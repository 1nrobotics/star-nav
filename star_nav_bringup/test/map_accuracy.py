"""Checks a sensor point cloud and the planner's map against the true world.

Used by sim_smoke_test.py with the MuJoCo simulator, whose bridge publishes the
exact obstacle geometry on /mujoco/obstacles (pillars as cylinders, rings as the
polyline of capsules the simulator builds). Distances are to the nearest
surface: the ground plane, a pillar or a ring tube.
"""

import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


def cloud_to_xyz(msg):
    """x, y, z float32 fields of a PointCloud2 as an (n, 3) array."""
    offsets = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    cols = [raw[:, offsets[k]:offsets[k] + 4].copy().view("<f4").ravel() for k in "xyz"]
    return np.stack(cols, axis=1).astype(float)


class World(object):
    def __init__(self, markers):
        pillars, segments = [], []
        self.tube = 0.0
        for m in markers.markers:
            if m.type == Marker.CYLINDER:
                pillars.append((m.pose.position.x, m.pose.position.y, m.scale.x / 2.0, m.scale.z))
            elif m.type == Marker.LINE_STRIP:
                pts = np.array([(p.x, p.y, p.z) for p in m.points])
                segments.append(np.concatenate([pts[:-1], pts[1:]], axis=1))
                self.tube = m.scale.x / 2.0
        self.pillars = np.array(pillars).reshape(-1, 4)
        self.segments = np.concatenate(segments) if segments else np.zeros((0, 6))

    def distance(self, p, solid=False):
        """Distance from points (n, 3) to the nearest surface.

        With solid=True, points inside an obstacle or below the ground count
        as 0 (for map voxels); otherwise the distance is unsigned.
        """
        best = np.maximum(p[:, 2], 0.0) if solid else np.abs(p[:, 2])
        if len(self.pillars):
            x, y, a, b = self.pillars.T
            dr = np.hypot(p[:, None, 0] - x, p[:, None, 1] - y) - a
            dz = p[:, None, 2] - b
            inside = (dr <= 0) & (dz <= 0)
            d = np.where(inside, 0.0 if solid else np.minimum(-dr, -dz),
                         np.hypot(np.maximum(dr, 0), np.maximum(dz, 0)))
            best = np.minimum(best, d.min(axis=1))
        if len(self.segments):
            a, b = self.segments[:, :3], self.segments[:, 3:]
            ab = b - a
            ap = p[:, None, :] - a[None]
            t = np.clip((ap * ab).sum(2) / (ab * ab).sum(1), 0.0, 1.0)
            closest = a[None] + t[..., None] * ab[None]
            d = np.linalg.norm(p[:, None, :] - closest, axis=2) - self.tube
            d = np.maximum(d, 0.0) if solid else np.abs(d)
            best = np.minimum(best, d.min(axis=1))
        return best

    def pillars_near(self, pos, radius, min_height):
        """Pillars whose surface is within radius of pos (horizontally)."""
        if not len(self.pillars):
            return self.pillars
        x, y, a, b = self.pillars.T
        d = np.hypot(x - pos[0], y - pos[1]) - a
        return self.pillars[(d < radius) & (d > 0.3) & (b >= min_height)]


class MapAccuracyMonitor(object):
    """Collects sensor and map errors while the mission runs.

    cloud_tol:   a sensor point is wrong if it is farther than this from any
                 surface (m)
    phantom_tol: an occupied map voxel is a phantom if it is farther than this
                 from any surface (m); covers inflation and voxel size
    recall_*:    tall pillars within recall_radius of the drone must have an
                 occupied voxel within recall_tol of their surface at flight
                 height
    """

    def __init__(self, obstacles_topic, map_topic, odom_topic, cloud_topic="",
                 cloud_tol=0.02, phantom_tol=0.45, recall_radius=2.5, recall_tol=0.25,
                 period=1.0):
        self.lock = threading.Lock()
        self.world = None
        self.pos = None
        self.cloud_topic = cloud_topic
        self.cloud_tol = cloud_tol
        self.phantom_tol = phantom_tol
        self.recall_radius = recall_radius
        self.recall_tol = recall_tol
        self.period = rospy.Duration(period)
        self.last_cloud = rospy.Time(0)
        self.last_map = rospy.Time(0)
        self.active = False
        self.stats = dict(cloud_points=0, cloud_bad=0, cloud_max=0.0,
                          voxels=0, phantoms=0, pillars=0, pillars_seen=0)
        rospy.Subscriber(obstacles_topic, MarkerArray, self._on_world, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(map_topic, PointCloud2, self._on_map, queue_size=1,
                         buff_size=2 ** 24)
        if cloud_topic:
            rospy.Subscriber(cloud_topic, PointCloud2, self._on_cloud, queue_size=1,
                             buff_size=2 ** 24)

    def start(self):
        with self.lock:
            self.active = True

    def stop(self):
        with self.lock:
            self.active = False
            return dict(self.stats)

    def _on_world(self, msg):
        world = World(msg)
        with self.lock:
            self.world = world

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.pos = np.array([p.x, p.y, p.z])

    def _ready(self, stamp_attr):
        with self.lock:
            if not self.active or self.world is None or self.pos is None:
                return None
            now = rospy.Time.now()
            if now - getattr(self, stamp_attr) < self.period:
                return None
            setattr(self, stamp_attr, now)
            return self.world, self.pos.copy()

    def _on_cloud(self, msg):
        ready = self._ready("last_cloud")
        if ready is None:
            return
        world, _ = ready
        pts = cloud_to_xyz(msg)
        if len(pts) > 4000:
            pts = pts[np.random.choice(len(pts), 4000, replace=False)]
        d = world.distance(pts)
        with self.lock:
            s = self.stats
            s["cloud_points"] += len(d)
            s["cloud_bad"] += int((d > self.cloud_tol).sum())
            s["cloud_max"] = max(s["cloud_max"], float(d.max()) if len(d) else 0.0)

    def _on_map(self, msg):
        ready = self._ready("last_map")
        if ready is None:
            return
        world, pos = ready
        vox = cloud_to_xyz(msg)
        if not len(vox):
            return
        sample = vox if len(vox) <= 5000 else vox[np.random.choice(len(vox), 5000, replace=False)]
        phantoms = int((world.distance(sample, solid=True) > self.phantom_tol).sum())

        # Recall: each tall pillar near the drone shows up at flight height.
        band = vox[np.abs(vox[:, 2] - pos[2]) < 0.3]
        seen = 0
        pillars = world.pillars_near(pos, self.recall_radius, pos[2] + 0.3)
        for x, y, a, _ in pillars:
            d = np.abs(np.hypot(band[:, 0] - x, band[:, 1] - y) - a)
            seen += int(len(d) > 0 and d.min() < self.recall_tol)
        with self.lock:
            s = self.stats
            s["voxels"] += len(sample)
            s["phantoms"] += phantoms
            s["pillars"] += len(pillars)
            s["pillars_seen"] += seen
