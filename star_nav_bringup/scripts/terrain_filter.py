#!/usr/bin/env python3
"""Terrain filter for walking robots: lidar cloud -> obstacle columns.

The planner's map treats every voxel above the floor as an obstacle, so a
ramp or a step would block a walking robot. This node keeps a 2.5D grid of
the lidar returns (world frame) and decides per cell whether it can be
walked on:

* Each cell keeps its lowest return (the ground there) and the highest
  return within ``robot_height`` above it. Higher returns (a table top)
  are overhead, not obstacles.
* A cell is an obstacle if those two differ by more than ``max_step`` (a
  wall, a pillar, a table leg) or if its ground is more than ``max_step``
  above a neighbour's, but less than ``robot_height`` (the top of a box, a
  cliff edge). Ramps and steps up to ``max_step`` stay free, and so does a
  cell whose only returns are overhead.

Obstacle cells within ``radius`` of the robot are published as vertical
columns of points (``column_min_z`` to ``column_max_z``) for grid_map's
point-cloud input in replace mode (``enable_pointcloud_raycasting`` false),
so the planner sees them at any height and never plans over them.

Static worlds only: cells are never cleared. Subscribes ``cloud`` (world
frame) and ``odom``; publishes ``obstacles``.
"""

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def cloud_xyz(msg):
    """x, y, z of a PointCloud2 with float32 fields, as an (n, 3) array."""
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(msg.data, dt, count=msg.width * msg.height)
    pts = np.stack([a["x"], a["y"], a["z"]], 1)
    return pts[np.isfinite(pts).all(1)]


def make_cloud(header, pts):
    msg = PointCloud2(header=header, height=1, width=len(pts), is_dense=True,
                      is_bigendian=False, point_step=12, row_step=12 * len(pts))
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate("xyz")]
    msg.data = np.ascontiguousarray(pts, "<f4").tobytes()
    return msg


class TerrainFilter:

    def __init__(self):
        self.res = rospy.get_param("~resolution", 0.1)
        size_x = rospy.get_param("~map_size_x", 50.0)
        size_y = rospy.get_param("~map_size_y", 50.0)
        self.robot_height = rospy.get_param("~robot_height", 0.55)
        self.max_step = rospy.get_param("~max_step", 0.09)
        self.radius = rospy.get_param("~radius", 7.0)
        z0 = rospy.get_param("~column_min_z", 0.0)
        z1 = rospy.get_param("~column_max_z", 1.9)
        self.frame = rospy.get_param("~frame_id", "world")

        self.origin = np.array([-size_x / 2.0, -size_y / 2.0])
        self.shape = (int(np.ceil(size_x / self.res)), int(np.ceil(size_y / self.res)))
        self.ground = np.full(self.shape, np.inf, np.float32)
        self.top = np.full(self.shape, -np.inf, np.float32)
        self.column = np.arange(z0, z1 + 1e-6, self.res, dtype=np.float32)
        self.pos = None

        self.pub = rospy.Publisher("obstacles", PointCloud2, queue_size=2)
        rospy.Subscriber("odom", Odometry, self.on_odom, queue_size=5)
        rospy.Subscriber("cloud", PointCloud2, self.on_cloud, queue_size=2,
                         buff_size=2 ** 24)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pos = np.array([p.x, p.y])

    def on_cloud(self, msg):
        if self.pos is None:
            return
        pts = cloud_xyz(msg)
        if len(pts):
            self.insert(pts)
        self.publish(msg.header.stamp)

    def insert(self, pts):
        ij = np.floor((pts[:, :2] - self.origin) / self.res).astype(np.int64)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[1])
        ij, z = ij[ok], pts[ok, 2]
        flat = ij[:, 0] * self.shape[1] + ij[:, 1]
        ground = self.ground.ravel()
        np.minimum.at(ground, flat, z)
        band = z <= ground[flat] + self.robot_height
        np.maximum.at(self.top.ravel(), flat[band], z[band])

    def obstacles(self):
        g, t = self.ground, self.top
        seen = np.isfinite(g)
        tall = seen & (t - g > self.max_step)
        # Lowest observed neighbouring ground (inf where none).
        pad = np.pad(g, 1, constant_values=np.inf)
        lowest = np.full_like(g, np.inf)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx or dy:
                    lowest = np.minimum(lowest, pad[1 + dx:1 + dx + g.shape[0], 1 + dy:1 + dy + g.shape[1]])
        # A cell more than robot_height above its neighbours is overhead (a
        # table top whose floor is not seen yet), not a ledge.
        with np.errstate(invalid="ignore"):
            rise = g - lowest
            ledge = seen & (rise > self.max_step) & (rise < self.robot_height)
        return tall | ledge

    def publish(self, stamp):
        occ = self.obstacles()
        cells = np.argwhere(occ)
        xy = self.origin + (cells + 0.5) * self.res
        xy = xy[np.hypot(*(xy - self.pos).T) < self.radius]
        n, m = len(xy), len(self.column)
        pts = np.empty((n * m, 3), np.float32)
        pts[:, :2] = np.repeat(xy, m, axis=0)
        pts[:, 2] = np.tile(self.column, n)
        header = Header(stamp=stamp, frame_id=self.frame)
        self.pub.publish(make_cloud(header, pts))


if __name__ == "__main__":
    rospy.init_node("terrain_filter")
    TerrainFilter()
    rospy.spin()
