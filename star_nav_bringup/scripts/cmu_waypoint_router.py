#!/usr/bin/env python3
"""Connects the exploration manager to the CMU local planner.

The CMU local planner looks about 3 m ahead and heads straight for its
waypoint, so a goal behind a wall leaves it facing the wall. In CMU's stack a
global planner (FAR planner, TARE) feeds it waypoints it can reach; here this
node does, for the exploration manager's goals:

* It keeps a 2D grid of the obstacles in CMU's terrain map (points higher
  than ``obstacle_height`` above the ground), which covers only the robot's
  surroundings. Cells near the robot follow the terrain map; cells farther
  away are remembered (static worlds only).
* It searches the grid (A*) for a route from the robot to the goal, keeping
  ``robot_radius`` clear of obstacles and preferring ``clearance``. Cells
  never seen are free.
* It sends the local planner the farthest point of the route, up to
  ``look_ahead`` along it, that is in a straight free line from the robot.
  Without a route (the goal is inside an obstacle) it sends the goal itself.

The local planner's paths are in the "vehicle" frame, the robot's position
and heading without roll and pitch, which this node also broadcasts so that
RViz can show them.

  goal (geometry_msgs/PoseStamped), odom (nav_msgs/Odometry),
  terrain_map (sensor_msgs/PointCloud2, intensity = height above the ground)
    ->  way_point (geometry_msgs/PointStamped), route (nav_msgs/Path),
        tf: odometry frame -> vehicle
"""

import heapq
import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2


def dilate(grid, radius):
    """``grid`` (bool) with every true cell grown to a disc of ``radius`` cells."""
    out = grid.copy()
    r = int(math.ceil(radius))
    n, m = grid.shape
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if (dx or dy) and dx * dx + dy * dy <= radius * radius:
                out[max(dx, 0):n + min(dx, 0), max(dy, 0):m + min(dy, 0)] |= \
                    grid[max(-dx, 0):n + min(-dx, 0), max(-dy, 0):m + min(-dy, 0)]
    return out


def find_route(blocked, cost, start, goal, max_nodes=200000):
    """Cheapest 8-connected route from ``start`` to ``goal`` (cells), or None.

    ``blocked`` cells cannot be entered; entering a cell costs its distance
    plus ``cost`` of the cell.
    """
    n, m = blocked.shape
    if blocked[goal]:
        return None
    diag = math.sqrt(2.0)
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, diag), (1, -1, diag), (-1, 1, diag), (-1, -1, diag)]

    def h(c):
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return max(dx, dy) + (diag - 1.0) * min(dx, dy)

    best = {start: 0.0}
    parent = {}
    heap = [(h(start), 0.0, start)]
    done = set()
    while heap and len(done) < max_nodes:
        _, g, c = heapq.heappop(heap)
        if c in done:
            continue
        if c == goal:
            route = [c]
            while c in parent:
                c = parent[c]
                route.append(c)
            return route[::-1]
        done.add(c)
        for dx, dy, d in moves:
            q = (c[0] + dx, c[1] + dy)
            if not (0 <= q[0] < n and 0 <= q[1] < m) or blocked[q] or q in done:
                continue
            gq = g + d + cost[q]
            if gq < best.get(q, np.inf):
                best[q] = gq
                parent[q] = c
                heapq.heappush(heap, (gq + h(q), gq, q))
    return None


def line_free(blocked, a, b):
    """Whether the straight line between cells ``a`` and ``b`` is free."""
    steps = int(2 * max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
    for t in np.linspace(0.0, 1.0, steps + 1):
        c = (int(round(a[0] + t * (b[0] - a[0]))), int(round(a[1] + t * (b[1] - a[1]))))
        if blocked[c]:
            return False
    return True


def cloud_xyzi(msg):
    """x, y, z, intensity of a PointCloud2 with float32 fields, (n, 4)."""
    off = {f.name: f.offset for f in msg.fields}
    names = ["x", "y", "z", "intensity"]
    dt = np.dtype({"names": names, "formats": ["<f4"] * 4,
                   "offsets": [off[k] for k in names], "itemsize": msg.point_step})
    a = np.frombuffer(msg.data, dt, count=msg.width * msg.height)
    pts = np.stack([a[k] for k in names], 1)
    return pts[np.isfinite(pts).all(1)]


class Router:
    """The obstacle grid and the search on it (no ROS)."""

    def __init__(self, size_x=50.0, size_y=50.0, resolution=0.1, robot_radius=0.25,
                 clearance=0.5, look_ahead=3.0, refresh_radius=3.5):
        self.res = resolution
        self.origin = np.array([-size_x / 2.0, -size_y / 2.0])
        self.shape = (int(math.ceil(size_x / resolution)), int(math.ceil(size_y / resolution)))
        self.occ = np.zeros(self.shape, bool)
        self.robot_radius = robot_radius
        self.clearance = clearance
        self.look_ahead = look_ahead
        self.refresh_radius = refresh_radius

    def cell(self, xy):
        ij = np.floor((np.asarray(xy, float) - self.origin) / self.res).astype(int)
        return (int(np.clip(ij[0], 0, self.shape[0] - 1)), int(np.clip(ij[1], 0, self.shape[1] - 1)))

    def point(self, cell):
        return self.origin + (np.asarray(cell) + 0.5) * self.res

    def update(self, obstacles_xy, robot_xy):
        """Obstacle points now in the terrain map around ``robot_xy``."""
        centres = self.origin + (np.indices(self.shape).transpose(1, 2, 0) + 0.5) * self.res
        near = np.hypot(centres[..., 0] - robot_xy[0], centres[..., 1] - robot_xy[1]) < self.refresh_radius
        self.occ[near] = False
        ij = np.floor((np.asarray(obstacles_xy, float).reshape(-1, 2) - self.origin) / self.res).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[1])
        self.occ[ij[ok, 0], ij[ok, 1]] = True

    def plan(self, robot_xy, goal_xy):
        """(waypoint, route): xy arrays; the goal and None without a route."""
        goal_xy = np.asarray(goal_xy, float)
        start, goal = self.cell(robot_xy), self.cell(goal_xy)
        blocked = dilate(self.occ, self.robot_radius / self.res)
        # The robot is where it is: let it leave a cell that counts as blocked.
        r = int(math.ceil(self.robot_radius / self.res)) + 1
        blocked[max(start[0] - r, 0):start[0] + r + 1, max(start[1] - r, 0):start[1] + r + 1] &= \
            self.occ[max(start[0] - r, 0):start[0] + r + 1, max(start[1] - r, 0):start[1] + r + 1]
        blocked[start] = False
        cost = 4.0 * dilate(self.occ, self.clearance / self.res)
        cells = find_route(blocked, cost, start, goal)
        if cells is None:
            return goal_xy, None
        route = np.array([self.point(c) for c in cells])
        route[0], route[-1] = robot_xy, goal_xy
        along = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(route, axis=0).T))])
        last = int(np.searchsorted(along, self.look_ahead, side="right")) - 1
        pick = min(1, len(cells) - 1)
        for i in range(last, pick, -1):
            if line_free(blocked, start, cells[i]):
                pick = i
                break
        return route[pick], route


class RouterNode:

    def __init__(self):
        self.router = Router(
            size_x=rospy.get_param("~map_size_x", 50.0),
            size_y=rospy.get_param("~map_size_y", 50.0),
            resolution=rospy.get_param("~resolution", 0.1),
            robot_radius=rospy.get_param("~robot_radius", 0.25),
            clearance=rospy.get_param("~clearance", 0.5),
            look_ahead=rospy.get_param("~look_ahead", 3.0),
            refresh_radius=rospy.get_param("~refresh_radius", 3.5))
        self.obstacle_height = rospy.get_param("~obstacle_height", 0.15)
        self.vehicle_frame = rospy.get_param("~vehicle_frame", "vehicle")
        self.pos = None
        self.goal = None
        self.frame = "world"

        self.way_point_pub = rospy.Publisher("way_point", PointStamped, queue_size=5)
        self.route_pub = rospy.Publisher("route", Path, queue_size=2)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("goal", PoseStamped, self.on_goal, queue_size=5)
        rospy.Subscriber("odom", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("terrain_map", PointCloud2, self.on_terrain, queue_size=1,
                         buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param("~rate", 4.0)), self.replan)

    def on_goal(self, msg):
        self.goal = msg.pose.position
        self.replan()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pos = np.array([p.x, p.y])
        self.frame = msg.header.frame_id
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = self.vehicle_frame
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_pub.sendTransform(t)

    def on_terrain(self, msg):
        if self.pos is None:
            return
        pts = cloud_xyzi(msg)
        self.router.update(pts[pts[:, 3] > self.obstacle_height, :2], self.pos)

    def replan(self, _=None):
        if self.pos is None or self.goal is None:
            return
        goal = self.goal
        way_point, route = self.router.plan(self.pos, [goal.x, goal.y])
        stamp = rospy.Time.now()
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame
        msg.point.x, msg.point.y, msg.point.z = way_point[0], way_point[1], goal.z
        self.way_point_pub.publish(msg)
        path = Path()
        path.header = msg.header
        for x, y in (route if route is not None else []):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, goal.z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.route_pub.publish(path)


if __name__ == "__main__":
    rospy.init_node("cmu_waypoint_router")
    RouterNode()
    rospy.spin()
