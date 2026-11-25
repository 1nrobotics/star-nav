#!/usr/bin/env python3
"""Frontier exploration for walking robots: map a place nobody described.

Takes the place of exploration_node and its preset waypoints
(``explore:=frontier`` of exploration_ground.launch). From the lidar scans it
keeps a 2D map of the floor (frontier_map.py: free, obstacle or unknown) and
sends the planner to the nearest *frontier*, a place the robot can walk to
that borders on floor it has not seen. It looks again whenever that frontier
has been seen, and is done when none is left; then it walks back to where it
started.

The goal it publishes is the frontier itself if the way there is straight,
otherwise the furthest point of the way (through seen, free floor) that is,
and at least ``min_goal_distance`` along it.
A frontier the robot does not get to in ``goal_timeout`` seconds, or stands
still in front of, is given up.

Subscribes ``cloud`` (lidar, world frame), ``lidar_pose``, ``odom`` and
/exploration/trigger. Publishes /move_base_simple/goal and
/exploration/state like exploration_node (WAIT_TRIGGER, PLAN_TO_FRONTIER,
EXEC_FRONTIER, RETURN_HOME, FINISH), and for RViz ``~map``
(nav_msgs/OccupancyGrid), ``~frontiers`` and ``~path``.
"""

import os
import sys
import threading

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# Next to this file (catkin's wrapper in devel/ does not add its directory).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frontier_map  # noqa: E402
from terrain_filter import cloud_xyz  # noqa: E402


class FrontierExplorer:

    def __init__(self):
        self.frame = rospy.get_param("~frame_id", "world")
        self.altitude = rospy.get_param("~altitude", 0.34)
        self.tolerance = rospy.get_param("~goal_tolerance", 0.3)
        self.goal_timeout = rospy.get_param("~goal_timeout", 60.0)
        self.stall_time = rospy.get_param("~stall_time", 6.0)
        self.max_time = rospy.get_param("~max_exploration_time", 900.0)
        self.return_home = rospy.get_param("~return_home", True)
        self.give_up_radius = rospy.get_param("~give_up_radius", 0.5)
        self.min_goal_distance = rospy.get_param("~min_goal_distance", 1.0)
        # Straight ahead: no obstacle within this distance of the line (m)
        self.sight_clearance = rospy.get_param("~sight_clearance", 0.25)
        # The planner finds its own way round the walls (the CMU planner's
        # waypoint router): it gets the frontier itself, not a point on the way.
        self.direct_goals = rospy.get_param("~direct_goals", False)
        clearance = rospy.get_param("~clearance", 0.4)
        bounds = None
        if rospy.has_param("~area_width"):
            cx, cy = rospy.get_param("~area_center_x", 0.0), rospy.get_param("~area_center_y", 0.0)
            w, h = rospy.get_param("~area_width"), rospy.get_param("~area_height")
            bounds = (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)
        self.bounds = bounds
        self.map = frontier_map.FrontierMap(
            size=(rospy.get_param("~map_size_x", 30.0), rospy.get_param("~map_size_y", 30.0)),
            resolution=rospy.get_param("~resolution", 0.1),
            robot_height=rospy.get_param("~robot_height", 0.55),
            max_step=rospy.get_param("~max_step", 0.09),
            clearance=clearance,
            max_range=rospy.get_param("~max_range", 8.0),
            min_cluster=rospy.get_param("~min_frontier_cells", 5),
            bounds=bounds)

        self.lock = threading.Lock()
        self.pos = None
        self.speed = 0.0
        self.sensor = None
        self.state = "WAIT_TRIGGER"
        self.triggered = False
        self.home = None
        self.target = None      # the frontier (or home) being walked to
        self.sent = None        # the goal last published
        self.sent_at = None
        self.target_since = None
        self.stopped_since = None
        self.start_time = None
        self.empty = 0
        self.finish_ticks = 0
        self.visited = 0

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.state_pub = rospy.Publisher("/exploration/state", String, queue_size=10)
        self.map_pub = rospy.Publisher("~map", OccupancyGrid, queue_size=1, latch=True)
        self.frontier_pub = rospy.Publisher("~frontiers", MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher("~path", Path, queue_size=1, latch=True)
        rospy.Subscriber("odom", Odometry, self.on_odom, queue_size=5)
        rospy.Subscriber("lidar_pose", PoseStamped, self.on_lidar_pose, queue_size=5)
        rospy.Subscriber("cloud", PointCloud2, self.on_cloud, queue_size=2, buff_size=2 ** 24)
        rospy.Subscriber("/exploration/trigger", PoseStamped, self.on_trigger, queue_size=10)
        rospy.Timer(rospy.Duration(rospy.get_param("~period", 0.5)), self.tick)

    # -- input -----------------------------------------------------------

    def on_odom(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.pos = np.array([p.x, p.y, p.z])
        self.speed = float(np.hypot(v.x, v.y))

    def on_lidar_pose(self, msg):
        p = msg.pose.position
        self.sensor = np.array([p.x, p.y, p.z])

    def on_cloud(self, msg):
        if self.sensor is None or self.pos is None:
            return
        pts = cloud_xyz(msg)
        with self.lock:
            self.map.insert(self.sensor, pts)
            self.map.mark_seen(self.pos[:2], 0.5)

    def on_trigger(self, _msg):
        self.triggered = True

    # -- state machine ---------------------------------------------------

    def tick(self, _event):
        if self.pos is None:
            return
        now = rospy.Time.now()
        if self.state == "WAIT_TRIGGER":
            if self.triggered and self.sensor is not None:
                self.triggered = False
                self.home = self.pos[:2].copy()
                self.start_time = now
                self.empty = 0
                self.visited = 0
                rospy.loginfo("[frontier] exploration triggered at (%.2f, %.2f)", *self.home)
                self.state = "PLAN_TO_FRONTIER"
        elif self.state == "FINISH":
            self.finish_ticks += 1
            if self.finish_ticks > 2:
                self.triggered = False
                self.state = "WAIT_TRIGGER"
        else:
            self.explore(now)
        self.state_pub.publish(String(data=self.state))

    def explore(self, now):
        with self.lock:
            grid = self.map.grid()
            clusters, dist = self.map.plan(self.pos[:2], grid)
            self.sight = ~frontier_map.dilate(grid == frontier_map.OBSTACLE,
                                              self.sight_clearance / self.map.res)
        self.publish_map(grid, now)
        self.publish_frontiers(clusters, now)

        if (now - self.start_time).to_sec() > self.max_time:
            rospy.logwarn("[frontier] Max exploration time reached")
            self.finish(grid)
            return

        if self.state == "RETURN_HOME":
            if self.arrived(self.home):
                self.finish(grid)
            elif self.stuck(now):
                rospy.logwarn("[frontier] TIMEOUT: did not get home, %.2f m away at (%.2f, %.2f)",
                              np.hypot(*(self.pos[:2] - self.home)), self.pos[0], self.pos[1])
                self.finish(grid)
            else:
                self.steer(self.home, dist, now)
            return

        if self.state == "EXEC_FRONTIER":
            if not self.still_frontier(clusters):
                rospy.loginfo("[frontier] frontier (%.2f, %.2f) has been seen", *self.target)
                self.visited += 1
                self.state = "PLAN_TO_FRONTIER"
            elif self.arrived(self.target):
                # Standing on it and it is still there: nothing more to see
                # from here.
                rospy.loginfo("[frontier] reached frontier (%.2f, %.2f)", *self.target)
                self.drop(self.target)
                self.visited += 1
                self.state = "PLAN_TO_FRONTIER"
            elif self.stuck(now):
                rospy.logwarn("[frontier] TIMEOUT: gave up frontier (%.2f, %.2f), %.2f m away at (%.2f, %.2f)",
                              self.target[0], self.target[1],
                              np.hypot(*(self.pos[:2] - self.target)), self.pos[0], self.pos[1])
                self.drop(self.target)
                self.state = "PLAN_TO_FRONTIER"
            else:
                self.steer(self.target, dist, now)
                return
            with self.lock:
                clusters, dist = self.map.plan(self.pos[:2], grid)

        # PLAN_TO_FRONTIER
        if not clusters:
            # A scan or two may still be on its way.
            self.empty += 1
            if self.empty < 4:
                return
            rospy.loginfo("[frontier] no frontier left after %d, %.0f s",
                          self.visited, (now - self.start_time).to_sec())
            if self.return_home and not self.arrived(self.home):
                self.begin(self.home, now)
                self.state = "RETURN_HOME"
                self.steer(self.home, dist, now)
            else:
                self.finish(grid)
            return
        self.empty = 0
        best = clusters[0]
        self.begin(self.map.point(best.goal), now)
        rospy.loginfo("[frontier] frontier %d: (%.2f, %.2f), %.1f m away, %d cells, %d frontiers left",
                      self.visited, self.target[0], self.target[1], best.cost, len(best), len(clusters))
        self.state = "EXEC_FRONTIER"
        self.steer(self.target, dist, now)

    def begin(self, target, now):
        self.target = np.array(target, float)
        self.target_since = now
        self.stopped_since = None
        self.sent = None
        self.sent_at = None

    def drop(self, target):
        with self.lock:
            self.map.give_up(target, self.give_up_radius)

    def finish(self, grid):
        known = float((grid != frontier_map.UNKNOWN).sum()) * self.map.res ** 2
        rospy.loginfo("[frontier] exploration finished: %d frontiers, %.1f m2 mapped", self.visited, known)
        self.finish_ticks = 0
        self.state = "FINISH"

    def arrived(self, target):
        return np.hypot(*(self.pos[:2] - target)) < self.tolerance

    def stuck(self, now):
        if (now - self.target_since).to_sec() > self.goal_timeout:
            return True
        if self.speed > 0.05:
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = now
        elif (now - self.stopped_since).to_sec() > self.stall_time:
            return True
        return False

    def still_frontier(self, clusters):
        """Is a frontier left within give_up_radius of the target?"""
        cell = np.array(self.map.cell(self.target))
        r2 = (self.give_up_radius / self.map.res) ** 2
        return any((((np.array(c.cells) - cell) ** 2).sum(1) <= r2).any() for c in clusters)

    # -- goals -----------------------------------------------------------

    def steer(self, target, dist, now):
        """Publish the goal on the way to ``target``: the furthest point of
        the way that lies straight ahead."""
        with self.lock:
            cell = self.map._nearest(np.isfinite(dist), self.map.cell(target), 1.0)
            cells = self.map.path(dist, cell) if cell is not None else []
        goal = np.array(target, float)
        if cells and self.direct_goals:
            goal = self.map.point(cells[-1])
        elif cells:
            here = self.map.cell(self.pos[:2])
            k = len(cells) - 1
            while k > 0 and not self.straight(self.sight, here, cells[k]):
                k -= 1
            # Not a goal at the robot's feet: the planner would stop there.
            k = max(k, min(len(cells) - 1, int(self.min_goal_distance / self.map.res)))
            if k < len(cells) - 1:
                goal = self.map.point(cells[k])
        self.publish_path(cells, now)
        # Again only if it moved, or if the robot stands still with the old one.
        waiting = self.speed < 0.05 and (now - self.sent_at).to_sec() > 3.0 if self.sent_at else False
        if self.sent is not None and np.hypot(*(goal - self.sent)) < 0.5 and not waiting:
            return
        self.sent = goal
        self.sent_at = now
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = now
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal[0], goal[1], self.altitude
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    @staticmethod
    def straight(mask, a, b):
        """All cells on the segment a-b are in ``mask``."""
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
        i = np.rint(np.linspace(a[0], b[0], n)).astype(int)
        j = np.rint(np.linspace(a[1], b[1], n)).astype(int)
        return bool(mask[i, j].all())

    # -- RViz ------------------------------------------------------------

    def publish_map(self, grid, now):
        if not self.map_pub.get_num_connections():
            return
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.header.stamp = now
        msg.info.resolution = self.map.res
        msg.info.width, msg.info.height = self.map.shape
        msg.info.origin.position.x, msg.info.origin.position.y = self.map.origin
        msg.info.origin.orientation.w = 1.0
        # Row-major, x along the row: grid is indexed [x, y].
        msg.data = grid.T.ravel().tolist()
        self.map_pub.publish(msg)

    def publish_frontiers(self, clusters, now):
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        cells = Marker(type=Marker.CUBE_LIST, ns="frontier_cells", id=0)
        cells.header.frame_id = self.frame
        cells.header.stamp = now
        cells.pose.orientation.w = 1.0
        cells.scale.x = cells.scale.y = self.map.res
        cells.scale.z = 0.02
        cells.color.r, cells.color.g, cells.color.b, cells.color.a = 1.0, 0.6, 0.0, 0.9
        for i, c in enumerate(clusters):
            for cell in c.cells:
                x, y = self.map.point(cell)
                cells.points.append(Point(x=x, y=y, z=0.02))
            goal = Marker(type=Marker.SPHERE, ns="frontiers", id=i)
            goal.header = cells.header
            goal.pose.position.x, goal.pose.position.y = self.map.point(c.goal)
            goal.pose.position.z = self.altitude
            goal.pose.orientation.w = 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.2
            goal.color.r, goal.color.g, goal.color.b, goal.color.a = 1.0, 0.25, 0.85, 0.7
            markers.markers.append(goal)
        if cells.points:
            markers.markers.append(cells)
        self.frontier_pub.publish(markers)

    def publish_path(self, cells, now):
        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = now
        for cell in cells:
            pose = PoseStamped(header=msg.header)
            pose.pose.position.x, pose.pose.position.y = self.map.point(cell)
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("frontier_explorer")
    FrontierExplorer()
    rospy.spin()
