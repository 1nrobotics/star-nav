#!/usr/bin/env python3
"""End-to-end smoke test for the exploration simulation.

Triggers exploration and checks that the mission reaches FINISH. Waypoint mode
(~mode): it visits the expected number of waypoints without timeouts and the
flown path covers the exploration area. Frontier mode: it observes at least
~min_coverage of the area, returns to the start and blocks at most
~max_blocked goals. With ~contacts_topic set (MuJoCo simulator), it also
checks that the quadrotor never touched an obstacle. With ~obstacles_topic set,
it also checks the sensor cloud and the planner's map against the true world
(map_accuracy.py).
"""

import math
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import Float32, String, UInt32

from map_accuracy import MapAccuracyMonitor

PKG = "star_nav_bringup"


class SimSmokeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.states = []
        self.problems = []
        self.min_xy = [float("inf"), float("inf")]
        self.max_xy = [float("-inf"), float("-inf")]
        self.last_xy = None
        self.path_length = 0.0

        self.odom_topic = rospy.get_param("~odom_topic", "/drone_0_visual_slam/odom")
        self.mode = rospy.get_param("~mode", "waypoint")
        self.min_coverage = rospy.get_param("~min_coverage", 0.9)
        self.max_blocked = rospy.get_param("~max_blocked", 2)
        self.coverage = None
        self.blocked = []
        self.expected_waypoints = rospy.get_param("~expected_waypoints", 12)
        self.width = rospy.get_param("~exploration_width", 9.0)
        self.height = rospy.get_param("~exploration_height", 9.0)
        self.coverage_ratio = rospy.get_param("~coverage_ratio", 0.8)
        self.startup_timeout = rospy.get_param("~startup_timeout", 60.0)
        self.settle_time = rospy.get_param("~settle_time", 5.0)
        self.mission_timeout = rospy.get_param("~mission_timeout", 180.0)
        self.contacts_topic = rospy.get_param("~contacts_topic", "")
        self.contacts = None

        self.accuracy = None
        obstacles_topic = rospy.get_param("~obstacles_topic", "")
        if obstacles_topic:
            self.accuracy = MapAccuracyMonitor(
                obstacles_topic,
                rospy.get_param("~map_topic", "/drone_0_ego_planner_node/grid_map/occupancy_inflate"),
                self.odom_topic,
                cloud_topic=rospy.get_param("~sensor_cloud_topic", ""))
            self.max_cloud_error_ratio = rospy.get_param("~max_cloud_error_ratio", 0.001)
            self.max_phantom_ratio = rospy.get_param("~max_phantom_ratio", 0.01)
            self.min_map_recall = rospy.get_param("~min_map_recall", -1.0)

        rospy.Subscriber("/exploration/state", String, self.state_cb, queue_size=100)
        rospy.Subscriber("/exploration/coverage", Float32, self.coverage_cb, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        # /rosout, not /rosout_agg: rostest runs no rosout node to aggregate.
        rospy.Subscriber("/rosout", Log, self.log_cb, queue_size=100)
        self.trigger_pub = rospy.Publisher("/exploration/trigger", PoseStamped, queue_size=1)
        if self.contacts_topic:
            rospy.Subscriber(self.contacts_topic, UInt32, self.contacts_cb, queue_size=10)

    def state_cb(self, msg):
        # The state is republished periodically; keep transitions only.
        with self.lock:
            if not self.states or self.states[-1] != msg.data:
                self.states.append(msg.data)

    def coverage_cb(self, msg):
        with self.lock:
            self.coverage = msg.data

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.min_xy = [min(self.min_xy[0], p.x), min(self.min_xy[1], p.y)]
            self.max_xy = [max(self.max_xy[0], p.x), max(self.max_xy[1], p.y)]
            if self.last_xy is not None:
                self.path_length += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
            self.last_xy = (p.x, p.y)

    def contacts_cb(self, msg):
        with self.lock:
            self.contacts = msg.data

    def log_cb(self, msg):
        if msg.name != "/exploration_node":
            return
        with self.lock:
            if ("TIMEOUT" in msg.msg or "Max exploration time" in msg.msg
                    or "Could not return" in msg.msg):
                self.problems.append(msg.msg)
            elif "blocking it" in msg.msg:
                self.blocked.append(msg.msg)

    def wait_for(self, predicate, timeout, what):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if predicate():
                    return
            rate.sleep()
        self.fail("Timed out after %.0f s waiting for %s" % (timeout, what))

    def test_exploration_mission(self):
        rospy.wait_for_message(self.odom_topic, Odometry, timeout=self.startup_timeout)
        self.wait_for(lambda: self.trigger_pub.get_num_connections() > 0,
                      self.startup_timeout, "exploration_node to subscribe to the trigger")
        # Give the planner time to receive the map before the first goal arrives.
        rospy.sleep(self.settle_time)

        # Re-publish until the FSM leaves WAIT_TRIGGER; the trigger is not latched.
        started = "EXPLORE" if self.mode == "frontier" else "GENERATE_WAYPOINTS"
        deadline = rospy.Time.now() + rospy.Duration(self.startup_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if started in self.states:
                    break
            self.trigger_pub.publish(PoseStamped())
            rospy.sleep(1.0)
        with self.lock:
            self.assertIn(started, self.states, "exploration was never triggered")
            self.min_xy = [float("inf"), float("inf")]
            self.max_xy = [float("-inf"), float("-inf")]
            self.path_length = 0.0
            start = rospy.Time.now()
        if self.accuracy:
            self.accuracy.start()

        self.wait_for(lambda: "FINISH" in self.states, self.mission_timeout, "FINISH state")

        with self.lock:
            reached = self.states.count("EXEC_WAYPOINT")
            span_x = self.max_xy[0] - self.min_xy[0]
            span_y = self.max_xy[1] - self.min_xy[1]
            problems = list(self.problems)
            blocked = list(self.blocked)
            coverage = self.coverage
            contacts = self.contacts
            path_length = self.path_length

        # print: the summary also goes to the rostest output.
        print("[sim_smoke] mode=%s time=%.0fs path=%.1fm waypoints=%d span=(%.2f, %.2f) "
              "coverage=%s blocked=%d problems=%d contacts=%s"
              % (self.mode, (rospy.Time.now() - start).to_sec(), path_length, reached, span_x,
                 span_y, coverage, len(blocked), len(problems), contacts))
        self.assertEqual(problems, [], "exploration reported: %s" % problems)
        if self.mode == "frontier":
            self.assertIsNotNone(coverage, "no message on /exploration/coverage")
            self.assertGreaterEqual(coverage, self.min_coverage, "too little of the area observed")
            self.assertLessEqual(len(blocked), self.max_blocked, "goals blocked: %s" % blocked)
        else:
            self.assertEqual(reached, self.expected_waypoints, "unexpected number of waypoints")
            self.assertGreaterEqual(span_x, self.coverage_ratio * self.width, "x coverage too small")
            self.assertGreaterEqual(span_y, self.coverage_ratio * self.height, "y coverage too small")
        if self.contacts_topic:
            self.assertIsNotNone(contacts, "no message on %s" % self.contacts_topic)
            self.assertEqual(contacts, 0, "the quadrotor collided with an obstacle")
        if self.accuracy:
            self.check_accuracy(self.accuracy.stop())

    def check_accuracy(self, s):
        rospy.loginfo("[sim_smoke] map accuracy: %s", s)
        self.assertGreater(s["voxels"], 0, "no planner map received")
        phantom_ratio = float(s["phantoms"]) / s["voxels"]
        self.assertLessEqual(phantom_ratio, self.max_phantom_ratio,
                             "map has occupied voxels away from any obstacle")
        if self.accuracy.cloud_topic:
            self.assertGreater(s["cloud_points"], 0, "no sensor cloud received")
            bad_ratio = float(s["cloud_bad"]) / s["cloud_points"]
            self.assertLessEqual(bad_ratio, self.max_cloud_error_ratio,
                                 "sensor points off the obstacle surfaces (max %.3f m)"
                                 % s["cloud_max"])
        if self.min_map_recall >= 0:
            self.assertGreater(s["pillars"], 0, "no pillars passed near the drone")
            recall = float(s["pillars_seen"]) / s["pillars"]
            self.assertGreaterEqual(recall, self.min_map_recall,
                                    "pillars near the drone missing from the map")


if __name__ == "__main__":
    rospy.init_node("sim_smoke_test")
    rostest.rosrun(PKG, "sim_smoke", SimSmokeTest)
