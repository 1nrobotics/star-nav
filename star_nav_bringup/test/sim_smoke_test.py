#!/usr/bin/env python3
"""End-to-end smoke test for the waypoint exploration simulation.

Triggers exploration and checks that the mission reaches FINISH, visits the
expected number of waypoints without timeouts, and that the flown path covers
the exploration area. With ~contacts_topic set (MuJoCo simulator), it also
checks that the quadrotor never touched an obstacle.
"""

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import String, UInt32

PKG = "star_nav_bringup"


class SimSmokeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.states = []
        self.problems = []
        self.min_xy = [float("inf"), float("inf")]
        self.max_xy = [float("-inf"), float("-inf")]

        self.odom_topic = rospy.get_param("~odom_topic", "/drone_0_visual_slam/odom")
        self.expected_waypoints = rospy.get_param("~expected_waypoints", 12)
        self.width = rospy.get_param("~exploration_width", 9.0)
        self.height = rospy.get_param("~exploration_height", 9.0)
        self.coverage_ratio = rospy.get_param("~coverage_ratio", 0.8)
        self.startup_timeout = rospy.get_param("~startup_timeout", 60.0)
        self.settle_time = rospy.get_param("~settle_time", 5.0)
        self.mission_timeout = rospy.get_param("~mission_timeout", 180.0)
        self.contacts_topic = rospy.get_param("~contacts_topic", "")
        self.contacts = None

        rospy.Subscriber("/exploration/state", String, self.state_cb, queue_size=100)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/rosout_agg", Log, self.log_cb, queue_size=100)
        self.trigger_pub = rospy.Publisher("/exploration/trigger", PoseStamped, queue_size=1)
        if self.contacts_topic:
            rospy.Subscriber(self.contacts_topic, UInt32, self.contacts_cb, queue_size=10)

    def state_cb(self, msg):
        # The state is republished periodically; keep transitions only.
        with self.lock:
            if not self.states or self.states[-1] != msg.data:
                self.states.append(msg.data)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.min_xy = [min(self.min_xy[0], p.x), min(self.min_xy[1], p.y)]
            self.max_xy = [max(self.max_xy[0], p.x), max(self.max_xy[1], p.y)]

    def contacts_cb(self, msg):
        with self.lock:
            self.contacts = msg.data

    def log_cb(self, msg):
        if msg.name != "/exploration_node":
            return
        if "TIMEOUT" in msg.msg or "Max exploration time" in msg.msg:
            with self.lock:
                self.problems.append(msg.msg)

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
        deadline = rospy.Time.now() + rospy.Duration(self.startup_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if "GENERATE_WAYPOINTS" in self.states:
                    break
            self.trigger_pub.publish(PoseStamped())
            rospy.sleep(1.0)
        with self.lock:
            self.assertIn("GENERATE_WAYPOINTS", self.states, "exploration was never triggered")
            self.min_xy = [float("inf"), float("inf")]
            self.max_xy = [float("-inf"), float("-inf")]

        self.wait_for(lambda: "FINISH" in self.states, self.mission_timeout, "FINISH state")

        with self.lock:
            reached = self.states.count("EXEC_WAYPOINT")
            span_x = self.max_xy[0] - self.min_xy[0]
            span_y = self.max_xy[1] - self.min_xy[1]
            problems = list(self.problems)
            contacts = self.contacts

        rospy.loginfo("[sim_smoke] waypoints=%d span=(%.2f, %.2f) problems=%d contacts=%s",
                      reached, span_x, span_y, len(problems), contacts)
        self.assertEqual(problems, [], "exploration reported: %s" % problems)
        self.assertEqual(reached, self.expected_waypoints, "unexpected number of waypoints")
        self.assertGreaterEqual(span_x, self.coverage_ratio * self.width, "x coverage too small")
        self.assertGreaterEqual(span_y, self.coverage_ratio * self.height, "y coverage too small")
        if self.contacts_topic:
            self.assertIsNotNone(contacts, "no message on %s" % self.contacts_topic)
            self.assertEqual(contacts, 0, "the quadrotor collided with an obstacle")


if __name__ == "__main__":
    rospy.init_node("sim_smoke_test")
    rostest.rosrun(PKG, "sim_smoke", SimSmokeTest)
