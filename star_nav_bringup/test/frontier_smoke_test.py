#!/usr/bin/env python3
"""End-to-end test of frontier exploration (scripts/frontier_explorer.py).

Triggers exploration and checks that it reaches FINISH without giving up a
frontier, that the explorer's map then covers ~min_coverage of ~area (xmin,
ymin, xmax, ymax: the floor of the place), that the robot ended where it
started and, with ~contacts_topic set, that it never bumped into anything.
"""

import threading
import unittest

import numpy as np
import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import String, UInt32

PKG = "star_nav_bringup"


class FrontierSmokeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.states = []
        self.problems = []
        self.frontiers = 0
        self.pos = None
        self.track = 0.0
        self.contacts = None
        self.map = None

        self.odom_topic = rospy.get_param("~odom_topic", "/drone_0_visual_slam/odom")
        self.area = rospy.get_param("~area", [-5.4, -5.4, 5.4, 5.4])
        self.min_coverage = rospy.get_param("~min_coverage", 0.95)
        self.home_tolerance = rospy.get_param("~home_tolerance", 0.6)
        self.startup_timeout = rospy.get_param("~startup_timeout", 60.0)
        self.settle_time = rospy.get_param("~settle_time", 5.0)
        self.mission_timeout = rospy.get_param("~mission_timeout", 480.0)
        self.contacts_topic = rospy.get_param("~contacts_topic", "")

        rospy.Subscriber("/exploration/state", String, self.state_cb, queue_size=100)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/frontier_explorer/map", OccupancyGrid, self.map_cb, queue_size=1)
        # /rosout, not /rosout_agg: rostest runs no rosout node to aggregate.
        rospy.Subscriber("/rosout", Log, self.log_cb, queue_size=100)
        self.trigger_pub = rospy.Publisher("/exploration/trigger", PoseStamped, queue_size=1)
        if self.contacts_topic:
            rospy.Subscriber(self.contacts_topic, UInt32, self.contacts_cb, queue_size=10)

    def state_cb(self, msg):
        with self.lock:
            if not self.states or self.states[-1] != msg.data:
                self.states.append(msg.data)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y])
        with self.lock:
            if self.pos is not None:
                self.track += float(np.hypot(*(pos - self.pos)))
            self.pos = pos

    def map_cb(self, msg):
        with self.lock:
            self.map = msg

    def contacts_cb(self, msg):
        with self.lock:
            self.contacts = msg.data

    def log_cb(self, msg):
        if msg.name != "/frontier_explorer":
            return
        with self.lock:
            if "TIMEOUT" in msg.msg or "Max exploration time" in msg.msg:
                self.problems.append(msg.msg)
            elif "has been seen" in msg.msg or "reached frontier" in msg.msg:
                self.frontiers += 1

    def wait_for(self, predicate, timeout, what):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if predicate():
                    return
            rate.sleep()
        self.fail("Timed out after %.0f s waiting for %s" % (timeout, what))

    def coverage(self, msg):
        """Fraction of the cells of ~area that are free or an obstacle."""
        grid = np.array(msg.data, np.int8).reshape(msg.info.height, msg.info.width)
        res, o = msg.info.resolution, msg.info.origin.position
        x = o.x + (np.arange(msg.info.width) + 0.5) * res
        y = o.y + (np.arange(msg.info.height) + 0.5) * res
        part = grid[np.ix_((y >= self.area[1]) & (y <= self.area[3]),
                           (x >= self.area[0]) & (x <= self.area[2]))]
        return float((part >= 0).mean()), float((part == 0).sum()) * res * res

    def test_frontier_exploration(self):
        rospy.wait_for_message(self.odom_topic, Odometry, timeout=self.startup_timeout)
        self.wait_for(lambda: self.trigger_pub.get_num_connections() > 0,
                      self.startup_timeout, "frontier_explorer to subscribe to the trigger")
        rospy.sleep(self.settle_time)

        deadline = rospy.Time.now() + rospy.Duration(self.startup_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if "EXEC_FRONTIER" in self.states:
                    break
            self.trigger_pub.publish(PoseStamped())
            rospy.sleep(1.0)
        with self.lock:
            self.assertIn("EXEC_FRONTIER", self.states, "exploration was never triggered")
            home = self.pos.copy()
            self.track = 0.0
        start = rospy.Time.now()

        self.wait_for(lambda: "FINISH" in self.states, self.mission_timeout, "FINISH state")
        took = (rospy.Time.now() - start).to_sec()

        with self.lock:
            frontiers = self.frontiers
            problems = list(self.problems)
            contacts = self.contacts
            last_map = self.map
            off_home = float(np.hypot(*(self.pos - home)))
            track = self.track
        self.assertIsNotNone(last_map, "no map from frontier_explorer")
        coverage, free_area = self.coverage(last_map)
        rospy.loginfo("[frontier_smoke] frontiers=%d coverage=%.3f free=%.1fm2 time=%.0fs track=%.1fm "
                      "home=%.2fm problems=%d contacts=%s",
                      frontiers, coverage, free_area, took, track, off_home, len(problems), contacts)
        self.assertEqual(problems, [], "exploration reported: %s" % problems)
        self.assertGreaterEqual(coverage, self.min_coverage, "map does not cover the place")
        self.assertLessEqual(off_home, self.home_tolerance, "did not come back to the start")
        if self.contacts_topic:
            self.assertIsNotNone(contacts, "no message on %s" % self.contacts_topic)
            self.assertEqual(contacts, 0, "the robot collided with an obstacle")


if __name__ == "__main__":
    rospy.init_node("frontier_smoke_test")
    rostest.rosrun(PKG, "frontier_smoke", FrontierSmokeTest)
