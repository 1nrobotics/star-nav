#!/usr/bin/env python3
"""Tests of the planner interface (launch/planner.xml), without starting ROS:

    python3 -B star_nav_bringup/test/test_planner_interface.py

Loads each robot's launch file the way roslaunch does and checks which
planner it would start, that the planner is wired to the robot's topics, and
that it is given the robot's sizes (config/robots/<robot>.yaml, written from
the robot's module in the simulator).
Needs the ROS environment (roslaunch, the workspace's packages).
"""

import math
import os
import unittest

import roslaunch.config
import yaml

LAUNCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "launch")
EGO = {"drone_0_ego_planner_node", "drone_0_traj_server"}
CMU = {"terrainAnalysis", "localPlanner", "pathFollower", "cmu_waypoint_router"}
# robot type -> its planner
PLANNERS = {"aerial": "ego", "legged": "cmu"}
ROBOTS = {"sim": "aerial", "mujoco": "aerial", "lite3": "legged", "t1": "legged"}
COMMAND = {"ego": "position", "cmu": "velocity"}
# launch file -> the robot's description (the simulator's robots; "sim" is
# EGO-Planner's own simulator, which has none)
DESCRIPTIONS = {"mujoco": "quadrotor", "lite3": "lite3", "t1": "t1"}
ROBOTS_DIR = os.path.join(LAUNCH, "..", "config", "robots")
PLANNER_XML = os.path.join(LAUNCH, "planner.xml")
PLANNER_ARGS = ["drone_id:=0", "odom_topic:=odom", "map_size_x:=50", "map_size_y:=50",
                "map_size_z:=5", "max_vel:=1", "max_acc:=1", "obstacles_inflation:=0.2",
                "cx:=320", "cy:=240", "fx:=387", "fy:=387"]


def load(path, args):
    return roslaunch.config.load_config_default([(path, list(args))], None, verbose=False)


class Launch:
    """What ``roslaunch exploration_<robot>.launch args`` would start."""

    def __init__(self, robot, *args):
        cfg = load(os.path.join(LAUNCH, "exploration_%s.launch" % robot), args)
        self.nodes = {n.name: n for n in cfg.nodes}
        self.params = {k: p.value for k, p in cfg.params.items()}

    def remaps(self, node):
        return dict(self.nodes[node].remap_args)

    def under(self, ns):
        """The parameters below a namespace, by their names in it."""
        return {k[len(ns):]: v for k, v in self.params.items() if k.startswith(ns)}


class PlannerInterfaceTest(unittest.TestCase):

    def check_backend(self, launch, backend, robot_type):
        """Exactly one planner runs, and says which it is."""
        names = set(launch.nodes)
        self.assertEqual(names & (EGO | CMU), EGO if backend == "ego" else CMU)
        self.assertEqual(launch.params["/planner/backend"], backend)
        self.assertEqual(launch.params["/planner/robot_type"], robot_type)
        self.assertEqual(launch.params["/planner/command"], COMMAND[backend])

    def test_each_robot_type_gets_its_planner(self):
        for robot, robot_type in ROBOTS.items():
            planner = PLANNERS[robot_type]
            with self.subTest(robot=robot):
                self.check_backend(Launch(robot), planner, robot_type)
            if robot_type == "legged":  # asked for by name
                with self.subTest(robot=robot, planner=planner):
                    self.check_backend(Launch(robot, "planner:=" + planner), planner, robot_type)

    def test_a_planner_the_robot_cannot_use_is_refused(self):
        # The EGO planner plans a quadrotor's flight: not for the walking robots.
        for robot in ("lite3", "t1"):
            with self.assertRaisesRegex(Exception, "legged:ego"):
                Launch(robot, "planner:=ego")
        with self.assertRaisesRegex(Exception, "legged:rrt"):
            Launch("lite3", "planner:=rrt")
        # The cmu backend is the ground version of CMU's planner: not for
        # the quadrotor.
        with self.assertRaisesRegex(Exception, "aerial:cmu"):
            load(PLANNER_XML, PLANNER_ARGS + ["robot_type:=aerial", "planner:=cmu"])
        with self.assertRaisesRegex(Exception, "wheeled:auto"):
            load(PLANNER_XML, PLANNER_ARGS + ["robot_type:=wheeled"])
        # The CMU planner's terrain analysis takes lidar scans.
        with self.assertRaisesRegex(Exception, "cmu:depth"):
            load(PLANNER_XML, PLANNER_ARGS + ["robot_type:=legged", "sensor:=depth"])
        load(PLANNER_XML, PLANNER_ARGS + ["robot_type:=legged", "sensor:=lidar"])

    def test_planners_take_the_same_goal_and_odometry(self):
        odom = "/drone_0_visual_slam/odom"
        ego = Launch("mujoco", "sensor:=lidar")
        cmu = Launch("lite3")
        self.assertEqual(ego.remaps("drone_0_ego_planner_node")["~odom_world"], odom)
        self.assertEqual(cmu.remaps("localPlanner")["/state_estimation"], odom)
        self.assertEqual(cmu.remaps("cmu_waypoint_router")["goal"], "/move_base_simple/goal")
        for launch in (ego, cmu):
            self.assertEqual(launch.remaps("mujoco_bridge")["odom"], odom)
        # The lidar's scans, and the command to the simulator
        self.assertEqual(ego.remaps("drone_0_ego_planner_node")["~grid_map/cloud"],
                         ego.remaps("mujoco_bridge")["lidar_cloud"])
        self.assertEqual(ego.remaps("drone_0_traj_server")["/position_cmd"],
                         ego.remaps("mujoco_bridge")["pos_cmd"])
        self.assertEqual(cmu.remaps("localPlanner")["/registered_scan"],
                         cmu.remaps("mujoco_bridge")["lidar_cloud"])
        self.assertEqual(cmu.remaps("mujoco_bridge")["cmd_vel"], "/cmd_vel")

    def test_a_backend_is_one_launch_file(self):
        """Every backend of the table is launch/planners/<name>.xml, and
        nothing else is there."""
        backends = set(PLANNERS.values())
        files = {f[:-len(".xml")] for f in os.listdir(os.path.join(LAUNCH, "planners"))}
        self.assertEqual(files, backends)
        self.assertEqual(set(COMMAND), backends)

    def test_the_robot_description_is_loaded(self):
        """/robot is the robot's description file, and says the type the
        launch file chooses the planner by."""
        self.assertEqual({f[:-len(".yaml")] for f in os.listdir(ROBOTS_DIR)},
                         set(DESCRIPTIONS.values()))
        for robot, name in DESCRIPTIONS.items():
            with self.subTest(robot=robot):
                launch = Launch(robot)
                with open(os.path.join(ROBOTS_DIR, name + ".yaml")) as f:
                    self.assertEqual(launch.under("/robot/"), yaml.safe_load(f))
                self.assertEqual(launch.params["/robot/name"], name)
                self.assertEqual(launch.params["/robot/type"], ROBOTS[robot])
                self.assertEqual(launch.params["/robot/type"],
                                 launch.params["/planner/robot_type"])
        self.assertEqual(Launch("sim").under("/robot/"), {})

    def test_the_planner_is_given_the_robots_sizes(self):
        """What a launch file repeats from the description agrees with it,
        and the planner's own settings fit the robot."""
        for robot in ("lite3", "t1"):
            for args in ((), ("explore:=frontier",)):
                with self.subTest(robot=robot, args=args):
                    launch = Launch(robot, *args)
                    p, d = launch.params, launch.under("/robot/")
                    self.assertEqual(p["/localPlanner/vehicleLength"], d["length"])
                    self.assertEqual(p["/localPlanner/vehicleWidth"], d["width"])
                    # Goals at the height the body stands at
                    goals = ("/frontier_explorer/altitude" if args else
                             "/exploration_node/exploration/altitude")
                    self.assertEqual(p[goals], d["stand_height"])
                    self.assertAlmostEqual(p["/mujoco_bridge/draw_height"], 0.04 - d["stand_height"])
                    # The robot itself is not overhead
                    self.assertGreaterEqual(p["/terrainAnalysis/vehicleHeight"], d["height"])
                    # Not sent faster than it can be commanded
                    for node in ("localPlanner", "pathFollower"):
                        self.assertLessEqual(p["/%s/maxSpeed" % node], d["max_speed"])
                    self.assertLessEqual(math.radians(p["/pathFollower/maxYawRate"]),
                                         d["max_turn"])
                    # Its route keeps its sides clear
                    self.assertGreaterEqual(p["/cmu_waypoint_router/robot_radius"],
                                            d["width"] / 2)
        # The quadrotor's lidar, for the planner's map
        mujoco = Launch("mujoco", "sensor:=lidar").params
        self.assertEqual(mujoco["/drone_0_ego_planner_node/grid_map/lidar_offset"],
                         mujoco["/robot/lidar_mount"])

    def test_frontier_exploration_keeps_the_planner(self):
        for robot, robot_type in (("lite3", "legged"), ("mujoco", "aerial")):
            with self.subTest(robot=robot):
                launch = Launch(robot, "explore:=frontier")
                self.check_backend(launch, PLANNERS[robot_type], robot_type)
                self.assertIn("frontier_explorer", launch.nodes)
                self.assertNotIn("exploration_node", launch.nodes)

    def test_the_quadrotors_frontier_exploration_uses_the_lidar(self):
        # The frontier map is made from lidar scans, whatever ``sensor`` says
        launch = Launch("mujoco", "explore:=frontier")
        self.assertIn("--sensor lidar", launch.nodes["mujoco_sim"].args)
        self.assertTrue(launch.params["/drone_0_ego_planner_node/grid_map/enable_pointcloud_raycasting"])
        # Goals at the height the planner flies at
        self.assertEqual(launch.params["/frontier_explorer/altitude"], 1.0)
        self.assertIn("--start 0 0 1.0", launch.nodes["mujoco_sim"].args)


if __name__ == "__main__":
    unittest.main()
