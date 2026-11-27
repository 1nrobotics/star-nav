#!/usr/bin/env python3
"""Records the map of one frontier exploration, for the inset of its video.

Run with the MuJoCo stack up and ``explore:=frontier``; it triggers the
exploration and writes the file when the exploration finishes:

    python3 site/tools/record_map.py /tmp/lite3-frontier.npz

What it keeps, with the simulator's time (the stamps; the simulator's video
has a frame every 1/30 s of the same clock):

    map_t, map      the explorer's map every time it is published
                    (/frontier_explorer/map): -1 unknown, 0 free, 100 obstacle
    frontier        the cells that were frontiers in that map
    origin, resolution
    track           t, x, y, yaw of the robot, 20 times a second
    goal            t, x, y of every goal the explorer sent
    start, finish   when the exploration was triggered and when it finished

``map_inset.py`` draws them into the video.
"""

import math
import sys
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

RATE = 20.0  # Hz of the track
ODOM_TOPIC = "/drone_0_visual_slam/odom"


def main():
    out = sys.argv[1]
    rospy.init_node("record_map")
    lock = threading.Lock()
    maps, frontiers, track, goals = [], {}, [], []
    info = {}
    state = {"started": None, "done": None, "last": -1.0}

    def on_map(m):
        grid = np.array(m.data, np.int8).reshape(m.info.height, m.info.width)
        with lock:
            info["resolution"] = m.info.resolution
            info["origin"] = (m.info.origin.position.x, m.info.origin.position.y)
            maps.append((m.header.stamp.to_sec(), grid))

    def on_frontiers(m):
        for marker in m.markers:
            if marker.ns == "frontier_cells":
                with lock:
                    frontiers[marker.header.stamp.to_sec()] = np.array(
                        [(p.x, p.y) for p in marker.points], np.float32).reshape(-1, 2)

    def on_odom(m):
        t = m.header.stamp.to_sec()
        if t - state["last"] < 1.0 / RATE - 1e-6:
            return
        state["last"] = t
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with lock:
            track.append((t, p.x, p.y, yaw))

    def on_goal(m):
        with lock:
            goals.append((rospy.Time.now().to_sec(), m.pose.position.x, m.pose.position.y))

    def on_state(m):
        now = rospy.Time.now().to_sec()
        if m.data == "FINISH":
            if state["started"] is not None and state["done"] is None:
                state["done"] = now
        elif m.data != "WAIT_TRIGGER" and state["started"] is None:
            state["started"] = now

    rospy.Subscriber("/frontier_explorer/map", OccupancyGrid, on_map, queue_size=10)
    rospy.Subscriber("/frontier_explorer/frontiers", MarkerArray, on_frontiers, queue_size=10)
    rospy.Subscriber(rospy.get_param("~odom", ODOM_TOPIC), Odometry, on_odom, queue_size=100)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, on_goal, queue_size=10)
    rospy.Subscriber("/exploration/state", String, on_state)
    trigger = rospy.Publisher("/exploration/trigger", PoseStamped, queue_size=1)
    rospy.sleep(3.0)
    while not rospy.is_shutdown() and state["started"] is None:
        trigger.publish(PoseStamped())
        rospy.sleep(1.0)
    while not rospy.is_shutdown() and state["done"] is None:
        rospy.sleep(0.5)
    rospy.sleep(1.0)  # the last map

    with lock:
        if not maps:
            sys.exit("no map was published: is the stack running with explore:=frontier?")
        res, origin = info["resolution"], np.array(info["origin"])
        shape = maps[0][1].shape
        cells = np.zeros((len(maps),) + shape, bool)
        for k, (t, _) in enumerate(maps):
            xy = frontiers.get(t)
            if xy is not None and len(xy):
                ij = np.floor((xy - origin) / res).astype(int)
                ok = (ij[:, 0] >= 0) & (ij[:, 0] < shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[0])
                cells[k, ij[ok, 1], ij[ok, 0]] = True
        np.savez_compressed(
            out, map_t=np.array([t for t, _ in maps]), map=np.stack([g for _, g in maps]),
            frontier=cells, origin=origin, resolution=res,
            track=np.array(track, np.float64).reshape(-1, 4),
            goal=np.array(goals, np.float64).reshape(-1, 3),
            start=state["started"] or 0.0, finish=state["done"] or state["last"])
    print("wrote %d maps, %d track points (%.1f s to %.1f s) to %s"
          % (len(maps), len(track), state["started"] or 0.0, state["done"] or state["last"], out))


if __name__ == "__main__":
    main()
