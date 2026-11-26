#!/usr/bin/env python3
"""Records the path a walking robot takes during one exploration mission.

The browser demo walks the robot along it with the same walking policy, so
the page shows the way the CMU planner led the robot through this house.
Run with the MuJoCo stack up (exploration_lite3.launch or
exploration_t1.launch); it triggers the mission and writes JSON when the
exploration finishes:

    python3 site/tools/record_path.py site/assets/mujoco/lite3-path.json

The body sways with every step. The file holds the path without the sway (a
moving average over WINDOW seconds) and its velocity, which is what the
robot follows in the browser.
"""

import json
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

RATE = 20.0  # Hz kept in the file
WINDOW = 0.6  # s, moving average
ODOM_TOPIC = "/drone_0_visual_slam/odom"


def smooth(values, half):
    out = []
    for i in range(len(values)):
        part = values[max(0, i - half):i + half + 1]
        out.append(sum(part) / len(part))
    return out


def main():
    out = sys.argv[1]
    rospy.init_node("record_path")
    rows, state = [], {"started": False, "done": False, "t0": None, "last": -1.0}

    def on_odom(m):
        if not state["started"] or state["done"]:
            return
        t = m.header.stamp.to_sec()
        if state["t0"] is None:
            state["t0"] = t
        t -= state["t0"]
        if t - state["last"] < 1.0 / RATE - 1e-6:
            return
        state["last"] = t
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if rows:  # keep the yaw continuous for the average
            yaw = rows[-1][4] + math.remainder(yaw - rows[-1][4], 2.0 * math.pi)
        rows.append([t, p.x, p.y, p.z, yaw])

    def on_state(m):
        if m.data == "FINISH":
            state["done"] = state["started"]
        elif m.data != "WAIT_TRIGGER":
            state["started"] = True

    rospy.Subscriber(ODOM_TOPIC, Odometry, on_odom, queue_size=100)
    rospy.Subscriber("/exploration/state", String, on_state)
    trigger = rospy.Publisher("/exploration/trigger", PoseStamped, queue_size=1)
    rospy.sleep(3.0)
    while not rospy.is_shutdown() and not state["started"]:
        trigger.publish(PoseStamped())
        rospy.sleep(1.0)
    while not rospy.is_shutdown() and not state["done"]:
        rospy.sleep(0.5)

    half = int(round(WINDOW * RATE / 2.0))
    cols = [smooth([r[i] for r in rows], half) for i in range(1, 5)]
    t = [r[0] for r in rows]
    path = []
    for i in range(len(rows)):
        a, b = max(0, i - 1), min(len(rows) - 1, i + 1)
        dt = t[b] - t[a]
        vx, vy = ((cols[k][b] - cols[k][a]) / dt for k in (0, 1))
        path.append([round(v, 4) for v in
                     (t[i], cols[0][i], cols[1][i], cols[2][i], cols[3][i], vx, vy)])
    with open(out, "w") as f:
        json.dump({"fields": ["t", "x", "y", "z", "yaw", "vx", "vy"], "rate": RATE,
                   "rows": path}, f, separators=(",", ":"))
    print("wrote %d points (%.1f s) to %s" % (len(path), path[-1][0], out))


if __name__ == "__main__":
    main()
