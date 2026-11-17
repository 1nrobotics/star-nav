#!/usr/bin/env python3
"""Records the planner's setpoints during one exploration mission.

The browser demo replays them through the same controller as the simulator
(the quadrotor's SE(3) controller, or the Microduck's tracker and policy), so
the robot follows the path EGO-Planner actually planned in this world. Run
with the MuJoCo stack up; it triggers the mission and writes JSON when the
exploration finishes:

    # with exploration_mujoco.launch running
    python3 site/tools/record_setpoints.py site/assets/mujoco/setpoints.json
    # with exploration_microduck.launch running (slow robot: fewer samples)
    python3 site/tools/record_setpoints.py site/assets/mujoco/microduck_setpoints.json 10
"""

import json
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String

RATE = 25.0  # Hz kept in the file (default)


def main():
    out = sys.argv[1]
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else RATE
    rospy.init_node("record_setpoints")
    rows, state = [], {"done": False, "t0": None, "last": -1.0}

    def on_cmd(m):
        t = m.header.stamp.to_sec()
        if state["t0"] is None:
            state["t0"] = t
        t -= state["t0"]
        if t - state["last"] < 1.0 / rate - 1e-6:
            return
        state["last"] = t
        p, v, a = m.position, m.velocity, m.acceleration
        rows.append([round(x, 4) for x in
                     (t, p.x, p.y, p.z, v.x, v.y, v.z, a.x, a.y, a.z, m.yaw, m.yaw_dot)])

    def on_state(m):
        if m.data == "FINISH":
            state["done"] = True

    rospy.Subscriber("/drone_0_planning/pos_cmd", PositionCommand, on_cmd, queue_size=100)
    rospy.Subscriber("/exploration/state", String, on_state)
    trigger = rospy.Publisher("/exploration/trigger", PoseStamped, queue_size=1)
    rospy.sleep(3.0)
    while not rospy.is_shutdown() and not rows:
        trigger.publish(PoseStamped())
        rospy.sleep(1.0)
    while not rospy.is_shutdown() and not state["done"]:
        rospy.sleep(0.5)
    with open(out, "w") as f:
        json.dump({"fields": ["t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az",
                              "yaw", "yaw_rate"], "rate": rate, "rows": rows},
                  f, separators=(",", ":"))
    print("wrote %d setpoints (%.1f s) to %s" % (len(rows), rows[-1][0], out))


if __name__ == "__main__":
    main()
