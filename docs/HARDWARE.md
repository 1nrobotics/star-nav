# Hardware Guide

`main` contains no sensor-specific hardware code. Real-flight integrations live
on dedicated branches that track `main`:

| Branch | Hardware | Contents |
| --- | --- | --- |
| `hardware/vk180` | VK180 vision module + PX4 (Jetson or x86 onboard computer) | `vk180-ros` bridge, real-flight launch files, Jetson Docker image and onboard setup scripts |

Hardware branches depend on proprietary vendor SDKs that are not part of the
open-source release.

## Hardware-independent parts on `main`

- `px4ctrl`: PX4 attitude/thrust controller over MAVROS. It consumes
  `quadrotor_msgs/PositionCommand` on `/drone_<id>_planning/pos_cmd` and
  odometry on `/mavros/local_position/odom`.
- The planner expects odometry and a forward depth image
  (`sensor_msgs/Image`, 16UC1 in millimetres); any sensor that provides these
  can be integrated the same way the VK180 branch does.

## Working on a hardware branch

```bash
git checkout hardware/vk180
git merge main            # pick up planner and simulation changes
```

Keep hardware-only changes on the hardware branch and send generic planner or
simulation fixes to `main` first.
