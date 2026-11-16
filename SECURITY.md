# Security Policy

Star Nav is research and development software for simulated and
experimental drones. Do not rely on it as a safety system.

## Reporting a vulnerability

Please report vulnerabilities privately through
[GitHub security advisories](https://github.com/1nrobotics/star-nav/security/advisories/new)
rather than a public issue. Include the affected component, steps to
reproduce and the impact you expect. We aim to acknowledge reports within a
week.

## Network exposure

The simulation stack is meant for trusted machines and networks:

- The MuJoCo web viewer (port 8080) listens on all interfaces; MuJoCo Studio
  does not offer a bind address yet. Do not expose it to untrusted networks.
- The simulator's ZeroMQ sockets (5555, 5556) bind to `127.0.0.1` by default
  and carry unauthenticated commands. Only use `--bind 0.0.0.0` on a trusted
  network.
- ROS 1 has no authentication; keep `ROS_MASTER_URI` on a trusted network.
