---
description: Read-only MirrorMe BPX quadruped body-state odometry.
---

# MirrorMe BPX Quadruped

The published package identity exposes only
`robonix/primitive/quadruped/odom`. Its SDK runtime has no command interface and
does not create or declare `twist_in`. The normalized-replay runtime has the
same read-only capability surface.

An explicitly non-release `motion_dev` package identity keeps the guarded fake
Twist path available for offline safety tests. It is not a supported hardware
capability and must not be submitted to the Robonix Package Catalog as a
release variant.

All runtimes publish the internal ROS 2 `/joint_states` topic with the 12 names
defined by the pinned BPX SDK and official URDF. This topic is not advertised
as a Robonix capability because no quadruped JointState contract exists in the
fixed baseline.

Current restrictions:

- Mapping, localization, autonomous navigation, obstacle avoidance, lidar and
  camera perception are outside the BPX product scope; odometry is body-state
  feedback and does not imply navigation support.
- Motion is disabled by default and activation does not arm the robot.
- `backend=sdk` requires `allow_motion=false`, loads a separately mounted and
  checksum-verified v1.0.8 wheel, and constructs `RequestRobotState` only.
- `backend=replay` requires an absolute versioned JSONL path, remains read-only,
  and preserves recorded freshness and disconnect events.
- Unknown, non-finite, stale or out-of-state commands are rejected or reduced
  to a strict zero command.
- Flip, running, jumping, handstand and low-level joint control are not exposed.
- Posture and bounded move contracts are not declared in the current milestone.
- JointState effort is omitted until the SDK torque unit is confirmed.
