# Runtime configuration accepted by the BPX quadruped primitive.
#
# This file documents the mapping passed in the deployment instance's config.
# Runtime validation is implemented by bpx_quadruped.config.

config:
  # enum string: fake, sdk, or replay; default: fake.
  # sdk requires a verified, externally mounted BPX_SDK_WHEEL. The production
  # manifest enables only the guarded stand/sit posture service; the explicit
  # sdk-read-only manifest keeps allow_motion=false.
  backend: fake

  # Absolute JSONL capture path; required only for backend=replay. The replay
  # adapter is read-only and preserves recorded freshness/disconnect events.
  replay_path:

  # boolean, default: false.
  # Process-level gate only. It never replaces local arm or a physical E-stop.
  allow_motion: false

  # boolean, default: false. Valid only with backend=sdk and
  # allow_motion=true. This selects the stand/sit-only SDK runtime;
  # it does not enable twist, gait, damping, or joint-level commands.
  enable_posture_service: false

  # string IPv4 address, default: 10.21.20.1.
  robot_ip: 10.21.20.1

  # integer UDP port, default: 9873; range: 1024..65535.
  robot_state_port: 9873

  # integer Hz, default: 50; range: 1..200.
  state_rate_hz: 50

  # integer Hz, default: 50; range: 1..200.
  command_rate_hz: 50

  # Read-only SDK connection settings. No motion controller is constructed.
  sdk_tcp_local_port: 0
  sdk_connect_timeout_s: 10.0
  sdk_poll_period_s: 0.05

  # Bounds for the isolated stand/sit Robonix posture RPC.
  posture_stand_timeout_s: 15.0
  posture_sit_timeout_s: 15.0
  posture_poll_period_s: 0.2
  posture_cleanup_sit_flush_s: 1.0

  # positive float seconds, default: 0.25.
  command_timeout_s: 0.25

  # positive float seconds, default: 0.50.
  state_timeout_s: 0.50

  # non-negative floats in m/s and rad/s.
  max_linear_x_mps: 0.30
  max_linear_y_mps: 0.20
  max_angular_z_rps: 0.50

  # positive acceleration limits in m/s^2 and rad/s^2.
  max_linear_accel_mps2: 0.50
  max_angular_accel_rps2: 1.00

  # positive integer, default: 3.
  zero_preamble_count: 3

  # ROS 2 names and frames.
  cmd_vel_topic: /cmd_vel
  odom_topic: /odom
  # Internal ROS 2 data-plane topic. Robonix has no quadruped JointState
  # contract in the fixed baseline, so this is not declared to Atlas.
  joint_states_topic: /joint_states
  odom_frame: odom
  base_frame: base_link
  odom_rate_hz: 50.0

  # Publish odom -> base_link from the exact Odometry sample/stamp. Default
  # false; enabled only in checked-in fake/replay profiles until hardware axes
  # are confirmed.
  publish_odom_tf: false

  # Fake-only test convenience. Must remain false in checked-in deployment.
  auto_arm_fake: false
