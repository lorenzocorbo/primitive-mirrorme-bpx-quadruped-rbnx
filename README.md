# MirrorMe BPX Quadruped Robonix Primitive

Safety-first Robonix adapter for the MirrorMe BPX quadruped. The first release
surface is an integrated, strictly read-only BPX SDK runtime. The repository
also contains normalized state replay and an explicitly non-release fake-motion
profile for offline integration and safety tests. It does not include or
redistribute the BPX SDK.

The SDK baseline is v1.0.8 from the upstream `master` branch, pinned to public
repository commit `19cb373a8d2f8e19eaab6a1441167f3287cc95dd`. The adapter does not
follow the moving `master` HEAD implicitly.

## Current capability surface

- Release manifests (`package_manifest.yaml`, `package_manifest.x86-docker.yaml`
  and `package_manifest.sdk-read-only.yaml`):
  `robonix/primitive/quadruped/odom` only.
- Replay profile: `robonix/primitive/quadruped/odom` only.
- Non-release `package_manifest.motion-dev.yaml`: fake backend only, with
  `robonix/primitive/quadruped/twist_in` and
  `robonix/primitive/quadruped/odom` for offline safety tests.

The production SDK runtime never creates the `twist_in` subscription. All
release manifests force `backend=sdk`; a later configuration mismatch fails
initialization. The fake command path has a distinct
`robonix.primitive.mirrorme.bpx.quadruped.motion_dev` package identity so it
cannot silently expand the published package's capability surface.

`posture` and `move` are deliberately absent from the manifest until their
implementations and safety tests are complete.

BPX does not support mapping, localization, or autonomous navigation. This
Primitive therefore exposes no map, navigation-goal, path-planning, obstacle,
lidar, or camera capability. `odom` is body-state feedback only.

The provider also publishes the standard internal ROS 2 topic
`/joint_states` (`sensor_msgs/JointState`). The fixed SDK index order maps
directly to the 12 names in the official BPX URDF. This stream is deliberately
not declared to Atlas because the fixed Robonix baseline has no quadruped
JointState contract. Position and velocity are published; `effort` remains
empty until the SDK torque unit is confirmed.

## Safety defaults

- The release start entry requires the pinned SDK wheel and forces
  `backend=sdk`.
- The generic runtime config still defaults to `fake`, but only the explicitly
  selected motion-development manifest may use that backend.
- `allow_motion` defaults to `false`.
- `backend=sdk` requires `allow_motion=false` and constructs
  `RequestRobotState` only.
- Activation leaves the controller `DISARMED`.
- The controlled fake path emits zero for invalid, stale, disconnected,
  deactivated and shutdown states; the SDK path has no command method.
- No code in this package calls `setZeroPositionsFlag()`.

## Offline tests

```bash
bash scripts/test.sh
```

## Docker build

The image uses Ubuntu 22.04 through `ros:humble-ros-base`:

```bash
bash scripts/build-container.sh
```

This builds a test target first and only tags the runtime image after the tests
pass. The default image name is `primitive-mirrorme-bpx-quadruped-rbnx:dev`.
The Robonix wire dependencies are pinned in the image to the versions used by
the fixed Robonix source baseline; `grpcio-tools` remains confined to the
code-generation tool image.

## Verify the fixed SDK wheel

The wheel stays outside this repository. Verification mounts it read-only into
an isolated container and checks the recorded SHA-256 before importing it:

```bash
bash scripts/verify-sdk-wheel.sh /path/to/bpx_sdk_open-1.0.8-cp310-...x86_64.whl
```

For a bounded, state-only hardware capture:

```bash
bash scripts/run-read-only-probe.sh /path/to/the-wheel \
  --robot-ip 10.21.20.1 --duration-s 10
```

The probe constructs `RequestRobotState` only. It never constructs a motion or
joint controller and never calls `setZeroPositionsFlag()`.

The reusable `SdkStateSource` follows the same boundary. Its readiness wait is
bounded, it maps leg odometry/IMU/joints/temperatures/motion/battery telemetry,
and its host freshness timestamp advances only when the BPX odometry timestamp
changes. It intentionally has no command method.

All state sources pass through one telemetry boundary that rejects malformed
dimensions, non-finite values, invalid timestamps and zero-norm quaternions;
valid non-unit odometry/IMU quaternions are normalized before publication.

To create a bounded normalized recording once hardware is available:

```bash
bash scripts/record-sdk-replay.sh /path/to/the-wheel /path/to/capture.jsonl \
  --robot-ip 10.21.20.1 --duration-s 30
```

The recorder is read-only, refuses to overwrite an existing file, disconnects
in `finally`, and prints an audit report covering rate, event gaps, timestamp
duplicates/regressions, disconnects and estimated missing odometry frames.

## State replay profile

`package_manifest.state-replay.yaml` forces `backend=replay` and declares only
`odom`. The bundled synthetic fixture can exercise the production Robonix/ROS
publication path without the wheel or robot:

```bash
cd ../robot-mirrorme-bpx
bash scripts/test-offline-integration.sh /path/to/fixed/robonix replay
```

This test performs Driver DEACTIVATE/ACTIVATE, receives `/odom` and
`/joint_states` plus `odom -> base_link`, checks the single-publisher topology,
and verifies shutdown cleanup. The fake variant also publishes a non-planar
Twist and requires the provider to reject it.

For the integrated SDK runtime, supply the same pinned wheel as an external
host artifact. The default and x86 Docker release manifests already select the
read-only wrapper. It verifies the recorded SHA-256, mounts that one file
read-only, and installs it without contacting a package index:

```bash
export BPX_SDK_WHEEL=/absolute/path/to/bpx_sdk_open-1.0.8-cp310-cp310-manylinux_2_24_x86_64.manylinux_2_28_x86_64.whl
rbnx start -p . --set backend=sdk --set allow_motion=false
```

For normal deployment, use the Robot package's
`robonix_manifest.sdk-read-only.yaml` profile. Hardware activation remains
bounded by `sdk_connect_timeout_s` and fails if no fresh odometry frame arrives.

## Robonix build and start

Once `rbnx` is installed and configured against a fixed Robonix source tree:

```bash
rbnx validate .
rbnx build -p .
export BPX_SDK_WHEEL=/absolute/path/to/the-pinned-wheel
rbnx start -p . --config ./local-config.yaml \
  --set backend=sdk --set allow_motion=false
```

For a fake, motion-disabled development smoke test without hardware, explicitly
select the non-release identity:

```bash
rbnx start -p . --manifest package_manifest.motion-dev.yaml \
  --set backend=fake --set allow_motion=false
```

For standalone `rbnx start`, the wrapper defaults `RBNX_INSTANCE_NAME` to the
manifest package name, `robonix.primitive.mirrorme.bpx.quadruped`. An instance
name supplied by a deployment remains authoritative.

To validate and generate against the fixed Robonix source entirely in Docker:

```bash
bash scripts/verify-rbnx.sh /path/to/robonix
```

The script checks the expected Robonix commit and recorded interface submodule
commits, builds `rbnx` in a pinned Rust image, and uses the `grpcio-tools`
versions from that Robonix lockfile. All tool caches stay under `.artifacts/`.

The start wrapper bind-mounts the package and `robonix-api` into the container.
When `BPX_SDK_WHEEL` is set, it additionally verifies the pinned artifact and
bind-mounts only that wheel under `/vendor-sdk` with `:ro`. The artifact remains
outside the repository and runtime image so the release can preserve its own
Apache-2.0 LICENSE, NOTICE, third-party notices, checksum, and SBOM boundary.

The fixed-baseline test suite currently passes all 62 package tests. The
previous Atlas lifecycle smoke test used the distinct fake motion-development
identity: it reached `ACTIVE` with Driver, `twist_in`, and `odom`; its internal
`/joint_states` stream was consumed by ROS 2 `robot_state_publisher` to resolve
all four toe TF chains. Driver `CMD_SHUTDOWN` transitioned it to `TERMINATED`
without leaving a runtime container behind.

## Development status

See the workspace-level `TODO.md`. The published package identity is suitable
only for state-read validation; it cannot arm or accept a velocity command. Do
not promote the motion-development identity or use this adapter to command a
real robot until the controlled-motion milestone and hardware acceptance
checklist are complete.

## License

This Primitive is licensed under the Apache License 2.0. See [LICENSE](LICENSE)
and [NOTICE](NOTICE). The separately distributed BPX SDK is also Apache-2.0,
but retains its own LICENSE, NOTICE, third-party notices, and release metadata.
