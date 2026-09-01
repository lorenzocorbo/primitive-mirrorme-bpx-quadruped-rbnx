"""Robonix provider entry point for fake and read-only SDK runtimes."""

from __future__ import annotations

import logging
import math
import os
import threading
import time
from typing import Any, Dict, Optional

from .config import ProviderConfig, validate_required_backend
from .joint_state import JointStateProjector
from .model import PlanarTwist
from .runtime import ProviderRuntime, build_runtime
from .telemetry import state_for_publication


log = logging.getLogger("mirrorme_bpx_quadruped")

try:
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from robonix_api import Err, Ok, Primitive
    from robonix_api.ros import RosBackend
    from tf2_ros import TransformBroadcaster
except ImportError as exc:  # pragma: no cover - exercised only in runtime image
    raise RuntimeError(
        "ROS 2 and robonix-api are required to run the provider; "
        "use scripts/test.sh for offline core tests"
    ) from exc


primitive = Primitive(
    id="mirrorme_bpx_quadruped",
    namespace="robonix/primitive/quadruped",
)

_runtime: Optional[ProviderRuntime] = None
_config: Optional[ProviderConfig] = None
_publish_thread: Optional[threading.Thread] = None
_stop_publish = threading.Event()
_joint_state_projector = JointStateProjector()
_tf_broadcaster: Optional[TransformBroadcaster] = None

_INTERNAL_JOINT_STATES = "internal/mirrorme/bpx/joint_states"


def _on_twist(msg: Twist) -> None:
    runtime = _runtime
    if runtime is None or not runtime.supports_twist:
        return
    non_planar = (msg.linear.z, msg.angular.x, msg.angular.y)
    if not all(math.isfinite(float(value)) for value in non_planar):
        runtime.submit_twist(PlanarTwist(float("nan"), 0.0, 0.0))
        return
    if any(abs(float(value)) > 1e-9 for value in non_planar):
        log.warning("rejected non-planar Twist")
        return
    decision = runtime.submit_twist(
        PlanarTwist(
            linear_x_mps=float(msg.linear.x),
            linear_y_mps=float(msg.linear.y),
            angular_z_rps=float(msg.angular.z),
        )
    )
    if not decision.accepted:
        log.warning("Twist rejected: %s", decision.reason)


def _publish_loop() -> None:
    assert _runtime is not None
    assert _config is not None
    period_s = 1.0 / _config.odom_rate_hz
    while not _stop_publish.is_set():
        try:
            state = _runtime.tick()
        except RuntimeError as exc:
            log.warning("state sample rejected: %s", exc)
            _stop_publish.wait(period_s)
            continue
        state = state_for_publication(
            state,
            now_s=time.monotonic(),
            timeout_s=_config.controller.state_timeout_s,
        )
        if state is None:
            _stop_publish.wait(period_s)
            continue
        stamp = RosBackend.get().node.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = _config.odom_frame
        msg.child_frame_id = _config.base_frame
        msg.pose.pose.position.x = state.position_m[0]
        msg.pose.pose.position.y = state.position_m[1]
        msg.pose.pose.position.z = state.position_m[2]
        msg.pose.pose.orientation.x = state.orientation_xyzw[0]
        msg.pose.pose.orientation.y = state.orientation_xyzw[1]
        msg.pose.pose.orientation.z = state.orientation_xyzw[2]
        msg.pose.pose.orientation.w = state.orientation_xyzw[3]
        msg.twist.twist.linear.x = state.linear_velocity_body_mps[0]
        msg.twist.twist.linear.y = state.linear_velocity_body_mps[1]
        msg.twist.twist.linear.z = state.linear_velocity_body_mps[2]
        msg.twist.twist.angular.x = state.angular_velocity_body_rps[0]
        msg.twist.twist.angular.y = state.angular_velocity_body_rps[1]
        msg.twist.twist.angular.z = state.angular_velocity_body_rps[2]
        primitive.emit("robonix/primitive/quadruped/odom", msg)

        if _tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = _config.odom_frame
            transform.child_frame_id = _config.base_frame
            transform.transform.translation.x = state.position_m[0]
            transform.transform.translation.y = state.position_m[1]
            transform.transform.translation.z = state.position_m[2]
            transform.transform.rotation.x = state.orientation_xyzw[0]
            transform.transform.rotation.y = state.orientation_xyzw[1]
            transform.transform.rotation.z = state.orientation_xyzw[2]
            transform.transform.rotation.w = state.orientation_xyzw[3]
            _tf_broadcaster.sendTransform(transform)

        joint_payload = _joint_state_projector.project(state)
        if joint_payload is not None:
            joint_msg = JointState()
            joint_msg.header.stamp = stamp
            joint_msg.name = list(joint_payload.names)
            joint_msg.position = list(joint_payload.position)
            joint_msg.velocity = list(joint_payload.velocity)
            joint_msg.effort = list(joint_payload.effort)
            primitive.emit(_INTERNAL_JOINT_STATES, joint_msg)
        _stop_publish.wait(period_s)


@primitive.on_init
def on_init(cfg: Dict[str, Any]):
    global _config, _runtime, _tf_broadcaster
    try:
        _config = ProviderConfig.from_mapping(cfg)
        validate_required_backend(_config, os.environ.get("BPX_REQUIRED_BACKEND"))
        _runtime = build_runtime(_config)
        if _runtime.supports_twist:
            primitive.create_subscription(
                contract_id="robonix/primitive/quadruped/twist_in",
                topic=_config.cmd_vel_topic,
                msg_type=Twist,
                callback=_on_twist,
                qos="reliable",
            )
        primitive.create_publisher(
            contract_id="robonix/primitive/quadruped/odom",
            topic=_config.odom_topic,
            msg_type=Odometry,
            qos="best_effort",
        )
        primitive.create_publisher(
            contract_id=_INTERNAL_JOINT_STATES,
            topic=_config.joint_states_topic,
            msg_type=JointState,
            qos="best_effort",
            declare=False,
        )
        _tf_broadcaster = (
            TransformBroadcaster(RosBackend.get().node)
            if _config.publish_odom_tf
            else None
        )
        return Ok()
    except (TypeError, ValueError) as exc:
        return Err(str(exc))


@primitive.on_activate
def on_activate():
    global _publish_thread
    if _runtime is None:
        return Err("provider was not initialized")
    try:
        _runtime.activate()
        assert _config is not None
        if _config.auto_arm_fake:
            decision = _runtime.arm()
            if not decision.accepted:
                return Err("fake auto-arm rejected: {}".format(decision.reason))
        _joint_state_projector.reset()
        _stop_publish.clear()
        _publish_thread = threading.Thread(
            target=_publish_loop,
            name="bpx-quadruped-odom",
            daemon=True,
        )
        _publish_thread.start()
        return Ok()
    except (ImportError, OSError, RuntimeError, TimeoutError, ValueError) as exc:
        return Err(str(exc))


@primitive.on_deactivate
def on_deactivate():
    _stop_publish.set()
    if _publish_thread is not None:
        _publish_thread.join(timeout=2.0)
    if _runtime is not None:
        _runtime.deactivate()
    return Ok()


@primitive.on_shutdown
def on_shutdown():
    _stop_publish.set()
    if _publish_thread is not None:
        _publish_thread.join(timeout=2.0)
    if _runtime is not None:
        _runtime.shutdown()
    return Ok()


def main() -> int:
    primitive.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
