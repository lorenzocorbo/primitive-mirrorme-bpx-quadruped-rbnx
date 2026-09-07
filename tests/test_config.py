from __future__ import annotations

import os
import sys
import unittest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.config import (
    ProviderConfig,
    validate_required_backend,
    validate_required_posture_capability,
)


class ProviderConfigTest(unittest.TestCase):
    def test_defaults_are_fake_and_motion_disabled(self) -> None:
        config = ProviderConfig.from_mapping({})
        self.assertEqual("fake", config.backend)
        self.assertFalse(config.controller.allow_motion)
        self.assertFalse(config.auto_arm_fake)
        self.assertEqual("10.21.20.1", config.robot_ip)
        self.assertEqual("/joint_states", config.joint_states_topic)
        self.assertEqual(0, config.sdk_tcp_local_port)
        self.assertEqual(10.0, config.sdk_connect_timeout_s)
        self.assertEqual(0.05, config.sdk_poll_period_s)
        self.assertIsNone(config.replay_path)
        self.assertFalse(config.publish_odom_tf)
        self.assertFalse(config.enable_posture_service)

    def test_string_false_is_not_accepted_as_boolean(self) -> None:
        with self.assertRaisesRegex(ValueError, "allow_motion must be a boolean"):
            ProviderConfig.from_mapping({"allow_motion": "false"})
        with self.assertRaisesRegex(ValueError, "publish_odom_tf must be a boolean"):
            ProviderConfig.from_mapping({"publish_odom_tf": "false"})

    def test_unknown_fields_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown provider config"):
            ProviderConfig.from_mapping({"surprise": True})

    def test_invalid_network_and_ros_names_are_rejected(self) -> None:
        invalid = (
            {"robot_ip": "robot.local"},
            {"robot_state_port": 80},
            {"cmd_vel_topic": "cmd_vel"},
            {"joint_states_topic": "joint_states"},
            {"odom_frame": "/odom"},
            {"odom_rate_hz": 0.0},
        )
        for values in invalid:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    ProviderConfig.from_mapping(values)

    def test_fake_auto_arm_cannot_be_selected_for_sdk(self) -> None:
        with self.assertRaisesRegex(ValueError, "only valid with backend=fake"):
            ProviderConfig.from_mapping({"backend": "sdk", "auto_arm_fake": True})

    def test_sdk_backend_is_strictly_read_only(self) -> None:
        config = ProviderConfig.from_mapping({"backend": "sdk"})
        self.assertFalse(config.controller.allow_motion)
        with self.assertRaisesRegex(ValueError, "backend=sdk requires allow_motion=false"):
            ProviderConfig.from_mapping({"backend": "sdk", "allow_motion": True})

    def test_sdk_posture_service_is_an_explicit_narrow_motion_exception(self) -> None:
        config = ProviderConfig.from_mapping(
            {
                "backend": "sdk",
                "allow_motion": True,
                "enable_posture_service": True,
            }
        )
        self.assertTrue(config.controller.allow_motion)
        self.assertTrue(config.enable_posture_service)

        with self.assertRaisesRegex(ValueError, "requires allow_motion=true"):
            ProviderConfig.from_mapping(
                {"backend": "sdk", "enable_posture_service": True}
            )
        with self.assertRaisesRegex(ValueError, "only valid with backend=sdk"):
            ProviderConfig.from_mapping(
                {
                    "backend": "fake",
                    "allow_motion": True,
                    "enable_posture_service": True,
                }
            )

    def test_manifest_posture_requirement_must_match_runtime(self) -> None:
        enabled = ProviderConfig.from_mapping(
            {
                "backend": "sdk",
                "allow_motion": True,
                "enable_posture_service": True,
            }
        )
        disabled = ProviderConfig.from_mapping({"backend": "sdk"})

        validate_required_posture_capability(enabled, "true")
        validate_required_posture_capability(disabled, "false")
        with self.assertRaisesRegex(ValueError, "enable_posture_service=true"):
            validate_required_posture_capability(disabled, "true")
        with self.assertRaisesRegex(ValueError, "must be true or false"):
            validate_required_posture_capability(enabled, "yes")

    def test_replay_backend_requires_an_absolute_path_and_is_read_only(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires replay_path"):
            ProviderConfig.from_mapping({"backend": "replay"})
        with self.assertRaisesRegex(ValueError, "absolute"):
            ProviderConfig.from_mapping(
                {"backend": "replay", "replay_path": "capture.jsonl"}
            )
        config = ProviderConfig.from_mapping(
            {"backend": "replay", "replay_path": "/pkg/capture.jsonl"}
        )
        self.assertFalse(config.controller.allow_motion)
        with self.assertRaisesRegex(ValueError, "backend=replay"):
            ProviderConfig.from_mapping(
                {
                    "backend": "replay",
                    "replay_path": "/pkg/capture.jsonl",
                    "allow_motion": True,
                }
            )

    def test_selected_package_manifest_can_require_the_sdk_backend(self) -> None:
        sdk = ProviderConfig.from_mapping({"backend": "sdk"})
        validate_required_backend(sdk, "sdk")
        validate_required_backend(sdk, None)

        fake = ProviderConfig.from_mapping({"backend": "fake"})
        with self.assertRaisesRegex(ValueError, "requires backend=sdk"):
            validate_required_backend(fake, "sdk")
        with self.assertRaisesRegex(ValueError, "BPX_REQUIRED_BACKEND"):
            validate_required_backend(sdk, "unknown")

        replay = ProviderConfig.from_mapping(
            {"backend": "replay", "replay_path": "/pkg/capture.jsonl"}
        )
        validate_required_backend(replay, "replay")


if __name__ == "__main__":
    unittest.main()
