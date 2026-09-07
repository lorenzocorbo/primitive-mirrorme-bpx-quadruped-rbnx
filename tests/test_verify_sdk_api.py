from __future__ import annotations

import os
import sys
from types import SimpleNamespace
import unittest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.verify_sdk_api import (
    REQUIRED_POSTURE_METHODS,
    REQUIRED_READ_METHODS,
    validate_posture_sdk_module,
    validate_sdk_module,
    validate_yaw_circle_sdk_module,
)


class VerifySdkApiTest(unittest.TestCase):
    def test_accepts_exact_read_only_api(self) -> None:
        request_type = type(
            "RequestRobotState",
            (),
            {name: lambda self: None for name in REQUIRED_READ_METHODS},
        )
        result = validate_sdk_module(
            SimpleNamespace(__version__="1.0.8", RequestRobotState=request_type)
        )
        self.assertEqual("1.0.8", result["sdk_version"])
        self.assertFalse(result["request_robot_state_has_motion_methods"])

    def test_rejects_a_motion_capable_request_type(self) -> None:
        methods = {name: lambda self: None for name in REQUIRED_READ_METHODS}
        methods["setVelocity"] = lambda self, *args: True
        request_type = type("RequestRobotState", (), methods)
        with self.assertRaisesRegex(RuntimeError, "motion methods"):
            validate_sdk_module(
                SimpleNamespace(__version__="1.0.8", RequestRobotState=request_type)
            )

    def test_accepts_narrow_posture_api(self) -> None:
        control_type = type(
            "MotionLevelControl",
            (),
            {
                name: lambda self: None
                for name in REQUIRED_READ_METHODS + REQUIRED_POSTURE_METHODS
            },
        )
        result = validate_posture_sdk_module(
            SimpleNamespace(
                __version__="1.0.8",
                MotionLevelControl=control_type,
                MotionState=SimpleNamespace(LyingDown=0, Motion=6),
            )
        )
        self.assertEqual(["setStandUp", "setSitDown"], result["posture_commands"])
        self.assertFalse(result["supports_twist"])
        self.assertFalse(result["uses_joint_level_control"])

    def test_rejects_incomplete_posture_api(self) -> None:
        methods = {name: lambda self: None for name in REQUIRED_READ_METHODS}
        methods.update(
            {
                name: lambda self: None
                for name in REQUIRED_POSTURE_METHODS
                if name != "setSitDown"
            }
        )
        control_type = type("MotionLevelControl", (), methods)
        with self.assertRaisesRegex(RuntimeError, "setSitDown"):
            validate_posture_sdk_module(
                SimpleNamespace(
                    __version__="1.0.8",
                    MotionLevelControl=control_type,
                    MotionState=SimpleNamespace(LyingDown=0, Motion=6),
                )
            )

    def test_rejects_changed_posture_state_values(self) -> None:
        control_type = type(
            "MotionLevelControl",
            (),
            {
                name: lambda self: None
                for name in REQUIRED_READ_METHODS + REQUIRED_POSTURE_METHODS
            },
        )
        with self.assertRaisesRegex(RuntimeError, "MotionState.Motion"):
            validate_posture_sdk_module(
                SimpleNamespace(
                    __version__="1.0.8",
                    MotionLevelControl=control_type,
                    MotionState=SimpleNamespace(LyingDown=0, Motion=99),
                )
            )

    def test_accepts_yaw_circle_api_without_declaring_twist(self) -> None:
        control_type = type(
            "MotionLevelControl",
            (),
            {
                name: lambda self: None
                for name in REQUIRED_READ_METHODS
                + REQUIRED_POSTURE_METHODS
                + ("setVelocity",)
            },
        )
        result = validate_yaw_circle_sdk_module(
            SimpleNamespace(
                __version__="1.0.8",
                MotionLevelControl=control_type,
                MotionState=SimpleNamespace(LyingDown=0, Motion=6),
            )
        )
        self.assertEqual(["setVelocity"], result["yaw_circle_probe_methods"])
        self.assertTrue(result["yaw_circle_probe_only"])
        self.assertFalse(result["supports_twist"])


if __name__ == "__main__":
    unittest.main()
