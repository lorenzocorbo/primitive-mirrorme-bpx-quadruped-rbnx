from __future__ import annotations

import os
import sys
from types import SimpleNamespace
import unittest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.verify_sdk_api import REQUIRED_READ_METHODS, validate_sdk_module


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


if __name__ == "__main__":
    unittest.main()
