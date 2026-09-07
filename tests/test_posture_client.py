from __future__ import annotations

import os
from types import SimpleNamespace
import sys
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.posture_client import POSTURE_CONTRACT, call_posture


class FakeBinding:
    endpoint = "127.0.0.1:54321"

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        return None


class FakeAtlas:
    def __init__(self) -> None:
        self.registered = []
        self.unregistered = []
        self.connection = None

    def register_service(self, consumer_id, namespace):
        self.registered.append((consumer_id, namespace))

    def connect_capability(self, **kwargs):
        self.connection = kwargs
        return FakeBinding()

    def unregister(self, consumer_id):
        self.unregistered.append(consumer_id)


class FakeRpcChannel:
    def __init__(self, endpoint) -> None:
        self.endpoint = endpoint
        self.closed = False

    def close(self):
        self.closed = True


class FakeGrpc:
    def __init__(self) -> None:
        self.channel = None

    def insecure_channel(self, endpoint, options):
        self.channel = FakeRpcChannel(endpoint)
        self.options = options
        return self.channel


class FakeStub:
    response = SimpleNamespace(success=True, message="standing confirmed")
    request = None
    timeout = None

    def __init__(self, channel) -> None:
        self.channel = channel

    def SetPosture(self, request, timeout):
        type(self).request = request
        type(self).timeout = timeout
        return type(self).response


class FakeRequest:
    def __init__(self, posture_name) -> None:
        self.posture_name = posture_name


class PostureClientTest(unittest.TestCase):
    def test_discovers_channel_calls_rpc_and_cleans_up_consumer(self) -> None:
        atlas = FakeAtlas()
        grpc = FakeGrpc()

        result = call_posture(
            "stand",
            provider_id="mirrorme_bpx_posture",
            timeout_s=20.0,
            atlas=atlas,
            grpc_module=grpc,
            stub_type=FakeStub,
            request_type=FakeRequest,
        )

        self.assertTrue(result["success"])
        self.assertEqual("stand", FakeStub.request.posture_name)
        self.assertEqual(20.0, FakeStub.timeout)
        self.assertEqual(POSTURE_CONTRACT, atlas.connection["contract_id"])
        self.assertEqual("grpc", atlas.connection["transport"])
        self.assertTrue(grpc.channel.closed)
        self.assertEqual(atlas.registered[0][0], atlas.unregistered[0])

    def test_rejected_rpc_is_returned_without_becoming_transport_error(self) -> None:
        atlas = FakeAtlas()
        grpc = FakeGrpc()
        FakeStub.response = SimpleNamespace(success=False, message="wrong state")

        result = call_posture(
            "sit",
            provider_id="mirrorme_bpx_posture",
            timeout_s=10.0,
            atlas=atlas,
            grpc_module=grpc,
            stub_type=FakeStub,
            request_type=FakeRequest,
        )

        self.assertFalse(result["success"])
        self.assertEqual("wrong state", result["message"])
        self.assertTrue(grpc.channel.closed)

    def test_rejects_unsupported_posture_before_atlas_registration(self) -> None:
        atlas = FakeAtlas()
        with self.assertRaisesRegex(ValueError, "stand or sit"):
            call_posture(
                "damping",
                provider_id="mirrorme_bpx_posture",
                timeout_s=10.0,
                atlas=atlas,
                grpc_module=FakeGrpc(),
                stub_type=FakeStub,
                request_type=FakeRequest,
            )
        self.assertEqual([], atlas.registered)


if __name__ == "__main__":
    unittest.main()
