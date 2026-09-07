"""Small manual client for the BPX Robonix posture RPC."""

from __future__ import annotations

import argparse
import json
import os
import uuid
from typing import Any, Optional, Sequence


POSTURE_CONTRACT = "robonix/primitive/quadruped/posture"


def call_posture(
    posture_name: str,
    *,
    provider_id: str,
    timeout_s: float,
    atlas: Any = None,
    grpc_module: Any = None,
    stub_type: Any = None,
    request_type: Any = None,
) -> dict:
    if posture_name not in {"stand", "sit"}:
        raise ValueError("posture must be stand or sit")
    if not 0.1 <= float(timeout_s) <= 120.0:
        raise ValueError("timeout_s must be in 0.1..120.0")

    if atlas is None:
        from robonix_api import ATLAS

        atlas = ATLAS
    if grpc_module is None:
        import grpc

        grpc_module = grpc
    if stub_type is None:
        from robonix_contracts_pb2_grpc import (
            RobonixPrimitiveQuadrupedPostureStub,
        )

        stub_type = RobonixPrimitiveQuadrupedPostureStub
    if request_type is None:
        from quadruped_pb2 import SetPosture_Request

        request_type = SetPosture_Request

    consumer_id = "bpx_posture_client_{}_{}".format(
        os.getpid(), uuid.uuid4().hex[:8]
    )
    registered = False
    rpc_channel = None
    try:
        atlas.register_service(consumer_id, "robonix/client/manual")
        registered = True
        with atlas.connect_capability(
            consumer_id=consumer_id,
            provider_id=provider_id,
            contract_id=POSTURE_CONTRACT,
            transport="grpc",
        ) as binding:
            rpc_channel = grpc_module.insecure_channel(
                binding.endpoint,
                options=[("grpc.enable_http_proxy", 0)],
            )
            response = stub_type(rpc_channel).SetPosture(
                request_type(posture_name=posture_name),
                timeout=float(timeout_s),
            )
            return {
                "provider_id": provider_id,
                "posture_name": posture_name,
                "success": bool(response.success),
                "message": str(response.message),
            }
    finally:
        if rpc_channel is not None:
            rpc_channel.close()
        if registered:
            atlas.unregister(consumer_id)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Call the BPX Robonix posture RPC"
    )
    parser.add_argument("posture", choices=("stand", "sit"))
    parser.add_argument("--provider-id", default="mirrorme_bpx")
    parser.add_argument("--timeout-s", type=float, default=20.0)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = call_posture(
            args.posture,
            provider_id=args.provider_id,
            timeout_s=args.timeout_s,
        )
    except Exception as exc:
        print(json.dumps({"success": False, "message": str(exc)}, sort_keys=True))
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
