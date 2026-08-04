import math

import pytest

from go2w_bridge.sport_gateway_protocol import (
    MAX_FRAME_BYTES,
    ProtocolError,
    decode_request,
    decode_response,
    encode_frame,
)


def test_request_accepts_only_versioned_allowlisted_effects():
    request = decode_request({
        "version": 1,
        "request_id": "req-1",
        "operation": "Move",
        "arguments": [0.1, 0.0, 0.0],
    })

    assert request.operation == "Move"
    assert request.arguments == (0.1, 0.0, 0.0)


@pytest.mark.parametrize("operation", ["Damp", "StandDown", "RecoveryStand"])
def test_request_rejects_unload_operations(operation):
    with pytest.raises(ProtocolError, match="unsupported operation"):
        decode_request({
            "version": 1,
            "request_id": "req-1",
            "operation": operation,
            "arguments": [],
        })


@pytest.mark.parametrize(
    ("payload", "reason"),
    [
        ({"version": 2, "request_id": "r", "operation": "CheckMode",
          "arguments": []}, "unsupported protocol version"),
        ({"version": 1, "request_id": "", "operation": "CheckMode",
          "arguments": []}, "request_id"),
        ({"version": 1, "request_id": "r", "operation": "Move",
          "arguments": [0.1, 0.0]}, "requires 3 arguments"),
        ({"version": 1, "request_id": "r", "operation": "Move",
          "arguments": [math.nan, 0.0, 0.0]}, "finite"),
    ],
)
def test_request_rejects_malformed_or_unsafe_values(payload, reason):
    with pytest.raises(ProtocolError, match=reason):
        decode_request(payload)


def test_response_requires_matching_request_id_and_integer_code():
    response = decode_response({
        "version": 1,
        "request_id": "req-1",
        "operation": "MoveZero",
        "code": 0,
        "motion_service": "ai-w",
    }, expected_request_id="req-1")

    assert response.code == 0
    assert encode_frame(response.to_dict()).endswith(b"\n")


def test_response_rejects_wrong_request_and_boolean_code():
    payload = {
        "version": 1,
        "request_id": "other",
        "operation": "MoveZero",
        "code": True,
        "motion_service": "ai-w",
    }
    with pytest.raises(ProtocolError, match="request_id mismatch"):
        decode_response(payload, expected_request_id="req-1")
    payload["request_id"] = "req-1"
    with pytest.raises(ProtocolError, match="integer code"):
        decode_response(payload, expected_request_id="req-1")


def test_frame_is_canonical_and_bounded():
    assert encode_frame({"b": 2, "a": 1}) == b'{"a":1,"b":2}\n'
    with pytest.raises(ProtocolError, match="frame exceeds"):
        encode_frame({"data": "x" * MAX_FRAME_BYTES})
