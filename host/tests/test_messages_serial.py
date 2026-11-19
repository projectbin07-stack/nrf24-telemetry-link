from __future__ import annotations

import struct
from typing import Any

import pytest

from nrflink.messages import (
    Battery,
    Command,
    Imu,
    KeyValue,
    KvEntry,
    KvType,
    MessageError,
    Odometry,
    decode_message,
    describe,
    encode_message,
    message_from_dict,
    message_name,
    message_to_dict,
)
from nrflink.serial_frame import (
    GATEWAY_STATS_SIZE,
    FrameError,
    GatewayStats,
    SerialKind,
    StreamDecoder,
    decode_command_result,
    decode_gateway_stats,
    decode_message_body,
    decode_packet,
    decode_wire_frame,
    encode_gateway_stats,
    encode_packet,
    encode_wire,
)


def test_message_vectors(vectors: dict[str, Any]) -> None:
    assert len(vectors["messages"]) >= 8
    for case in vectors["messages"]:
        m = message_from_dict(case["type"], case["fields"])
        assert encode_message(m).hex() == case["hex"], case["name"]
        decoded = decode_message(bytes.fromhex(case["hex"]))
        assert decoded == m
        assert message_name(decoded) == case["type"]
        assert message_to_dict(decoded) == case["fields"]


def test_invalid_message_vectors(vectors: dict[str, Any]) -> None:
    # Each case must not decode as its declared type (it may raise, or be a
    # valid message of another type, like the IMU body under "odometry").
    for case in vectors["invalid_messages"]:
        try:
            m = decode_message(bytes.fromhex(case["hex"]))
        except MessageError:
            continue
        assert message_name(m) != case["type"], case["name"]


def test_battery_layout_hand_derived() -> None:
    raw = encode_message(Battery(12034, 1523400, 18333000, -2500, 873, 0))
    assert raw[:2] == b"\x01\x01"
    assert raw[2:4] == (12034).to_bytes(2, "little")
    assert raw[4:8] == (1523400).to_bytes(4, "little")
    assert raw[12:16] == (-2500).to_bytes(4, "little", signed=True)
    assert len(raw) == 19


def test_kv_encoding_rules() -> None:
    kv = KeyValue((KvEntry("a", KvType.INT, 1), KvEntry("s", KvType.STRING, "hi")))
    assert encode_message(kv) == bytes.fromhex("01040201610001000000017303026869")
    with pytest.raises(MessageError):
        encode_message(KeyValue((KvEntry("bad key", KvType.INT, 1),)))
    with pytest.raises(MessageError):
        encode_message(KeyValue((KvEntry("k", KvType.STRING, "x" * 33),)))
    with pytest.raises(MessageError):
        encode_message(KeyValue(tuple(KvEntry(f"k{i}", KvType.BOOL, True) for i in range(16))))
    assert kv.get("s") == "hi" and kv.get("zz") is None


def test_unknown_type_and_describe() -> None:
    with pytest.raises(MessageError, match="unknown message type"):
        decode_message(b"\x01\x7f")
    assert describe(Battery(7400, soc_permille=0xFFFF, flags=0x80)).startswith("battery 7.400 V")
    assert "soc=50.0%" in describe(Battery(7400, soc_permille=500))
    assert describe(Command(1, 100)) == "command set_telemetry_period_ms arg=100"
    assert describe(Command(99, 0)) == "command op99 arg=0"
    assert "theta=-3.141" in describe(Odometry(1, 2, 3, -3141, 4, 5))
    assert "temp=25.12 C" in describe(Imu(1, (0, 0, 1000), (100, 0, 0), 2512))
    assert describe(KeyValue((KvEntry("v", KvType.FLOAT, 3.5),))) == "kv v=3.5"
    with pytest.raises(MessageError):
        message_from_dict("nope", {})


def test_serial_vectors(vectors: dict[str, Any]) -> None:
    for case in vectors["serial_frames"]:
        wire = encode_wire(case["kind"], case["seq"], bytes.fromhex(case["body"]))
        assert wire.hex() == case["wire"]
        pkt = decode_wire_frame(wire[:-1])
        assert (pkt.kind, pkt.seq, pkt.body.hex()) == (case["kind"], case["seq"], case["body"])
    gs = vectors["gateway_stats"]
    stats = GatewayStats(**gs["fields"])
    assert encode_gateway_stats(stats).hex() == gs["hex"]
    assert decode_gateway_stats(bytes.fromhex(gs["hex"])) == stats
    assert GATEWAY_STATS_SIZE == 56


def test_packet_layout_hand_derived() -> None:
    pkt = encode_packet(SerialKind.LOG, 3, b"hi")
    assert pkt[:6] == bytes((1, 4, 3, 2)) + b"hi"
    (crc,) = struct.unpack("<H", pkt[6:])
    from nrflink.framing import crc16_ccitt

    assert crc == crc16_ccitt(bytes((1, 4, 3, 2)) + b"hi")


@pytest.mark.parametrize(
    ("packet", "kind"),
    [
        (b"\x01\x01\x00", "short"),
        (bytes((2, 1, 0, 0, 0, 0)), "version"),
        (bytes((1, 1, 0, 5, 0, 0)), "length"),
        (bytes((1, 1, 0, 0, 0, 0)), "crc"),
    ],
)
def test_packet_errors(packet: bytes, kind: str) -> None:
    with pytest.raises(FrameError) as info:
        decode_packet(packet)
    assert info.value.kind == kind


def test_stream_decoder_counts_errors_and_gaps() -> None:
    d = StreamDecoder(max_frame=64)
    good = [encode_wire(SerialKind.LOG, s, b"x") for s in (1, 2, 5)]
    stream = b"\x11\x22\x00" + good[0] + b"\x00\x00" + good[1] + b"\x55" * 80 + b"\x00"
    stream += bytearray(good[2][:-1]) + b"\x00"
    corrupt = bytearray(encode_wire(SerialKind.LOG, 6, b"y"))
    corrupt[5] ^= 0x01  # the "y" body byte: CRC mismatch
    pkts = d.feed(stream) + d.feed(bytes(corrupt))
    assert [p.seq for p in pkts] == [1, 2, 5]
    assert d.lost == 2
    assert d.errors == {"cobs": 1, "overlong": 1, "crc": 1}


def test_body_decoders() -> None:
    ev = decode_message_body(bytes((1, 1, 1)) + b"\x01\x10")
    assert (ev.channel, ev.pipe, ev.rpd, ev.message) == (1, 1, True, b"\x01\x10")
    assert decode_command_result(b"\x07\x01") == (7, True)
    with pytest.raises(FrameError):
        decode_message_body(b"\x00")
    with pytest.raises(FrameError):
        decode_command_result(b"\x00")
    with pytest.raises(FrameError):
        decode_gateway_stats(bytes(55))
    with pytest.raises(ValueError, match="255"):
        encode_packet(1, 0, bytes(256))
