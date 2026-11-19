"""Gateway USB serial framing, mirror of ``lib/linkcore/src/linkcore/serial_frame.h``.

::

    wire   = COBS(packet) 0x00
    packet = version:u8 (=1) kind:u8 seq:u8 length:u8 body[length] crc:u16 LE

CRC-16/CCITT-FALSE over ``version .. body`` (the same layout as the
encoder-odometry-firmware protocol that serial-scope decodes).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, fields
from enum import IntEnum

from .framing import CobsError, cobs_decode, cobs_encode, crc16_ccitt

SERIAL_VERSION = 1
SERIAL_OVERHEAD = 6
MAX_BODY = 255
MESSAGE_FLAG_RPD = 0x01


class SerialKind(IntEnum):
    MESSAGE = 0x01
    LINK_STATS = 0x02
    COMMAND_RESULT = 0x03
    LOG = 0x04
    SEND_COMMAND = 0x81
    REQUEST_STATS = 0x82


class FrameError(ValueError):
    """A rejected serial frame; ``kind`` is cobs, short, version, length or crc."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


@dataclass(frozen=True)
class SerialPacket:
    kind: int
    seq: int
    body: bytes


def encode_packet(kind: int, seq: int, body: bytes) -> bytes:
    if len(body) > MAX_BODY:
        raise ValueError("body longer than 255 bytes")
    head = bytes((SERIAL_VERSION, kind & 0xFF, seq & 0xFF, len(body))) + bytes(body)
    return head + struct.pack("<H", crc16_ccitt(head))


def encode_wire(kind: int, seq: int, body: bytes) -> bytes:
    return cobs_encode(encode_packet(kind, seq, body)) + b"\x00"


def decode_packet(packet: bytes) -> SerialPacket:
    if len(packet) < SERIAL_OVERHEAD:
        raise FrameError("short", f"packet of {len(packet)} bytes is shorter than 6")
    if packet[0] != SERIAL_VERSION:
        raise FrameError("version", f"unsupported serial version {packet[0]}")
    if packet[3] + SERIAL_OVERHEAD != len(packet):
        raise FrameError("length", "length byte does not match the packet size")
    (crc,) = struct.unpack_from("<H", packet, len(packet) - 2)
    if crc16_ccitt(packet[:-2]) != crc:
        raise FrameError("crc", "CRC mismatch")
    return SerialPacket(packet[1], packet[2], bytes(packet[4:-2]))


def decode_wire_frame(frame: bytes) -> SerialPacket:
    """Decode the bytes between two 0x00 delimiters."""
    try:
        packet = cobs_decode(frame)
    except CobsError as exc:
        raise FrameError("cobs", str(exc)) from None
    return decode_packet(packet)


class StreamDecoder:
    """Splits a byte stream on 0x00 and decodes each frame.

    ``feed`` returns the packets completed by the new bytes; rejected frames
    are counted in ``errors`` (by kind) and ``seq`` gaps in ``lost``.
    """

    def __init__(self, max_frame: int = 1024) -> None:
        self.max_frame = max_frame
        self._buf = bytearray()
        self._overflow = False
        self.packets = 0
        self.errors: dict[str, int] = {}
        self.lost = 0
        self._last_seq: int | None = None

    def _error(self, kind: str) -> None:
        self.errors[kind] = self.errors.get(kind, 0) + 1

    def feed(self, data: bytes) -> list[SerialPacket]:
        out: list[SerialPacket] = []
        for byte in data:
            if byte != 0:
                if len(self._buf) < self.max_frame:
                    self._buf.append(byte)
                else:
                    self._overflow = True
                continue
            frame = bytes(self._buf)
            overflow = self._overflow
            self._buf.clear()
            self._overflow = False
            if overflow:
                self._error("overlong")
                continue
            if not frame:
                continue
            try:
                pkt = decode_wire_frame(frame)
            except FrameError as exc:
                self._error(exc.kind)
                continue
            if self._last_seq is not None:
                self.lost += (pkt.seq - self._last_seq - 1) & 0xFF
            self._last_seq = pkt.seq
            self.packets += 1
            out.append(pkt)
        return out


@dataclass(frozen=True)
class GatewayStats:
    uptime_ms: int = 0
    frames_received: int = 0
    malformed: int = 0
    crc_errors: int = 0
    duplicates: int = 0
    delivered_reliable: int = 0
    delivered_unreliable: int = 0
    unreliable_lost: int = 0
    reliable_sent: int = 0
    reliable_acked: int = 0
    reliable_failed: int = 0
    reliable_retries: int = 0
    resyncs: int = 0
    hop_index: int = 0
    hop_locked: int = 0
    channel: int = 0
    rpd_percent: int = 0


_STATS_FORMAT = "<13I4B"
GATEWAY_STATS_SIZE = struct.calcsize(_STATS_FORMAT)  # 56


def encode_gateway_stats(s: GatewayStats) -> bytes:
    return struct.pack(_STATS_FORMAT, *(getattr(s, f.name) for f in fields(s)))


def decode_gateway_stats(body: bytes) -> GatewayStats:
    if len(body) != GATEWAY_STATS_SIZE:
        raise FrameError("length", f"link_stats body must be {GATEWAY_STATS_SIZE} bytes")
    return GatewayStats(*struct.unpack(_STATS_FORMAT, body))


@dataclass(frozen=True)
class MessageEvent:
    channel: int
    pipe: int
    rpd: bool
    message: bytes


def decode_message_body(body: bytes) -> MessageEvent:
    if len(body) < 3:
        raise FrameError("length", "message body shorter than 3 bytes")
    return MessageEvent(body[0], body[1], bool(body[2] & MESSAGE_FLAG_RPD), bytes(body[3:]))


def decode_command_result(body: bytes) -> tuple[int, bool]:
    if len(body) != 2:
        raise FrameError("length", "command_result body must be 2 bytes")
    return body[0], body[1] == 1
