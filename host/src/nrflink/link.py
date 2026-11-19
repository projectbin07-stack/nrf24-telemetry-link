"""Line-by-line Python port of ``lib/linkcore/src/linkcore/link.{h,cpp}``.

The link simulator runs this code, so its results describe the same state
machines the firmware runs. ``test/vectors/link_vectors.json`` contains
scripted scenarios (generated from this module) that the C++ unit test
``test/test_vectors`` replays byte for byte, which is what keeps the two
implementations in step.

Frame layout (at most 32 bytes, one ESB payload)::

    byte 0  type[7:5] | ack_valid[4] | phase[3:0]
    byte 1  seq
    byte 2  frag_index[7:4] | (frag_count - 1)[3:0]
    byte 3  ack   (last reliable seq delivered by the sender of this frame)
    byte 4  hop   (hop index the sender is on)
    5..31   fragment of (message || crc16 LE)
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from enum import IntEnum

from .framing import crc16_ccitt

FRAME_MAX = 32
HEADER_SIZE = 5
FRAGMENT_PAYLOAD = FRAME_MAX - HEADER_SIZE  # 27
MAX_FRAGMENTS = 16
DEFAULT_MAX_MESSAGE = 160


class FrameType(IntEnum):
    IDLE = 0
    RELIABLE = 1
    UNRELIABLE = 2


class Channel(IntEnum):
    RELIABLE = 0
    UNRELIABLE = 1


@dataclass(frozen=True)
class FrameHeader:
    type: FrameType = FrameType.IDLE
    ack_valid: bool = False
    phase: int = 0
    seq: int = 0
    frag_index: int = 0
    frag_count: int = 1
    ack: int = 0
    hop: int = 0


def encode_header(h: FrameHeader) -> bytes:
    return bytes(
        (
            ((int(h.type) << 5) | (0x10 if h.ack_valid else 0) | (h.phase & 0x0F)) & 0xFF,
            h.seq & 0xFF,
            (((h.frag_index & 0x0F) << 4) | ((h.frag_count - 1) & 0x0F)) & 0xFF,
            h.ack & 0xFF,
            h.hop & 0xFF,
        )
    )


def decode_header(frame: bytes) -> FrameHeader | None:
    """Mirror of ``decode_header``: ``None`` where the C++ returns false."""
    n = len(frame)
    if n < HEADER_SIZE or n > FRAME_MAX:
        return None
    t = frame[0] >> 5
    if t > FrameType.UNRELIABLE:
        return None
    h = FrameHeader(
        type=FrameType(t),
        ack_valid=bool(frame[0] & 0x10),
        phase=frame[0] & 0x0F,
        seq=frame[1],
        frag_index=frame[2] >> 4,
        frag_count=(frame[2] & 0x0F) + 1,
        ack=frame[3],
        hop=frame[4],
    )
    if h.frag_index >= h.frag_count:
        return None
    if h.type == FrameType.IDLE:
        if n != HEADER_SIZE:
            return None
    elif n == HEADER_SIZE:
        return None
    return h


def fragment_count(length: int) -> int:
    return (length + 2 + FRAGMENT_PAYLOAD - 1) // FRAGMENT_PAYLOAD


def message_crc(seq: int, message: bytes) -> int:
    return crc16_ccitt(message, crc16_ccitt(bytes((seq & 0xFF,))))


class ReassemblyResult(IntEnum):
    INCOMPLETE = 0
    COMPLETE = 1
    DUPLICATE_FRAGMENT = 2
    CRC_ERROR = 3
    MALFORMED = 4


class Reassembler:
    def __init__(self, max_message: int = DEFAULT_MAX_MESSAGE) -> None:
        self.capacity = max_message + 2
        self.buffer = bytearray(self.capacity)
        self.total = 0
        self.message_length = 0
        self.mask = 0
        self.seq = 0
        self.count = 0
        self.active = False

    def reset(self) -> None:
        self.active = False

    def add(self, seq: int, index: int, count: int, data: bytes) -> ReassemblyResult:
        length = len(data)
        if (
            count == 0
            or count > MAX_FRAGMENTS
            or index >= count
            or length == 0
            or length > FRAGMENT_PAYLOAD
        ):
            return ReassemblyResult.MALFORMED
        if not self.active or seq != self.seq or count != self.count:
            self.active = True
            self.seq = seq
            self.count = count
            self.mask = 0
            self.total = 0
        bit = 1 << index
        if self.mask & bit:
            return ReassemblyResult.DUPLICATE_FRAGMENT
        offset = index * FRAGMENT_PAYLOAD
        last = index == count - 1
        if (not last and length != FRAGMENT_PAYLOAD) or offset + length > self.capacity:
            self.active = False
            return ReassemblyResult.MALFORMED
        self.buffer[offset : offset + length] = data
        if last:
            self.total = offset + length
        self.mask |= bit
        if self.mask != (1 << count) - 1:
            return ReassemblyResult.INCOMPLETE
        self.active = False
        if self.total < 3:
            return ReassemblyResult.MALFORMED
        mlen = self.total - 2
        crc = self.buffer[mlen] | (self.buffer[mlen + 1] << 8)
        if message_crc(seq, bytes(self.buffer[:mlen])) != crc:
            return ReassemblyResult.CRC_ERROR
        self.message_length = mlen
        return ReassemblyResult.COMPLETE

    def message(self) -> bytes:
        return bytes(self.buffer[: self.message_length])


@dataclass
class LinkStats:
    frames_sent: int = 0
    frames_failed: int = 0
    frames_received: int = 0
    malformed: int = 0
    crc_errors: int = 0
    duplicates: int = 0
    delivered_reliable: int = 0
    delivered_unreliable: int = 0
    unreliable_lost: int = 0
    unreliable_replaced: int = 0
    unreliable_aborted: int = 0
    reliable_sent: int = 0
    reliable_acked: int = 0
    reliable_failed: int = 0
    reliable_retries: int = 0


@dataclass
class LinkConfig:
    reliable_timeout_ms: int = 100
    reliable_max_attempts: int = 8
    initial_reliable_seq: int = 0
    initial_unreliable_seq: int = 0
    max_message: int = DEFAULT_MAX_MESSAGE


DeliverFn = Callable[[Channel, bytes], None]
ResultFn = Callable[[int, bool], None]

_TX_IDLE, _TX_SENDING, _TX_WAIT_ACK = 0, 1, 2
_LAST_NONE, _LAST_IDLE, _LAST_RELIABLE, _LAST_UNRELIABLE = 0, 1, 2, 3


def _u32(x: int) -> int:
    return x & 0xFFFFFFFF


def _prepare(seq: int, message: bytes) -> bytes:
    crc = message_crc(seq, message)
    return message + bytes((crc & 0xFF, crc >> 8))


@dataclass
class LinkEndpoint:
    config: LinkConfig = field(default_factory=LinkConfig)
    deliver: DeliverFn | None = None
    result: ResultFn | None = None

    def __post_init__(self) -> None:
        self.stats = LinkStats()
        self.max_message = self.config.max_message
        self._tx_buffer = b""
        self._tx_seq = 0
        self._tx_next_seq = self.config.initial_reliable_seq & 0xFF
        self._tx_count = 0
        self._tx_frag_next = 0
        self._tx_attempts = 0
        self._tx_sent_at = 0
        self._tx_state = _TX_IDLE
        self._u_buffer = b""
        self._u_seq = 0
        self._u_next_seq = self.config.initial_unreliable_seq & 0xFF
        self._u_count = 0
        self._u_frag_next = 0
        self._u_pending = False
        self._last_kind = _LAST_NONE
        self._last_index = 0
        self._last_seq = 0
        self._rx_reliable = Reassembler(self.max_message)
        self._rx_unreliable = Reassembler(self.max_message)
        self._rx_reliable_last = 0
        self._rx_reliable_have = False
        self._rx_unreliable_last = 0
        self._rx_unreliable_have = False
        self._ack_pending = False

    # --- public API (same names as the C++ class) ---

    def set_next_sequences(self, reliable: int, unreliable: int) -> None:
        self._tx_next_seq = reliable & 0xFF
        self._u_next_seq = unreliable & 0xFF

    @property
    def reliable_busy(self) -> bool:
        return self._tx_state != _TX_IDLE

    @property
    def reliable_seq(self) -> int:
        return self._tx_seq

    @property
    def unreliable_pending(self) -> bool:
        return self._u_pending

    def send_reliable(self, message: bytes, now_ms: int) -> bool:
        if self._tx_state != _TX_IDLE or len(message) == 0 or len(message) > self.max_message:
            return False
        self._tx_seq = self._tx_next_seq
        self._tx_next_seq = (self._tx_next_seq + 1) & 0xFF
        self._tx_buffer = _prepare(self._tx_seq, bytes(message))
        self._tx_count = fragment_count(len(message))
        self._tx_frag_next = 0
        self._tx_attempts = 1
        self._tx_sent_at = _u32(now_ms)
        self._tx_state = _TX_SENDING
        self.stats.reliable_sent += 1
        return True

    def send_unreliable(self, message: bytes) -> bool:
        if len(message) == 0 or len(message) > self.max_message:
            return False
        if self._u_pending:
            if self._u_frag_next > 0:
                return False
            self.stats.unreliable_replaced += 1
        self._u_seq = self._u_next_seq
        self._u_next_seq = (self._u_next_seq + 1) & 0xFF
        self._u_buffer = _prepare(self._u_seq, bytes(message))
        self._u_count = fragment_count(len(message))
        self._u_frag_next = 0
        self._u_pending = True
        return True

    def _finish_reliable(self, delivered: bool) -> None:
        self._tx_state = _TX_IDLE
        if delivered:
            self.stats.reliable_acked += 1
        else:
            self.stats.reliable_failed += 1
        if self.result is not None:
            self.result(self._tx_seq, delivered)

    def poll(self, now_ms: int) -> None:
        if (
            self._tx_state == _TX_WAIT_ACK
            and _u32(now_ms - self._tx_sent_at) >= self.config.reliable_timeout_ms
        ):
            if self._tx_attempts >= self.config.reliable_max_attempts:
                self._finish_reliable(False)
            else:
                self._tx_attempts += 1
                self.stats.reliable_retries += 1
                self._tx_frag_next = 0
                self._tx_state = _TX_SENDING

    def has_traffic(self, now_ms: int) -> bool:
        self.poll(now_ms)
        return self._tx_state == _TX_SENDING or self._u_pending or self._ack_pending

    def _build(
        self,
        ftype: FrameType,
        seq: int,
        index: int,
        count: int,
        buffer: bytes | None,
        hop: int,
        phase: int,
    ) -> bytes:
        header = encode_header(
            FrameHeader(
                type=ftype,
                ack_valid=self._rx_reliable_have,
                phase=phase & 0x0F,
                seq=seq,
                frag_index=index,
                frag_count=count,
                ack=self._rx_reliable_last if self._rx_reliable_have else 0,
                hop=hop,
            )
        )
        payload = b""
        if buffer is not None:
            offset = index * FRAGMENT_PAYLOAD
            payload = buffer[offset : offset + FRAGMENT_PAYLOAD]
        self._ack_pending = False
        self.stats.frames_sent += 1
        self._last_seq = seq
        self._last_index = index
        return header + payload

    def next_frame(self, now_ms: int, hop: int, phase: int, idle_allowed: bool) -> bytes:
        """Next frame to transmit, or ``b""`` if there is nothing to send."""
        self.poll(now_ms)
        if self._tx_state == _TX_SENDING:
            index = self._tx_frag_next
            frame = self._build(
                FrameType.RELIABLE,
                self._tx_seq,
                index,
                self._tx_count,
                self._tx_buffer,
                hop,
                phase,
            )
            self._last_kind = _LAST_RELIABLE
            self._tx_frag_next += 1
            self._tx_sent_at = _u32(now_ms)
            if self._tx_frag_next >= self._tx_count:
                self._tx_state = _TX_WAIT_ACK
            return frame
        if self._u_pending:
            index = self._u_frag_next
            frame = self._build(
                FrameType.UNRELIABLE,
                self._u_seq,
                index,
                self._u_count,
                self._u_buffer,
                hop,
                phase,
            )
            self._last_kind = _LAST_UNRELIABLE
            self._u_frag_next += 1
            if self._u_frag_next >= self._u_count:
                self._u_pending = False
            return frame
        if self._ack_pending or idle_allowed:
            frame = self._build(FrameType.IDLE, 0, 0, 1, None, hop, phase)
            self._last_kind = _LAST_IDLE
            return frame
        self._last_kind = _LAST_NONE
        return b""

    def on_tx_result(self, acked: bool) -> None:
        kind = self._last_kind
        self._last_kind = _LAST_NONE
        if acked or kind == _LAST_NONE:
            return
        self.stats.frames_failed += 1
        if kind == _LAST_RELIABLE:
            if self._tx_state != _TX_IDLE and self._last_seq == self._tx_seq:
                if self._tx_attempts >= self.config.reliable_max_attempts:
                    self._finish_reliable(False)
                else:
                    self._tx_attempts += 1
                    self.stats.reliable_retries += 1
                    self._tx_frag_next = self._last_index
                    self._tx_state = _TX_SENDING
        elif kind == _LAST_UNRELIABLE:
            self.stats.unreliable_aborted += 1
            if self._u_pending and self._u_seq == self._last_seq:
                self._u_pending = False

    def on_frame(self, frame: bytes, now_ms: int) -> FrameHeader | None:
        """Process a received frame; returns its header, or ``None`` if rejected."""
        del now_ms  # kept for signature parity with the C++ API
        h = decode_header(frame)
        if h is None:
            self.stats.malformed += 1
            return None
        self.stats.frames_received += 1
        if h.ack_valid and self._tx_state != _TX_IDLE and h.ack == self._tx_seq:
            self._finish_reliable(True)
        payload = bytes(frame[HEADER_SIZE:])

        if h.type == FrameType.RELIABLE:
            if self._rx_reliable_have and h.seq == self._rx_reliable_last:
                self.stats.duplicates += 1
                self._ack_pending = True
                return h
            r = self._rx_reliable.add(h.seq, h.frag_index, h.frag_count, payload)
            if r == ReassemblyResult.COMPLETE:
                self._rx_reliable_last = h.seq
                self._rx_reliable_have = True
                self._ack_pending = True
                self.stats.delivered_reliable += 1
                if self.deliver is not None:
                    self.deliver(Channel.RELIABLE, self._rx_reliable.message())
            elif r == ReassemblyResult.DUPLICATE_FRAGMENT:
                self.stats.duplicates += 1
            elif r == ReassemblyResult.CRC_ERROR:
                self.stats.crc_errors += 1
            elif r == ReassemblyResult.MALFORMED:
                self.stats.malformed += 1
                return None
        elif h.type == FrameType.UNRELIABLE:
            if self._rx_unreliable_have and h.seq == self._rx_unreliable_last:
                self.stats.duplicates += 1
                return h
            r = self._rx_unreliable.add(h.seq, h.frag_index, h.frag_count, payload)
            if r == ReassemblyResult.COMPLETE:
                if self._rx_unreliable_have:
                    gap = (h.seq - self._rx_unreliable_last - 1) & 0xFF
                    if gap < 128:
                        self.stats.unreliable_lost += gap
                self._rx_unreliable_last = h.seq
                self._rx_unreliable_have = True
                self.stats.delivered_unreliable += 1
                if self.deliver is not None:
                    self.deliver(Channel.UNRELIABLE, self._rx_unreliable.message())
            elif r == ReassemblyResult.DUPLICATE_FRAGMENT:
                self.stats.duplicates += 1
            elif r == ReassemblyResult.CRC_ERROR:
                self.stats.crc_errors += 1
            elif r == ReassemblyResult.MALFORMED:
                self.stats.malformed += 1
                return None
        return h
