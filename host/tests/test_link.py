"""Link layer port: same cases as test/test_link (C++), plus scenario replay."""

from __future__ import annotations

import random
from typing import Any

import pytest

from nrflink.framing import crc16_ccitt
from nrflink.link import (
    HEADER_SIZE,
    Channel,
    FrameHeader,
    FrameType,
    LinkConfig,
    LinkEndpoint,
    Reassembler,
    ReassemblyResult,
    decode_header,
    encode_header,
    fragment_count,
    message_crc,
)


def pattern(n: int, start: int = 0) -> bytes:
    return bytes((start + i * 7) & 0xFF for i in range(n))


class Box:
    def __init__(self) -> None:
        self.reliable: list[bytes] = []
        self.unreliable: list[bytes] = []
        self.results: list[tuple[int, bool]] = []

    def deliver(self, ch: Channel, data: bytes) -> None:
        (self.reliable if ch == Channel.RELIABLE else self.unreliable).append(data)

    def result(self, seq: int, ok: bool) -> None:
        self.results.append((seq, ok))


class Pair:
    def __init__(self, cfg: LinkConfig | None = None) -> None:
        self.in_a = Box()
        self.in_b = Box()
        self.a = LinkEndpoint(cfg or LinkConfig(), self.in_a.deliver, self.in_a.result)
        self.b = LinkEndpoint(cfg or LinkConfig(), self.in_b.deliver, self.in_b.result)


def hop(
    src: LinkEndpoint, dst: LinkEndpoint, now: int, drop: bool = False, idle: bool = False
) -> int:
    f = src.next_frame(now, 0, 0, idle)
    if not f:
        return 0
    src.on_tx_result(not drop)
    if not drop:
        dst.on_frame(f, now)
    return len(f)


def test_header_round_trip() -> None:
    h = FrameHeader(FrameType.UNRELIABLE, True, 9, 200, 3, 5, 17, 6)
    raw = encode_header(h)
    assert raw[0] == 0x59 and raw[2] == 0x34
    assert decode_header(raw + b"\xab") == h


@pytest.mark.parametrize(
    "frame",
    [
        bytes(6),  # idle with payload
        bytes(4),  # too short
        bytes((0x20, 0, 0, 0, 0)),  # data without payload
        bytes((0x60, 0, 0, 0, 0, 1)),  # unknown type
        bytes((0x20, 0, 0x21, 0, 0, 1)),  # index 2 of 2
        bytes(33),
    ],
)
def test_header_rejects(frame: bytes) -> None:
    assert decode_header(frame) is None


def test_fragment_count() -> None:
    assert [fragment_count(n) for n in (1, 25, 26, 52, 53, 16 * 27 - 2)] == [1, 1, 2, 2, 3, 16]


def test_message_crc_covers_seq() -> None:
    assert message_crc(5, b"\x01\x02\x03") == crc16_ccitt(b"\x05\x01\x02\x03")
    assert message_crc(5, b"\x01") != message_crc(6, b"\x01")


def test_reassembler_rejects_bad_fragments() -> None:
    r = Reassembler(max_message=40)
    assert r.add(0, 0, 0, b"x") == ReassemblyResult.MALFORMED
    assert r.add(0, 0, 17, b"x") == ReassemblyResult.MALFORMED
    assert r.add(0, 0, 2, b"short") == ReassemblyResult.MALFORMED  # non-final must be full
    assert r.add(0, 2, 3, bytes(27)) == ReassemblyResult.MALFORMED  # beyond 42-byte buffer
    assert r.add(0, 0, 1, b"\x01\x02") == ReassemblyResult.MALFORMED  # no room for a message


def test_reliable_single_fragment() -> None:
    p = Pair()
    msg = pattern(10)
    assert p.a.send_reliable(msg, 0)
    assert not p.a.send_reliable(msg, 0)
    assert hop(p.a, p.b, 1) == HEADER_SIZE + 12
    assert p.in_b.reliable == [msg]
    assert hop(p.b, p.a, 2) == HEADER_SIZE
    assert not p.a.reliable_busy
    assert p.in_a.results == [(0, True)]
    assert hop(p.b, p.a, 3) == 0


def test_multi_fragment_with_failed_fragment() -> None:
    p = Pair()
    msg = pattern(100)
    p.a.send_reliable(msg, 0)
    hop(p.a, p.b, 1)
    hop(p.a, p.b, 2, drop=True)
    hop(p.a, p.b, 3)
    hop(p.a, p.b, 4)
    assert p.in_b.reliable == []
    hop(p.a, p.b, 5)
    assert p.in_b.reliable == [msg]
    s = p.a.stats
    assert (s.frames_failed, s.reliable_retries, s.frames_sent) == (1, 1, 5)


def test_ack_lost_and_duplicate_suppressed() -> None:
    p = Pair(LinkConfig(reliable_timeout_ms=50))
    msg = pattern(30)
    p.a.send_reliable(msg, 0)
    hop(p.a, p.b, 0)
    hop(p.a, p.b, 1)
    hop(p.b, p.a, 2, drop=True)
    assert p.a.next_frame(40, 0, 0, False) == b""
    hop(p.a, p.b, 51)
    hop(p.a, p.b, 52)
    assert p.in_b.reliable == [msg]
    assert p.b.stats.duplicates >= 1
    hop(p.b, p.a, 53)
    assert not p.a.reliable_busy
    assert p.in_a.results[-1][1]


def test_reliable_gives_up_then_recovers() -> None:
    p = Pair(LinkConfig(reliable_timeout_ms=10, reliable_max_attempts=3))
    msg = pattern(5)
    p.a.send_reliable(msg, 0)
    now = 0
    for _ in range(10):
        if not p.a.reliable_busy:
            break
        hop(p.a, p.b, now)
        now += 11
        p.a.poll(now)
    assert p.a.stats.reliable_failed == 1
    assert p.a.stats.reliable_retries == 2
    assert p.in_a.results[-1] == (0, False)
    p.a.send_reliable(msg, now)
    hop(p.a, p.b, now)
    assert len(p.in_b.reliable) == 2


def test_fragment_failures_count_as_attempts() -> None:
    p = Pair(LinkConfig(reliable_max_attempts=2))
    p.a.send_reliable(pattern(5), 0)
    hop(p.a, p.b, 0, drop=True)
    assert p.a.reliable_busy
    hop(p.a, p.b, 1, drop=True)
    assert not p.a.reliable_busy
    assert p.a.stats.reliable_failed == 1


def test_unreliable_replace_gap_abort() -> None:
    p = Pair()
    assert p.a.send_unreliable(pattern(8, 1))
    assert p.a.send_unreliable(pattern(8, 2))
    assert p.a.stats.unreliable_replaced == 1
    hop(p.a, p.b, 0)
    assert p.in_b.unreliable == [pattern(8, 2)]
    p.a.send_unreliable(pattern(8, 1))
    hop(p.a, p.b, 1, drop=True)
    p.a.send_unreliable(pattern(8, 3))
    hop(p.a, p.b, 2)
    assert p.b.stats.unreliable_lost == 1
    assert p.a.stats.unreliable_aborted == 1


def test_unreliable_not_cut_in_half() -> None:
    p = Pair()
    p.a.send_unreliable(pattern(60))
    hop(p.a, p.b, 0)
    assert not p.a.send_unreliable(pattern(4))
    hop(p.a, p.b, 1, drop=True)
    assert not p.a.unreliable_pending
    assert p.a.send_unreliable(pattern(4))
    hop(p.a, p.b, 2)
    assert p.in_b.unreliable == [pattern(4)]


def test_priority_crc_duplicates_and_limits() -> None:
    p = Pair()
    p.a.send_unreliable(pattern(4, 9))
    p.a.send_reliable(pattern(4, 1), 0)
    assert p.a.next_frame(0, 0, 0, False)[0] & 0xE0 == 0x20
    assert p.a.next_frame(0, 0, 0, False)[0] & 0xE0 == 0x40

    p = Pair()
    p.a.send_unreliable(pattern(12))
    f = bytearray(p.a.next_frame(0, 0, 0, False))
    f[7] ^= 1
    assert p.b.on_frame(bytes(f), 0) is not None
    assert p.b.stats.crc_errors == 1 and p.in_b.unreliable == []

    p = Pair()
    p.a.send_unreliable(pattern(40))
    f0 = p.a.next_frame(0, 0, 0, False)
    f1 = p.a.next_frame(0, 0, 0, False)
    for frame in (f0, f0, f1, f1):
        p.b.on_frame(frame, 0)
    assert len(p.in_b.unreliable) == 1 and p.b.stats.duplicates == 2

    p = Pair()
    assert not p.a.send_reliable(pattern(161), 0)
    assert not p.a.send_unreliable(pattern(161))
    assert not p.a.send_reliable(b"", 0)
    assert p.a.send_reliable(pattern(160), 0)
    for t in range(20):
        hop(p.a, p.b, t)
    assert p.in_b.reliable == [pattern(160)]


def test_idle_frames_and_initial_sequences() -> None:
    p = Pair()
    assert p.a.next_frame(0, 3, 4, False) == b""
    f = p.a.next_frame(0, 3, 4, True)
    assert f == bytes((0x04, 0, 0, 0, 3))
    h = p.b.on_frame(f, 0)
    assert h is not None and (h.hop, h.phase) == (3, 4)
    assert p.b.on_frame(f[:3], 0) is None
    assert p.b.stats.malformed == 1

    p = Pair()
    p.a.set_next_sequences(250, 99)
    p.a.send_reliable(b"abc", 0)
    assert p.a.reliable_seq == 250
    p.a.send_unreliable(b"abc")
    assert p.a.next_frame(0, 0, 0, False)[1] == 250
    assert p.a.next_frame(0, 0, 0, False)[1] == 99


def test_reliable_under_heavy_loss() -> None:
    p = Pair(LinkConfig(reliable_timeout_ms=20, reliable_max_attempts=50))
    rng = random.Random(12345)
    now = 0
    sent = 0
    expected = [pattern(5 + (i * 13) % 90, i) for i in range(40)]
    while sent < 40 or p.a.reliable_busy:
        if not p.a.reliable_busy and sent < 40:
            assert p.a.send_reliable(expected[sent], now)
            sent += 1
        hop(p.a, p.b, now, rng.random() < 0.3, True)
        hop(p.b, p.a, now, rng.random() < 0.3, True)
        now += 2
        assert now < 200_000
    assert p.in_b.reliable == expected
    assert p.a.stats.reliable_acked == 40


def _replay(scenario: dict[str, Any]) -> None:
    cfg = LinkConfig(**scenario["config"])
    events: dict[str, dict[str, list[Any]]] = {n: {"d": [], "r": []} for n in ("a", "b")}

    def make(name: str) -> LinkEndpoint:
        ev = events[name]
        return LinkEndpoint(
            LinkConfig(**scenario["config"]),
            deliver=lambda ch, data: ev["d"].append({"channel": int(ch), "data": data.hex()}),
            result=lambda seq, ok: ev["r"].append({"seq": seq, "ok": ok}),
        )

    del cfg
    ep = {"a": make("a"), "b": make("b")}
    for step in scenario["steps"]:
        e = ep[step["ep"]]
        op = step["op"]
        if op == "send_reliable":
            assert e.send_reliable(bytes.fromhex(step["data"]), step["now"]) == step["ret"]
        elif op == "send_unreliable":
            assert e.send_unreliable(bytes.fromhex(step["data"])) == step["ret"]
        elif op == "next_frame":
            f = e.next_frame(step["now"], step["hop"], step["phase"], step["idle"])
            assert f.hex() == step["frame"]
        elif op == "tx_result":
            e.on_tx_result(step["acked"])
        elif op == "on_frame":
            assert (e.on_frame(bytes.fromhex(step["frame"]), step["now"]) is not None) == step[
                "ret"
            ]
        elif op == "poll":
            e.poll(step["now"])
        elif op == "stats":
            assert e.stats.__dict__ == step["stats"]
        ev = events[step["ep"]]
        assert ev["d"] == step["delivered"]
        assert ev["r"] == step["results"]
        ev["d"].clear()
        ev["r"].clear()


def test_scenarios_replay(vectors: dict[str, Any]) -> None:
    assert len(vectors["link_scenarios"]) == 4
    for scenario in vectors["link_scenarios"]:
        _replay(scenario)


def test_link_frame_vectors(vectors: dict[str, Any]) -> None:
    for case in vectors["link_frames"]:
        h = dict(case["header"])
        h["type"] = FrameType(h["type"])
        header = FrameHeader(**h)
        assert encode_header(header).hex() == case["hex"]
