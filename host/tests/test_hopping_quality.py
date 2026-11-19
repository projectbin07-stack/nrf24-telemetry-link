from __future__ import annotations

from typing import Any

import pytest

from nrflink.hopping import HopFollower, HopMaster, HopSequence, xorshift32
from nrflink.quality import LinkQuality


def test_xorshift_reference() -> None:
    s = xorshift32(1)
    assert s == 0x00042021
    assert xorshift32(s) == 0x04080601


def test_sequence_properties() -> None:
    seq = HopSequence.generated(0x12345678, 2, 80, 2, 16)
    assert len(set(seq.channels)) == 16
    assert all(2 <= c <= 80 and c % 2 == 0 for c in seq.channels)
    assert seq.channels == HopSequence.generated(0x12345678, 2, 80, 2, 16).channels
    assert seq.channels != HopSequence.generated(0x12345679, 2, 80, 2, 16).channels
    assert seq.channel(16) == seq.channel(0)


@pytest.mark.parametrize(
    "params",
    [(1, 10, 5, 1, 2), (1, 0, 126, 1, 2), (1, 0, 10, 0, 2), (1, 0, 10, 5, 4), (1, 0, 125, 1, 33)],
)
def test_sequence_rejects(params: tuple[int, int, int, int, int]) -> None:
    seq = HopSequence()
    assert not seq.generate(*params)
    assert seq.length == 1
    with pytest.raises(ValueError, match="invalid"):
        HopSequence.generated(*params)


def test_hop_vectors(vectors: dict[str, Any]) -> None:
    for case in vectors["hop_sequences"]:
        seq = HopSequence.generated(
            case["seed"], case["first"], case["last"], case["spacing"], case["count"]
        )
        assert seq.channels == case["channels"]


def test_master_phase_and_guard() -> None:
    seq = HopSequence.generated(7, 2, 80, 2, 4)
    m = HopMaster(seq, 32)
    m.start(1000)
    assert [m.phase(t) for t in (1000, 1016, 1031)] == [0, 8, 15]
    assert not m.can_transmit(1000) and m.can_transmit(1002)
    assert m.can_transmit(1029) and not m.can_transmit(1031)
    assert not m.update(1031) and m.update(1032) and m.index == 1
    assert m.update(1032 + 96) and m.index == 0


def test_follower_tracks_master() -> None:
    seq = HopSequence.generated(99, 2, 80, 2, 8)
    m = HopMaster(seq, 20)
    f = HopFollower(seq, 20, 3)
    m.start(0)
    f.start(0)
    m.update(5)
    f.on_frame(m.index, m.phase(5), 5)
    for t in range(6, 2000):
        m.update(t)
        f.update(t)
        if t % 7 == 0 and m.can_transmit(t):
            assert m.channel == f.channel
            f.on_frame(m.index, m.phase(t), t)
    assert f.locked and f.resyncs == 0


def test_follower_loses_sync_and_parks() -> None:
    seq = HopSequence.generated(99, 2, 80, 2, 8)
    f = HopFollower(seq, 20, 3)
    f.start(0)
    f.on_frame(2, 0, 0)
    for t in (20, 40, 60):
        f.update(t)
    assert f.locked
    f.update(80)
    assert not f.locked and f.resyncs == 1 and f.index == 6
    f.update(80 + 179)
    assert f.index == 6
    f.update(80 + 180)
    assert f.index == 7
    assert not f.on_frame(8, 0, 300)


def test_resync_with_jammed_channel() -> None:
    seq = HopSequence.generated(0xBEEF, 2, 80, 2, 8)
    jammed = seq.channel(0)
    m = HopMaster(seq, 10)
    f = HopFollower(seq, 10, 4)
    m.start(0)
    f.start(3)
    locked_at = None
    for t in range(3, 2000):
        m.update(t)
        f.update(t)
        if m.can_transmit(t) and m.channel == f.channel and m.channel != jammed:
            f.on_frame(m.index, m.phase(t), t)
            locked_at = locked_at or t
    assert locked_at is not None and locked_at < 3 + 2 * 9 * 10 + 10
    assert f.locked


def test_quality_window_and_score() -> None:
    q = LinkQuality()
    assert (q.delivery_percent(), q.mean_retries_x10(), q.rpd_percent(), q.score()) == (
        100,
        0,
        0,
        100,
    )
    for i in range(4):
        q.record_tx(i != 0, 3)
    assert (q.delivery_percent(), q.mean_retries_x10(), q.score()) == (75, 30, 60)
    for _ in range(40):
        q.record_tx(True, 0)
    assert (q.delivery_percent(), q.mean_retries_x10()) == (100, 0)
    assert (q.total_tx, q.total_failed, q.total_retransmits) == (44, 1, 12)
    q.record_tx(False, 200)
    assert q.total_retransmits == 27
    for b in (True, False, True):
        q.record_rpd(b)
    assert q.rpd_percent() == 67
