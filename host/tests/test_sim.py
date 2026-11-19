from __future__ import annotations

import random

import pytest

from nrflink.hopping import HopSequence
from nrflink.sim import (
    Bernoulli,
    GilbertElliott,
    Interference,
    NoLoss,
    RadioParams,
    SimConfig,
    SimResult,
    _Gateway,
    _percentile,
    describe_config,
    simulate,
    summary_line,
    sweep,
)


def test_airtime_hand_derived() -> None:
    # 1 preamble + 5 address + 32 payload + 2 CRC bytes and a 9-bit PCF: 329 bits.
    assert RadioParams().airtime_us(32) == 329
    assert RadioParams(data_rate_bps=2_000_000).airtime_us(32) == 165  # 164.5 rounded up
    assert RadioParams(data_rate_bps=250_000).airtime_us(0) == 4 * (8 * 8 + 9)


def test_bernoulli_rate() -> None:
    rng = random.Random(3)
    m = Bernoulli(0.3)
    n = sum(m.lost(rng) for _ in range(20000))
    assert 0.28 < n / 20000 < 0.32


def test_gilbert_elliott_rate_and_bursts() -> None:
    rng = random.Random(4)
    m = GilbertElliott(0.2, 10.0)
    seq = [m.lost(rng) for _ in range(200000)]
    assert 0.18 < sum(seq) / len(seq) < 0.22
    bursts = []
    run = 0
    for lost in seq:
        if lost:
            run += 1
        elif run:
            bursts.append(run)
            run = 0
    assert 8.5 < sum(bursts) / len(bursts) < 11.5
    with pytest.raises(ValueError, match="mean_loss"):
        GilbertElliott(1.0)


def test_wifi_interference_channels() -> None:
    w = Interference.wifi(6)
    assert min(w.channels) == 26 and max(w.channels) == 48  # 2426..2448 MHz


def test_no_loss_delivers_everything() -> None:
    r = simulate(SimConfig(duration_s=10))
    assert r.telemetry_generated == 159  # t = 50, 100, ... 7950 ms (2 s drain)
    assert r.telemetry_delivery == 1.0
    assert r.command_delivery == 1.0 and r.commands_issued == 15
    assert r.frames_failed == 0 and r.mean_attempts == 1.0
    assert r.duplicates_at_gateway == 0 and r.crc_errors == 0
    # A command waits at most one poll interval (10 ms) plus one ESB transaction.
    assert max(r.command_latency_ms) < 12


def test_deterministic_for_a_seed() -> None:
    a = simulate(SimConfig(duration_s=5, seed=9, loss=Bernoulli(0.3)))
    b = simulate(SimConfig(duration_s=5, seed=9, loss=Bernoulli(0.3)))
    assert summary_line(a) == summary_line(b)


def test_retries_hide_moderate_independent_loss() -> None:
    r = simulate(SimConfig(duration_s=10, loss=Bernoulli(0.3)))
    assert r.telemetry_delivery > 0.99
    assert 1.5 < r.mean_attempts < 2.6  # 1 / (1 - (1 - 0.7^2)) ~ 2.0 attempts


def test_fewer_retransmits_lose_more() -> None:
    hi = simulate(SimConfig(duration_s=10, loss=Bernoulli(0.6)))
    lo_cfg = SimConfig(duration_s=10, loss=Bernoulli(0.6))
    lo_cfg.radio.arc = 3
    lo = simulate(lo_cfg)
    assert lo.telemetry_delivery < hi.telemetry_delivery


def test_hopping_survives_wifi_but_fixed_jammed_channel_does_not() -> None:
    fixed = simulate(
        SimConfig(duration_s=6, hopping=False, fixed_channel=40, interference=Interference.wifi(6))
    )
    hopping = simulate(SimConfig(duration_s=6, interference=Interference.wifi(6)))
    assert fixed.telemetry_delivered == 0
    assert hopping.telemetry_delivery > 0.3
    assert hopping.command_delivery > 0.9


def test_gateway_reboot_resynchronises() -> None:
    r = simulate(SimConfig(duration_s=6, gateway_boot_ms=1500))
    assert r.first_lock_ms is not None
    # Park period is (8 + 1) slots of 50 ms.
    assert 1500 <= r.first_lock_ms <= 1500 + 450 + 50
    assert r.commands_delivered == r.commands_issued > 0


def test_ack_payload_fifo_semantics() -> None:
    cfg = SimConfig()
    hops = HopSequence.generated(1, 2, 80, 2, 8)
    gw = _Gateway(cfg, hops, SimResult(cfg))
    gw.tick(0)
    assert gw.booted
    gw.fifo.extend([])
    gw.link.send_reliable(b"\x01\x10\x04\x01\x00\x00\x00", 0)
    gw.refill(0)
    assert len(gw.fifo) == 1
    idle = bytes((0x04, 0, 0, 0, 0))
    first = gw.on_packet(1, idle, 1000)
    assert first == gw.fifo[0].data and gw.fifo[0].sent
    # Same PID and payload: a retransmission, ACKed with the same payload.
    assert gw.on_packet(1, idle, 2000) == first
    # A new PID retires the payload that has been sent (TX_DS on the PRX).
    assert gw.on_packet(2, idle, 3000) == b""
    assert not gw.fifo


def test_sweep_and_helpers() -> None:
    s = sweep((0.0, 0.5), burst=10.0, duration_s=4)
    assert s.label == "burst 10 pkts, ARC=15"
    assert s.results[0].telemetry_delivery == 1.0
    assert _percentile([1.0, 2.0, 3.0, 4.0], 0.5) == 2.5
    assert _percentile([], 0.5) != _percentile([], 0.5)  # NaN
    assert "no loss" in describe_config(SimConfig())
    assert "fixed channel 76" in describe_config(SimConfig(hopping=False))
    assert NoLoss().describe() == "no loss"
    with pytest.raises(ValueError, match="drain"):
        simulate(SimConfig(duration_s=1, drain_s=2))
