"""Link simulator: two modelled nRF24L01+ radios running the ported link layer.

What is modelled
----------------
* **Enhanced ShockBurst timing** (product specification sections 6.1.7, 7.4,
  7.7): Tstby2a (130 us) before the first transmission, packet air time from
  preamble + address + 9-bit packet control field + payload + CRC at the
  configured data rate, ARD between the end of a packet and its
  retransmission, up to ARC retransmissions, an ACK turnaround of 130 us and
  ACK air time including the ACK payload, and a 250 us ACK timeout after the
  last attempt. A fixed per-transaction MCU overhead stands in for SPI
  upload, status polling and payload reads.
* **PTX/PRX semantics**: the 2-bit PID and the PRX's PID+payload duplicate
  check; ACK payloads that stay at the head of the PRX TX FIFO until a packet
  with a new PID arrives (section 7.8.6), a three-level ACK payload FIFO, and
  the PRX MCU refilling it only after the ACK for the current packet has gone.
* **Channel**: independent (Bernoulli) loss or Gilbert-Elliott burst loss per
  over-the-air packet (data and ACK alike), plus optional interference that
  adds loss on a set of RF channels (e.g. a Wi-Fi channel).
* **Firmware logic**: the node and gateway loops of ``src/node_main.cpp`` and
  ``src/gateway_main.cpp`` re-expressed in Python on top of the ported
  :mod:`nrflink.link`, :mod:`nrflink.hopping` and :mod:`nrflink.quality`.

What is not modelled
--------------------
Path loss, multipath, RSSI/RPD, capture effect, crystal drift between the two
boards, SPI timing jitter, serial-port back-pressure on the gateway and
power-supply problems. Results are simulated and say nothing about range.
"""

from __future__ import annotations

import math
import random
import statistics
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Protocol

from .hopping import HopFollower, HopMaster, HopSequence
from .link import Channel, LinkConfig, LinkEndpoint
from .messages import (
    BATTERY_FLAG_NO_CURRENT_SENSOR,
    SOC_UNKNOWN,
    Battery,
    Command,
    CommandOp,
    Imu,
    KeyValue,
    KvEntry,
    KvType,
    MessageError,
    Odometry,
    decode_message,
    encode_message,
)
from .quality import LinkQuality

TSTBY2A_US = 130
ACK_TIMEOUT_US = 250


class LossModel(Protocol):
    def lost(self, rng: random.Random) -> bool: ...

    def describe(self) -> str: ...


@dataclass
class NoLoss:
    def lost(self, rng: random.Random) -> bool:
        return False

    def describe(self) -> str:
        return "no loss"


@dataclass
class Bernoulli:
    """Each packet is lost independently with probability ``p``."""

    p: float

    def lost(self, rng: random.Random) -> bool:
        return rng.random() < self.p

    def describe(self) -> str:
        return f"independent loss {self.p:.0%}"


@dataclass
class GilbertElliott:
    """Two-state burst-loss channel advanced once per packet.

    Parametrised by the long-run loss rate and the mean burst length (packets).
    The bad state loses ``loss_bad`` of its packets, the good state
    ``loss_good``; with the defaults (1.0 / 0.0) this is the classic Gilbert
    model, where ``mean_loss`` is the stationary probability of the bad state.
    """

    mean_loss: float
    mean_burst: float = 10.0
    loss_good: float = 0.0
    loss_bad: float = 1.0
    bad: bool = False

    def __post_init__(self) -> None:
        if not 0.0 <= self.mean_loss < 1.0 or self.mean_burst < 1.0:
            raise ValueError("need 0 <= mean_loss < 1 and mean_burst >= 1")
        span = self.loss_bad - self.loss_good
        pi_bad = 0.0 if span <= 0 else (self.mean_loss - self.loss_good) / span
        pi_bad = min(max(pi_bad, 0.0), 0.999)
        self.p_bg = 1.0 / self.mean_burst
        self.p_gb = 0.0 if pi_bad == 0 else min(1.0, pi_bad * self.p_bg / (1.0 - pi_bad))

    def lost(self, rng: random.Random) -> bool:
        if self.bad:
            if rng.random() < self.p_bg:
                self.bad = False
        elif rng.random() < self.p_gb:
            self.bad = True
        return rng.random() < (self.loss_bad if self.bad else self.loss_good)

    def describe(self) -> str:
        return f"burst loss {self.mean_loss:.0%} (mean burst {self.mean_burst:g} packets)"


@dataclass
class Interference:
    """Extra loss on a set of RF channels (``RF_CH`` numbers)."""

    channels: frozenset[int] = frozenset()
    loss: float = 1.0

    @classmethod
    def wifi(cls, wifi_channel: int, loss: float = 1.0) -> Interference:
        """A 2.4 GHz Wi-Fi channel: centre 2407 + 5 n MHz, about 22 MHz wide."""
        centre = 7 + 5 * wifi_channel  # RF_CH = f - 2400
        return cls(frozenset(range(centre - 11, centre + 12)), loss)


@dataclass
class RadioParams:
    data_rate_bps: int = 1_000_000
    address_width: int = 5
    crc_bytes: int = 2
    ard_us: int = 500
    arc: int = 15
    mcu_overhead_us: int = 150

    def airtime_us(self, payload: int) -> int:
        bits = 8 * (1 + self.address_width + payload + self.crc_bytes) + 9
        return math.ceil(bits * 1_000_000 / self.data_rate_bps)


@dataclass
class SimConfig:
    duration_s: float = 60.0
    seed: int = 1
    radio: RadioParams = field(default_factory=RadioParams)
    loss: LossModel = field(default_factory=NoLoss)
    interference: Interference = field(default_factory=Interference)
    telemetry_period_ms: int = 50
    poll_interval_ms: int = 10
    command_period_ms: int = 503  # not a multiple of the 50 ms hop slot: samples all phases
    hopping: bool = True
    fixed_channel: int = 76
    hop_seed: int = 0x4E524634
    hop_first: int = 2
    hop_last: int = 80
    hop_spacing: int = 2
    hop_count: int = 8
    hop_dwell_ms: int = 50
    hop_lost_slots: int = 4
    gateway_boot_ms: int = 0
    link: LinkConfig = field(default_factory=LinkConfig)
    demo_sensors: bool = True
    drain_s: float = 2.0
    """No new telemetry or commands in the last ``drain_s`` seconds, so messages
    still in flight when the run ends are not counted as lost."""

    @property
    def traffic_end_ms(self) -> int:
        return int((self.duration_s - self.drain_s) * 1000)


@dataclass
class SimResult:
    config: SimConfig
    telemetry_generated: int = 0
    telemetry_delivered: int = 0
    telemetry_bytes: int = 0
    commands_issued: int = 0
    commands_delivered: int = 0
    commands_acked: int = 0
    commands_failed: int = 0
    command_latency_ms: list[float] = field(default_factory=list)
    command_ack_latency_ms: list[float] = field(default_factory=list)
    frames: int = 0
    frames_failed: int = 0
    esb_attempts: int = 0
    resyncs: int = 0
    first_lock_ms: float | None = None
    node_quality_score: int = 0
    duplicates_at_gateway: int = 0
    crc_errors: int = 0

    @property
    def telemetry_delivery(self) -> float:
        g = self.telemetry_generated
        return self.telemetry_delivered / g if g else float("nan")

    @property
    def command_delivery(self) -> float:
        n = self.commands_issued
        return self.commands_delivered / n if n else float("nan")

    @property
    def goodput_bps(self) -> float:
        return 8 * self.telemetry_bytes / (self.config.traffic_end_ms / 1000)

    @property
    def mean_attempts(self) -> float:
        return self.esb_attempts / self.frames if self.frames else float("nan")

    def latency_percentile(self, q: float) -> float:
        return _percentile(self.command_latency_ms, q)


def _percentile(values: Sequence[float], q: float) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    k = (len(s) - 1) * q
    lo = math.floor(k)
    hi = math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


@dataclass
class _AckPayload:
    data: bytes
    sent: bool = False


class _Gateway:
    """PRX side: the loop of ``src/gateway_main.cpp`` plus the chip's ACK payload FIFO."""

    def __init__(self, cfg: SimConfig, hops: HopSequence, result: SimResult) -> None:
        self.cfg = cfg
        self.result = result
        self.follower = HopFollower(hops, cfg.hop_dwell_ms, cfg.hop_lost_slots)
        self.link = LinkEndpoint(cfg.link, deliver=self._deliver, result=self._result)
        self.fifo: deque[_AckPayload] = deque()
        self.last_rx: tuple[int, bytes] | None = None
        self.booted = False
        self.next_command_ms = cfg.gateway_boot_ms + cfg.command_period_ms
        self.command_counter = 0
        self.pending_command_ms: dict[int, int] = {}  # link seq -> scheduled time
        self.issue_ms: dict[int, int] = {}  # command id -> scheduled time
        self.now_us = 0

    @property
    def channel(self) -> int:
        return self.follower.channel if self.cfg.hopping else self.cfg.fixed_channel

    def listening(self) -> bool:
        return self.booted

    def _deliver(self, channel: Channel, data: bytes) -> None:
        if channel == Channel.UNRELIABLE:
            self.result.telemetry_delivered += 1
            self.result.telemetry_bytes += len(data)

    def _result(self, seq: int, delivered: bool) -> None:
        scheduled = self.pending_command_ms.pop(seq, None)
        if delivered:
            self.result.commands_acked += 1
            if scheduled is not None:
                self.result.command_ack_latency_ms.append(self.now_us / 1000 - scheduled)
        else:
            self.result.commands_failed += 1

    def tick(self, now_us: int) -> None:
        self.now_us = now_us
        now_ms = now_us // 1000
        if not self.booted:
            if now_ms < self.cfg.gateway_boot_ms:
                return
            self.booted = True
            self.follower.start(now_ms)
        if self.cfg.hopping:
            self.follower.update(now_ms)
        if (
            now_ms >= self.next_command_ms
            and self.next_command_ms < self.cfg.traffic_end_ms
            and not self.link.reliable_busy
        ):
            scheduled = self.next_command_ms
            self.command_counter += 1
            msg = encode_message(Command(int(CommandOp.IDENTIFY), self.command_counter))
            if self.link.send_reliable(msg, now_ms):
                self.pending_command_ms[self.link.reliable_seq] = scheduled
                self.issue_ms[self.command_counter] = scheduled
                self.result.commands_issued += 1
            self.next_command_ms += self.cfg.command_period_ms
        self.refill(now_ms)

    def refill(self, now_ms: int) -> None:
        while len(self.fifo) < 3:
            frame = self.link.next_frame(now_ms, self.follower.index, 0, False)
            if not frame:
                break
            self.fifo.append(_AckPayload(frame))

    def on_packet(self, pid: int, frame: bytes, now_us: int) -> bytes:
        """A packet that reached the PRX; returns the ACK payload (may be empty)."""
        key = (pid, frame)
        if key == self.last_rx:
            # Retransmission of a packet already received: discarded, ACKed again.
            head = self.fifo[0] if self.fifo else None
            if head is not None:
                head.sent = True
                return head.data
            return b""
        self.last_rx = key
        if self.fifo and self.fifo[0].sent:
            self.fifo.popleft()  # TX_DS on the PRX: a new PID means the PTX got it
        head = self.fifo[0] if self.fifo else None
        ack = b""
        if head is not None:
            head.sent = True
            ack = head.data
        # The MCU handles RX_DR after the ACK has gone.
        self.now_us = now_us
        now_ms = now_us // 1000
        header = self.link.on_frame(frame, now_ms)
        if header is not None and self.cfg.hopping:
            was_locked = self.follower.locked
            self.follower.on_frame(header.hop, header.phase, now_ms)
            if not was_locked and self.result.first_lock_ms is None:
                self.result.first_lock_ms = now_us / 1000
        self.refill(now_ms)
        return ack


class _Node:
    """PTX side: the loop of ``src/node_main.cpp``."""

    def __init__(self, cfg: SimConfig, hops: HopSequence, gateway: _Gateway, result: SimResult):
        self.cfg = cfg
        self.result = result
        self.gateway = gateway
        self.master = HopMaster(hops, cfg.hop_dwell_ms)
        self.link = LinkEndpoint(cfg.link, deliver=self._deliver)
        self.quality = LinkQuality()
        self.rng = random.Random(cfg.seed)
        self.pid = 0
        self.last_telemetry = 0
        self.last_tx = -(10**9)
        self.rotation = 0
        self.now_us = 0
        self.seen_commands: set[int] = set()

    @property
    def channel(self) -> int:
        return self.master.channel if self.cfg.hopping else self.cfg.fixed_channel

    def _deliver(self, channel: Channel, data: bytes) -> None:
        if channel != Channel.RELIABLE:
            return
        try:
            msg = decode_message(data)
        except MessageError:
            return
        if isinstance(msg, Command) and msg.arg not in self.seen_commands:
            self.seen_commands.add(msg.arg)
            self.result.commands_delivered += 1
            scheduled = self.gateway.issue_ms.get(msg.arg)
            if scheduled is not None:
                self.result.command_latency_ms.append(self.now_us / 1000 - scheduled)

    def _telemetry(self, now_ms: int) -> bytes | None:
        r = self.rotation
        self.rotation = (self.rotation + 1) % 30
        if r == 0:
            q = self.quality
            entries = (
                KvEntry("up", KvType.INT, now_ms // 1000),
                KvEntry("q", KvType.INT, q.score()),
                KvEntry("dlv", KvType.INT, q.delivery_percent()),
                KvEntry("arc", KvType.INT, q.mean_retries_x10()),
                KvEntry("tx", KvType.INT, q.total_tx),
                KvEntry("fail", KvType.INT, q.total_failed),
                KvEntry("pa", KvType.INT, 2),
            )
            return encode_message(KeyValue(entries))
        if r % 3 == 1:
            return encode_message(
                Battery(7400, 0, 0, 0, SOC_UNKNOWN, BATTERY_FLAG_NO_CURRENT_SENSOR)
            )
        if not self.cfg.demo_sensors:
            return None
        t = now_ms / 1000
        if r % 3 == 2:
            return encode_message(
                Odometry(
                    now_ms,
                    int(400 * math.sin(0.5 * t)),
                    int(400 * (1 - math.cos(0.5 * t))),
                    int(1000 * math.fmod(0.5 * t, 2 * math.pi)),
                    200,
                    500,
                )
            )
        return encode_message(
            Imu(
                now_ms,
                (int(100 * math.cos(0.5 * t)), int(100 * math.sin(0.5 * t)), 1000),
                (0, 0, 2865),
                2500,
            )
        )

    def lost(self, channel: int) -> bool:
        lost = self.cfg.loss.lost(self.rng)
        jam = self.cfg.interference
        if channel in jam.channels and self.rng.random() < jam.loss:
            lost = True
        return lost

    def transmit_one(self, t: int, now_ms: int, idle_allowed: bool) -> int:
        hop_index = self.master.index if self.cfg.hopping else 0
        phase = self.master.phase(now_ms) if self.cfg.hopping else 0
        frame = self.link.next_frame(now_ms, hop_index, phase, idle_allowed)
        if not frame:
            return t + 20
        self.last_tx = now_ms
        radio = self.cfg.radio
        self.pid = (self.pid + 1) & 3
        self.result.frames += 1
        t += radio.mcu_overhead_us // 2 + TSTBY2A_US
        retransmits = 0
        acked = False
        ack_payload = b""
        channel = self.channel
        while True:
            data_end = t + radio.airtime_us(len(frame))
            self.result.esb_attempts += 1
            self.gateway.tick(data_end)
            received = (
                self.gateway.listening()
                and self.gateway.channel == channel
                and not self.lost(channel)
            )
            if received:
                payload = self.gateway.on_packet(self.pid, frame, data_end)
                if not self.lost(channel):
                    acked = True
                    ack_payload = payload
                    t = data_end + TSTBY2A_US + radio.airtime_us(len(payload))
                    break
            if retransmits >= radio.arc:
                t = data_end + ACK_TIMEOUT_US
                break
            retransmits += 1
            t = data_end + radio.ard_us
        t += radio.mcu_overhead_us - radio.mcu_overhead_us // 2
        self.now_us = t
        if acked:
            self.quality.record_tx(True, retransmits)
            self.link.on_tx_result(True)
            if ack_payload:
                self.link.on_frame(ack_payload, now_ms)
        else:
            self.result.frames_failed += 1
            self.quality.record_tx(False, 15)
            self.link.on_tx_result(False)
        return t

    def run(self, end_us: int) -> None:
        t = 0
        self.master.start(0)
        while t < end_us:
            now_ms = t // 1000
            if self.cfg.hopping:
                self.master.update(now_ms)
            if (
                now_ms - self.last_telemetry >= self.cfg.telemetry_period_ms
                and now_ms < self.cfg.traffic_end_ms
            ):
                self.last_telemetry = now_ms
                msg = self._telemetry(now_ms)
                if msg is not None:
                    self.result.telemetry_generated += 1
                    self.link.send_unreliable(msg)
            self.gateway.tick(t)
            poll_due = now_ms - self.last_tx >= self.cfg.poll_interval_ms
            slot_ok = not self.cfg.hopping or self.master.can_transmit(now_ms)
            if slot_ok and (poll_due or self.link.has_traffic(now_ms)):
                t = self.transmit_one(t, now_ms, poll_due)
            else:
                t = (now_ms + 1) * 1000


def simulate(cfg: SimConfig) -> SimResult:
    """Run one simulation; deterministic for a given ``cfg.seed``."""
    if not 0 <= cfg.drain_s < cfg.duration_s:
        raise ValueError("need 0 <= drain_s < duration_s")
    result = SimResult(cfg)
    hops = HopSequence()
    if cfg.hopping:
        hops.generate(cfg.hop_seed, cfg.hop_first, cfg.hop_last, cfg.hop_spacing, cfg.hop_count)
    gateway = _Gateway(cfg, hops, result)
    node = _Node(cfg, hops, gateway, result)
    node.run(int(cfg.duration_s * 1_000_000))
    result.resyncs = gateway.follower.resyncs
    result.node_quality_score = node.quality.score()
    result.duplicates_at_gateway = gateway.link.stats.duplicates
    result.crc_errors = gateway.link.stats.crc_errors
    if not cfg.hopping and gateway.booted:
        result.first_lock_ms = float(cfg.gateway_boot_ms)
    return result


@dataclass(frozen=True)
class SweepSeries:
    label: str
    losses: tuple[float, ...]
    results: tuple[SimResult, ...]


def sweep(
    losses: Sequence[float],
    *,
    burst: float | None = None,
    arc: int = 15,
    duration_s: float = 30.0,
    seed: int = 1,
    label: str | None = None,
) -> SweepSeries:
    """Delivery vs loss for independent (``burst=None``) or Gilbert-Elliott loss."""
    results = []
    for loss in losses:
        model: LossModel
        if loss <= 0:
            model = NoLoss()
        elif burst is None:
            model = Bernoulli(loss)
        else:
            model = GilbertElliott(loss, burst)
        cfg = SimConfig(duration_s=duration_s, seed=seed, loss=model)
        cfg.radio.arc = arc
        results.append(simulate(cfg))
    if label is None:
        kind = "independent" if burst is None else f"burst {burst:g} pkts"
        label = f"{kind}, ARC={arc}"
    return SweepSeries(label, tuple(losses), tuple(results))


def summary_line(r: SimResult) -> str:
    lat50 = r.latency_percentile(0.5)
    lat95 = r.latency_percentile(0.95)
    return (
        f"telemetry {r.telemetry_delivered}/{r.telemetry_generated} "
        f"({100 * r.telemetry_delivery:.1f}%) goodput {r.goodput_bps / 1000:.2f} kbit/s | "
        f"commands {r.commands_delivered}/{r.commands_issued} "
        f"({100 * r.command_delivery:.1f}%) latency p50 {lat50:.1f} ms p95 {lat95:.1f} ms | "
        f"frames {r.frames} failed {r.frames_failed} attempts/frame {r.mean_attempts:.2f} | "
        f"resyncs {r.resyncs}"
    )


def describe_config(cfg: SimConfig) -> str:
    hop = (
        f"hopping {cfg.hop_count} ch, dwell {cfg.hop_dwell_ms} ms"
        if cfg.hopping
        else f"fixed channel {cfg.fixed_channel}"
    )
    return (
        f"{cfg.loss.describe()}; {cfg.radio.data_rate_bps // 1000} kbit/s, "
        f"ARD {cfg.radio.ard_us} us, ARC {cfg.radio.arc}; telemetry every "
        f"{cfg.telemetry_period_ms} ms, command every {cfg.command_period_ms} ms; {hop}; "
        f"{cfg.duration_s:g} s simulated, seed {cfg.seed}"
    )


def mean(values: Sequence[float]) -> float:
    return statistics.fmean(values) if values else float("nan")
