"""Port of ``lib/linkcore/src/linkcore/hopping.{h,cpp}``: hop sequence, master, follower."""

from __future__ import annotations

from enum import IntEnum

MAX_SEQUENCE = 32
MAX_CHANNEL = 125


def xorshift32(state: int) -> int:
    """One step of Marsaglia's xorshift32 (13, 17, 5); returns the new state."""
    x = state & 0xFFFFFFFF
    x ^= (x << 13) & 0xFFFFFFFF
    x ^= x >> 17
    x ^= (x << 5) & 0xFFFFFFFF
    return x


class HopSequence:
    def __init__(self) -> None:
        self.channels: list[int] = [76]

    @classmethod
    def generated(cls, seed: int, first: int, last: int, spacing: int, count: int) -> HopSequence:
        seq = cls()
        if not seq.generate(seed, first, last, spacing, count):
            raise ValueError("invalid hop sequence parameters")
        return seq

    def generate(self, seed: int, first: int, last: int, spacing: int, count: int) -> bool:
        self.channels = [first]
        if spacing == 0 or first > last or last > MAX_CHANNEL or count == 0 or count > MAX_SEQUENCE:
            return False
        candidates = list(range(first, last + 1, spacing))
        if count > len(candidates):
            return False
        state = seed & 0xFFFFFFFF or 0x9E3779B9
        for i in range(len(candidates) - 1, 0, -1):
            state = xorshift32(state)
            j = state % (i + 1)
            candidates[i], candidates[j] = candidates[j], candidates[i]
        self.channels = candidates[:count]
        return True

    @property
    def length(self) -> int:
        return len(self.channels)

    def channel(self, index: int) -> int:
        return self.channels[index % len(self.channels)]


class HopMaster:
    def __init__(self, sequence: HopSequence, dwell_ms: int) -> None:
        self.sequence = sequence
        self.dwell_ms = max(1, dwell_ms)
        self.slot_start = 0
        self.index = 0

    def start(self, now_ms: int) -> None:
        self.slot_start = now_ms
        self.index = 0

    def update(self, now_ms: int) -> bool:
        changed = False
        while now_ms - self.slot_start >= self.dwell_ms:
            self.slot_start += self.dwell_ms
            self.index = (self.index + 1) % self.sequence.length
            changed = True
        return changed

    @property
    def channel(self) -> int:
        return self.sequence.channel(self.index)

    def phase(self, now_ms: int) -> int:
        return min(15, (now_ms - self.slot_start) * 16 // self.dwell_ms)

    def can_transmit(self, now_ms: int) -> bool:
        return 1 <= self.phase(now_ms) <= 14


class FollowerState(IntEnum):
    SCANNING = 0
    LOCKED = 1


class HopFollower:
    def __init__(self, sequence: HopSequence, dwell_ms: int, lost_slots: int) -> None:
        self.sequence = sequence
        self.dwell_ms = max(1, dwell_ms)
        self.lost_slots = max(1, lost_slots)
        self.state = FollowerState.SCANNING
        self.slot_start = 0
        self.scan_start = 0
        self.index = 0
        self.missed = 0
        self.heard = False
        self.resyncs = 0

    @property
    def locked(self) -> bool:
        return self.state == FollowerState.LOCKED

    @property
    def channel(self) -> int:
        return self.sequence.channel(self.index)

    def start(self, now_ms: int) -> None:
        self.state = FollowerState.SCANNING
        self.index = 0
        self.scan_start = now_ms
        self.slot_start = now_ms
        self.missed = 0
        self.heard = False

    def update(self, now_ms: int) -> bool:
        before = self.index
        if self.state == FollowerState.LOCKED:
            while self.state == FollowerState.LOCKED and now_ms - self.slot_start >= self.dwell_ms:
                self.slot_start += self.dwell_ms
                self.index = (self.index + 1) % self.sequence.length
                if self.heard:
                    self.missed = 0
                else:
                    self.missed += 1
                    if self.missed >= self.lost_slots:
                        self.state = FollowerState.SCANNING
                        self.scan_start = now_ms
                        self.resyncs += 1
                self.heard = False
        else:
            park_ms = (self.sequence.length + 1) * self.dwell_ms
            if now_ms - self.scan_start >= park_ms:
                self.index = (self.index + 1) % self.sequence.length
                self.scan_start = now_ms
        return self.sequence.channel(self.index) != self.sequence.channel(before)

    def on_frame(self, hop: int, phase: int, now_ms: int) -> bool:
        if hop >= self.sequence.length:
            return False
        before = self.index
        self.index = hop
        offset = (2 * (phase & 0x0F) + 1) * self.dwell_ms // 32
        self.slot_start = now_ms - offset
        self.state = FollowerState.LOCKED
        self.missed = 0
        self.heard = True
        return self.sequence.channel(self.index) != self.sequence.channel(before)
