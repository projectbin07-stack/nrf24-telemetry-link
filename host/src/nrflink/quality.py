"""Port of ``lib/linkcore/src/linkcore/quality.{h,cpp}`` (link-quality heuristics)."""

from __future__ import annotations


class LinkQuality:
    def __init__(self) -> None:
        self._tx_window = 0
        self.tx_samples = 0
        self._arc_ring = [0] * 16
        self._arc_samples = 0
        self._arc_pos = 0
        self._rpd_window = 0
        self.rpd_samples = 0
        self.total_tx = 0
        self.total_failed = 0
        self.total_retransmits = 0

    def record_tx(self, acked: bool, retransmits: int) -> None:
        retransmits = min(15, retransmits)
        self._tx_window = ((self._tx_window << 1) | int(acked)) & 0xFFFFFFFF
        self.tx_samples = min(32, self.tx_samples + 1)
        self._arc_ring[self._arc_pos] = retransmits
        self._arc_pos = (self._arc_pos + 1) % 16
        self._arc_samples = min(16, self._arc_samples + 1)
        self.total_tx += 1
        if not acked:
            self.total_failed += 1
        self.total_retransmits += retransmits

    def record_rpd(self, above_threshold: bool) -> None:
        self._rpd_window = ((self._rpd_window << 1) | int(above_threshold)) & 0xFFFFFFFF
        self.rpd_samples = min(32, self.rpd_samples + 1)

    @staticmethod
    def _mask(samples: int) -> int:
        return (1 << samples) - 1

    def delivery_percent(self) -> int:
        if self.tx_samples == 0:
            return 100
        ok = bin(self._tx_window & self._mask(self.tx_samples)).count("1")
        return (ok * 100 + self.tx_samples // 2) // self.tx_samples

    def mean_retries_x10(self) -> int:
        if self._arc_samples == 0:
            return 0
        total = sum(self._arc_ring[: self._arc_samples])
        return (total * 10 + self._arc_samples // 2) // self._arc_samples

    def rpd_percent(self) -> int:
        if self.rpd_samples == 0:
            return 0
        hits = bin(self._rpd_window & self._mask(self.rpd_samples)).count("1")
        return (hits * 100 + self.rpd_samples // 2) // self.rpd_samples

    def score(self) -> int:
        return (self.delivery_percent() * (150 - self.mean_retries_x10()) + 75) // 150
