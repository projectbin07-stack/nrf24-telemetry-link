#include "linkcore/quality.h"

namespace linkcore {

uint8_t LinkQuality::popcount32(uint32_t v) {
  uint8_t n = 0;
  while (v != 0) {
    v &= v - 1u;
    ++n;
  }
  return n;
}

void LinkQuality::record_tx(bool acked, uint8_t retransmits) {
  if (retransmits > 15) {
    retransmits = 15;
  }
  tx_window_ = (tx_window_ << 1) | (acked ? 1u : 0u);
  if (tx_samples_ < 32) {
    ++tx_samples_;
  }
  arc_ring_[arc_pos_] = retransmits;
  arc_pos_ = static_cast<uint8_t>((arc_pos_ + 1) % 16);
  if (arc_samples_ < 16) {
    ++arc_samples_;
  }
  ++total_tx_;
  if (!acked) {
    ++total_failed_;
  }
  total_retransmits_ += retransmits;
}

void LinkQuality::record_rpd(bool above_threshold) {
  rpd_window_ = (rpd_window_ << 1) | (above_threshold ? 1u : 0u);
  if (rpd_samples_ < 32) {
    ++rpd_samples_;
  }
}

static uint32_t window_mask(uint8_t samples) {
  return samples >= 32 ? 0xFFFFFFFFu : ((1UL << samples) - 1UL);
}

uint8_t LinkQuality::delivery_percent() const {
  if (tx_samples_ == 0) {
    return 100;
  }
  const uint8_t ok = popcount32(tx_window_ & window_mask(tx_samples_));
  return static_cast<uint8_t>((static_cast<uint16_t>(ok) * 100u + tx_samples_ / 2u) / tx_samples_);
}

uint8_t LinkQuality::mean_retries_x10() const {
  if (arc_samples_ == 0) {
    return 0;
  }
  uint16_t sum = 0;
  for (uint8_t i = 0; i < arc_samples_; ++i) {
    sum = static_cast<uint16_t>(sum + arc_ring_[i]);
  }
  return static_cast<uint8_t>((sum * 10u + arc_samples_ / 2u) / arc_samples_);
}

uint8_t LinkQuality::rpd_percent() const {
  if (rpd_samples_ == 0) {
    return 0;
  }
  const uint8_t hits = popcount32(rpd_window_ & window_mask(rpd_samples_));
  return static_cast<uint8_t>((static_cast<uint16_t>(hits) * 100u + rpd_samples_ / 2u) /
                              rpd_samples_);
}

uint8_t LinkQuality::score() const {
  const uint16_t delivery = delivery_percent();
  const uint16_t retries = mean_retries_x10();  // 0..150
  return static_cast<uint8_t>((delivery * (150u - retries) + 75u) / 150u);
}

}  // namespace linkcore
