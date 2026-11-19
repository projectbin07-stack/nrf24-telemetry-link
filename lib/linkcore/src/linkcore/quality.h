// Link-quality metrics from what the radio can actually observe.
//
// The nRF24L01+ has no RSSI. What it does offer (product specification
// sections 6.4 and 7.4.2, OBSERVE_TX and RPD in Table 28):
//
//   * ARC_CNT: retransmissions the last packet needed (0..15);
//   * MAX_RT: the packet was not acknowledged after ARC retransmissions;
//   * RPD: on a receiver, "received power above -64 dBm" latched for the last
//     packet - a one-bit signal-strength proxy.
//
// LinkQuality keeps the last 32 transmissions in a bit window (delivery
// ratio), a 16-sample ring of ARC_CNT values and the last 32 RPD samples,
// and folds them into a 0..100 score. The score is a heuristic for display
// and for deciding when to raise PA power, not a calibrated measurement.
#pragma once

#include <stdint.h>

namespace linkcore {

class LinkQuality {
 public:
  void record_tx(bool acked, uint8_t retransmits);
  void record_rpd(bool above_threshold);

  uint8_t tx_samples() const { return tx_samples_; }
  /// Percentage of acknowledged packets in the window (100 when empty).
  uint8_t delivery_percent() const;
  /// Mean ARC_CNT over the last 16 transmissions, times 10 (0..150).
  uint8_t mean_retries_x10() const;
  /// Percentage of RPD samples above -64 dBm (0 when empty).
  uint8_t rpd_percent() const;
  uint8_t rpd_samples() const { return rpd_samples_; }
  /// delivery% scaled by (1 - mean_retries / 15). 0..100.
  uint8_t score() const;

  uint32_t total_tx() const { return total_tx_; }
  uint32_t total_failed() const { return total_failed_; }
  uint32_t total_retransmits() const { return total_retransmits_; }

 private:
  static uint8_t popcount32(uint32_t v);

  uint32_t tx_window_ = 0;  // bit set = acknowledged
  uint8_t tx_samples_ = 0;  // 0..32
  uint8_t arc_ring_[16] = {};
  uint8_t arc_samples_ = 0;  // 0..16
  uint8_t arc_pos_ = 0;
  uint32_t rpd_window_ = 0;
  uint8_t rpd_samples_ = 0;
  uint32_t total_tx_ = 0;
  uint32_t total_failed_ = 0;
  uint32_t total_retransmits_ = 0;
};

}  // namespace linkcore
