// Deterministic channel hopping with resynchronisation.
//
// Both ends derive the same hop sequence from a shared 32-bit seed: the
// candidate channels first, first+spacing, ..., <= last are shuffled with a
// xorshift32-driven Fisher-Yates shuffle and the first `count` are kept.
//
// The node (PTX) is the timing master: it dwells `dwell_ms` on each channel
// and stamps every frame with its hop index and the phase within the slot
// (0..15, in 1/16ths of the dwell time). The gateway (PRX) follows: every
// received frame re-aligns its slot timer. If it hears nothing for
// `lost_slots` consecutive slots it parks on one channel for a whole
// sequence period plus one slot, which the master is guaranteed to visit;
// if that channel stays silent (for example it is jammed) it parks on the
// next index of the sequence, and so on.
#pragma once

#include <stdint.h>

namespace linkcore {

uint32_t xorshift32(uint32_t* state);

class HopSequence {
 public:
  static constexpr uint8_t kMaxLength = 32;

  /// Returns false (and leaves a one-channel sequence on `first`) if the
  /// arguments are invalid: first > last, last > 125, spacing 0, count 0 or
  /// count larger than the number of candidates or kMaxLength.
  bool generate(uint32_t seed, uint8_t first, uint8_t last, uint8_t spacing, uint8_t count);
  uint8_t length() const { return length_; }
  uint8_t channel(uint8_t index) const { return channels_[index % length_]; }

 private:
  uint8_t channels_[kMaxLength] = {76};
  uint8_t length_ = 1;
};

class HopMaster {
 public:
  HopMaster(const HopSequence& sequence, uint32_t dwell_ms);
  void start(uint32_t now_ms);
  /// Advances the slot if its time is up. Returns true if the channel changed.
  bool update(uint32_t now_ms);
  uint8_t index() const { return index_; }
  uint8_t channel() const { return sequence_.channel(index_); }
  /// Elapsed part of the slot in 1/16ths (0..15).
  uint8_t phase(uint32_t now_ms) const;
  /// False in the first and last 1/16 of a slot: guard time that absorbs the
  /// follower's slot-alignment error (at most one phase bucket), so frames
  /// are not sent while the two ends may be on different channels.
  bool can_transmit(uint32_t now_ms) const {
    const uint8_t p = phase(now_ms);
    return p >= 1 && p <= 14;
  }

 private:
  const HopSequence& sequence_;
  uint32_t dwell_ms_;
  uint32_t slot_start_ = 0;
  uint8_t index_ = 0;
};

class HopFollower {
 public:
  enum class State : uint8_t { kScanning = 0, kLocked = 1 };

  HopFollower(const HopSequence& sequence, uint32_t dwell_ms, uint8_t lost_slots);
  /// Starts scanning on index 0.
  void start(uint32_t now_ms);
  /// Returns true if the channel changed.
  bool update(uint32_t now_ms);
  /// Re-aligns to a received frame's hop index and phase. Returns true if
  /// the channel changed. Out-of-range indices are ignored.
  bool on_frame(uint8_t hop, uint8_t phase, uint32_t now_ms);
  State state() const { return state_; }
  bool locked() const { return state_ == State::kLocked; }
  uint8_t index() const { return index_; }
  uint8_t channel() const { return sequence_.channel(index_); }
  uint32_t resyncs() const { return resyncs_; }

 private:
  const HopSequence& sequence_;
  uint32_t dwell_ms_;
  uint8_t lost_slots_;
  State state_ = State::kScanning;
  uint32_t slot_start_ = 0;
  uint32_t scan_start_ = 0;
  uint8_t index_ = 0;
  uint8_t missed_ = 0;
  bool heard_ = false;
  uint32_t resyncs_ = 0;
};

}  // namespace linkcore
