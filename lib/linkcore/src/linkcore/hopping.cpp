#include "linkcore/hopping.h"

namespace linkcore {

uint32_t xorshift32(uint32_t* state) {
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

bool HopSequence::generate(uint32_t seed, uint8_t first, uint8_t last, uint8_t spacing,
                           uint8_t count) {
  channels_[0] = first;
  length_ = 1;
  if (spacing == 0 || first > last || last > 125 || count == 0 || count > kMaxLength) {
    return false;
  }
  uint8_t candidates[126];
  uint8_t n = 0;
  for (uint16_t ch = first; ch <= last; ch = static_cast<uint16_t>(ch + spacing)) {
    candidates[n++] = static_cast<uint8_t>(ch);
  }
  if (count > n) {
    return false;
  }
  uint32_t state = seed != 0 ? seed : 0x9E3779B9u;
  for (uint8_t i = static_cast<uint8_t>(n - 1); i > 0; --i) {
    const uint8_t j = static_cast<uint8_t>(xorshift32(&state) % (static_cast<uint32_t>(i) + 1u));
    const uint8_t tmp = candidates[i];
    candidates[i] = candidates[j];
    candidates[j] = tmp;
  }
  for (uint8_t i = 0; i < count; ++i) {
    channels_[i] = candidates[i];
  }
  length_ = count;
  return true;
}

// --- HopMaster ---

HopMaster::HopMaster(const HopSequence& sequence, uint32_t dwell_ms)
    : sequence_(sequence), dwell_ms_(dwell_ms == 0 ? 1 : dwell_ms) {}

void HopMaster::start(uint32_t now_ms) {
  slot_start_ = now_ms;
  index_ = 0;
}

bool HopMaster::update(uint32_t now_ms) {
  bool changed = false;
  while (static_cast<uint32_t>(now_ms - slot_start_) >= dwell_ms_) {
    slot_start_ += dwell_ms_;
    index_ = static_cast<uint8_t>((index_ + 1) % sequence_.length());
    changed = true;
  }
  return changed;
}

uint8_t HopMaster::phase(uint32_t now_ms) const {
  const uint32_t elapsed = now_ms - slot_start_;
  const uint32_t p = elapsed * 16u / dwell_ms_;
  return static_cast<uint8_t>(p > 15 ? 15 : p);
}

// --- HopFollower ---

HopFollower::HopFollower(const HopSequence& sequence, uint32_t dwell_ms, uint8_t lost_slots)
    : sequence_(sequence),
      dwell_ms_(dwell_ms == 0 ? 1 : dwell_ms),
      lost_slots_(lost_slots == 0 ? 1 : lost_slots) {}

void HopFollower::start(uint32_t now_ms) {
  state_ = State::kScanning;
  index_ = 0;
  scan_start_ = now_ms;
  slot_start_ = now_ms;
  missed_ = 0;
  heard_ = false;
}

bool HopFollower::update(uint32_t now_ms) {
  const uint8_t before = index_;
  if (state_ == State::kLocked) {
    while (state_ == State::kLocked && static_cast<uint32_t>(now_ms - slot_start_) >= dwell_ms_) {
      slot_start_ += dwell_ms_;
      index_ = static_cast<uint8_t>((index_ + 1) % sequence_.length());
      if (heard_) {
        missed_ = 0;
      } else if (++missed_ >= lost_slots_) {
        state_ = State::kScanning;
        scan_start_ = now_ms;
        ++resyncs_;
      }
      heard_ = false;
    }
  } else {
    const uint32_t park_ms = (static_cast<uint32_t>(sequence_.length()) + 1u) * dwell_ms_;
    if (static_cast<uint32_t>(now_ms - scan_start_) >= park_ms) {
      index_ = static_cast<uint8_t>((index_ + 1) % sequence_.length());
      scan_start_ = now_ms;
    }
  }
  return sequence_.channel(index_) != sequence_.channel(before);
}

bool HopFollower::on_frame(uint8_t hop, uint8_t phase, uint32_t now_ms) {
  if (hop >= sequence_.length()) {
    return false;
  }
  const uint8_t before = index_;
  index_ = hop;
  // The frame was sent somewhere inside phase bucket `phase`; assume its middle.
  const uint32_t offset = (2u * (phase & 0x0F) + 1u) * dwell_ms_ / 32u;
  slot_start_ = now_ms - offset;
  state_ = State::kLocked;
  missed_ = 0;
  heard_ = true;
  return sequence_.channel(index_) != sequence_.channel(before);
}

}  // namespace linkcore
