#include "linkcore/link.h"

#include <string.h>

#include "linkcore/crc16.h"

namespace linkcore {

void encode_header(const FrameHeader& h, uint8_t* out) {
  out[0] = static_cast<uint8_t>((static_cast<uint8_t>(h.type) << 5) | (h.ack_valid ? 0x10 : 0x00) |
                                (h.phase & 0x0F));
  out[1] = h.seq;
  out[2] = static_cast<uint8_t>(((h.frag_index & 0x0F) << 4) | ((h.frag_count - 1) & 0x0F));
  out[3] = h.ack;
  out[4] = h.hop;
}

bool decode_header(const uint8_t* frame, size_t length, FrameHeader* h) {
  if (length < kHeaderSize || length > kFrameMax) {
    return false;
  }
  const uint8_t type = static_cast<uint8_t>(frame[0] >> 5);
  if (type > static_cast<uint8_t>(FrameType::kUnreliable)) {
    return false;
  }
  FrameHeader out;
  out.type = static_cast<FrameType>(type);
  out.ack_valid = (frame[0] & 0x10) != 0;
  out.phase = static_cast<uint8_t>(frame[0] & 0x0F);
  out.seq = frame[1];
  out.frag_index = static_cast<uint8_t>(frame[2] >> 4);
  out.frag_count = static_cast<uint8_t>((frame[2] & 0x0F) + 1);
  out.ack = frame[3];
  out.hop = frame[4];
  if (out.frag_index >= out.frag_count) {
    return false;
  }
  if (out.type == FrameType::kIdle) {
    if (length != kHeaderSize) {
      return false;
    }
  } else if (length == kHeaderSize) {
    return false;
  }
  *h = out;
  return true;
}

uint8_t fragment_count(size_t length) {
  return static_cast<uint8_t>((length + 2 + kFragmentPayload - 1) / kFragmentPayload);
}

uint16_t message_crc(uint8_t seq, const uint8_t* message, size_t length) {
  return crc16_ccitt(message, length, crc16_ccitt(&seq, 1));
}

// --- Reassembler ---

Reassembler::Result Reassembler::add(uint8_t seq, uint8_t index, uint8_t count,
                                     const uint8_t* data, size_t length) {
  if (count == 0 || count > kMaxFragments || index >= count || length == 0 ||
      length > kFragmentPayload) {
    return Result::kMalformed;
  }
  if (!active_ || seq != seq_ || count != count_) {
    active_ = true;
    seq_ = seq;
    count_ = count;
    mask_ = 0;
    total_ = 0;
  }
  const uint16_t bit = static_cast<uint16_t>(1u << index);
  if (mask_ & bit) {
    return Result::kDuplicateFragment;
  }
  const size_t offset = static_cast<size_t>(index) * kFragmentPayload;
  const bool last = index == count - 1;
  if ((!last && length != kFragmentPayload) || offset + length > kMessageBuffer) {
    active_ = false;
    return Result::kMalformed;
  }
  memcpy(buffer_ + offset, data, length);
  if (last) {
    total_ = offset + length;
  }
  mask_ = static_cast<uint16_t>(mask_ | bit);
  const uint16_t full = static_cast<uint16_t>((1UL << count) - 1UL);
  if (mask_ != full) {
    return Result::kIncomplete;
  }
  active_ = false;
  if (total_ < 3) {
    return Result::kMalformed;
  }
  const size_t message_length = total_ - 2;
  const uint16_t crc = static_cast<uint16_t>(buffer_[message_length] |
                                             (static_cast<uint16_t>(buffer_[message_length + 1]) << 8));
  if (message_crc(seq, buffer_, message_length) != crc) {
    return Result::kCrcError;
  }
  message_length_ = message_length;
  return Result::kComplete;
}

// --- LinkEndpoint ---

LinkEndpoint::LinkEndpoint(const LinkConfig& config)
    : config_(config),
      tx_next_seq_(config.initial_reliable_seq),
      u_next_seq_(config.initial_unreliable_seq) {}

void LinkEndpoint::set_handlers(DeliverFn deliver, ResultFn result, void* context) {
  deliver_ = deliver;
  result_ = result;
  context_ = context;
}

static size_t prepare(uint8_t* buffer, uint8_t seq, const uint8_t* message, size_t length) {
  memcpy(buffer, message, length);
  const uint16_t crc = message_crc(seq, message, length);
  buffer[length] = static_cast<uint8_t>(crc & 0xFF);
  buffer[length + 1] = static_cast<uint8_t>(crc >> 8);
  return length + 2;
}

bool LinkEndpoint::send_reliable(const uint8_t* message, size_t length, uint32_t now_ms) {
  if (tx_state_ != TxState::kIdle || length == 0 || length > kMaxMessage) {
    return false;
  }
  tx_seq_ = tx_next_seq_++;
  tx_total_ = prepare(tx_buffer_, tx_seq_, message, length);
  tx_count_ = fragment_count(length);
  tx_frag_next_ = 0;
  tx_attempts_ = 1;
  tx_sent_at_ = now_ms;
  tx_state_ = TxState::kSending;
  ++stats_.reliable_sent;
  return true;
}

bool LinkEndpoint::send_unreliable(const uint8_t* message, size_t length) {
  if (length == 0 || length > kMaxMessage) {
    return false;
  }
  if (u_pending_) {
    if (u_frag_next_ > 0) {
      return false;
    }
    ++stats_.unreliable_replaced;
  }
  u_seq_ = u_next_seq_++;
  u_total_ = prepare(u_buffer_, u_seq_, message, length);
  u_count_ = fragment_count(length);
  u_frag_next_ = 0;
  u_pending_ = true;
  return true;
}

void LinkEndpoint::finish_reliable(bool delivered) {
  tx_state_ = TxState::kIdle;
  if (delivered) {
    ++stats_.reliable_acked;
  } else {
    ++stats_.reliable_failed;
  }
  if (result_ != nullptr) {
    result_(context_, tx_seq_, delivered);
  }
}

void LinkEndpoint::poll(uint32_t now_ms) {
  if (tx_state_ == TxState::kWaitAck &&
      static_cast<uint32_t>(now_ms - tx_sent_at_) >= config_.reliable_timeout_ms) {
    if (tx_attempts_ >= config_.reliable_max_attempts) {
      finish_reliable(false);
    } else {
      ++tx_attempts_;
      ++stats_.reliable_retries;
      tx_frag_next_ = 0;
      tx_state_ = TxState::kSending;
    }
  }
}

bool LinkEndpoint::has_traffic(uint32_t now_ms) {
  poll(now_ms);
  return tx_state_ == TxState::kSending || u_pending_ || ack_pending_;
}

size_t LinkEndpoint::build(uint8_t* out, FrameType type, uint8_t seq, uint8_t index, uint8_t count,
                           const uint8_t* buffer, size_t total, uint8_t hop, uint8_t phase) {
  FrameHeader h;
  h.type = type;
  h.ack_valid = rx_reliable_have_;
  h.phase = static_cast<uint8_t>(phase & 0x0F);
  h.seq = seq;
  h.frag_index = index;
  h.frag_count = count;
  h.ack = rx_reliable_have_ ? rx_reliable_last_ : 0;
  h.hop = hop;
  encode_header(h, out);
  size_t length = 0;
  if (buffer != nullptr) {
    const size_t offset = static_cast<size_t>(index) * kFragmentPayload;
    length = total - offset;
    if (length > kFragmentPayload) {
      length = kFragmentPayload;
    }
    memcpy(out + kHeaderSize, buffer + offset, length);
  }
  ack_pending_ = false;
  ++stats_.frames_sent;
  last_seq_ = seq;
  last_index_ = index;
  return kHeaderSize + length;
}

size_t LinkEndpoint::next_frame(uint8_t* out, uint32_t now_ms, uint8_t hop, uint8_t phase,
                                bool idle_allowed) {
  poll(now_ms);
  if (tx_state_ == TxState::kSending) {
    const uint8_t index = tx_frag_next_;
    const size_t n = build(out, FrameType::kReliable, tx_seq_, index, tx_count_, tx_buffer_,
                           tx_total_, hop, phase);
    last_kind_ = LastFrame::kReliable;
    ++tx_frag_next_;
    tx_sent_at_ = now_ms;
    if (tx_frag_next_ >= tx_count_) {
      tx_state_ = TxState::kWaitAck;
    }
    return n;
  }
  if (u_pending_) {
    const uint8_t index = u_frag_next_;
    const size_t n = build(out, FrameType::kUnreliable, u_seq_, index, u_count_, u_buffer_,
                           u_total_, hop, phase);
    last_kind_ = LastFrame::kUnreliable;
    ++u_frag_next_;
    if (u_frag_next_ >= u_count_) {
      u_pending_ = false;
    }
    return n;
  }
  if (ack_pending_ || idle_allowed) {
    const size_t n = build(out, FrameType::kIdle, 0, 0, 1, nullptr, 0, hop, phase);
    last_kind_ = LastFrame::kIdle;
    return n;
  }
  last_kind_ = LastFrame::kNone;
  return 0;
}

void LinkEndpoint::on_tx_result(bool acked) {
  const LastFrame kind = last_kind_;
  last_kind_ = LastFrame::kNone;
  if (acked || kind == LastFrame::kNone) {
    return;
  }
  ++stats_.frames_failed;
  if (kind == LastFrame::kReliable) {
    if (tx_state_ != TxState::kIdle && last_seq_ == tx_seq_) {
      if (tx_attempts_ >= config_.reliable_max_attempts) {
        finish_reliable(false);
      } else {
        ++tx_attempts_;
        ++stats_.reliable_retries;
        tx_frag_next_ = last_index_;
        tx_state_ = TxState::kSending;
      }
    }
  } else if (kind == LastFrame::kUnreliable) {
    ++stats_.unreliable_aborted;
    if (u_pending_ && u_seq_ == last_seq_) {
      u_pending_ = false;
    }
  }
}

bool LinkEndpoint::on_frame(const uint8_t* frame, size_t length, uint32_t now_ms,
                            FrameHeader* header) {
  (void)now_ms;
  FrameHeader h;
  if (!decode_header(frame, length, &h)) {
    ++stats_.malformed;
    return false;
  }
  ++stats_.frames_received;
  if (header != nullptr) {
    *header = h;
  }
  if (h.ack_valid && tx_state_ != TxState::kIdle && h.ack == tx_seq_) {
    finish_reliable(true);
  }
  const uint8_t* payload = frame + kHeaderSize;
  const size_t payload_length = length - kHeaderSize;

  if (h.type == FrameType::kReliable) {
    if (rx_reliable_have_ && h.seq == rx_reliable_last_) {
      ++stats_.duplicates;
      ack_pending_ = true;
      return true;
    }
    const Reassembler::Result r =
        rx_reliable_.add(h.seq, h.frag_index, h.frag_count, payload, payload_length);
    if (r == Reassembler::Result::kComplete) {
      rx_reliable_last_ = h.seq;
      rx_reliable_have_ = true;
      ack_pending_ = true;
      ++stats_.delivered_reliable;
      if (deliver_ != nullptr) {
        deliver_(context_, Channel::kReliable, rx_reliable_.message(),
                 rx_reliable_.message_length());
      }
    } else if (r == Reassembler::Result::kDuplicateFragment) {
      ++stats_.duplicates;
    } else if (r == Reassembler::Result::kCrcError) {
      ++stats_.crc_errors;
    } else if (r == Reassembler::Result::kMalformed) {
      ++stats_.malformed;
      return false;
    }
  } else if (h.type == FrameType::kUnreliable) {
    if (rx_unreliable_have_ && h.seq == rx_unreliable_last_) {
      ++stats_.duplicates;
      return true;
    }
    const Reassembler::Result r =
        rx_unreliable_.add(h.seq, h.frag_index, h.frag_count, payload, payload_length);
    if (r == Reassembler::Result::kComplete) {
      if (rx_unreliable_have_) {
        const uint8_t gap = static_cast<uint8_t>(h.seq - rx_unreliable_last_ - 1);
        if (gap < 128) {
          stats_.unreliable_lost += gap;
        }
      }
      rx_unreliable_last_ = h.seq;
      rx_unreliable_have_ = true;
      ++stats_.delivered_unreliable;
      if (deliver_ != nullptr) {
        deliver_(context_, Channel::kUnreliable, rx_unreliable_.message(),
                 rx_unreliable_.message_length());
      }
    } else if (r == Reassembler::Result::kDuplicateFragment) {
      ++stats_.duplicates;
    } else if (r == Reassembler::Result::kCrcError) {
      ++stats_.crc_errors;
    } else if (r == Reassembler::Result::kMalformed) {
      ++stats_.malformed;
      return false;
    }
  }
  return true;
}

}  // namespace linkcore
