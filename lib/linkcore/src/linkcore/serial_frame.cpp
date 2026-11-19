#include "linkcore/serial_frame.h"

#include <string.h>

#include "linkcore/bytes.h"
#include "linkcore/crc16.h"

namespace linkcore {

size_t encode_serial_packet(SerialKind kind, uint8_t seq, const uint8_t* body, size_t length,
                            uint8_t* out, size_t capacity) {
  if (length > kSerialMaxBody || capacity < length + kSerialOverhead) {
    return 0;
  }
  out[0] = kSerialVersion;
  out[1] = static_cast<uint8_t>(kind);
  out[2] = seq;
  out[3] = static_cast<uint8_t>(length);
  if (length > 0) {
    memcpy(out + 4, body, length);
  }
  put_u16(out + 4 + length, crc16_ccitt(out, 4 + length));
  return length + kSerialOverhead;
}

size_t encode_serial_wire(SerialKind kind, uint8_t seq, const uint8_t* body, size_t length,
                          uint8_t* out, size_t capacity) {
  uint8_t packet[kSerialMaxPacket];
  const size_t n = encode_serial_packet(kind, seq, body, length, packet, sizeof(packet));
  if (n == 0 || capacity == 0) {
    return 0;
  }
  const size_t encoded = cobs_encode(packet, n, out, capacity - 1);
  if (encoded == 0) {
    return 0;
  }
  out[encoded] = 0x00;
  return encoded + 1;
}

size_t encode_gateway_stats(const GatewayStats& s, uint8_t* out, size_t capacity) {
  if (capacity < kGatewayStatsSize) {
    return 0;
  }
  const uint32_t words[13] = {s.uptime_ms,        s.frames_received,      s.malformed,
                              s.crc_errors,       s.duplicates,           s.delivered_reliable,
                              s.delivered_unreliable, s.unreliable_lost,  s.reliable_sent,
                              s.reliable_acked,   s.reliable_failed,      s.reliable_retries,
                              s.resyncs};
  for (uint8_t i = 0; i < 13; ++i) {
    put_u32(out + 4 * i, words[i]);
  }
  out[52] = s.hop_index;
  out[53] = s.hop_locked;
  out[54] = s.channel;
  out[55] = s.rpd_percent;
  return kGatewayStatsSize;
}

bool decode_gateway_stats(const uint8_t* b, size_t length, GatewayStats* s) {
  if (length != kGatewayStatsSize) {
    return false;
  }
  uint32_t* words[13] = {&s->uptime_ms,        &s->frames_received,      &s->malformed,
                         &s->crc_errors,       &s->duplicates,           &s->delivered_reliable,
                         &s->delivered_unreliable, &s->unreliable_lost,  &s->reliable_sent,
                         &s->reliable_acked,   &s->reliable_failed,      &s->reliable_retries,
                         &s->resyncs};
  for (uint8_t i = 0; i < 13; ++i) {
    *words[i] = get_u32(b + 4 * i);
  }
  s->hop_index = b[52];
  s->hop_locked = b[53];
  s->channel = b[54];
  s->rpd_percent = b[55];
  return true;
}

SerialFrameDecoder::Result SerialFrameDecoder::feed(uint8_t byte) {
  if (byte != 0x00) {
    if (wire_length_ < sizeof(wire_)) {
      wire_[wire_length_++] = byte;
    } else {
      overflow_ = true;
    }
    return Result::kNone;
  }
  return finish();
}

SerialFrameDecoder::Result SerialFrameDecoder::finish() {
  const size_t n = wire_length_;
  const bool overflow = overflow_;
  wire_length_ = 0;
  overflow_ = false;
  if (n == 0 && !overflow) {
    return Result::kNone;  // back-to-back delimiters are harmless
  }
  size_t length = 0;
  if (overflow || !cobs_decode(wire_, n, packet_, sizeof(packet_), &length) ||
      length < kSerialOverhead || packet_[0] != kSerialVersion ||
      static_cast<size_t>(packet_[3]) + kSerialOverhead != length ||
      crc16_ccitt(packet_, length - 2) != get_u16(packet_ + length - 2)) {
    ++errors_;
    return Result::kError;
  }
  ++packets_;
  return Result::kPacket;
}

}  // namespace linkcore
