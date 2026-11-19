// Gateway <-> host USB serial framing.
//
//   wire   = COBS(packet) 0x00
//   packet = version:u8 (=1) kind:u8 seq:u8 length:u8 body[length] crc:u16 LE
//
// The CRC is CRC-16/CCITT-FALSE over version..body. This is the same
// layout as the encoder-odometry-firmware protocol, so a serial-scope style
// decoder only needs a new kind table. Kinds:
//
//   gateway -> host
//     0x01 message         channel:u8 pipe:u8 flags:u8 message[..]
//                          channel 0 = reliable, 1 = best effort; flags bit0 = RPD
//     0x02 link_stats      see GatewayStats / encode_gateway_stats (56 bytes)
//     0x03 command_result  link_seq:u8 delivered:u8
//     0x04 log             ASCII text
//   host -> gateway
//     0x81 send_command    message[..], sent to the node on the reliable channel
//     0x82 request_stats   (empty)
//
// `seq` counts packets per direction (mod 256) so the reader can see drops.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "linkcore/cobs.h"

#ifndef LINKCORE_SERIAL_MAX_BODY
#define LINKCORE_SERIAL_MAX_BODY 200
#endif

namespace linkcore {

constexpr uint8_t kSerialVersion = 1;
constexpr size_t kSerialMaxBody = LINKCORE_SERIAL_MAX_BODY;
static_assert(kSerialMaxBody <= 255, "body length is one byte");
constexpr size_t kSerialOverhead = 6;  // version kind seq length + crc16
constexpr size_t kSerialMaxPacket = kSerialMaxBody + kSerialOverhead;
constexpr size_t kSerialMaxWire = cobs_max_encoded_size(kSerialMaxPacket) + 1;

enum class SerialKind : uint8_t {
  kMessage = 0x01,
  kLinkStats = 0x02,
  kCommandResult = 0x03,
  kLog = 0x04,
  kSendCommand = 0x81,
  kRequestStats = 0x82,
};

constexpr uint8_t kMessageFlagRpd = 0x01;

/// Raw packet (no COBS). Returns its size, or 0 if body or capacity is too large/small.
size_t encode_serial_packet(SerialKind kind, uint8_t seq, const uint8_t* body, size_t length,
                            uint8_t* out, size_t capacity);

/// COBS-encoded packet plus the 0x00 delimiter, ready for Serial.write().
size_t encode_serial_wire(SerialKind kind, uint8_t seq, const uint8_t* body, size_t length,
                          uint8_t* out, size_t capacity);

struct GatewayStats {
  uint32_t uptime_ms = 0;
  uint32_t frames_received = 0;
  uint32_t malformed = 0;
  uint32_t crc_errors = 0;
  uint32_t duplicates = 0;
  uint32_t delivered_reliable = 0;
  uint32_t delivered_unreliable = 0;
  uint32_t unreliable_lost = 0;
  uint32_t reliable_sent = 0;
  uint32_t reliable_acked = 0;
  uint32_t reliable_failed = 0;
  uint32_t reliable_retries = 0;
  uint32_t resyncs = 0;
  uint8_t hop_index = 0;
  uint8_t hop_locked = 0;
  uint8_t channel = 0;
  uint8_t rpd_percent = 0;
};

constexpr size_t kGatewayStatsSize = 13 * 4 + 4;

size_t encode_gateway_stats(const GatewayStats& s, uint8_t* out, size_t capacity);
bool decode_gateway_stats(const uint8_t* body, size_t length, GatewayStats* s);

/// Byte-at-a-time decoder for the wire format.
class SerialFrameDecoder {
 public:
  enum class Result : uint8_t { kNone = 0, kPacket = 1, kError = 2 };

  Result feed(uint8_t byte);
  SerialKind kind() const { return static_cast<SerialKind>(packet_[1]); }
  uint8_t seq() const { return packet_[2]; }
  const uint8_t* body() const { return packet_ + 4; }
  size_t body_length() const { return packet_[3]; }

  uint32_t packets() const { return packets_; }
  uint32_t errors() const { return errors_; }

 private:
  Result finish();

  uint8_t wire_[kSerialMaxWire] = {};
  uint8_t packet_[kSerialMaxPacket] = {};
  size_t wire_length_ = 0;
  bool overflow_ = false;
  uint32_t packets_ = 0;
  uint32_t errors_ = 0;
};

}  // namespace linkcore
