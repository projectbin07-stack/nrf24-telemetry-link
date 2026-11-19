// Link layer on top of Enhanced ShockBurst.
//
// ESB already gives per-packet acknowledgement and retransmission between
// two radios, but it cannot carry messages longer than 32 bytes, it reports
// MAX_RT on the transmitter even when the receiver got the packet and only
// the ACK was lost, and an ACK payload is considered delivered by the PRX as
// soon as the next packet with a new PID arrives, whether or not the PTX
// actually received it (product specification sections 7.4.1, 7.8.6). This
// layer adds, end to end:
//
//   * fragmentation / reassembly of messages up to kMaxMessage bytes;
//   * CRC-16/CCITT-FALSE over the sequence number and the reassembled message;
//   * a stop-and-wait reliable channel (commands) with cumulative ack,
//     timeout, retries and duplicate suppression;
//   * a best-effort channel (telemetry) with newest-value-wins semantics,
//     duplicate suppression and loss counting from sequence gaps;
//   * room in every frame for the channel-hopping index and slot phase.
//
// Frame (one ESB payload, at most 32 bytes):
//
//   byte 0  type[7:5] | ack_valid[4] | phase[3:0]
//   byte 1  seq          message sequence number (per channel)
//   byte 2  frag[7:4] | (count - 1)[3:0]
//   byte 3  ack          last reliable seq delivered by the sender of this frame
//   byte 4  hop          hop index the sender is on
//   5..31   fragment     up to 27 bytes of (message || crc16 LE)
//
// docs/protocol.md has the full description. host/src/nrflink/link.py is a
// line-by-line port used by the link simulator; test/vectors/link_vectors.json
// holds scripted scenarios both implementations must reproduce byte for byte.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifndef LINKCORE_MAX_MESSAGE
#define LINKCORE_MAX_MESSAGE 160
#endif

namespace linkcore {

constexpr size_t kFrameMax = 32;
constexpr size_t kHeaderSize = 5;
constexpr size_t kFragmentPayload = kFrameMax - kHeaderSize;  // 27
constexpr uint8_t kMaxFragments = 16;
constexpr size_t kMaxMessage = LINKCORE_MAX_MESSAGE;
constexpr size_t kMessageBuffer = kMaxMessage + 2;  // + CRC-16
static_assert(kMaxMessage >= 1, "LINKCORE_MAX_MESSAGE must be positive");
static_assert(kMessageBuffer <= kMaxFragments * kFragmentPayload,
              "LINKCORE_MAX_MESSAGE too large for 16 fragments");

enum class FrameType : uint8_t { kIdle = 0, kReliable = 1, kUnreliable = 2 };
enum class Channel : uint8_t { kReliable = 0, kUnreliable = 1 };

struct FrameHeader {
  FrameType type = FrameType::kIdle;
  bool ack_valid = false;
  uint8_t phase = 0;  // 0..15
  uint8_t seq = 0;
  uint8_t frag_index = 0;
  uint8_t frag_count = 1;  // 1..16
  uint8_t ack = 0;
  uint8_t hop = 0;
};

void encode_header(const FrameHeader& header, uint8_t* out);
/// False if `length` is shorter than a header, the type is unknown, the
/// fragment index is not below the count, or an idle frame carries payload.
bool decode_header(const uint8_t* frame, size_t length, FrameHeader* header);

/// Number of fragments for a message of `length` application bytes.
uint8_t fragment_count(size_t length);

/// CRC-16/CCITT-FALSE over [seq, message...].
uint16_t message_crc(uint8_t seq, const uint8_t* message, size_t length);

class Reassembler {
 public:
  enum class Result : uint8_t {
    kIncomplete = 0,
    kComplete = 1,
    kDuplicateFragment = 2,
    kCrcError = 3,
    kMalformed = 4,
  };

  void reset() { active_ = false; }
  bool active() const { return active_; }
  uint8_t seq() const { return seq_; }
  /// Adds one fragment. A fragment of a different (seq, count) than the one
  /// in progress abandons the old message and starts a new one.
  Result add(uint8_t seq, uint8_t index, uint8_t count, const uint8_t* data, size_t length);
  const uint8_t* message() const { return buffer_; }
  size_t message_length() const { return message_length_; }

 private:
  uint8_t buffer_[kMessageBuffer] = {};
  size_t total_ = 0;
  size_t message_length_ = 0;
  uint16_t mask_ = 0;
  uint8_t seq_ = 0;
  uint8_t count_ = 0;
  bool active_ = false;
};

struct LinkStats {
  uint32_t frames_sent = 0;        // frames handed to the radio by next_frame()
  uint32_t frames_failed = 0;      // on_tx_result(false)
  uint32_t frames_received = 0;    // valid frames passed to on_frame()
  uint32_t malformed = 0;          // frames rejected by decode_header / reassembly
  uint32_t crc_errors = 0;         // reassembled messages with a bad CRC
  uint32_t duplicates = 0;         // frames of an already delivered message
  uint32_t delivered_reliable = 0;
  uint32_t delivered_unreliable = 0;
  uint32_t unreliable_lost = 0;    // gaps in the best-effort sequence
  uint32_t unreliable_replaced = 0;  // queued telemetry overwritten before it went out
  uint32_t unreliable_aborted = 0;   // telemetry abandoned after a fragment failed
  uint32_t reliable_sent = 0;      // reliable messages accepted by send_reliable()
  uint32_t reliable_acked = 0;
  uint32_t reliable_failed = 0;    // gave up after max_attempts
  uint32_t reliable_retries = 0;   // whole-message retransmissions after a timeout
};

struct LinkConfig {
  uint32_t reliable_timeout_ms = 100;  // from last fragment sent to retransmission
  uint8_t reliable_max_attempts = 8;
  uint8_t initial_reliable_seq = 0;
  uint8_t initial_unreliable_seq = 0;
};

class LinkEndpoint {
 public:
  using DeliverFn = void (*)(void* context, Channel channel, const uint8_t* data, size_t length);
  using ResultFn = void (*)(void* context, uint8_t seq, bool delivered);

  explicit LinkEndpoint(const LinkConfig& config = LinkConfig());

  void set_handlers(DeliverFn deliver, ResultFn result, void* context);

  /// Next sequence numbers to use. Firmware calls this once at start-up with
  /// random values so a rebooted sender does not reuse the sequence number the
  /// peer delivered last (which the peer would drop as a duplicate).
  void set_next_sequences(uint8_t reliable, uint8_t unreliable) {
    tx_next_seq_ = reliable;
    u_next_seq_ = unreliable;
  }

  /// Starts a reliable message. False if one is still outstanding or the
  /// length is 0 or above kMaxMessage.
  bool send_reliable(const uint8_t* message, size_t length, uint32_t now_ms);
  bool reliable_busy() const { return tx_state_ != TxState::kIdle; }
  uint8_t reliable_seq() const { return tx_seq_; }

  /// Queues a best-effort message, replacing one that has not started to go
  /// out yet. False if the length is 0 or above kMaxMessage, or if a previous
  /// message is part way through its fragments (it is never cut in half).
  bool send_unreliable(const uint8_t* message, size_t length);
  bool unreliable_pending() const { return u_pending_; }

  /// True if next_frame() would return data or an ack without idle_allowed.
  bool has_traffic(uint32_t now_ms);

  /// Builds the next frame into `out` (kFrameMax bytes). Priority: reliable
  /// fragments, best-effort fragments, then an idle frame if an ack is owed
  /// or `idle_allowed`. Returns 0 if there is nothing to send.
  size_t next_frame(uint8_t* out, uint32_t now_ms, uint8_t hop, uint8_t phase, bool idle_allowed);

  /// Result of the last frame returned by next_frame() when the radio knows
  /// it (PTX: TX_DS or MAX_RT). A failed reliable fragment is resent next; a
  /// failed best-effort fragment abandons that message.
  void on_tx_result(bool acked);

  /// Processes a received frame. Returns false if it was rejected; on
  /// success `header` (optional) receives the decoded header.
  bool on_frame(const uint8_t* frame, size_t length, uint32_t now_ms,
                FrameHeader* header = nullptr);

  /// Runs reliable-channel timeouts (also called by next_frame).
  void poll(uint32_t now_ms);

  const LinkStats& stats() const { return stats_; }

 private:
  enum class TxState : uint8_t { kIdle = 0, kSending = 1, kWaitAck = 2 };
  enum class LastFrame : uint8_t { kNone = 0, kIdle = 1, kReliable = 2, kUnreliable = 3 };

  size_t build(uint8_t* out, FrameType type, uint8_t seq, uint8_t index, uint8_t count,
               const uint8_t* buffer, size_t total, uint8_t hop, uint8_t phase);
  void finish_reliable(bool delivered);

  LinkConfig config_;
  LinkStats stats_;
  DeliverFn deliver_ = nullptr;
  ResultFn result_ = nullptr;
  void* context_ = nullptr;

  // Reliable sender.
  uint8_t tx_buffer_[kMessageBuffer] = {};
  size_t tx_total_ = 0;
  uint8_t tx_seq_ = 0;
  uint8_t tx_next_seq_ = 0;
  uint8_t tx_count_ = 0;
  uint8_t tx_frag_next_ = 0;
  uint8_t tx_attempts_ = 0;
  uint32_t tx_sent_at_ = 0;
  TxState tx_state_ = TxState::kIdle;

  // Best-effort sender.
  uint8_t u_buffer_[kMessageBuffer] = {};
  size_t u_total_ = 0;
  uint8_t u_seq_ = 0;
  uint8_t u_next_seq_ = 0;
  uint8_t u_count_ = 0;
  uint8_t u_frag_next_ = 0;
  bool u_pending_ = false;

  // Last frame handed out, for on_tx_result().
  LastFrame last_kind_ = LastFrame::kNone;
  uint8_t last_index_ = 0;
  uint8_t last_seq_ = 0;

  // Receivers.
  Reassembler rx_reliable_;
  Reassembler rx_unreliable_;
  uint8_t rx_reliable_last_ = 0;
  bool rx_reliable_have_ = false;
  uint8_t rx_unreliable_last_ = 0;
  bool rx_unreliable_have_ = false;
  bool ack_pending_ = false;
};

}  // namespace linkcore
