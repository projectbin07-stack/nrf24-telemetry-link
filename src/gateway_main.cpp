// Gateway / base station (ESB primary receiver).
//
// Receives frames from the node on pipe 1, reassembles messages and writes
// them to USB serial as COBS + CRC-16 packets (lib/linkcore/src/linkcore/
// serial_frame.h, docs/protocol.md). Commands from the host are sent to the
// node on the reliable channel, carried in ACK payloads. Follows the node's
// hop sequence (HopFollower) and re-synchronises when it loses it.
#include <Arduino.h>
#include <SPI.h>

#include <board_config.h>
#include <linkcore/hopping.h>
#include <linkcore/link.h>
#include <linkcore/nrf24.h>
#include <linkcore/quality.h>
#include <linkcore/serial_frame.h>

#include "arduino_hal.h"
#include "radio_setup.h"

using namespace linkcore;

namespace {

constexpr uint8_t kPipe = 1;

ArduinoSpi g_spi(NTL_SPI_HZ);
ArduinoPin g_csn(NTL_PIN_CSN);
ArduinoPin g_ce(NTL_PIN_CE);
ArduinoClock g_clock;
Nrf24 g_radio(g_spi, g_csn, g_ce, g_clock);
HopSequence g_hops;
HopFollower g_follower(g_hops, NTL_HOP_DWELL_MS, NTL_HOP_LOST_SLOTS);
LinkEndpoint g_link(make_link_config());
LinkQuality g_quality;
SerialFrameDecoder g_decoder;

uint8_t g_serial_seq = 0;
uint8_t g_rx_pipe = 0;
bool g_rx_rpd = false;
uint32_t g_last_stats = 0;

void emit(SerialKind kind, const uint8_t* body, size_t length) {
  uint8_t wire[kSerialMaxWire];
  const size_t n = encode_serial_wire(kind, g_serial_seq, body, length, wire, sizeof(wire));
  if (n > 0) {
    ++g_serial_seq;
    Serial.write(wire, n);
  }
}

void log_text(const char* text) {
  emit(SerialKind::kLog, reinterpret_cast<const uint8_t*>(text), strlen(text));
}

void on_deliver(void*, Channel channel, const uint8_t* data, size_t length) {
  uint8_t body[3 + kMaxMessage];
  if (length > kMaxMessage) {
    return;
  }
  body[0] = static_cast<uint8_t>(channel);
  body[1] = g_rx_pipe;
  body[2] = g_rx_rpd ? kMessageFlagRpd : 0;
  memcpy(body + 3, data, length);
  emit(SerialKind::kMessage, body, 3 + length);
}

void on_result(void*, uint8_t seq, bool delivered) {
  const uint8_t body[2] = {seq, static_cast<uint8_t>(delivered ? 1 : 0)};
  emit(SerialKind::kCommandResult, body, sizeof(body));
}

void send_stats(uint32_t now) {
  const LinkStats& s = g_link.stats();
  GatewayStats g;
  g.uptime_ms = now;
  g.frames_received = s.frames_received;
  g.malformed = s.malformed;
  g.crc_errors = s.crc_errors;
  g.duplicates = s.duplicates;
  g.delivered_reliable = s.delivered_reliable;
  g.delivered_unreliable = s.delivered_unreliable;
  g.unreliable_lost = s.unreliable_lost;
  g.reliable_sent = s.reliable_sent;
  g.reliable_acked = s.reliable_acked;
  g.reliable_failed = s.reliable_failed;
  g.reliable_retries = s.reliable_retries;
  g.resyncs = g_follower.resyncs();
  g.hop_index = NTL_HOPPING ? g_follower.index() : 0;
  g.hop_locked = NTL_HOPPING ? (g_follower.locked() ? 1 : 0) : 1;
  g.channel = NTL_HOPPING ? g_follower.channel() : NTL_FIXED_CHANNEL;
  g.rpd_percent = g_quality.rpd_percent();
  uint8_t body[kGatewayStatsSize];
  emit(SerialKind::kLinkStats, body, encode_gateway_stats(g, body, sizeof(body)));
}

void retune(uint8_t channel) {
  g_radio.stop_listening();
  g_radio.set_channel(channel);
  g_radio.start_listening();
}

void handle_host_packet(uint32_t now) {
  switch (g_decoder.kind()) {
    case SerialKind::kSendCommand:
      if (!g_link.send_reliable(g_decoder.body(), g_decoder.body_length(), now)) {
        log_text("command rejected: previous command still in flight or bad length");
      }
      break;
    case SerialKind::kRequestStats:
      send_stats(now);
      break;
    default:
      log_text("unknown packet kind from host");
      break;
  }
}

void refill_ack_payloads(uint32_t now) {
  uint8_t frame[kFrameMax];
  while (!g_radio.tx_full()) {
    const size_t n = g_link.next_frame(frame, now, g_follower.index(), 0, false);
    if (n == 0) {
      break;
    }
    g_radio.write_ack_payload(kPipe, frame, static_cast<uint8_t>(n));
  }
}

}  // namespace

void setup() {
  Serial.begin(NTL_SERIAL_BAUD);
  delay(100);  // let the module's supply and the chip settle after power-on
  g_csn.begin(true);
  g_ce.begin(false);
  SPI.begin();
  make_hop_sequence(&g_hops);
  const bool ok = g_radio.begin(make_radio_config(g_hops));
  g_radio.open_reading_pipe(kPipe, kNodeAddress);
  g_radio.start_listening();
  g_link.set_handlers(on_deliver, on_result, nullptr);
  g_link.set_next_sequences(startup_entropy8(NTL_PIN_VBAT), startup_entropy8(NTL_PIN_VBAT));
  const uint32_t now = millis();
  g_follower.start(now);
  if (NTL_HOPPING) {
    retune(g_follower.channel());
  }
  g_last_stats = now;
  log_text(ok ? "gateway ready" : "gateway: nRF24L01+ not responding");
}

void loop() {
  const uint32_t now = millis();
  if (NTL_HOPPING && g_follower.update(now)) {
    retune(g_follower.channel());
  }

  while (Serial.available() > 0) {
    if (g_decoder.feed(static_cast<uint8_t>(Serial.read())) ==
        SerialFrameDecoder::Result::kPacket) {
      handle_host_packet(now);
    }
  }

  uint8_t pipe = 0;
  uint8_t frame[kFrameMax];
  while (g_radio.available(&pipe)) {
    const int len = g_radio.read(frame, sizeof(frame), &pipe);
    if (len <= 0) {
      continue;
    }
    g_rx_pipe = pipe;
    g_rx_rpd = g_radio.received_power_detector();
    g_quality.record_rpd(g_rx_rpd);
    FrameHeader header;
    if (g_link.on_frame(frame, static_cast<size_t>(len), now, &header) && NTL_HOPPING &&
        g_follower.on_frame(header.hop, header.phase, now)) {
      retune(g_follower.channel());
    }
  }

  refill_ack_payloads(now);

  if (now - g_last_stats >= 1000) {
    g_last_stats = now;
    send_stats(now);
  }
}
