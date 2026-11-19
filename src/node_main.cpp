// Telemetry node (ESB primary transmitter).
//
// Sends best-effort telemetry (battery voltage from an ADC divider, optional
// synthetic odometry / IMU samples, periodic link status) and receives
// commands that the gateway queues as ACK payloads. Polls with empty frames
// every NTL_POLL_INTERVAL_MS so commands get through even when there is no
// telemetry to send. Owns the hop timing (HopMaster).
#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include <board_config.h>
#include <linkcore/hopping.h>
#include <linkcore/link.h>
#include <linkcore/messages.h>
#include <linkcore/nrf24.h>
#include <linkcore/quality.h>

#include "arduino_hal.h"
#include "radio_setup.h"

using namespace linkcore;

namespace {

ArduinoSpi g_spi(NTL_SPI_HZ);
ArduinoPin g_csn(NTL_PIN_CSN);
ArduinoPin g_ce(NTL_PIN_CE);
ArduinoClock g_clock;
Nrf24 g_radio(g_spi, g_csn, g_ce, g_clock);
HopSequence g_hops;
HopMaster g_hop(g_hops, NTL_HOP_DWELL_MS);
LinkEndpoint g_link(make_link_config());
LinkQuality g_quality;

bool g_radio_ok = false;
uint32_t g_period_ms = NTL_TELEMETRY_PERIOD_MS;
uint32_t g_last_telemetry = 0;
uint32_t g_last_tx = 0;
uint32_t g_last_print = 0;
uint32_t g_identify_until = 0;
uint8_t g_rotation = 0;
uint8_t g_pa_level = NTL_PA_LEVEL;
bool g_status_requested = false;
uint32_t g_commands = 0;

constexpr uint32_t kTxTimeoutUs = 25000;  // > 16 x (ARD + air time) at 1 Mbps

void on_deliver(void*, Channel channel, const uint8_t* data, size_t length) {
  Command cmd;
  if (channel != Channel::kReliable || !decode_command(data, length, &cmd)) {
    return;
  }
  ++g_commands;
  switch (static_cast<CommandOp>(cmd.opcode)) {
    case CommandOp::kSetTelemetryPeriodMs:
      if (cmd.arg >= 20 && cmd.arg <= 60000) {
        g_period_ms = static_cast<uint32_t>(cmd.arg);
      }
      break;
    case CommandOp::kSetPaLevel:
      if (cmd.arg >= 0 && cmd.arg <= 3) {
        g_pa_level = static_cast<uint8_t>(cmd.arg);
        g_radio.set_pa_level(static_cast<PaLevel>(g_pa_level));
      }
      break;
    case CommandOp::kRequestStatus:
      g_status_requested = true;
      break;
    case CommandOp::kIdentify:
      if (cmd.arg > 0 && cmd.arg <= 60000) {
        g_identify_until = millis() + static_cast<uint32_t>(cmd.arg);
      }
      break;
  }
}

size_t build_battery(uint8_t* out, size_t capacity) {
  Battery b;
  const uint32_t raw = static_cast<uint32_t>(analogRead(NTL_PIN_VBAT));
  const uint32_t mv = raw * NTL_ADC_REF_MV / NTL_ADC_MAX * NTL_VBAT_DIVIDER_X100 / 100u;
  b.bus_mv = static_cast<uint16_t>(mv > 65535u ? 65535u : mv);
  b.soc_permille = 0xFFFF;  // unknown: no coulomb counter on this node
  b.flags = kBatteryFlagNoCurrentSensor;
  return encode_battery(b, out, capacity);
}

#if NTL_DEMO_SENSORS
// Synthetic samples so the link can be exercised without sensors: a robot
// driving a 0.4 m radius circle at 0.2 m/s. Clearly not measurements.
size_t build_demo_odometry(uint8_t* out, size_t capacity, uint32_t now) {
  const float t = static_cast<float>(now) * 0.001f;
  const float w = 0.5f;
  const float r = 0.4f;
  Odometry o;
  o.t_ms = now;
  o.x_mm = static_cast<int32_t>(1000.0f * r * sinf(w * t));
  o.y_mm = static_cast<int32_t>(1000.0f * r * (1.0f - cosf(w * t)));
  o.theta_mrad = static_cast<int16_t>(1000.0f * fmodf(w * t, 6.2831853f));
  o.v_mm_s = 200;
  o.w_mrad_s = 500;
  return encode_odometry(o, out, capacity);
}

size_t build_demo_imu(uint8_t* out, size_t capacity, uint32_t now) {
  const float t = static_cast<float>(now) * 0.001f;
  Imu m;
  m.t_ms = now;
  m.accel_mg[0] = static_cast<int16_t>(100.0f * cosf(0.5f * t));  // centripetal 0.1 g
  m.accel_mg[1] = static_cast<int16_t>(100.0f * sinf(0.5f * t));
  m.accel_mg[2] = 1000;
  m.gyro_cdps[2] = 2865;  // 0.5 rad/s
  m.temp_cc = 2500;
  return encode_imu(m, out, capacity);
}
#endif

size_t build_status(uint8_t* out, size_t capacity, uint32_t now) {
  KvWriter kv(out, capacity);
  kv.add_int("up", static_cast<int32_t>(now / 1000u));
  kv.add_int("q", g_quality.score());
  kv.add_int("dlv", g_quality.delivery_percent());
  kv.add_int("arc", g_quality.mean_retries_x10());
  kv.add_int("tx", static_cast<int32_t>(g_quality.total_tx()));
  kv.add_int("fail", static_cast<int32_t>(g_quality.total_failed()));
  kv.add_int("pa", g_pa_level);
  return kv.finish();
}

void queue_telemetry(uint32_t now) {
  uint8_t msg[kMaxMessage];
  size_t n = 0;
  if (g_status_requested || g_rotation == 0) {
    n = build_status(msg, sizeof(msg), now);
    g_status_requested = false;
  } else if (g_rotation % 3 == 1) {
    n = build_battery(msg, sizeof(msg));
#if NTL_DEMO_SENSORS
  } else if (g_rotation % 3 == 2) {
    n = build_demo_odometry(msg, sizeof(msg), now);
  } else {
    n = build_demo_imu(msg, sizeof(msg), now);
#endif
  }
  g_rotation = static_cast<uint8_t>((g_rotation + 1) % 30);
  if (n > 0) {
    g_link.send_unreliable(msg, n);
  }
}

void transmit_one(uint32_t now, bool idle_allowed) {
  uint8_t frame[kFrameMax];
  const size_t n = g_link.next_frame(frame, now, g_hop.index(), g_hop.phase(now), idle_allowed);
  if (n == 0) {
    return;
  }
  g_last_tx = now;
  if (!g_radio.write(frame, static_cast<uint8_t>(n))) {
    g_radio.flush_tx();
    g_link.on_tx_result(false);
    return;
  }
  const uint32_t start = micros();
  TxResult result = TxResult::kPending;
  while (result == TxResult::kPending && micros() - start < kTxTimeoutUs) {
    result = g_radio.poll_tx();
  }
  if (result == TxResult::kSent) {
    g_quality.record_tx(true, g_radio.observe_tx().retransmits);
    g_link.on_tx_result(true);
    uint8_t ack[kFrameMax];
    while (g_radio.available()) {
      const int len = g_radio.read(ack, sizeof(ack));
      if (len > 0) {
        g_link.on_frame(ack, static_cast<size_t>(len), now);
      }
    }
  } else {
    // MAX_RT (or no answer from the chip): the payload is still in the FIFO.
    g_radio.flush_tx();
    g_quality.record_tx(false, 15);
    g_link.on_tx_result(false);
  }
}

void print_status(uint32_t now) {
  const LinkStats& s = g_link.stats();
  Serial.print(F("node t="));
  Serial.print(now);
  Serial.print(F(" radio="));
  Serial.print(g_radio_ok ? F("ok") : F("MISSING"));
  Serial.print(F(" ch="));
  Serial.print(NTL_HOPPING ? g_hop.channel() : NTL_FIXED_CHANNEL);
  Serial.print(F(" q="));
  Serial.print(g_quality.score());
  Serial.print(F(" dlv%="));
  Serial.print(g_quality.delivery_percent());
  Serial.print(F(" sent="));
  Serial.print(s.frames_sent);
  Serial.print(F(" failed="));
  Serial.print(s.frames_failed);
  Serial.print(F(" cmds="));
  Serial.println(g_commands);
}

}  // namespace

void setup() {
  Serial.begin(NTL_SERIAL_BAUD);
#if NTL_PIN_LED >= 0
  pinMode(NTL_PIN_LED, OUTPUT);
#endif
  delay(100);  // power-on reset (spec Table 14) and supply settling on the module
  g_csn.begin(true);
  g_ce.begin(false);
  SPI.begin();
  make_hop_sequence(&g_hops);
  g_radio_ok = g_radio.begin(make_radio_config(g_hops));
  g_radio.open_writing_pipe(kNodeAddress);
  g_radio.stop_listening();
  g_link.set_handlers(on_deliver, nullptr, nullptr);
  g_link.set_next_sequences(startup_entropy8(NTL_PIN_VBAT), startup_entropy8(NTL_PIN_VBAT));
  const uint32_t now = millis();
  g_hop.start(now);
  g_last_telemetry = now;
  g_last_print = now;
  Serial.println(g_radio_ok ? F("node ready") : F("node: nRF24L01+ not responding"));
}

void loop() {
  const uint32_t now = millis();
  if (NTL_HOPPING && g_hop.update(now)) {
    g_radio.set_channel(g_hop.channel());
  }
  if (now - g_last_telemetry >= g_period_ms) {
    g_last_telemetry = now;
    queue_telemetry(now);
  }
  const bool poll_due = now - g_last_tx >= NTL_POLL_INTERVAL_MS;
  const bool slot_ok = !NTL_HOPPING || g_hop.can_transmit(now);
  if (slot_ok && (poll_due || g_link.has_traffic(now))) {
    transmit_one(now, poll_due);
  }
  if (now - g_last_print >= 1000) {
    g_last_print = now;
    print_status(now);
  }
#if NTL_PIN_LED >= 0
  const bool identify = static_cast<int32_t>(g_identify_until - now) > 0;
  digitalWrite(NTL_PIN_LED, identify && ((now / 100) & 1) ? HIGH : LOW);
#endif
}
