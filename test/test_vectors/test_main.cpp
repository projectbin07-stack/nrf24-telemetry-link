// Checks the C++ core against test/vectors/link_vectors.json, the file the
// Python host package generates and its own tests read. The link scenarios
// replay scripted calls on two LinkEndpoints and require byte-identical
// frames, deliveries, result callbacks and statistics, which keeps the link
// simulator's Python port and the firmware's C++ link layer in step.
#include <stdint.h>
#include <stdio.h>
#include <unity.h>

#include <string>
#include <vector>

#include <linkcore/cobs.h>
#include <linkcore/crc16.h>
#include <linkcore/hopping.h>
#include <linkcore/link.h>
#include <linkcore/messages.h>
#include <linkcore/serial_frame.h>

#include "mini_json.h"

using minijson::Value;
using namespace linkcore;

static Value g_vectors;

void setUp() {}
void tearDown() {}

static std::string to_hex(const uint8_t* data, size_t n) {
  static const char* digits = "0123456789abcdef";
  std::string out;
  for (size_t i = 0; i < n; ++i) {
    out.push_back(digits[data[i] >> 4]);
    out.push_back(digits[data[i] & 0x0F]);
  }
  return out;
}

static std::vector<uint8_t> from_hex(const std::string& hex) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<uint8_t>(strtol(hex.substr(i, 2).c_str(), nullptr, 16)));
  }
  return out;
}

static bool load_vectors() {
  std::string here(__FILE__);
  const size_t slash = here.find_last_of("/\\");
  const std::string dir = slash == std::string::npos ? std::string(".") : here.substr(0, slash);
  const std::string candidates[] = {
      dir + "/../vectors/link_vectors.json",
      "test/vectors/link_vectors.json",
      "../test/vectors/link_vectors.json",
  };
  for (const std::string& path : candidates) {
    std::string text;
    if (minijson::read_file(path, &text)) {
      minijson::Parser parser(text);
      if (parser.parse(&g_vectors)) {
        return true;
      }
    }
  }
  return false;
}

static int32_t i32(const Value& v) { return static_cast<int32_t>(v.number); }
static uint32_t u32(const Value& v) { return static_cast<uint32_t>(v.number); }

static void test_vectors_loaded() {
  TEST_ASSERT_TRUE_MESSAGE(load_vectors(), "test/vectors/link_vectors.json not found");
  TEST_ASSERT_TRUE(g_vectors["link_scenarios"].size() >= 4);
}

static void test_crc_and_cobs() {
  const Value& crc = g_vectors["crc16"];
  for (size_t i = 0; i < crc.size(); ++i) {
    const std::string& s = crc.at(i)["ascii"].string;
    const uint16_t c = crc16_ccitt(reinterpret_cast<const uint8_t*>(s.data()), s.size());
    TEST_ASSERT_EQUAL_HEX16(static_cast<uint16_t>(strtol(crc.at(i)["crc"].string.c_str(), nullptr, 16)), c);
  }
  const Value& cobs = g_vectors["cobs"];
  for (size_t i = 0; i < cobs.size(); ++i) {
    const std::vector<uint8_t> raw = from_hex(cobs.at(i)["raw"].string);
    std::vector<uint8_t> enc(cobs_max_encoded_size(raw.size()));
    const size_t n = cobs_encode(raw.data(), raw.size(), enc.data(), enc.size());
    TEST_ASSERT_EQUAL_STRING(cobs.at(i)["encoded"].string.c_str(), to_hex(enc.data(), n).c_str());
    std::vector<uint8_t> dec(raw.size() + 1);
    size_t k = 0;
    TEST_ASSERT_TRUE(cobs_decode(enc.data(), n, dec.data(), dec.size(), &k));
    TEST_ASSERT_EQUAL_UINT(raw.size(), k);
    TEST_ASSERT_EQUAL_STRING(cobs.at(i)["raw"].string.c_str(), to_hex(dec.data(), k).c_str());
  }
}

static size_t encode_kv(const Value& fields, uint8_t* out, size_t cap) {
  KvWriter w(out, cap);
  const Value& entries = fields["entries"];
  for (size_t i = 0; i < entries.size(); ++i) {
    const Value& e = entries.at(i);
    const std::string& key = e["key"].string;
    const std::string& type = e["type"].string;
    if (type == "int") w.add_int(key.c_str(), i32(e["value"]));
    if (type == "float") w.add_float(key.c_str(), static_cast<float>(e["value"].number));
    if (type == "bool") w.add_bool(key.c_str(), e["value"].boolean);
    if (type == "string") w.add_string(key.c_str(), e["value"].string.c_str());
  }
  return w.finish();
}

static void check_kv_decode(const Value& fields, const std::vector<uint8_t>& raw) {
  KvReader r(raw.data(), raw.size());
  TEST_ASSERT_TRUE(r.valid());
  const Value& entries = fields["entries"];
  TEST_ASSERT_EQUAL_UINT8(entries.size(), r.count());
  for (size_t i = 0; i < entries.size(); ++i) {
    const Value& e = entries.at(i);
    KvEntry got;
    TEST_ASSERT_TRUE(r.next(&got));
    TEST_ASSERT_EQUAL_STRING(e["key"].string.c_str(), std::string(got.key, got.key_length).c_str());
    const std::string& type = e["type"].string;
    if (type == "int") {
      TEST_ASSERT_EQUAL(KvType::kInt, got.type);
      TEST_ASSERT_EQUAL_INT32(i32(e["value"]), got.int_value);
    } else if (type == "float") {
      TEST_ASSERT_EQUAL(KvType::kFloat, got.type);
      TEST_ASSERT_TRUE(static_cast<float>(e["value"].number) == got.float_value);
    } else if (type == "bool") {
      TEST_ASSERT_EQUAL(KvType::kBool, got.type);
      TEST_ASSERT_EQUAL(e["value"].boolean, got.bool_value);
    } else {
      TEST_ASSERT_EQUAL(KvType::kString, got.type);
      TEST_ASSERT_EQUAL_STRING(e["value"].string.c_str(),
                               std::string(got.string_value, got.string_length).c_str());
    }
  }
  KvEntry extra;
  TEST_ASSERT_FALSE(r.next(&extra));
}

static void test_messages() {
  const Value& msgs = g_vectors["messages"];
  TEST_ASSERT_TRUE(msgs.size() >= 8);
  for (size_t i = 0; i < msgs.size(); ++i) {
    const Value& c = msgs.at(i);
    const std::string& type = c["type"].string;
    const Value& f = c["fields"];
    const std::vector<uint8_t> expected = from_hex(c["hex"].string);
    uint8_t buf[200];
    size_t n = 0;
    if (type == "battery") {
      Battery m;
      m.bus_mv = static_cast<uint16_t>(f["bus_mv"].number);
      m.current_ua = i32(f["current_ua"]);
      m.power_uw = i32(f["power_uw"]);
      m.charge_uah = i32(f["charge_uah"]);
      m.soc_permille = static_cast<uint16_t>(f["soc_permille"].number);
      m.flags = static_cast<uint8_t>(f["flags"].number);
      n = encode_battery(m, buf, sizeof(buf));
      Battery d;
      TEST_ASSERT_TRUE(decode_battery(expected.data(), expected.size(), &d));
      TEST_ASSERT_EQUAL_UINT16(m.bus_mv, d.bus_mv);
      TEST_ASSERT_EQUAL_INT32(m.current_ua, d.current_ua);
      TEST_ASSERT_EQUAL_INT32(m.power_uw, d.power_uw);
      TEST_ASSERT_EQUAL_INT32(m.charge_uah, d.charge_uah);
      TEST_ASSERT_EQUAL_UINT16(m.soc_permille, d.soc_permille);
      TEST_ASSERT_EQUAL_UINT8(m.flags, d.flags);
    } else if (type == "odometry") {
      Odometry m;
      m.t_ms = u32(f["t_ms"]);
      m.x_mm = i32(f["x_mm"]);
      m.y_mm = i32(f["y_mm"]);
      m.theta_mrad = static_cast<int16_t>(f["theta_mrad"].number);
      m.v_mm_s = static_cast<int16_t>(f["v_mm_s"].number);
      m.w_mrad_s = static_cast<int16_t>(f["w_mrad_s"].number);
      n = encode_odometry(m, buf, sizeof(buf));
      Odometry d;
      TEST_ASSERT_TRUE(decode_odometry(expected.data(), expected.size(), &d));
      TEST_ASSERT_EQUAL_UINT32(m.t_ms, d.t_ms);
      TEST_ASSERT_EQUAL_INT32(m.x_mm, d.x_mm);
      TEST_ASSERT_EQUAL_INT32(m.y_mm, d.y_mm);
      TEST_ASSERT_EQUAL_INT16(m.theta_mrad, d.theta_mrad);
      TEST_ASSERT_EQUAL_INT16(m.v_mm_s, d.v_mm_s);
      TEST_ASSERT_EQUAL_INT16(m.w_mrad_s, d.w_mrad_s);
    } else if (type == "imu") {
      Imu m;
      m.t_ms = u32(f["t_ms"]);
      for (size_t k = 0; k < 3; ++k) {
        m.accel_mg[k] = static_cast<int16_t>(f["accel_mg"].at(k).number);
        m.gyro_cdps[k] = static_cast<int16_t>(f["gyro_cdps"].at(k).number);
      }
      m.temp_cc = static_cast<int16_t>(f["temp_cc"].number);
      n = encode_imu(m, buf, sizeof(buf));
      Imu d;
      TEST_ASSERT_TRUE(decode_imu(expected.data(), expected.size(), &d));
      TEST_ASSERT_EQUAL_UINT32(m.t_ms, d.t_ms);
      TEST_ASSERT_EQUAL_INT16_ARRAY(m.accel_mg, d.accel_mg, 3);
      TEST_ASSERT_EQUAL_INT16_ARRAY(m.gyro_cdps, d.gyro_cdps, 3);
      TEST_ASSERT_EQUAL_INT16(m.temp_cc, d.temp_cc);
    } else if (type == "command") {
      Command m;
      m.opcode = static_cast<uint8_t>(f["opcode"].number);
      m.arg = i32(f["arg"]);
      n = encode_command(m, buf, sizeof(buf));
      Command d;
      TEST_ASSERT_TRUE(decode_command(expected.data(), expected.size(), &d));
      TEST_ASSERT_EQUAL_UINT8(m.opcode, d.opcode);
      TEST_ASSERT_EQUAL_INT32(m.arg, d.arg);
    } else if (type == "key_value") {
      n = encode_kv(f, buf, sizeof(buf));
      check_kv_decode(f, expected);
    } else {
      TEST_FAIL_MESSAGE("unknown message type in vectors");
    }
    TEST_ASSERT_EQUAL_STRING_MESSAGE(c["hex"].string.c_str(), to_hex(buf, n).c_str(),
                                     c["name"].string.c_str());
  }
  // Encoders refuse a buffer that is one byte short.
  uint8_t small[kBatterySize - 1];
  TEST_ASSERT_EQUAL_UINT(0, encode_battery(Battery(), small, sizeof(small)));
}

static void test_invalid_messages() {
  const Value& bad = g_vectors["invalid_messages"];
  for (size_t i = 0; i < bad.size(); ++i) {
    const std::vector<uint8_t> raw = from_hex(bad.at(i)["hex"].string);
    const std::string& type = bad.at(i)["type"].string;
    bool accepted = true;
    if (type == "battery") {
      Battery m;
      accepted = decode_battery(raw.data(), raw.size(), &m);
    } else if (type == "odometry") {
      Odometry m;
      accepted = decode_odometry(raw.data(), raw.size(), &m);
    } else if (type == "command") {
      Command m;
      accepted = decode_command(raw.data(), raw.size(), &m);
    } else if (type == "key_value") {
      accepted = KvReader(raw.data(), raw.size()).valid();
    }
    TEST_ASSERT_FALSE_MESSAGE(accepted, bad.at(i)["name"].string.c_str());
  }
}

static void test_serial_frames() {
  const Value& frames = g_vectors["serial_frames"];
  for (size_t i = 0; i < frames.size(); ++i) {
    const Value& c = frames.at(i);
    const std::vector<uint8_t> body = from_hex(c["body"].string);
    uint8_t wire[kSerialMaxWire + 64];
    const size_t n =
        encode_serial_wire(static_cast<SerialKind>(static_cast<uint8_t>(c["kind"].number)),
                           static_cast<uint8_t>(c["seq"].number), body.data(), body.size(), wire,
                           sizeof(wire));
    TEST_ASSERT_EQUAL_STRING(c["wire"].string.c_str(), to_hex(wire, n).c_str());
    SerialFrameDecoder d;
    SerialFrameDecoder::Result r = SerialFrameDecoder::Result::kNone;
    for (size_t k = 0; k < n; ++k) r = d.feed(wire[k]);
    TEST_ASSERT_EQUAL(SerialFrameDecoder::Result::kPacket, r);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(c["seq"].number), d.seq());
    TEST_ASSERT_EQUAL_STRING(c["body"].string.c_str(), to_hex(d.body(), d.body_length()).c_str());
  }
  const Value& gs = g_vectors["gateway_stats"];
  const Value& f = gs["fields"];
  GatewayStats s;
  s.uptime_ms = u32(f["uptime_ms"]);
  s.frames_received = u32(f["frames_received"]);
  s.malformed = u32(f["malformed"]);
  s.crc_errors = u32(f["crc_errors"]);
  s.duplicates = u32(f["duplicates"]);
  s.delivered_reliable = u32(f["delivered_reliable"]);
  s.delivered_unreliable = u32(f["delivered_unreliable"]);
  s.unreliable_lost = u32(f["unreliable_lost"]);
  s.reliable_sent = u32(f["reliable_sent"]);
  s.reliable_acked = u32(f["reliable_acked"]);
  s.reliable_failed = u32(f["reliable_failed"]);
  s.reliable_retries = u32(f["reliable_retries"]);
  s.resyncs = u32(f["resyncs"]);
  s.hop_index = static_cast<uint8_t>(f["hop_index"].number);
  s.hop_locked = static_cast<uint8_t>(f["hop_locked"].number);
  s.channel = static_cast<uint8_t>(f["channel"].number);
  s.rpd_percent = static_cast<uint8_t>(f["rpd_percent"].number);
  uint8_t b[kGatewayStatsSize];
  const size_t n = encode_gateway_stats(s, b, sizeof(b));
  TEST_ASSERT_EQUAL_STRING(gs["hex"].string.c_str(), to_hex(b, n).c_str());
}

static void test_hop_sequences() {
  const Value& seqs = g_vectors["hop_sequences"];
  for (size_t i = 0; i < seqs.size(); ++i) {
    const Value& c = seqs.at(i);
    HopSequence h;
    TEST_ASSERT_TRUE(h.generate(u32(c["seed"]), static_cast<uint8_t>(c["first"].number),
                                static_cast<uint8_t>(c["last"].number),
                                static_cast<uint8_t>(c["spacing"].number),
                                static_cast<uint8_t>(c["count"].number)));
    const Value& ch = c["channels"];
    TEST_ASSERT_EQUAL_UINT8(ch.size(), h.length());
    for (size_t k = 0; k < ch.size(); ++k) {
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ch.at(k).number),
                              h.channel(static_cast<uint8_t>(k)));
    }
  }
}

static void test_link_frames() {
  const Value& frames = g_vectors["link_frames"];
  for (size_t i = 0; i < frames.size(); ++i) {
    const Value& h = frames.at(i)["header"];
    FrameHeader fh;
    fh.type = static_cast<FrameType>(static_cast<uint8_t>(h["type"].number));
    fh.ack_valid = h["ack_valid"].boolean;
    fh.phase = static_cast<uint8_t>(h["phase"].number);
    fh.seq = static_cast<uint8_t>(h["seq"].number);
    fh.frag_index = static_cast<uint8_t>(h["frag_index"].number);
    fh.frag_count = static_cast<uint8_t>(h["frag_count"].number);
    fh.ack = static_cast<uint8_t>(h["ack"].number);
    fh.hop = static_cast<uint8_t>(h["hop"].number);
    uint8_t out[kHeaderSize];
    encode_header(fh, out);
    TEST_ASSERT_EQUAL_STRING(frames.at(i)["hex"].string.c_str(), to_hex(out, kHeaderSize).c_str());
  }
}

// --- scenario replay ---

struct Events {
  std::vector<std::string> delivered;  // "<channel>:<hex>"
  std::vector<std::string> results;    // "<seq>:<0|1>"
};

static void on_deliver(void* ctx, Channel ch, const uint8_t* data, size_t n) {
  static_cast<Events*>(ctx)->delivered.push_back(std::to_string(static_cast<int>(ch)) + ":" +
                                                 to_hex(data, n));
}

static void on_result(void* ctx, uint8_t seq, bool ok) {
  static_cast<Events*>(ctx)->results.push_back(std::to_string(seq) + ":" + (ok ? "1" : "0"));
}

static void check_stats(const LinkStats& s, const Value& e, const char* where) {
  const struct {
    const char* name;
    uint32_t value;
  } fields[] = {
      {"frames_sent", s.frames_sent},
      {"frames_failed", s.frames_failed},
      {"frames_received", s.frames_received},
      {"malformed", s.malformed},
      {"crc_errors", s.crc_errors},
      {"duplicates", s.duplicates},
      {"delivered_reliable", s.delivered_reliable},
      {"delivered_unreliable", s.delivered_unreliable},
      {"unreliable_lost", s.unreliable_lost},
      {"unreliable_replaced", s.unreliable_replaced},
      {"unreliable_aborted", s.unreliable_aborted},
      {"reliable_sent", s.reliable_sent},
      {"reliable_acked", s.reliable_acked},
      {"reliable_failed", s.reliable_failed},
      {"reliable_retries", s.reliable_retries},
  };
  TEST_ASSERT_EQUAL_UINT_MESSAGE(sizeof(fields) / sizeof(fields[0]), e.size(), where);
  for (const auto& f : fields) {
    std::string msg = std::string(where) + " stat " + f.name;
    TEST_ASSERT_TRUE_MESSAGE(e.has(f.name), msg.c_str());
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(u32(e[f.name]), f.value, msg.c_str());
  }
}

static void replay(const Value& scenario) {
  const Value& cfg = scenario["config"];
  LinkConfig config;
  config.reliable_timeout_ms = u32(cfg["reliable_timeout_ms"]);
  config.reliable_max_attempts = static_cast<uint8_t>(cfg["reliable_max_attempts"].number);
  config.initial_reliable_seq = static_cast<uint8_t>(cfg["initial_reliable_seq"].number);
  config.initial_unreliable_seq = static_cast<uint8_t>(cfg["initial_unreliable_seq"].number);
  LinkEndpoint a(config);
  LinkEndpoint b(config);
  Events ev_a;
  Events ev_b;
  a.set_handlers(on_deliver, on_result, &ev_a);
  b.set_handlers(on_deliver, on_result, &ev_b);

  const Value& steps = scenario["steps"];
  for (size_t i = 0; i < steps.size(); ++i) {
    const Value& st = steps.at(i);
    const bool is_a = st["ep"].string == "a";
    LinkEndpoint& ep = is_a ? a : b;
    Events& ev = is_a ? ev_a : ev_b;
    const std::string& op = st["op"].string;
    char where[160];
    snprintf(where, sizeof(where), "%s step %u (%s %s)", scenario["name"].string.c_str(),
             static_cast<unsigned>(i), st["ep"].string.c_str(), op.c_str());
    if (op == "send_reliable") {
      const std::vector<uint8_t> d = from_hex(st["data"].string);
      TEST_ASSERT_EQUAL_MESSAGE(st["ret"].boolean, ep.send_reliable(d.data(), d.size(), u32(st["now"])),
                                where);
    } else if (op == "send_unreliable") {
      const std::vector<uint8_t> d = from_hex(st["data"].string);
      TEST_ASSERT_EQUAL_MESSAGE(st["ret"].boolean, ep.send_unreliable(d.data(), d.size()), where);
    } else if (op == "next_frame") {
      uint8_t f[kFrameMax];
      const size_t n = ep.next_frame(f, u32(st["now"]), static_cast<uint8_t>(st["hop"].number),
                                     static_cast<uint8_t>(st["phase"].number), st["idle"].boolean);
      TEST_ASSERT_EQUAL_STRING_MESSAGE(st["frame"].string.c_str(), to_hex(f, n).c_str(), where);
    } else if (op == "tx_result") {
      ep.on_tx_result(st["acked"].boolean);
    } else if (op == "on_frame") {
      const std::vector<uint8_t> d = from_hex(st["frame"].string);
      TEST_ASSERT_EQUAL_MESSAGE(st["ret"].boolean, ep.on_frame(d.data(), d.size(), u32(st["now"])),
                                where);
    } else if (op == "poll") {
      ep.poll(u32(st["now"]));
    } else if (op == "stats") {
      check_stats(ep.stats(), st["stats"], where);
    } else {
      TEST_FAIL_MESSAGE(where);
    }
    const Value& dl = st["delivered"];
    TEST_ASSERT_EQUAL_UINT_MESSAGE(dl.size(), ev.delivered.size(), where);
    for (size_t k = 0; k < dl.size(); ++k) {
      const std::string want = std::to_string(static_cast<int>(dl.at(k)["channel"].number)) + ":" +
                               dl.at(k)["data"].string;
      TEST_ASSERT_EQUAL_STRING_MESSAGE(want.c_str(), ev.delivered[k].c_str(), where);
    }
    const Value& rs = st["results"];
    TEST_ASSERT_EQUAL_UINT_MESSAGE(rs.size(), ev.results.size(), where);
    for (size_t k = 0; k < rs.size(); ++k) {
      const std::string want = std::to_string(static_cast<int>(rs.at(k)["seq"].number)) + ":" +
                               (rs.at(k)["ok"].boolean ? "1" : "0");
      TEST_ASSERT_EQUAL_STRING_MESSAGE(want.c_str(), ev.results[k].c_str(), where);
    }
    ev.delivered.clear();
    ev.results.clear();
  }
}

static void test_link_scenarios() {
  const Value& scenarios = g_vectors["link_scenarios"];
  for (size_t i = 0; i < scenarios.size(); ++i) {
    replay(scenarios.at(i));
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_vectors_loaded);
  RUN_TEST(test_crc_and_cobs);
  RUN_TEST(test_messages);
  RUN_TEST(test_invalid_messages);
  RUN_TEST(test_serial_frames);
  RUN_TEST(test_hop_sequences);
  RUN_TEST(test_link_frames);
  RUN_TEST(test_link_scenarios);
  return UNITY_END();
}
