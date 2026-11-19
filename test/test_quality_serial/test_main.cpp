// Link-quality metrics, COBS, CRC-16 and the gateway serial framing.
#include <string.h>
#include <unity.h>

#include <vector>

#include <linkcore/cobs.h>
#include <linkcore/crc16.h>
#include <linkcore/quality.h>
#include <linkcore/serial_frame.h>

using namespace linkcore;

void setUp() {}
void tearDown() {}

static void test_quality_empty() {
  LinkQuality q;
  TEST_ASSERT_EQUAL_UINT8(100, q.delivery_percent());
  TEST_ASSERT_EQUAL_UINT8(0, q.mean_retries_x10());
  TEST_ASSERT_EQUAL_UINT8(0, q.rpd_percent());
  TEST_ASSERT_EQUAL_UINT8(100, q.score());
}

static void test_quality_window() {
  LinkQuality q;
  for (int i = 0; i < 4; ++i) q.record_tx(i != 0, 3);  // 3 of 4 acked
  TEST_ASSERT_EQUAL_UINT8(75, q.delivery_percent());
  TEST_ASSERT_EQUAL_UINT8(30, q.mean_retries_x10());
  // 75 * (150 - 30) / 150 = 60
  TEST_ASSERT_EQUAL_UINT8(60, q.score());
  for (int i = 0; i < 40; ++i) q.record_tx(true, 0);  // old samples fall out
  TEST_ASSERT_EQUAL_UINT8(100, q.delivery_percent());
  TEST_ASSERT_EQUAL_UINT8(0, q.mean_retries_x10());
  TEST_ASSERT_EQUAL_UINT32(44, q.total_tx());
  TEST_ASSERT_EQUAL_UINT32(1, q.total_failed());
  TEST_ASSERT_EQUAL_UINT32(12, q.total_retransmits());
  q.record_tx(false, 200);  // clamped to 15
  TEST_ASSERT_EQUAL_UINT32(27, q.total_retransmits());
}

static void test_quality_rpd() {
  LinkQuality q;
  q.record_rpd(true);
  q.record_rpd(false);
  q.record_rpd(true);
  TEST_ASSERT_EQUAL_UINT8(67, q.rpd_percent());
  for (int i = 0; i < 32; ++i) q.record_rpd(false);
  TEST_ASSERT_EQUAL_UINT8(0, q.rpd_percent());
  TEST_ASSERT_EQUAL_UINT8(32, q.rpd_samples());
}

static void test_crc_check_value() {
  const char* s = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt(reinterpret_cast<const uint8_t*>(s), 9));
}

static void test_cobs_round_trip_all_lengths() {
  for (size_t n = 0; n < 600; n += 37) {
    std::vector<uint8_t> in(n);
    for (size_t i = 0; i < n; ++i) in[i] = static_cast<uint8_t>((i * 31) % 7 == 0 ? 0 : i);
    std::vector<uint8_t> enc(cobs_max_encoded_size(n));
    const size_t m = cobs_encode(in.data(), n, enc.data(), enc.size());
    TEST_ASSERT_TRUE(m > 0);
    for (size_t i = 0; i < m; ++i) TEST_ASSERT_NOT_EQUAL(0, enc[i]);
    std::vector<uint8_t> dec(n + 1);
    size_t k = 0;
    TEST_ASSERT_TRUE(cobs_decode(enc.data(), m, dec.data(), dec.size(), &k));
    TEST_ASSERT_EQUAL_UINT(n, k);
    if (n > 0) TEST_ASSERT_EQUAL_UINT8_ARRAY(in.data(), dec.data(), n);
  }
  uint8_t out[4];
  size_t k = 0;
  const uint8_t bad[3] = {0x05, 0x01, 0x02};  // code points past the end
  TEST_ASSERT_FALSE(cobs_decode(bad, 3, out, sizeof(out), &k));
  const uint8_t one[1] = {0x00};
  TEST_ASSERT_EQUAL_UINT(0, cobs_encode(one, 1, out, 1));  // capacity too small
}

static void feed(SerialFrameDecoder& d, const uint8_t* wire, size_t n, int* packets, int* errors) {
  for (size_t i = 0; i < n; ++i) {
    const SerialFrameDecoder::Result r = d.feed(wire[i]);
    if (r == SerialFrameDecoder::Result::kPacket) ++*packets;
    if (r == SerialFrameDecoder::Result::kError) ++*errors;
  }
}

static void test_serial_round_trip_and_errors() {
  const uint8_t body[5] = {0, 1, 0, 2, 3};
  uint8_t wire[kSerialMaxWire];
  const size_t n = encode_serial_wire(SerialKind::kMessage, 42, body, 5, wire, sizeof(wire));
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_EQUAL_HEX8(0x00, wire[n - 1]);
  SerialFrameDecoder d;
  int packets = 0;
  int errors = 0;
  const uint8_t junk[4] = {0x11, 0x22, 0x33, 0x00};
  feed(d, junk, 4, &packets, &errors);  // garbage before the first delimiter
  TEST_ASSERT_EQUAL_INT(1, errors);
  feed(d, wire, n, &packets, &errors);
  TEST_ASSERT_EQUAL_INT(1, packets);
  TEST_ASSERT_EQUAL(SerialKind::kMessage, d.kind());
  TEST_ASSERT_EQUAL_UINT8(42, d.seq());
  TEST_ASSERT_EQUAL_UINT(5, d.body_length());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(body, d.body(), 5);
  // Corrupt one byte: CRC error.
  uint8_t bad[kSerialMaxWire];
  memcpy(bad, wire, n);
  bad[2] ^= 0x40;
  if (bad[2] == 0) bad[2] = 0x7F;
  feed(d, bad, n, &packets, &errors);
  TEST_ASSERT_EQUAL_INT(2, errors);
  // Empty frames between delimiters are ignored.
  const uint8_t zeros[3] = {0, 0, 0};
  feed(d, zeros, 3, &packets, &errors);
  TEST_ASSERT_EQUAL_INT(2, errors);
  // An overlong run without a delimiter is an error, and the decoder recovers.
  for (size_t i = 0; i < kSerialMaxWire + 10; ++i) d.feed(0x55);
  TEST_ASSERT_EQUAL(SerialFrameDecoder::Result::kError, d.feed(0x00));
  feed(d, wire, n, &packets, &errors);
  TEST_ASSERT_EQUAL_INT(2, packets);
}

static void test_serial_limits() {
  uint8_t body[kSerialMaxBody + 1] = {};
  uint8_t out[kSerialMaxWire + 8];
  TEST_ASSERT_EQUAL_UINT(0, encode_serial_packet(SerialKind::kLog, 0, body, kSerialMaxBody + 1,
                                                 out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT(kSerialMaxBody + kSerialOverhead,
                         encode_serial_packet(SerialKind::kLog, 0, body, kSerialMaxBody, out,
                                              sizeof(out)));
  TEST_ASSERT_TRUE(encode_serial_wire(SerialKind::kLog, 0, body, kSerialMaxBody, out,
                                      sizeof(out)) <= kSerialMaxWire);
  TEST_ASSERT_EQUAL_UINT(0, encode_serial_wire(SerialKind::kLog, 0, body, 10, out, 5));
}

static void test_gateway_stats_round_trip() {
  GatewayStats s;
  s.uptime_ms = 123456;
  s.frames_received = 1;
  s.reliable_retries = 0xDEADBEEF;
  s.resyncs = 3;
  s.hop_index = 5;
  s.hop_locked = 1;
  s.channel = 42;
  s.rpd_percent = 90;
  uint8_t b[kGatewayStatsSize];
  TEST_ASSERT_EQUAL_UINT(56, encode_gateway_stats(s, b, sizeof(b)));
  GatewayStats d;
  TEST_ASSERT_TRUE(decode_gateway_stats(b, sizeof(b), &d));
  TEST_ASSERT_EQUAL_UINT32(123456, d.uptime_ms);
  TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, d.reliable_retries);
  TEST_ASSERT_EQUAL_UINT32(3, d.resyncs);
  TEST_ASSERT_EQUAL_UINT8(42, d.channel);
  TEST_ASSERT_EQUAL_UINT8(90, d.rpd_percent);
  TEST_ASSERT_FALSE(decode_gateway_stats(b, 55, &d));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_quality_empty);
  RUN_TEST(test_quality_window);
  RUN_TEST(test_quality_rpd);
  RUN_TEST(test_crc_check_value);
  RUN_TEST(test_cobs_round_trip_all_lengths);
  RUN_TEST(test_serial_round_trip_and_errors);
  RUN_TEST(test_serial_limits);
  RUN_TEST(test_gateway_stats_round_trip);
  return UNITY_END();
}
