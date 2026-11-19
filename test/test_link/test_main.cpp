// Link layer: fragmentation, CRC, reliable / best-effort channels, duplicate
// suppression and behaviour under loss, endpoint to endpoint.
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include <vector>

#include <linkcore/crc16.h>
#include <linkcore/link.h>

using namespace linkcore;

struct Inbox {
  std::vector<std::vector<uint8_t>> reliable;
  std::vector<std::vector<uint8_t>> unreliable;
  std::vector<std::pair<uint8_t, bool>> results;
};

static void deliver(void* ctx, Channel ch, const uint8_t* data, size_t n) {
  Inbox* box = static_cast<Inbox*>(ctx);
  std::vector<uint8_t> v(data, data + n);
  (ch == Channel::kReliable ? box->reliable : box->unreliable).push_back(v);
}

static void result(void* ctx, uint8_t seq, bool ok) {
  static_cast<Inbox*>(ctx)->results.push_back(std::make_pair(seq, ok));
}

static std::vector<uint8_t> pattern(size_t n, uint8_t start = 0) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>(start + i * 7);
  return v;
}

void setUp() {}
void tearDown() {}

static void test_header_round_trip() {
  FrameHeader h;
  h.type = FrameType::kUnreliable;
  h.ack_valid = true;
  h.phase = 9;
  h.seq = 200;
  h.frag_index = 3;
  h.frag_count = 5;
  h.ack = 17;
  h.hop = 6;
  uint8_t buf[6];
  encode_header(h, buf);
  buf[5] = 0xAB;
  TEST_ASSERT_EQUAL_HEX8(0x59, buf[0]);  // type 2, ack_valid, phase 9
  TEST_ASSERT_EQUAL_HEX8(0x34, buf[2]);  // frag 3 of 5
  FrameHeader d;
  TEST_ASSERT_TRUE(decode_header(buf, 6, &d));
  TEST_ASSERT_EQUAL(FrameType::kUnreliable, d.type);
  TEST_ASSERT_TRUE(d.ack_valid);
  TEST_ASSERT_EQUAL_UINT8(9, d.phase);
  TEST_ASSERT_EQUAL_UINT8(200, d.seq);
  TEST_ASSERT_EQUAL_UINT8(3, d.frag_index);
  TEST_ASSERT_EQUAL_UINT8(5, d.frag_count);
  TEST_ASSERT_EQUAL_UINT8(17, d.ack);
  TEST_ASSERT_EQUAL_UINT8(6, d.hop);
}

static void test_header_rejects_malformed() {
  FrameHeader d;
  uint8_t idle[6] = {0x00, 0, 0x00, 0, 0, 0};
  TEST_ASSERT_TRUE(decode_header(idle, 5, &d));
  TEST_ASSERT_FALSE(decode_header(idle, 6, &d));  // idle with payload
  TEST_ASSERT_FALSE(decode_header(idle, 4, &d));  // too short
  uint8_t data[5] = {0x20, 0, 0x00, 0, 0};
  TEST_ASSERT_FALSE(decode_header(data, 5, &d));  // data frame without payload
  uint8_t bad_type[6] = {0x60, 0, 0x00, 0, 0, 1};
  TEST_ASSERT_FALSE(decode_header(bad_type, 6, &d));
  uint8_t bad_frag[6] = {0x20, 0, 0x21, 0, 0, 1};  // index 2 of 2
  TEST_ASSERT_FALSE(decode_header(bad_frag, 6, &d));
  uint8_t big[33] = {0x20};
  TEST_ASSERT_FALSE(decode_header(big, 33, &d));
}

static void test_fragment_count() {
  TEST_ASSERT_EQUAL_UINT8(1, fragment_count(1));
  TEST_ASSERT_EQUAL_UINT8(1, fragment_count(25));  // 25 + 2 CRC = 27
  TEST_ASSERT_EQUAL_UINT8(2, fragment_count(26));
  TEST_ASSERT_EQUAL_UINT8(2, fragment_count(52));
  TEST_ASSERT_EQUAL_UINT8(3, fragment_count(53));
  TEST_ASSERT_EQUAL_UINT8(16, fragment_count(16 * 27 - 2));
}

static void test_message_crc_covers_seq() {
  const uint8_t msg[3] = {1, 2, 3};
  const uint8_t with_seq[4] = {5, 1, 2, 3};
  TEST_ASSERT_EQUAL_HEX16(crc16_ccitt(with_seq, 4), message_crc(5, msg, 3));
  TEST_ASSERT_NOT_EQUAL(message_crc(5, msg, 3), message_crc(6, msg, 3));
}

struct Pair {
  LinkEndpoint a;
  LinkEndpoint b;
  Inbox in_a;
  Inbox in_b;
  explicit Pair(const LinkConfig& c = LinkConfig()) : a(c), b(c) {
    a.set_handlers(deliver, result, &in_a);
    b.set_handlers(deliver, result, &in_b);
  }
};

// Moves one frame from `from` to `to`. Returns its length (0 if none).
static size_t hop_frame(LinkEndpoint& from, LinkEndpoint& to, uint32_t now, bool drop = false,
                        bool idle = false) {
  uint8_t f[kFrameMax];
  const size_t n = from.next_frame(f, now, 0, 0, idle);
  if (n == 0) return 0;
  from.on_tx_result(!drop);
  if (!drop) to.on_frame(f, n, now);
  return n;
}

static void test_reliable_single_fragment() {
  Pair p;
  const std::vector<uint8_t> msg = pattern(10);
  TEST_ASSERT_TRUE(p.a.send_reliable(msg.data(), msg.size(), 0));
  TEST_ASSERT_TRUE(p.a.reliable_busy());
  TEST_ASSERT_FALSE(p.a.send_reliable(msg.data(), msg.size(), 0));  // stop-and-wait
  TEST_ASSERT_EQUAL_UINT(kHeaderSize + 12, hop_frame(p.a, p.b, 1));
  TEST_ASSERT_EQUAL_UINT(1, p.in_b.reliable.size());
  TEST_ASSERT_TRUE(p.in_b.reliable[0] == msg);
  // B owes an ack: it sends an idle frame even without idle_allowed.
  TEST_ASSERT_EQUAL_UINT(kHeaderSize, hop_frame(p.b, p.a, 2));
  TEST_ASSERT_FALSE(p.a.reliable_busy());
  TEST_ASSERT_EQUAL_UINT(1, p.in_a.results.size());
  TEST_ASSERT_TRUE(p.in_a.results[0].second);
  TEST_ASSERT_EQUAL_UINT32(1, p.a.stats().reliable_acked);
  TEST_ASSERT_EQUAL_UINT(0, hop_frame(p.b, p.a, 3));  // nothing more owed
}

static void test_reliable_multi_fragment_with_failed_fragment() {
  Pair p;
  const std::vector<uint8_t> msg = pattern(100);  // 102 bytes with CRC -> 4 fragments
  p.a.send_reliable(msg.data(), msg.size(), 0);
  hop_frame(p.a, p.b, 1);
  hop_frame(p.a, p.b, 2, /*drop=*/true);  // fragment 1 hits MAX_RT
  hop_frame(p.a, p.b, 3);                 // fragment 1 again
  hop_frame(p.a, p.b, 4);
  TEST_ASSERT_EQUAL_UINT(0, p.in_b.reliable.size());
  hop_frame(p.a, p.b, 5);
  TEST_ASSERT_EQUAL_UINT(1, p.in_b.reliable.size());
  TEST_ASSERT_TRUE(p.in_b.reliable[0] == msg);
  TEST_ASSERT_EQUAL_UINT32(1, p.a.stats().frames_failed);
  TEST_ASSERT_EQUAL_UINT32(1, p.a.stats().reliable_retries);
  TEST_ASSERT_EQUAL_UINT32(5, p.a.stats().frames_sent);
}

static void test_ack_lost_retransmits_and_suppresses_duplicate() {
  LinkConfig c;
  c.reliable_timeout_ms = 50;
  Pair p(c);
  const std::vector<uint8_t> msg = pattern(30);  // 2 fragments
  p.a.send_reliable(msg.data(), msg.size(), 0);
  hop_frame(p.a, p.b, 0);
  hop_frame(p.a, p.b, 1);
  TEST_ASSERT_EQUAL_UINT(1, p.in_b.reliable.size());
  hop_frame(p.b, p.a, 2, /*drop=*/true);  // the ack is lost
  TEST_ASSERT_TRUE(p.a.reliable_busy());
  uint8_t f[kFrameMax];
  TEST_ASSERT_EQUAL_UINT(0, p.a.next_frame(f, 40, 0, 0, false));  // not yet timed out
  hop_frame(p.a, p.b, 51);  // retransmission, fragment 0
  hop_frame(p.a, p.b, 52);  // fragment 1
  TEST_ASSERT_EQUAL_UINT(1, p.in_b.reliable.size());  // not delivered twice
  TEST_ASSERT_TRUE(p.b.stats().duplicates >= 1);
  hop_frame(p.b, p.a, 53);  // re-ack
  TEST_ASSERT_FALSE(p.a.reliable_busy());
  TEST_ASSERT_TRUE(p.in_a.results.back().second);
  TEST_ASSERT_EQUAL_UINT32(1, p.a.stats().reliable_retries);
}

static void test_reliable_gives_up() {
  LinkConfig c;
  c.reliable_timeout_ms = 10;
  c.reliable_max_attempts = 3;
  Pair p(c);
  const std::vector<uint8_t> msg = pattern(5);
  p.a.send_reliable(msg.data(), msg.size(), 0);
  uint32_t now = 0;
  for (int i = 0; i < 10 && p.a.reliable_busy(); ++i) {
    hop_frame(p.a, p.b, now);  // delivered, but B's acks never come back
    now += 11;
    p.a.poll(now);
  }
  TEST_ASSERT_FALSE(p.a.reliable_busy());
  TEST_ASSERT_EQUAL_UINT32(1, p.a.stats().reliable_failed);
  TEST_ASSERT_EQUAL_UINT32(2, p.a.stats().reliable_retries);
  TEST_ASSERT_FALSE(p.in_a.results.back().second);
  TEST_ASSERT_EQUAL_UINT(1, p.in_b.reliable.size());
  // The next message uses the next sequence number and is delivered.
  p.a.send_reliable(msg.data(), msg.size(), now);
  hop_frame(p.a, p.b, now);
  TEST_ASSERT_EQUAL_UINT(2, p.in_b.reliable.size());
}

static void test_fragment_failures_count_as_attempts() {
  LinkConfig c;
  c.reliable_max_attempts = 2;
  Pair p(c);
  const std::vector<uint8_t> msg = pattern(5);
  p.a.send_reliable(msg.data(), msg.size(), 0);
  hop_frame(p.a, p.b, 0, true);
  TEST_ASSERT_TRUE(p.a.reliable_busy());
  hop_frame(p.a, p.b, 1, true);
  TEST_ASSERT_FALSE(p.a.reliable_busy());
  TEST_ASSERT_EQUAL_UINT32(1, p.a.stats().reliable_failed);
}

static void test_unreliable_replace_and_gap() {
  Pair p;
  const std::vector<uint8_t> m1 = pattern(8, 1);
  const std::vector<uint8_t> m2 = pattern(8, 2);
  const std::vector<uint8_t> m3 = pattern(8, 3);
  TEST_ASSERT_TRUE(p.a.send_unreliable(m1.data(), m1.size()));
  TEST_ASSERT_TRUE(p.a.send_unreliable(m2.data(), m2.size()));  // replaces m1
  TEST_ASSERT_EQUAL_UINT32(1, p.a.stats().unreliable_replaced);
  hop_frame(p.a, p.b, 0);
  TEST_ASSERT_EQUAL_UINT(1, p.in_b.unreliable.size());
  TEST_ASSERT_TRUE(p.in_b.unreliable[0] == m2);
  p.a.send_unreliable(m1.data(), m1.size());
  hop_frame(p.a, p.b, 1, true);  // lost
  p.a.send_unreliable(m3.data(), m3.size());
  hop_frame(p.a, p.b, 2);
  TEST_ASSERT_EQUAL_UINT(2, p.in_b.unreliable.size());
  // seq 0 replaced (never sent), 1 delivered, 2 lost, 3 delivered.
  TEST_ASSERT_EQUAL_UINT32(1, p.b.stats().unreliable_lost);
  TEST_ASSERT_EQUAL_UINT32(1, p.a.stats().unreliable_aborted);
}

static void test_unreliable_not_cut_in_half() {
  Pair p;
  const std::vector<uint8_t> big = pattern(60);  // 3 fragments
  const std::vector<uint8_t> small = pattern(4);
  p.a.send_unreliable(big.data(), big.size());
  hop_frame(p.a, p.b, 0);
  TEST_ASSERT_FALSE(p.a.send_unreliable(small.data(), small.size()));
  hop_frame(p.a, p.b, 1, true);  // fragment 1 fails: the rest is abandoned
  TEST_ASSERT_FALSE(p.a.unreliable_pending());
  TEST_ASSERT_TRUE(p.a.send_unreliable(small.data(), small.size()));
  hop_frame(p.a, p.b, 2);
  TEST_ASSERT_EQUAL_UINT(1, p.in_b.unreliable.size());
  TEST_ASSERT_TRUE(p.in_b.unreliable[0] == small);
}

static void test_reliable_has_priority_over_unreliable() {
  Pair p;
  const std::vector<uint8_t> t = pattern(4, 9);
  const std::vector<uint8_t> c = pattern(4, 1);
  p.a.send_unreliable(t.data(), t.size());
  p.a.send_reliable(c.data(), c.size(), 0);
  uint8_t f[kFrameMax];
  p.a.next_frame(f, 0, 0, 0, false);
  TEST_ASSERT_EQUAL_HEX8(0x20, f[0] & 0xE0);
  p.a.next_frame(f, 0, 0, 0, false);
  TEST_ASSERT_EQUAL_HEX8(0x40, f[0] & 0xE0);
}

static void test_crc_error_is_rejected() {
  Pair p;
  const std::vector<uint8_t> msg = pattern(12);
  p.a.send_unreliable(msg.data(), msg.size());
  uint8_t f[kFrameMax];
  const size_t n = p.a.next_frame(f, 0, 0, 0, false);
  f[7] ^= 0x01;
  TEST_ASSERT_TRUE(p.b.on_frame(f, n, 0));
  TEST_ASSERT_EQUAL_UINT(0, p.in_b.unreliable.size());
  TEST_ASSERT_EQUAL_UINT32(1, p.b.stats().crc_errors);
}

static void test_duplicate_fragment_and_frame() {
  Pair p;
  const std::vector<uint8_t> msg = pattern(40);  // 2 fragments
  p.a.send_unreliable(msg.data(), msg.size());
  uint8_t f0[kFrameMax];
  uint8_t f1[kFrameMax];
  const size_t n0 = p.a.next_frame(f0, 0, 0, 0, false);
  const size_t n1 = p.a.next_frame(f1, 0, 0, 0, false);
  p.b.on_frame(f0, n0, 0);
  p.b.on_frame(f0, n0, 0);  // duplicate fragment
  p.b.on_frame(f1, n1, 0);
  p.b.on_frame(f1, n1, 0);  // fragment of an already delivered message
  TEST_ASSERT_EQUAL_UINT(1, p.in_b.unreliable.size());
  TEST_ASSERT_EQUAL_UINT32(2, p.b.stats().duplicates);
}

static void test_message_length_limits() {
  Pair p;
  std::vector<uint8_t> max = pattern(kMaxMessage);
  std::vector<uint8_t> over = pattern(kMaxMessage + 1);
  TEST_ASSERT_FALSE(p.a.send_reliable(over.data(), over.size(), 0));
  TEST_ASSERT_FALSE(p.a.send_unreliable(over.data(), over.size()));
  TEST_ASSERT_FALSE(p.a.send_reliable(max.data(), 0, 0));
  TEST_ASSERT_TRUE(p.a.send_reliable(max.data(), max.size(), 0));
  for (uint32_t t = 0; t < 20 && p.in_b.reliable.empty(); ++t) hop_frame(p.a, p.b, t);
  TEST_ASSERT_EQUAL_UINT(1, p.in_b.reliable.size());
  TEST_ASSERT_TRUE(p.in_b.reliable[0] == max);
}

static void test_idle_frames() {
  Pair p;
  uint8_t f[kFrameMax];
  TEST_ASSERT_EQUAL_UINT(0, p.a.next_frame(f, 0, 3, 4, false));
  TEST_ASSERT_EQUAL_UINT(kHeaderSize, p.a.next_frame(f, 0, 3, 4, true));
  TEST_ASSERT_EQUAL_HEX8(0x04, f[0]);  // idle, no ack yet, phase 4
  TEST_ASSERT_EQUAL_HEX8(3, f[4]);     // hop index
  FrameHeader h;
  TEST_ASSERT_TRUE(p.b.on_frame(f, kHeaderSize, 0, &h));
  TEST_ASSERT_EQUAL_UINT8(3, h.hop);
  TEST_ASSERT_EQUAL_UINT8(4, h.phase);
  TEST_ASSERT_FALSE(p.b.on_frame(f, 3, 0));
  TEST_ASSERT_EQUAL_UINT32(1, p.b.stats().malformed);
}

static void test_initial_sequences() {
  Pair p;
  p.a.set_next_sequences(250, 99);
  const std::vector<uint8_t> msg = pattern(3);
  p.a.send_reliable(msg.data(), msg.size(), 0);
  TEST_ASSERT_EQUAL_UINT8(250, p.a.reliable_seq());
  p.a.send_unreliable(msg.data(), msg.size());
  uint8_t f[kFrameMax];
  p.a.next_frame(f, 0, 0, 0, false);
  TEST_ASSERT_EQUAL_UINT8(250, f[1]);
  p.a.next_frame(f, 0, 0, 0, false);
  TEST_ASSERT_EQUAL_UINT8(99, f[1]);
}

// Bidirectional traffic over a lossy channel with a fixed pseudo-random
// pattern: every reliable message is delivered exactly once and in order.
static void test_reliable_under_heavy_loss() {
  LinkConfig c;
  c.reliable_timeout_ms = 20;
  c.reliable_max_attempts = 50;
  Pair p(c);
  uint32_t rng = 12345;
  auto lost = [&rng]() {
    rng = rng * 1103515245u + 12345u;
    return ((rng >> 16) % 100) < 30;  // 30 % loss per frame, each direction
  };
  uint32_t now = 0;
  int sent = 0;
  while (sent < 40 || p.a.reliable_busy()) {
    if (!p.a.reliable_busy() && sent < 40) {
      const std::vector<uint8_t> m = pattern(static_cast<size_t>(5 + (sent * 13) % 90),
                                             static_cast<uint8_t>(sent));
      TEST_ASSERT_TRUE(p.a.send_reliable(m.data(), m.size(), now));
      ++sent;
    }
    hop_frame(p.a, p.b, now, lost(), true);
    hop_frame(p.b, p.a, now, lost(), true);
    now += 2;
    TEST_ASSERT_TRUE(now < 200000);
  }
  TEST_ASSERT_EQUAL_UINT(40, p.in_b.reliable.size());
  for (int i = 0; i < 40; ++i) {
    const std::vector<uint8_t> m =
        pattern(static_cast<size_t>(5 + (i * 13) % 90), static_cast<uint8_t>(i));
    TEST_ASSERT_TRUE(p.in_b.reliable[static_cast<size_t>(i)] == m);
  }
  TEST_ASSERT_EQUAL_UINT32(40, p.a.stats().reliable_acked);
  TEST_ASSERT_EQUAL_UINT32(0, p.a.stats().reliable_failed);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_header_round_trip);
  RUN_TEST(test_header_rejects_malformed);
  RUN_TEST(test_fragment_count);
  RUN_TEST(test_message_crc_covers_seq);
  RUN_TEST(test_reliable_single_fragment);
  RUN_TEST(test_reliable_multi_fragment_with_failed_fragment);
  RUN_TEST(test_ack_lost_retransmits_and_suppresses_duplicate);
  RUN_TEST(test_reliable_gives_up);
  RUN_TEST(test_fragment_failures_count_as_attempts);
  RUN_TEST(test_unreliable_replace_and_gap);
  RUN_TEST(test_unreliable_not_cut_in_half);
  RUN_TEST(test_reliable_has_priority_over_unreliable);
  RUN_TEST(test_crc_error_is_rejected);
  RUN_TEST(test_duplicate_fragment_and_frame);
  RUN_TEST(test_message_length_limits);
  RUN_TEST(test_idle_frames);
  RUN_TEST(test_initial_sequences);
  RUN_TEST(test_reliable_under_heavy_loss);
  return UNITY_END();
}
