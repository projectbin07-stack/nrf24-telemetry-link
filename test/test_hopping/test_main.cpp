// Channel hopping: sequence generation, master timing, follower lock-in,
// loss of sync and park-and-scan resynchronisation.
#include <unity.h>

#include <linkcore/hopping.h>

using namespace linkcore;

void setUp() {}
void tearDown() {}

static void test_xorshift_reference_values() {
  // Marsaglia's xorshift32 (13, 17, 5) from state 1.
  uint32_t s = 1;
  TEST_ASSERT_EQUAL_HEX32(0x00042021u, xorshift32(&s));
  TEST_ASSERT_EQUAL_HEX32(0x04080601u, xorshift32(&s));
}

static void test_sequence_is_a_permutation_subset() {
  HopSequence seq;
  TEST_ASSERT_TRUE(seq.generate(0x12345678u, 2, 80, 2, 16));
  TEST_ASSERT_EQUAL_UINT8(16, seq.length());
  for (uint8_t i = 0; i < 16; ++i) {
    const uint8_t ch = seq.channel(i);
    TEST_ASSERT_TRUE(ch >= 2 && ch <= 80);
    TEST_ASSERT_EQUAL_UINT8(0, ch % 2);
    for (uint8_t j = 0; j < i; ++j) {
      TEST_ASSERT_NOT_EQUAL(seq.channel(j), ch);
    }
  }
  HopSequence again;
  again.generate(0x12345678u, 2, 80, 2, 16);
  HopSequence other;
  other.generate(0x12345679u, 2, 80, 2, 16);
  bool differs = false;
  for (uint8_t i = 0; i < 16; ++i) {
    TEST_ASSERT_EQUAL_UINT8(seq.channel(i), again.channel(i));
    differs = differs || seq.channel(i) != other.channel(i);
  }
  TEST_ASSERT_TRUE(differs);
  TEST_ASSERT_EQUAL_UINT8(seq.channel(0), seq.channel(16));  // wraps
}

static void test_sequence_rejects_bad_arguments() {
  HopSequence seq;
  TEST_ASSERT_FALSE(seq.generate(1, 10, 5, 1, 2));
  TEST_ASSERT_FALSE(seq.generate(1, 0, 126, 1, 2));
  TEST_ASSERT_FALSE(seq.generate(1, 0, 10, 0, 2));
  TEST_ASSERT_FALSE(seq.generate(1, 0, 10, 5, 4));  // only 0, 5, 10
  TEST_ASSERT_FALSE(seq.generate(1, 0, 125, 1, 33));
  TEST_ASSERT_EQUAL_UINT8(1, seq.length());
  TEST_ASSERT_TRUE(seq.generate(0, 0, 125, 1, 32));  // seed 0 is remapped
  TEST_ASSERT_EQUAL_UINT8(32, seq.length());
}

static void test_master_slots_and_phase() {
  HopSequence seq;
  seq.generate(7, 2, 80, 2, 4);
  HopMaster m(seq, 32);
  m.start(1000);
  TEST_ASSERT_EQUAL_UINT8(0, m.index());
  TEST_ASSERT_EQUAL_UINT8(0, m.phase(1000));
  TEST_ASSERT_EQUAL_UINT8(8, m.phase(1016));
  TEST_ASSERT_EQUAL_UINT8(15, m.phase(1031));
  TEST_ASSERT_FALSE(m.can_transmit(1000));  // guard bucket at the start
  TEST_ASSERT_TRUE(m.can_transmit(1002));
  TEST_ASSERT_TRUE(m.can_transmit(1029));
  TEST_ASSERT_FALSE(m.can_transmit(1031));  // and at the end
  TEST_ASSERT_FALSE(m.update(1031));
  TEST_ASSERT_TRUE(m.update(1032));
  TEST_ASSERT_EQUAL_UINT8(1, m.index());
  TEST_ASSERT_EQUAL_UINT8(seq.channel(1), m.channel());
  TEST_ASSERT_TRUE(m.update(1032 + 32 * 3));  // skips several slots, wraps
  TEST_ASSERT_EQUAL_UINT8(0, m.index());
}

static void test_follower_locks_and_tracks() {
  HopSequence seq;
  seq.generate(99, 2, 80, 2, 8);
  HopMaster m(seq, 20);
  HopFollower f(seq, 20, 3);
  m.start(0);
  f.start(0);
  TEST_ASSERT_FALSE(f.locked());
  // A frame at t = 5 in master slot 0 (phase 4).
  m.update(5);
  f.on_frame(m.index(), m.phase(5), 5);
  TEST_ASSERT_TRUE(f.locked());
  // Master and follower stay on the same channel while frames keep coming.
  for (uint32_t t = 6; t < 2000; ++t) {
    m.update(t);
    f.update(t);
    if (t % 7 == 0 && m.can_transmit(t)) {
      TEST_ASSERT_EQUAL_UINT8(m.channel(), f.channel());
      f.on_frame(m.index(), m.phase(t), t);
    }
  }
  TEST_ASSERT_TRUE(f.locked());
  TEST_ASSERT_EQUAL_UINT32(0, f.resyncs());
}

static void test_follower_loses_sync_and_parks() {
  HopSequence seq;
  seq.generate(99, 2, 80, 2, 8);
  HopFollower f(seq, 20, 3);
  f.start(0);
  f.on_frame(2, 0, 0);
  // Slot 2 had a frame; slots 3, 4 and 5 stay silent, then it parks.
  f.update(20);
  f.update(40);
  f.update(60);
  TEST_ASSERT_TRUE(f.locked());
  f.update(80);
  TEST_ASSERT_FALSE(f.locked());
  TEST_ASSERT_EQUAL_UINT32(1, f.resyncs());
  const uint8_t parked = f.index();
  TEST_ASSERT_EQUAL_UINT8(6, parked);
  f.update(80 + 20 * 9 - 1);
  TEST_ASSERT_EQUAL_UINT8(parked, f.index());
  f.update(80 + 20 * 9);  // a whole period + 1 slot: try the next channel
  TEST_ASSERT_EQUAL_UINT8((parked + 1) % 8, f.index());
  TEST_ASSERT_FALSE(f.on_frame(8, 0, 300));  // out of range: ignored
  TEST_ASSERT_FALSE(f.locked());
}

// The master keeps hopping; a follower that starts at an arbitrary time and
// only hears the master when both are on the same channel must lock within
// one park period, even with one channel of the sequence jammed.
static void test_resync_with_jammed_channel() {
  HopSequence seq;
  seq.generate(0xBEEF, 2, 80, 2, 8);
  const uint8_t jammed = seq.channel(0);
  HopMaster m(seq, 10);
  HopFollower f(seq, 10, 4);
  m.start(0);
  f.start(3);  // parks on index 0, the jammed channel
  uint32_t locked_at = 0;
  for (uint32_t t = 3; t < 2000; ++t) {
    m.update(t);
    f.update(t);
    if (m.can_transmit(t) && m.channel() == f.channel() && m.channel() != jammed) {
      f.on_frame(m.index(), m.phase(t), t);
      if (locked_at == 0) locked_at = t;
    }
  }
  TEST_ASSERT_TRUE(locked_at > 0);
  TEST_ASSERT_TRUE(locked_at < 3 + 2 * 9 * 10 + 10);
  TEST_ASSERT_TRUE(f.locked());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_xorshift_reference_values);
  RUN_TEST(test_sequence_is_a_permutation_subset);
  RUN_TEST(test_sequence_rejects_bad_arguments);
  RUN_TEST(test_master_slots_and_phase);
  RUN_TEST(test_follower_locks_and_tracks);
  RUN_TEST(test_follower_loses_sync_and_parks);
  RUN_TEST(test_resync_with_jammed_channel);
  return UNITY_END();
}
