// nRF24L01+ driver against the register-level fake chip.
#include <unity.h>

#include <vector>

#include <linkcore/nrf24.h>

#include "../fakes/fake_nrf24.h"

using namespace linkcore;
using namespace linkcore::nrf24;

static fake::FakeClock* g_clock;
static fake::FakeNrf24* g_chip;
static Nrf24* g_radio;

void setUp() {
  g_clock = new fake::FakeClock();
  g_chip = new fake::FakeNrf24(g_clock);
  g_radio = new Nrf24(*g_chip, g_chip->csn, g_chip->ce, *g_clock);
}

void tearDown() {
  delete g_radio;
  delete g_chip;
  delete g_clock;
}

static Nrf24Config default_config() { return Nrf24Config(); }

static void test_begin_applies_configuration() {
  Nrf24Config c = default_config();
  c.channel = 76;
  TEST_ASSERT_TRUE(g_radio->begin(c));
  // EN_CRC | CRCO | PWR_UP, PTX, no IRQ masks.
  TEST_ASSERT_EQUAL_HEX8(kEnCrc | kCrco | kPwrUp, g_chip->reg(kRegConfig));
  TEST_ASSERT_EQUAL_HEX8(0x1F, g_chip->reg(kRegSetupRetr));  // ARD 500 us, ARC 15
  TEST_ASSERT_EQUAL_HEX8(0x03, g_chip->reg(kRegSetupAw));    // 5-byte addresses
  TEST_ASSERT_EQUAL_UINT8(76, g_chip->reg(kRegRfCh));
  TEST_ASSERT_EQUAL_HEX8(0x04, g_chip->reg(kRegRfSetup));    // 1 Mbps, -6 dBm
  TEST_ASSERT_EQUAL_HEX8(kEnDpl | kEnAckPay | kEnDynAck, g_chip->reg(kRegFeature));
  TEST_ASSERT_EQUAL_HEX8(0x3F, g_chip->reg(kRegDynpd));
  TEST_ASSERT_EQUAL_HEX8(0x3F, g_chip->reg(kRegEnAa));
  TEST_ASSERT_EQUAL_HEX8(0x01, g_chip->reg(kRegEnRxAddr));
  TEST_ASSERT_EQUAL_HEX8(kFifoTxEmpty | kFifoRxEmpty, g_chip->reg(kRegFifoStatus));
  // Tpd2stby: the driver waited at least 1.5 ms after setting PWR_UP.
  TEST_ASSERT_TRUE(g_clock->total_delay_us >= 1500);
  TEST_ASSERT_EQUAL_INT(0, g_chip->transactions_in_flight);
  TEST_ASSERT_TRUE(g_radio->is_connected());
}

static void test_begin_masks_and_crc() {
  Nrf24Config c = default_config();
  c.mask_rx_dr = true;
  c.mask_max_rt = true;
  c.crc = CrcLength::k8Bit;
  TEST_ASSERT_TRUE(g_radio->begin(c));
  TEST_ASSERT_EQUAL_HEX8(kMaskRxDr | kMaskMaxRt | kEnCrc | kPwrUp, g_chip->reg(kRegConfig));
  g_radio->set_crc(CrcLength::kDisabled);
  TEST_ASSERT_EQUAL_HEX8(kMaskRxDr | kMaskMaxRt | kPwrUp, g_chip->reg(kRegConfig));
  g_radio->set_crc(CrcLength::k16Bit);
  TEST_ASSERT_EQUAL_HEX8(kMaskRxDr | kMaskMaxRt | kEnCrc | kCrco | kPwrUp,
                         g_chip->reg(kRegConfig));
}

static void test_begin_fails_without_chip() {
  g_chip->present = false;
  TEST_ASSERT_FALSE(g_radio->begin(default_config()));
  TEST_ASSERT_FALSE(g_radio->is_connected());
}

static void test_non_plus_chip_rejects_250kbps() {
  g_chip->plus_variant = false;
  Nrf24Config c = default_config();
  c.data_rate = DataRate::k250Kbps;
  TEST_ASSERT_FALSE(g_radio->begin(c));
  c.data_rate = DataRate::k2Mbps;
  TEST_ASSERT_TRUE(g_radio->begin(c));
}

static void test_invalid_address_width() {
  Nrf24Config c = default_config();
  c.address_width = 6;
  TEST_ASSERT_FALSE(g_radio->begin(c));
  TEST_ASSERT_FALSE(g_radio->set_address_width(2));
  TEST_ASSERT_TRUE(g_radio->set_address_width(3));
  TEST_ASSERT_EQUAL_HEX8(0x01, g_chip->reg(kRegSetupAw));
  TEST_ASSERT_TRUE(g_radio->set_address_width(4));
  TEST_ASSERT_EQUAL_HEX8(0x02, g_chip->reg(kRegSetupAw));
}

static void test_data_rate_and_power_bits() {
  g_radio->begin(default_config());
  TEST_ASSERT_TRUE(g_radio->set_data_rate(DataRate::k250Kbps));
  TEST_ASSERT_EQUAL_HEX8(kRfDrLow, g_chip->reg(kRegRfSetup) & (kRfDrLow | kRfDrHigh));
  TEST_ASSERT_TRUE(g_radio->set_data_rate(DataRate::k2Mbps));
  TEST_ASSERT_EQUAL_HEX8(kRfDrHigh, g_chip->reg(kRegRfSetup) & (kRfDrLow | kRfDrHigh));
  TEST_ASSERT_TRUE(g_radio->set_data_rate(DataRate::k1Mbps));
  TEST_ASSERT_EQUAL_HEX8(0, g_chip->reg(kRegRfSetup) & (kRfDrLow | kRfDrHigh));
  const PaLevel levels[] = {PaLevel::kMinus18dBm, PaLevel::kMinus12dBm, PaLevel::kMinus6dBm,
                            PaLevel::k0dBm};
  for (uint8_t i = 0; i < 4; ++i) {
    g_radio->set_pa_level(levels[i]);
    TEST_ASSERT_EQUAL_HEX8(i << 1, g_chip->reg(kRegRfSetup) & kRfPwrMask);
    TEST_ASSERT_EQUAL_HEX8(0, g_chip->reg(kRegRfSetup) & (kRfDrLow | kRfDrHigh));
  }
}

static void test_channel_clamps_and_resets_plos() {
  g_radio->begin(default_config());
  g_chip->tx_fifo.push_back(fake::Payload());
  g_chip->complete_tx_failure();
  g_chip->complete_tx_failure();
  TEST_ASSERT_EQUAL_UINT8(2, g_radio->observe_tx().lost_packets);
  g_radio->set_channel(200);
  TEST_ASSERT_EQUAL_UINT8(125, g_radio->channel());
  TEST_ASSERT_EQUAL_UINT8(0, g_radio->observe_tx().lost_packets);
  g_radio->set_channel(40);
  TEST_ASSERT_EQUAL_UINT8(40, g_radio->channel());
}

static void test_retries_register() {
  g_radio->begin(default_config());
  g_radio->set_retries(5, 3);
  TEST_ASSERT_EQUAL_HEX8(0x53, g_chip->reg(kRegSetupRetr));
  g_radio->set_retries(0x1F, 0x1F);  // masked to 4 bits each
  TEST_ASSERT_EQUAL_HEX8(0xFF, g_chip->reg(kRegSetupRetr));
}

static void test_pipes_and_addresses() {
  g_radio->begin(default_config());
  const uint8_t tx[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
  g_radio->open_writing_pipe(tx);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(tx, g_chip->tx_addr, 5);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(tx, g_chip->rx_addr_p0, 5);  // for the auto-ack
  const uint8_t p1[5] = {0x11, 0x12, 0x13, 0x14, 0x15};
  TEST_ASSERT_TRUE(g_radio->open_reading_pipe(1, p1));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(p1, g_chip->rx_addr_p1, 5);
  const uint8_t p3[5] = {0x33, 0, 0, 0, 0};
  TEST_ASSERT_TRUE(g_radio->open_reading_pipe(3, p3));
  TEST_ASSERT_EQUAL_HEX8(0x33, g_chip->reg(0x0D));
  TEST_ASSERT_EQUAL_HEX8(0x0B, g_chip->reg(kRegEnRxAddr));  // pipes 0, 1, 3
  g_radio->close_reading_pipe(1);
  TEST_ASSERT_EQUAL_HEX8(0x09, g_chip->reg(kRegEnRxAddr));
  TEST_ASSERT_FALSE(g_radio->open_reading_pipe(6, p1));
  // Address bytes are sent LSByte first right after the W_REGISTER command.
  bool found = false;
  for (const auto& t : g_chip->transactions) {
    if (t.size() == 6 && t[0] == (kCmdWRegister | kRegTxAddr)) {
      found = t[1] == 0x01 && t[5] == 0x05;
    }
  }
  TEST_ASSERT_TRUE(found);
}

static void test_write_pulses_ce_and_fills_fifo() {
  g_radio->begin(default_config());
  const uint8_t data[4] = {1, 2, 3, 4};
  TEST_ASSERT_TRUE(g_radio->write(data, 4));
  TEST_ASSERT_EQUAL_UINT(1, g_chip->tx_fifo.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(data, g_chip->tx_fifo.front().bytes.data(), 4);
  TEST_ASSERT_FALSE(g_chip->tx_fifo.front().no_ack);
  TEST_ASSERT_EQUAL_INT(1, g_chip->ce_pulses);
  TEST_ASSERT_TRUE(g_chip->min_ce_pulse_us >= kCeHighMinUs);  // Thce
  TEST_ASSERT_FALSE(g_chip->ce.level);
  TEST_ASSERT_TRUE(g_radio->write(data, 4, true));
  TEST_ASSERT_TRUE(g_chip->tx_fifo.back().no_ack);  // W_TX_PAYLOAD_NOACK
  TEST_ASSERT_TRUE(g_radio->write(data, 1));
  TEST_ASSERT_TRUE(g_radio->tx_full());
  TEST_ASSERT_FALSE(g_radio->write(data, 4));  // three-level FIFO
  TEST_ASSERT_FALSE(g_radio->queue_payload(data, 0));
  uint8_t big[33] = {};
  g_radio->flush_tx();
  TEST_ASSERT_TRUE(g_radio->tx_empty());
  TEST_ASSERT_FALSE(g_radio->queue_payload(big, 33));
}

static void test_poll_tx_success_and_observe() {
  g_radio->begin(default_config());
  const uint8_t data[3] = {9, 8, 7};
  g_radio->write(data, 3);
  TEST_ASSERT_EQUAL(TxResult::kPending, g_radio->poll_tx());
  g_chip->complete_tx_success(3);
  TEST_ASSERT_EQUAL(TxResult::kSent, g_radio->poll_tx());
  TEST_ASSERT_EQUAL_HEX8(0, g_chip->status() & kTxDs);  // cleared
  TEST_ASSERT_EQUAL_UINT8(3, g_radio->observe_tx().retransmits);
  TEST_ASSERT_EQUAL(TxResult::kPending, g_radio->poll_tx());
}

static void test_poll_tx_failure_keeps_payload() {
  g_radio->begin(default_config());
  const uint8_t data[3] = {9, 8, 7};
  g_radio->write(data, 3);
  g_chip->complete_tx_failure();
  TEST_ASSERT_EQUAL(TxResult::kFailed, g_radio->poll_tx());
  TEST_ASSERT_EQUAL_HEX8(0, g_chip->status() & kMaxRt);
  TEST_ASSERT_EQUAL_UINT8(15, g_radio->observe_tx().retransmits);
  TEST_ASSERT_EQUAL_UINT8(1, g_radio->observe_tx().lost_packets);
  TEST_ASSERT_FALSE(g_radio->tx_empty());  // spec 7.8.7: not removed on MAX_RT
  const int pulses = g_chip->ce_pulses;
  g_radio->retry_last();
  TEST_ASSERT_EQUAL_INT(pulses + 1, g_chip->ce_pulses);
  g_radio->flush_tx();
  TEST_ASSERT_TRUE(g_radio->tx_empty());
}

static void test_ack_payload_is_read() {
  g_radio->begin(default_config());
  const uint8_t data[2] = {1, 2};
  g_radio->write(data, 2);
  g_chip->complete_tx_success(0, {0xAA, 0xBB, 0xCC});
  TEST_ASSERT_EQUAL(TxResult::kSent, g_radio->poll_tx());
  uint8_t pipe = 9;
  TEST_ASSERT_TRUE(g_radio->available(&pipe));
  TEST_ASSERT_EQUAL_UINT8(0, pipe);
  uint8_t buf[32];
  TEST_ASSERT_EQUAL_INT(3, g_radio->read(buf, sizeof(buf), &pipe));
  TEST_ASSERT_EQUAL_HEX8(0xAA, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0xCC, buf[2]);
  TEST_ASSERT_FALSE(g_radio->available());
  TEST_ASSERT_EQUAL_HEX8(0, g_chip->status() & kRxDr);
  TEST_ASSERT_EQUAL_INT(-1, g_radio->read(buf, sizeof(buf)));
}

static void test_invalid_width_flushes_rx() {
  g_radio->begin(default_config());
  g_chip->inject_rx(1, {1, 2, 3});
  g_chip->inject_rx(1, {4, 5, 6});
  g_chip->forced_width = 33;
  uint8_t buf[32];
  TEST_ASSERT_EQUAL_INT(-1, g_radio->read(buf, sizeof(buf)));
  TEST_ASSERT_TRUE(g_radio->rx_empty());  // FLUSH_RX per section 7.3.4
}

static void test_short_buffer_drains_payload() {
  g_radio->begin(default_config());
  g_chip->inject_rx(2, {1, 2, 3, 4, 5});
  uint8_t buf[3];
  TEST_ASSERT_EQUAL_INT(-1, g_radio->read(buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_HEX8(1, buf[0]);
  TEST_ASSERT_TRUE(g_radio->rx_empty());
}

static void test_prx_listening_and_ack_payloads() {
  g_radio->begin(default_config());
  const uint8_t addr[5] = {1, 2, 3, 4, 5};
  g_radio->open_reading_pipe(1, addr);
  g_radio->start_listening();
  TEST_ASSERT_EQUAL_HEX8(kPrimRx | kPwrUp, g_chip->reg(kRegConfig) & (kPrimRx | kPwrUp));
  TEST_ASSERT_TRUE(g_chip->ce.level);
  const uint8_t ack[4] = {7, 7, 7, 7};
  TEST_ASSERT_TRUE(g_radio->write_ack_payload(1, ack, 4));
  TEST_ASSERT_EQUAL_HEX8(kCmdWAckPayload | 1, g_chip->transactions.back()[0]);
  TEST_ASSERT_FALSE(g_radio->write_ack_payload(6, ack, 4));
  g_chip->prx_receive(1, {0x10, 0x20});
  TEST_ASSERT_EQUAL_UINT(1, g_chip->acks_sent.size());
  uint8_t pipe = 0;
  uint8_t buf[32];
  TEST_ASSERT_EQUAL_INT(2, g_radio->read(buf, sizeof(buf), &pipe));
  TEST_ASSERT_EQUAL_UINT8(1, pipe);
  g_radio->stop_listening();
  TEST_ASSERT_FALSE(g_chip->ce.level);
  TEST_ASSERT_EQUAL_HEX8(0, g_chip->reg(kRegConfig) & kPrimRx);
}

static void test_take_interrupts_clears_flags() {
  g_radio->begin(default_config());
  g_chip->inject_rx(0, {1});
  g_chip->tx_fifo.push_back(fake::Payload());
  g_chip->complete_tx_failure();
  const uint8_t flags = g_radio->take_interrupts();
  TEST_ASSERT_EQUAL_HEX8(kRxDr | kMaxRt, flags);
  TEST_ASSERT_EQUAL_HEX8(0, g_chip->status() & kIrqMask);
  TEST_ASSERT_EQUAL_HEX8(0, g_radio->take_interrupts());
}

static void test_rpd_and_power_down() {
  g_radio->begin(default_config());
  TEST_ASSERT_FALSE(g_radio->received_power_detector());
  g_chip->set_rpd(true);
  TEST_ASSERT_TRUE(g_radio->received_power_detector());
  g_radio->power_down();
  TEST_ASSERT_EQUAL_HEX8(0, g_chip->reg(kRegConfig) & kPwrUp);
  const uint64_t before = g_clock->total_delay_us;
  g_radio->power_up();
  TEST_ASSERT_EQUAL_HEX8(kPwrUp, g_chip->reg(kRegConfig) & kPwrUp);
  TEST_ASSERT_TRUE(g_clock->total_delay_us - before >= 1500);
}

static void test_static_payload_mode() {
  Nrf24Config c = default_config();
  c.dynamic_payloads = false;
  c.ack_payloads = false;
  TEST_ASSERT_TRUE(g_radio->begin(c));
  TEST_ASSERT_EQUAL_HEX8(kEnDynAck, g_chip->reg(kRegFeature));
  TEST_ASSERT_EQUAL_HEX8(0, g_chip->reg(kRegDynpd));
  const uint8_t addr[5] = {1, 2, 3, 4, 5};
  g_radio->open_reading_pipe(2, addr);
  TEST_ASSERT_EQUAL_UINT8(32, g_chip->reg(kRegRxPwP0 + 2));
  const uint8_t data[3] = {1, 2, 3};
  g_radio->write(data, 3);
  TEST_ASSERT_EQUAL_UINT(32, g_chip->tx_fifo.front().bytes.size());  // zero padded
  std::vector<uint8_t> full(32, 0x5A);
  g_chip->inject_rx(2, full);
  uint8_t buf[32];
  TEST_ASSERT_EQUAL_INT(32, g_radio->read(buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_HEX8(0x5A, buf[31]);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_begin_applies_configuration);
  RUN_TEST(test_begin_masks_and_crc);
  RUN_TEST(test_begin_fails_without_chip);
  RUN_TEST(test_non_plus_chip_rejects_250kbps);
  RUN_TEST(test_invalid_address_width);
  RUN_TEST(test_data_rate_and_power_bits);
  RUN_TEST(test_channel_clamps_and_resets_plos);
  RUN_TEST(test_retries_register);
  RUN_TEST(test_pipes_and_addresses);
  RUN_TEST(test_write_pulses_ce_and_fills_fifo);
  RUN_TEST(test_poll_tx_success_and_observe);
  RUN_TEST(test_poll_tx_failure_keeps_payload);
  RUN_TEST(test_ack_payload_is_read);
  RUN_TEST(test_invalid_width_flushes_rx);
  RUN_TEST(test_short_buffer_drains_payload);
  RUN_TEST(test_prx_listening_and_ack_payloads);
  RUN_TEST(test_take_interrupts_clears_flags);
  RUN_TEST(test_rpd_and_power_down);
  RUN_TEST(test_static_payload_mode);
  return UNITY_END();
}
