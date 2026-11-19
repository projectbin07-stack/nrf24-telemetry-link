// Register-level fake nRF24L01+ for native tests.
//
// Implements the SPI command set (Table 20), the register map with reset
// values (Table 28), three-level TX and RX FIFOs, write-1-to-clear interrupt
// flags, PLOS_CNT reset on RF_CH writes, CE edge timing and the ACK-payload
// FIFO of a PRX. It does not model RF: tests decide the outcome of each
// transmission with complete_tx_success() / complete_tx_failure() and
// inject received packets with inject_rx().
#pragma once

#include <stdint.h>
#include <string.h>

#include <deque>
#include <vector>

#include <linkcore/hal.h>
#include <linkcore/nrf24_regs.h>

namespace fake {

using namespace linkcore::nrf24;

struct Payload {
  std::vector<uint8_t> bytes;
  uint8_t pipe = 0;
  bool no_ack = false;
};

class FakeClock final : public linkcore::Clock {
 public:
  uint32_t micros() override { return now_us; }
  void delay_us(uint32_t us) override {
    now_us += us;
    total_delay_us += us;
  }
  uint32_t now_us = 0;
  uint64_t total_delay_us = 0;
};

class FakeNrf24;

class FakePin final : public linkcore::OutputPin {
 public:
  FakePin(FakeNrf24* chip, bool is_csn) : chip_(chip), is_csn_(is_csn) {}
  void write(bool high) override;
  bool level = true;

 private:
  FakeNrf24* chip_;
  bool is_csn_;
};

class FakeNrf24 final : public linkcore::SpiBus {
 public:
  explicit FakeNrf24(FakeClock* clock) : csn(this, true), ce(this, false), clock_(clock) {
    ce.level = false;
    reset();
  }

  // --- configuration of the fake ---
  bool present = true;        // false: MISO floats high (reads 0xFF)
  bool plus_variant = true;   // false: RF_DR_LOW cannot be set (nRF24L01 non-plus)
  int forced_width = -1;      // >= 0: R_RX_PL_WID returns this value

  FakePin csn;
  FakePin ce;

  // --- observations ---
  std::vector<std::vector<uint8_t>> transactions;  // MOSI bytes per CSN-low period
  int ce_pulses = 0;
  uint32_t last_ce_high_us = 0;
  uint32_t min_ce_pulse_us = 0xFFFFFFFFu;
  int transactions_in_flight = 0;  // SPI begin without end
  std::vector<Payload> sent;       // payloads that left the TX FIFO (PTX)
  std::vector<Payload> acks_sent;  // ACK payloads that left the TX FIFO (PRX)

  void reset() {
    memset(regs_, 0, sizeof(regs_));
    regs_[kRegConfig] = 0x08;
    regs_[kRegEnAa] = 0x3F;
    regs_[kRegEnRxAddr] = 0x03;
    regs_[kRegSetupAw] = 0x03;
    regs_[kRegSetupRetr] = 0x03;
    regs_[kRegRfCh] = 0x02;
    regs_[kRegRfSetup] = 0x0E;
    regs_[0x0C] = 0xC3;
    regs_[0x0D] = 0xC4;
    regs_[0x0E] = 0xC5;
    regs_[0x0F] = 0xC6;
    for (int i = 0; i < 5; ++i) {
      rx_addr_p0[i] = 0xE7;
      rx_addr_p1[i] = 0xC2;
      tx_addr[i] = 0xE7;
    }
    irq_ = 0;
    tx_fifo.clear();
    rx_fifo.clear();
  }

  // --- register access for assertions ---
  uint8_t reg(uint8_t r) const {
    if (r == kRegStatus) return status();
    if (r == kRegFifoStatus) return fifo_status();
    return regs_[r];
  }
  void set_rpd(bool on) { regs_[kRegRpd] = on ? 1 : 0; }
  uint8_t plos() const { return static_cast<uint8_t>(regs_[kRegObserveTx] >> 4); }

  uint8_t rx_addr_p0[5];
  uint8_t rx_addr_p1[5];
  uint8_t tx_addr[5];
  std::deque<Payload> tx_fifo;
  std::deque<Payload> rx_fifo;

  // --- RF outcomes, driven by the test ---
  /// The head of the TX FIFO was acknowledged after `retransmits` retries;
  /// optionally the ACK carried a payload (goes to the RX FIFO, pipe 0).
  void complete_tx_success(uint8_t retransmits, const std::vector<uint8_t>& ack_payload = {}) {
    Payload p = tx_fifo.front();
    tx_fifo.pop_front();
    sent.push_back(p);
    set_arc(retransmits);
    irq_ |= kTxDs;
    if (!ack_payload.empty()) {
      inject_rx(0, ack_payload);
    }
  }
  /// ARC retransmissions all went unanswered: MAX_RT, PLOS_CNT++, payload stays.
  void complete_tx_failure() {
    set_arc(regs_[kRegSetupRetr] & 0x0F);
    uint8_t plos_cnt = static_cast<uint8_t>(regs_[kRegObserveTx] >> 4);
    if (plos_cnt < 15) ++plos_cnt;
    regs_[kRegObserveTx] = static_cast<uint8_t>((plos_cnt << 4) | (regs_[kRegObserveTx] & 0x0F));
    irq_ |= kMaxRt;
  }
  bool inject_rx(uint8_t pipe, const std::vector<uint8_t>& bytes) {
    if (rx_fifo.size() >= kFifoDepth) return false;
    Payload p;
    p.bytes = bytes;
    p.pipe = pipe;
    rx_fifo.push_back(p);
    irq_ |= kRxDr;
    return true;
  }
  /// PRX: a packet arrives on `pipe`; the first pending ACK payload for that
  /// pipe leaves with the ACK.
  void prx_receive(uint8_t pipe, const std::vector<uint8_t>& bytes) {
    inject_rx(pipe, bytes);
    for (auto it = tx_fifo.begin(); it != tx_fifo.end(); ++it) {
      if (it->pipe == pipe) {
        acks_sent.push_back(*it);
        tx_fifo.erase(it);
        irq_ |= kTxDs;
        break;
      }
    }
  }

  // --- SpiBus ---
  void begin_transaction() override { ++transactions_in_flight; }
  void end_transaction() override { --transactions_in_flight; }

  uint8_t transfer(uint8_t mosi) override {
    if (!selected_) {
      return 0xFF;
    }
    current_.push_back(mosi);
    if (!present) {
      return 0xFF;
    }
    const size_t i = current_.size() - 1;
    if (i == 0) {
      const uint8_t s = status();
      start_command(mosi);
      return s;
    }
    return data_byte(i - 1);
  }

  // called by FakePin
  void on_csn(bool high) {
    if (!high && !selected_) {
      selected_ = true;
      current_.clear();
    } else if (high && selected_) {
      selected_ = false;
      if (present) finish_command();
      transactions.push_back(current_);
    }
  }
  void on_ce(bool high, bool was_high) {
    const uint32_t now = clock_->now_us;
    if (high && !was_high) {
      last_ce_high_us = now;
    } else if (!high && was_high) {
      const uint32_t width = now - last_ce_high_us;
      if (width < min_ce_pulse_us) min_ce_pulse_us = width;
      ++ce_pulses;
    }
  }

  uint8_t status() const {
    uint8_t s = static_cast<uint8_t>(irq_ & kIrqMask);
    const uint8_t pipe = rx_fifo.empty() ? kRxPNoEmpty : rx_fifo.front().pipe;
    s = static_cast<uint8_t>(s | (pipe << kRxPNoShift));
    if (tx_fifo.size() >= kFifoDepth) s |= kStatusTxFull;
    return s;
  }
  uint8_t fifo_status() const {
    uint8_t f = 0;
    if (tx_fifo.size() >= kFifoDepth) f |= kFifoTxFull;
    if (tx_fifo.empty()) f |= kFifoTxEmpty;
    if (rx_fifo.size() >= kFifoDepth) f |= kFifoRxFull;
    if (rx_fifo.empty()) f |= kFifoRxEmpty;
    return f;
  }

 private:
  void set_arc(uint8_t arc) {
    regs_[kRegObserveTx] = static_cast<uint8_t>((regs_[kRegObserveTx] & 0xF0) | (arc & 0x0F));
  }

  void start_command(uint8_t cmd) {
    cmd_ = cmd;
    if (cmd == kCmdFlushTx) tx_fifo.clear();
    if (cmd == kCmdFlushRx) rx_fifo.clear();
  }

  uint8_t data_byte(size_t i) {
    if ((cmd_ & 0xE0) == kCmdRRegister) {
      const uint8_t r = static_cast<uint8_t>(cmd_ & kRegisterMask);
      if (r == kRegRxAddrP0) return i < 5 ? rx_addr_p0[i] : 0;
      if (r == kRegRxAddrP0 + 1) return i < 5 ? rx_addr_p1[i] : 0;
      if (r == kRegTxAddr) return i < 5 ? tx_addr[i] : 0;
      return reg(r);
    }
    if (cmd_ == kCmdRRxPlWid) {
      if (forced_width >= 0) return static_cast<uint8_t>(forced_width);
      return rx_fifo.empty() ? 0 : static_cast<uint8_t>(rx_fifo.front().bytes.size());
    }
    if (cmd_ == kCmdRRxPayload) {
      if (rx_fifo.empty() || i >= rx_fifo.front().bytes.size()) return 0;
      return rx_fifo.front().bytes[i];
    }
    return 0;
  }

  void finish_command() {
    const size_t n = current_.size();
    if (n == 0) return;
    const uint8_t* data = current_.data() + 1;
    const size_t len = n - 1;
    if ((cmd_ & 0xE0) == kCmdWRegister && len > 0) {
      write_reg(static_cast<uint8_t>(cmd_ & kRegisterMask), data, len);
    } else if ((cmd_ == kCmdWTxPayload || cmd_ == kCmdWTxPayloadNoAck) && len > 0) {
      if (tx_fifo.size() < kFifoDepth && len <= 32) {
        Payload p;
        p.bytes.assign(data, data + len);
        p.no_ack = cmd_ == kCmdWTxPayloadNoAck;
        tx_fifo.push_back(p);
      }
    } else if ((cmd_ & 0xF8) == kCmdWAckPayload && len > 0) {
      if (tx_fifo.size() < kFifoDepth && len <= 32) {
        Payload p;
        p.bytes.assign(data, data + len);
        p.pipe = static_cast<uint8_t>(cmd_ & 0x07);
        tx_fifo.push_back(p);
      }
    } else if (cmd_ == kCmdRRxPayload && len > 0 && !rx_fifo.empty()) {
      rx_fifo.pop_front();
    }
  }

  void write_reg(uint8_t r, const uint8_t* data, size_t len) {
    if (r == kRegRxAddrP0 || r == kRegRxAddrP0 + 1 || r == kRegTxAddr) {
      uint8_t* dst = r == kRegRxAddrP0 ? rx_addr_p0 : (r == kRegTxAddr ? tx_addr : rx_addr_p1);
      for (size_t i = 0; i < len && i < 5; ++i) dst[i] = data[i];
      return;
    }
    const uint8_t v = data[0];
    switch (r) {
      case kRegStatus:
        irq_ = static_cast<uint8_t>(irq_ & ~(v & kIrqMask));
        break;
      case kRegRfCh:
        regs_[r] = static_cast<uint8_t>(v & 0x7F);
        regs_[kRegObserveTx] = static_cast<uint8_t>(regs_[kRegObserveTx] & 0x0F);  // PLOS_CNT = 0
        break;
      case kRegRfSetup:
        regs_[r] = plus_variant ? v : static_cast<uint8_t>(v & ~kRfDrLow);
        break;
      case kRegSetupAw:
        regs_[r] = static_cast<uint8_t>(v & 0x03);
        break;
      case kRegObserveTx:
      case kRegRpd:
      case kRegFifoStatus:
        break;  // read-only
      default:
        if (r <= kRegFeature) regs_[r] = v;
        break;
    }
  }

  uint8_t regs_[0x20];
  uint8_t irq_ = 0;
  uint8_t cmd_ = 0;
  bool selected_ = false;
  std::vector<uint8_t> current_;
  FakeClock* clock_;
};

inline void FakePin::write(bool high) {
  const bool was = level;
  level = high;
  if (is_csn_) {
    chip_->on_csn(high);
  } else {
    chip_->on_ce(high, was);
  }
}

}  // namespace fake
