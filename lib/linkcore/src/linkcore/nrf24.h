// nRF24L01+ driver over abstract SPI / GPIO / clock interfaces.
//
// Written from the Nordic product specification v1.0 (see docs/radio.md),
// not derived from any existing Arduino library. It is deliberately small:
// it exposes the chip's features as the specification describes them and
// leaves protocol decisions (when to send, what to do on MAX_RT) to the link
// layer and the application.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "linkcore/hal.h"
#include "linkcore/nrf24_regs.h"

namespace linkcore {

enum class DataRate : uint8_t { k250Kbps = 0, k1Mbps = 1, k2Mbps = 2 };

/// RF_PWR in RF_SETUP: -18, -12, -6 and 0 dBm (Table 17).
enum class PaLevel : uint8_t { kMinus18dBm = 0, kMinus12dBm = 1, kMinus6dBm = 2, k0dBm = 3 };

enum class CrcLength : uint8_t { kDisabled = 0, k8Bit = 1, k16Bit = 2 };

enum class TxResult : uint8_t { kPending = 0, kSent = 1, kFailed = 2 };

struct Nrf24Config {
  uint8_t channel = 76;
  DataRate data_rate = DataRate::k1Mbps;
  PaLevel pa_level = PaLevel::kMinus6dBm;
  CrcLength crc = CrcLength::k16Bit;
  uint8_t address_width = 5;  // 3, 4 or 5 bytes
  /// ARD code 0..15: delay = 250 us * (code + 1). With ack payloads at 1 or
  /// 2 Mbps the specification requires at least 500 us (code 1); at 250 kbps
  /// 1500 us (code 5) covers every ack payload length (section 7.4.2).
  uint8_t retransmit_delay = 1;
  uint8_t retransmit_count = 15;  // ARC 0..15
  bool dynamic_payloads = true;   // EN_DPL + DYNPD on every enabled pipe
  bool ack_payloads = true;       // EN_ACK_PAY (requires dynamic payloads)
  bool dynamic_ack = true;        // EN_DYN_ACK: allows W_TX_PAYLOAD_NOACK
  bool mask_rx_dr = false;        // mask bits keep the IRQ pin quiet for that event
  bool mask_tx_ds = false;
  bool mask_max_rt = false;
};

struct ObserveTx {
  uint8_t lost_packets;  // PLOS_CNT: packets that reached MAX_RT since the last RF_CH write (saturates at 15)
  uint8_t retransmits;   // ARC_CNT: retransmissions of the current / last packet
};

class Nrf24 {
 public:
  Nrf24(SpiBus& spi, OutputPin& csn, OutputPin& ce, Clock& clock);

  /// Applies `config`, flushes both FIFOs, clears interrupts and powers up in
  /// standby-I (PTX). Returns false if the chip does not answer (SETUP_AW or
  /// RF_SETUP read back wrong), which is also what an nRF24L01 (non-plus) does
  /// for 250 kbps.
  bool begin(const Nrf24Config& config);

  /// True if register writes read back: a cheap presence check.
  bool is_connected();

  void set_channel(uint8_t channel);  // clamps to 0..125; also resets PLOS_CNT
  uint8_t channel();
  bool set_data_rate(DataRate rate);  // false if the read-back does not match
  void set_pa_level(PaLevel level);
  void set_crc(CrcLength crc);
  void set_retries(uint8_t delay_code, uint8_t count);
  bool set_address_width(uint8_t bytes);
  uint8_t address_width() const { return address_width_; }

  /// TX_ADDR and RX_ADDR_P0 (needed to receive the auto-ack), `address_width_` bytes, LSByte first.
  void open_writing_pipe(const uint8_t* address);
  /// Pipes 0 and 1 take a full address; pipes 2..5 share bytes 1.. with pipe 1
  /// and only address[0] is written. Enables auto-ack and, if configured,
  /// dynamic payloads on the pipe.
  bool open_reading_pipe(uint8_t pipe, const uint8_t* address);
  void close_reading_pipe(uint8_t pipe);

  void power_up();    // PWR_UP and wait Tpd2stby
  void power_down();
  void start_listening();  // PRX: PRIM_RX = 1, CE high, wait Tstby2a
  void stop_listening();   // CE low, PRIM_RX = 0

  /// Queue a payload (1..32 bytes) in the TX FIFO without starting it.
  bool queue_payload(const uint8_t* data, uint8_t length, bool no_ack = false);
  /// CE pulse of at least Thce (10 us) to send one queued packet.
  void pulse_ce();
  /// queue_payload() followed by pulse_ce().
  bool write(const uint8_t* data, uint8_t length, bool no_ack = false);
  /// Re-arm the last packet after MAX_RT without re-uploading it.
  void retry_last();
  /// Polls STATUS: kSent on TX_DS, kFailed on MAX_RT (both flags are
  /// cleared; the failed payload is left in the FIFO, see flush_tx()).
  TxResult poll_tx();

  /// PRX: queue a payload that rides on the next auto-ack for `pipe`.
  bool write_ack_payload(uint8_t pipe, const uint8_t* data, uint8_t length);

  /// True if the RX FIFO holds a payload; `pipe` receives its pipe number.
  bool available(uint8_t* pipe = nullptr);
  /// Reads the oldest payload. Returns its length, or -1 if the FIFO was
  /// empty or the reported width was invalid (> 32: the specification says
  /// to discard it, so the RX FIFO is flushed).
  int read(uint8_t* buffer, uint8_t capacity, uint8_t* pipe = nullptr);

  uint8_t status();  // NOP: STATUS is shifted out on every command
  /// Reads STATUS, clears the RX_DR / TX_DS / MAX_RT flags that were set and
  /// returns them (use from the IRQ pin handler or a poll loop).
  uint8_t take_interrupts();
  void clear_interrupts(uint8_t flags);
  uint8_t fifo_status();
  bool tx_full();
  bool tx_empty();
  bool rx_empty();
  void flush_tx();
  void flush_rx();
  ObserveTx observe_tx();
  /// Received Power Detector: a signal above -64 dBm was present (valid only
  /// in RX mode after Tstby2a + Tdelay_AGC, section 6.4).
  bool received_power_detector();

  uint8_t read_register(uint8_t reg);
  void read_register(uint8_t reg, uint8_t* out, uint8_t length);
  void write_register(uint8_t reg, uint8_t value);
  void write_register(uint8_t reg, const uint8_t* data, uint8_t length);

 private:
  uint8_t command(uint8_t cmd);
  uint8_t command_write(uint8_t cmd, const uint8_t* data, uint8_t length);
  uint8_t command_read(uint8_t cmd, uint8_t* out, uint8_t length);
  void update_config(uint8_t set, uint8_t clear);

  SpiBus& spi_;
  OutputPin& csn_;
  OutputPin& ce_;
  Clock& clock_;
  uint8_t address_width_ = 5;
  bool dynamic_payloads_ = true;
  bool ce_high_ = false;
};

}  // namespace linkcore
