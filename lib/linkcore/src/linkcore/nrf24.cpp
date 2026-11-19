#include "linkcore/nrf24.h"

namespace linkcore {

using namespace nrf24;

Nrf24::Nrf24(SpiBus& spi, OutputPin& csn, OutputPin& ce, Clock& clock)
    : spi_(spi), csn_(csn), ce_(ce), clock_(clock) {}

// --- low-level SPI commands (section 8.3) ---

uint8_t Nrf24::command(uint8_t cmd) { return command_write(cmd, nullptr, 0); }

uint8_t Nrf24::command_write(uint8_t cmd, const uint8_t* data, uint8_t length) {
  spi_.begin_transaction();
  csn_.write(false);
  const uint8_t status = spi_.transfer(cmd);
  for (uint8_t i = 0; i < length; ++i) {
    spi_.transfer(data[i]);
  }
  csn_.write(true);
  spi_.end_transaction();
  return status;
}

uint8_t Nrf24::command_read(uint8_t cmd, uint8_t* out, uint8_t length) {
  spi_.begin_transaction();
  csn_.write(false);
  const uint8_t status = spi_.transfer(cmd);
  for (uint8_t i = 0; i < length; ++i) {
    out[i] = spi_.transfer(kCmdNop);
  }
  csn_.write(true);
  spi_.end_transaction();
  return status;
}

uint8_t Nrf24::read_register(uint8_t reg) {
  uint8_t value = 0;
  command_read(static_cast<uint8_t>(kCmdRRegister | (reg & kRegisterMask)), &value, 1);
  return value;
}

void Nrf24::read_register(uint8_t reg, uint8_t* out, uint8_t length) {
  command_read(static_cast<uint8_t>(kCmdRRegister | (reg & kRegisterMask)), out, length);
}

void Nrf24::write_register(uint8_t reg, uint8_t value) {
  command_write(static_cast<uint8_t>(kCmdWRegister | (reg & kRegisterMask)), &value, 1);
}

void Nrf24::write_register(uint8_t reg, const uint8_t* data, uint8_t length) {
  command_write(static_cast<uint8_t>(kCmdWRegister | (reg & kRegisterMask)), data, length);
}

void Nrf24::update_config(uint8_t set, uint8_t clear) {
  const uint8_t value = read_register(kRegConfig);
  write_register(kRegConfig, static_cast<uint8_t>((value & ~clear) | set));
}

// --- configuration ---

bool Nrf24::begin(const Nrf24Config& config) {
  ce_.write(false);
  ce_high_ = false;
  csn_.write(true);

  uint8_t cfg = 0;
  if (config.mask_rx_dr) cfg |= kMaskRxDr;
  if (config.mask_tx_ds) cfg |= kMaskTxDs;
  if (config.mask_max_rt) cfg |= kMaskMaxRt;
  write_register(kRegConfig, cfg);  // powered down, PTX
  set_crc(config.crc);
  set_retries(config.retransmit_delay, config.retransmit_count);
  if (!set_address_width(config.address_width)) {
    return false;
  }
  set_pa_level(config.pa_level);
  const bool rate_ok = set_data_rate(config.data_rate);
  set_channel(config.channel);

  dynamic_payloads_ = config.dynamic_payloads;
  uint8_t feature = 0;
  if (config.dynamic_payloads) {
    feature |= kEnDpl;
    if (config.ack_payloads) feature |= kEnAckPay;
  }
  if (config.dynamic_ack) feature |= kEnDynAck;
  write_register(kRegFeature, feature);
  // Auto-ack on every pipe; DPL_Px requires ENAA_Px (Table 28, DYNPD).
  write_register(kRegEnAa, 0x3F);
  write_register(kRegDynpd, config.dynamic_payloads ? 0x3F : 0x00);
  // Only pipe 0 (auto-ack reception) until reading pipes are opened.
  write_register(kRegEnRxAddr, 0x01);
  if (!config.dynamic_payloads) {
    write_register(kRegRxPwP0, kMaxPayload);
  }

  flush_rx();
  flush_tx();
  clear_interrupts(kIrqMask);

  const uint8_t aw_expected = static_cast<uint8_t>(config.address_width - 2);
  if (read_register(kRegSetupAw) != aw_expected || !rate_ok) {
    return false;
  }
  power_up();
  return true;
}

bool Nrf24::is_connected() {
  const uint8_t aw = read_register(kRegSetupAw);
  return aw >= 1 && aw <= 3 && aw == static_cast<uint8_t>(address_width_ - 2);
}

void Nrf24::set_channel(uint8_t channel) {
  write_register(kRegRfCh, channel > kMaxChannel ? kMaxChannel : channel);
}

uint8_t Nrf24::channel() { return read_register(kRegRfCh); }

bool Nrf24::set_data_rate(DataRate rate) {
  uint8_t setup = read_register(kRegRfSetup);
  setup = static_cast<uint8_t>(setup & ~(kRfDrLow | kRfDrHigh));
  if (rate == DataRate::k250Kbps) {
    setup |= kRfDrLow;
  } else if (rate == DataRate::k2Mbps) {
    setup |= kRfDrHigh;
  }
  write_register(kRegRfSetup, setup);
  return read_register(kRegRfSetup) == setup;
}

void Nrf24::set_pa_level(PaLevel level) {
  uint8_t setup = read_register(kRegRfSetup);
  setup = static_cast<uint8_t>((setup & ~kRfPwrMask) |
                               ((static_cast<uint8_t>(level) << kRfPwrShift) & kRfPwrMask));
  write_register(kRegRfSetup, setup);
}

void Nrf24::set_crc(CrcLength crc) {
  if (crc == CrcLength::kDisabled) {
    update_config(0, static_cast<uint8_t>(kEnCrc | kCrco));
  } else if (crc == CrcLength::k8Bit) {
    update_config(kEnCrc, kCrco);
  } else {
    update_config(static_cast<uint8_t>(kEnCrc | kCrco), 0);
  }
}

void Nrf24::set_retries(uint8_t delay_code, uint8_t count) {
  write_register(kRegSetupRetr,
                 static_cast<uint8_t>(((delay_code & 0x0F) << 4) | (count & 0x0F)));
}

bool Nrf24::set_address_width(uint8_t bytes) {
  if (bytes < 3 || bytes > 5) {
    return false;
  }
  address_width_ = bytes;
  write_register(kRegSetupAw, static_cast<uint8_t>(bytes - 2));
  return true;
}

void Nrf24::open_writing_pipe(const uint8_t* address) {
  write_register(kRegTxAddr, address, address_width_);
  write_register(kRegRxAddrP0, address, address_width_);
  write_register(kRegEnRxAddr, static_cast<uint8_t>(read_register(kRegEnRxAddr) | 0x01));
}

bool Nrf24::open_reading_pipe(uint8_t pipe, const uint8_t* address) {
  if (pipe >= kPipes) {
    return false;
  }
  if (pipe < 2) {
    write_register(static_cast<uint8_t>(kRegRxAddrP0 + pipe), address, address_width_);
  } else {
    write_register(static_cast<uint8_t>(kRegRxAddrP0 + pipe), address[0]);
  }
  if (!dynamic_payloads_) {
    write_register(static_cast<uint8_t>(kRegRxPwP0 + pipe), kMaxPayload);
  }
  write_register(kRegEnRxAddr,
                 static_cast<uint8_t>(read_register(kRegEnRxAddr) | (1u << pipe)));
  return true;
}

void Nrf24::close_reading_pipe(uint8_t pipe) {
  if (pipe >= kPipes) {
    return;
  }
  write_register(kRegEnRxAddr,
                 static_cast<uint8_t>(read_register(kRegEnRxAddr) & ~(1u << pipe)));
}

// --- modes (section 6.1) ---

void Nrf24::power_up() {
  const uint8_t value = read_register(kRegConfig);
  if (!(value & kPwrUp)) {
    write_register(kRegConfig, static_cast<uint8_t>(value | kPwrUp));
    clock_.delay_us(kPowerUpUs);
  }
}

void Nrf24::power_down() {
  ce_.write(false);
  ce_high_ = false;
  update_config(0, kPwrUp);
}

void Nrf24::start_listening() {
  update_config(static_cast<uint8_t>(kPwrUp | kPrimRx), 0);
  clear_interrupts(kIrqMask);
  ce_.write(true);
  ce_high_ = true;
  clock_.delay_us(kStandbyToActiveUs);
}

void Nrf24::stop_listening() {
  ce_.write(false);
  ce_high_ = false;
  update_config(0, kPrimRx);
}

// --- transmit (sections 6.1.5, 7.4) ---

bool Nrf24::queue_payload(const uint8_t* data, uint8_t length, bool no_ack) {
  if (length == 0 || length > kMaxPayload || tx_full()) {
    return false;
  }
  const uint8_t cmd = no_ack ? kCmdWTxPayloadNoAck : kCmdWTxPayload;
  spi_.begin_transaction();
  csn_.write(false);
  spi_.transfer(cmd);
  for (uint8_t i = 0; i < length; ++i) {
    spi_.transfer(data[i]);
  }
  if (!dynamic_payloads_) {
    for (uint8_t i = length; i < kMaxPayload; ++i) {
      spi_.transfer(0);
    }
  }
  csn_.write(true);
  spi_.end_transaction();
  return true;
}

void Nrf24::pulse_ce() {
  ce_.write(true);
  clock_.delay_us(kCeHighMinUs + 5);
  ce_.write(false);
  ce_high_ = false;
}

bool Nrf24::write(const uint8_t* data, uint8_t length, bool no_ack) {
  if (!queue_payload(data, length, no_ack)) {
    return false;
  }
  pulse_ce();
  return true;
}

void Nrf24::retry_last() {
  clear_interrupts(kMaxRt);
  pulse_ce();
}

TxResult Nrf24::poll_tx() {
  const uint8_t s = status();
  if (s & kTxDs) {
    clear_interrupts(kTxDs);
    return TxResult::kSent;
  }
  if (s & kMaxRt) {
    clear_interrupts(kMaxRt);
    return TxResult::kFailed;
  }
  return TxResult::kPending;
}

bool Nrf24::write_ack_payload(uint8_t pipe, const uint8_t* data, uint8_t length) {
  if (pipe >= kPipes || length == 0 || length > kMaxPayload || tx_full()) {
    return false;
  }
  command_write(static_cast<uint8_t>(kCmdWAckPayload | pipe), data, length);
  return true;
}

// --- receive (sections 7.3.4, 8.4) ---

bool Nrf24::available(uint8_t* pipe) {
  const uint8_t p = static_cast<uint8_t>((status() & kRxPNoMask) >> kRxPNoShift);
  if (p >= kPipes) {
    return false;
  }
  if (pipe != nullptr) {
    *pipe = p;
  }
  return true;
}

int Nrf24::read(uint8_t* buffer, uint8_t capacity, uint8_t* pipe) {
  uint8_t p = 0;
  if (!available(&p)) {
    return -1;
  }
  uint8_t width = 0;
  if (dynamic_payloads_) {
    command_read(kCmdRRxPlWid, &width, 1);
    if (width == 0 || width > kMaxPayload) {
      // Section 7.3.4: a width above 32 means the packet is corrupt and must
      // be discarded with FLUSH_RX.
      flush_rx();
      clear_interrupts(kRxDr);
      return -1;
    }
  } else {
    width = read_register(static_cast<uint8_t>(kRegRxPwP0 + p));
    if (width == 0 || width > kMaxPayload) {
      width = kMaxPayload;
    }
  }
  spi_.begin_transaction();
  csn_.write(false);
  spi_.transfer(kCmdRRxPayload);
  for (uint8_t i = 0; i < width; ++i) {
    const uint8_t b = spi_.transfer(kCmdNop);
    if (i < capacity) {
      buffer[i] = b;
    }
  }
  csn_.write(true);
  spi_.end_transaction();
  clear_interrupts(kRxDr);
  if (pipe != nullptr) {
    *pipe = p;
  }
  return width > capacity ? -1 : static_cast<int>(width);
}

// --- status and FIFOs (sections 8.4, 8.5, Table 28) ---

uint8_t Nrf24::status() { return command(kCmdNop); }

uint8_t Nrf24::take_interrupts() {
  const uint8_t flags = static_cast<uint8_t>(status() & kIrqMask);
  if (flags != 0) {
    write_register(kRegStatus, flags);
  }
  return flags;
}

void Nrf24::clear_interrupts(uint8_t flags) {
  write_register(kRegStatus, static_cast<uint8_t>(flags & kIrqMask));
}

uint8_t Nrf24::fifo_status() { return read_register(kRegFifoStatus); }

bool Nrf24::tx_full() { return (fifo_status() & kFifoTxFull) != 0; }

bool Nrf24::tx_empty() { return (fifo_status() & kFifoTxEmpty) != 0; }

bool Nrf24::rx_empty() { return (fifo_status() & kFifoRxEmpty) != 0; }

void Nrf24::flush_tx() { command(kCmdFlushTx); }

void Nrf24::flush_rx() { command(kCmdFlushRx); }

ObserveTx Nrf24::observe_tx() {
  const uint8_t v = read_register(kRegObserveTx);
  ObserveTx o;
  o.lost_packets = static_cast<uint8_t>(v >> 4);
  o.retransmits = static_cast<uint8_t>(v & 0x0F);
  return o;
}

bool Nrf24::received_power_detector() { return (read_register(kRegRpd) & 0x01) != 0; }

}  // namespace linkcore
