// nRF24L01+ SPI commands, register addresses and bit positions.
//
// Source: nRF24L01+ Single Chip 2.4GHz Transceiver Product Specification
// v1.0, Nordic Semiconductor, Table 20 (SPI commands) and Table 28 (register
// map). Written from the specification; see docs/radio.md.
//
// Names follow the specification with a k prefix (kRfCh = RF_CH) so they
// cannot collide with the many register macros defined by MCU vendor headers.
#pragma once

#include <stdint.h>

namespace linkcore {
namespace nrf24 {

// --- SPI commands (Table 20) ---
constexpr uint8_t kCmdRRegister = 0x00;  // R_REGISTER  000A AAAA
constexpr uint8_t kCmdWRegister = 0x20;  // W_REGISTER  001A AAAA
constexpr uint8_t kRegisterMask = 0x1F;
constexpr uint8_t kCmdRRxPayload = 0x61;      // R_RX_PAYLOAD
constexpr uint8_t kCmdWTxPayload = 0xA0;      // W_TX_PAYLOAD
constexpr uint8_t kCmdFlushTx = 0xE1;         // FLUSH_TX
constexpr uint8_t kCmdFlushRx = 0xE2;         // FLUSH_RX
constexpr uint8_t kCmdReuseTxPl = 0xE3;       // REUSE_TX_PL
constexpr uint8_t kCmdRRxPlWid = 0x60;        // R_RX_PL_WID
constexpr uint8_t kCmdWAckPayload = 0xA8;     // W_ACK_PAYLOAD 1010 1PPP
constexpr uint8_t kCmdWTxPayloadNoAck = 0xB0;  // W_TX_PAYLOAD_NOACK
constexpr uint8_t kCmdNop = 0xFF;             // NOP

// --- Registers (Table 28) ---
constexpr uint8_t kRegConfig = 0x00;
constexpr uint8_t kRegEnAa = 0x01;
constexpr uint8_t kRegEnRxAddr = 0x02;
constexpr uint8_t kRegSetupAw = 0x03;
constexpr uint8_t kRegSetupRetr = 0x04;
constexpr uint8_t kRegRfCh = 0x05;
constexpr uint8_t kRegRfSetup = 0x06;
constexpr uint8_t kRegStatus = 0x07;
constexpr uint8_t kRegObserveTx = 0x08;
constexpr uint8_t kRegRpd = 0x09;
constexpr uint8_t kRegRxAddrP0 = 0x0A;  // P1 = 0x0B, P2..P5 = 0x0C..0x0F
constexpr uint8_t kRegTxAddr = 0x10;
constexpr uint8_t kRegRxPwP0 = 0x11;  // P1..P5 = 0x12..0x16
constexpr uint8_t kRegFifoStatus = 0x17;
constexpr uint8_t kRegDynpd = 0x1C;
constexpr uint8_t kRegFeature = 0x1D;

// CONFIG
constexpr uint8_t kMaskRxDr = 1u << 6;
constexpr uint8_t kMaskTxDs = 1u << 5;
constexpr uint8_t kMaskMaxRt = 1u << 4;
constexpr uint8_t kEnCrc = 1u << 3;
constexpr uint8_t kCrco = 1u << 2;
constexpr uint8_t kPwrUp = 1u << 1;
constexpr uint8_t kPrimRx = 1u << 0;

// RF_SETUP
constexpr uint8_t kContWave = 1u << 7;
constexpr uint8_t kRfDrLow = 1u << 5;
constexpr uint8_t kPllLock = 1u << 4;
constexpr uint8_t kRfDrHigh = 1u << 3;
constexpr uint8_t kRfPwrShift = 1;
constexpr uint8_t kRfPwrMask = 0x06;

// STATUS
constexpr uint8_t kRxDr = 1u << 6;
constexpr uint8_t kTxDs = 1u << 5;
constexpr uint8_t kMaxRt = 1u << 4;
constexpr uint8_t kIrqMask = kRxDr | kTxDs | kMaxRt;
constexpr uint8_t kRxPNoShift = 1;
constexpr uint8_t kRxPNoMask = 0x0E;
constexpr uint8_t kRxPNoEmpty = 0x07;
constexpr uint8_t kStatusTxFull = 1u << 0;

// FIFO_STATUS
constexpr uint8_t kFifoTxReuse = 1u << 6;
constexpr uint8_t kFifoTxFull = 1u << 5;
constexpr uint8_t kFifoTxEmpty = 1u << 4;
constexpr uint8_t kFifoRxFull = 1u << 1;
constexpr uint8_t kFifoRxEmpty = 1u << 0;

// FEATURE
constexpr uint8_t kEnDpl = 1u << 2;
constexpr uint8_t kEnAckPay = 1u << 1;
constexpr uint8_t kEnDynAck = 1u << 0;

// --- Limits and timing (sections 6.1.7, 6.3, 6.4, 7.3.4, 8.4) ---
constexpr uint8_t kMaxPayload = 32;
constexpr uint8_t kPipes = 6;
constexpr uint8_t kFifoDepth = 3;
constexpr uint8_t kMaxChannel = 125;          // F0 = 2400 + RF_CH MHz
constexpr uint32_t kPowerUpUs = 1500;         // Tpd2stby, crystal Ls < 30 mH
constexpr uint32_t kStandbyToActiveUs = 130;  // Tstby2a
constexpr uint32_t kCeHighMinUs = 10;         // Thce
constexpr uint32_t kRpdSettleUs = 170;        // Tstby2a + Tdelay_AGC

}  // namespace nrf24
}  // namespace linkcore
