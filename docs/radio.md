# The nRF24L01+ and how this project uses it

All section, table and page references are to the **nRF24L01+ Single Chip
2.4GHz Transceiver Product Specification v1.0** (Nordic Semiconductor,
78 pages), the document Nordic publishes as the product specification for
the "+" part. The driver in `lib/linkcore/src/linkcore/nrf24.{h,cpp}` was
written from this document; it does not use or derive from the RF24 Arduino
library.

## What the chip provides

| Feature | Specification | Used here |
|---|---|---|
| RF channel | `F0 = 2400 + RF_CH` MHz, RF_CH 0..125, 1 MHz steps; at 2 Mbps channels must be at least 2 MHz apart to not overlap (section 6.3) | Hop channels are generated with 2 MHz spacing by default (`NTL_HOP_SPACING`). `Nrf24::set_channel` clamps to 125. |
| Data rate | 250 kbps, 1 Mbps, 2 Mbps via RF_DR_LOW / RF_DR_HIGH in RF_SETUP (section 6.2, Table 28) | Default 1 Mbps. `set_data_rate` reads RF_SETUP back; a non-plus nRF24L01 cannot set RF_DR_LOW, so 250 kbps fails `begin()`. |
| Output power | RF_PWR: -18, -12, -6, 0 dBm (section 6.5, Table 17) | Default -6 dBm (`NTL_PA_LEVEL`), changeable by command. |
| Payload | 0..32 bytes, static (RX_PW_Px) or dynamic (DPL) length (section 7.3.4) | Dynamic payloads on every pipe; the link layer never sends more than 32 bytes. |
| Packet | preamble (1 B) + address (3..5 B) + 9-bit packet control field (6-bit length, 2-bit PID, NO_ACK) + payload + CRC (1 or 2 B) (section 7.3) | 5-byte addresses, 16-bit CRC. The simulator's air-time formula uses exactly these fields. |
| CRC | 8-bit (X^8+X^2+X+1, init 0xFF) or 16-bit (X^16+X^12+X^5+1, init 0xFFFF) over address, PCF and payload (section 7.3.5) | 16-bit. The link layer adds its own end-to-end CRC-16 over reassembled messages because the radio CRC protects one packet only. |
| Auto acknowledgement | PRX answers each valid packet with an ACK, optionally carrying an ACK payload (EN_ACK_PAY, needs DPL) (section 7.4.1) | Gateway-to-node traffic (commands, link-layer acks) rides in ACK payloads. |
| Auto retransmission | ARD: 250 us x (n + 1); ARC: 0..15 retries; MAX_RT when exhausted (section 7.4.2) | ARD 500 us, ARC 15 by default. |
| ARD vs ACK payload size | "ARD=500us is long enough for any ACK payload length in 1 or 2Mbps mode"; at 250 kbps Table 18 gives the limits (1500 us covers every size) (section 7.4.2) | `NTL_ARD_CODE 1` (500 us). If you select 250 kbps, set `NTL_ARD_CODE 5`. |
| Duplicate filter | PRX compares PID and CRC with the previous packet and discards a copy, but still ACKs it (sections 7.3.3.2, 7.5.2) | Modelled in the simulator. The link layer has its own sequence numbers because PID is only 2 bits and resets on MAX_RT flushes. |
| ACK payload lifetime | An ACK payload stays at the head of the PRX TX FIFO and is re-sent with ACKs to retransmissions; the PRX treats it as delivered (TX_DS) when a packet with a new PID arrives (section 7.8.6, Figure 23) | This is why the link layer acknowledges commands end to end: if the ACK carrying a command is lost and the PTX then gives up (MAX_RT) and sends a new packet, the PRX has already discarded the command. The simulator models this FIFO behaviour. |
| MultiCeiver | 6 pipes; pipes 2..5 share address bytes 1..4 with pipe 1; on a PTX, RX_ADDR_P0 must equal TX_ADDR to receive the ACK (section 7.6) | Node: TX_ADDR = RX_ADDR_P0 = node address. Gateway: pipe 1 = node address. One node per gateway in the shipped firmware. |
| FIFOs | three-level TX and RX FIFOs (section 8.4) | The gateway keeps up to three ACK payloads queued. |
| Interrupts | RX_DR, TX_DS, MAX_RT in STATUS, write 1 to clear; "If MAX_RT is asserted it must be cleared to enable further communication"; IRQ pin active low, maskable in CONFIG (section 8.5, Table 28) | `poll_tx()` and `take_interrupts()` clear exactly the flags they report. The firmware polls STATUS over SPI; the IRQ pin is wired but not required. |
| Observe TX | ARC_CNT: retransmissions of the current packet, reset when a new transaction starts. PLOS_CNT: lost packets, saturates at 15, reset by writing RF_CH (section 7.4.2, Table 28) | ARC_CNT feeds the link-quality window. PLOS_CNT is not used for metrics because every hop writes RF_CH and resets it. |
| RPD | set when a signal above -64 dBm is present on the channel, latched for a received packet; valid after Tstby2a + Tdelay_AGC = 130 us + 40 us in RX (section 6.4) | Gateway records RPD per received packet as a one-bit signal-strength proxy (`rpd_percent` in link stats). |
| Dynamic payload width | "If its width is longer than 32 bytes then the packet contains errors and must be discarded" with FLUSH_RX (section 7.3.4) | `Nrf24::read()` does exactly this and returns -1. |
| SPI | mode 0, MSB first, max 10 Mbps; multi-byte registers LSByte first; STATUS is shifted out on MISO during the command byte (section 8.3, Table 20) | 8 MHz on all boards (Nano: F_CPU / 2). Addresses are written LSByte first. |

### Timing used by the driver (section 6.1.7, Table 16)

| Symbol | Value | Where |
|---|---|---|
| Tpd2stby (power down to standby) | 1.5 ms with a crystal of Ls < 30 mH (150 us with an external clock) | `power_up()` waits 1.5 ms |
| Tstby2a (standby to TX/RX) | 130 us | `start_listening()` waits 130 us; simulator adds it before the first transmission and before each ACK |
| Thce (minimum CE high to start TX) | 10 us | `pulse_ce()` holds CE high for 15 us |
| Tpece2csn (CE rising edge to CSN low) | 4 us | met implicitly: CSN is not pulled low inside a CE pulse |

### Register map (Table 28) as used

| Addr | Register | Written by `begin()` |
|---|---|---|
| 0x00 | CONFIG | IRQ masks from the config, EN_CRC, CRCO (2 bytes), then PWR_UP; PRIM_RX set by `start_listening()` |
| 0x01 | EN_AA | 0x3F (auto-ack on all pipes; DPL requires it) |
| 0x02 | EN_RXADDR | 0x01; reading pipes added by `open_reading_pipe()` |
| 0x03 | SETUP_AW | width - 2 (0x03 for 5 bytes); also the presence check |
| 0x04 | SETUP_RETR | ARD << 4 \| ARC |
| 0x05 | RF_CH | start channel (hop index 0) |
| 0x06 | RF_SETUP | RF_DR_LOW / RF_DR_HIGH and RF_PWR, read back |
| 0x07 | STATUS | IRQ flags cleared (write 1 to clear) |
| 0x08 | OBSERVE_TX | read only: PLOS_CNT[7:4], ARC_CNT[3:0] |
| 0x09 | RPD | read only, bit 0 |
| 0x0A..0x0F | RX_ADDR_P0..P5 | via `open_writing_pipe()` / `open_reading_pipe()` |
| 0x10 | TX_ADDR | via `open_writing_pipe()` |
| 0x11..0x16 | RX_PW_P0..P5 | 32 only when dynamic payloads are disabled |
| 0x17 | FIFO_STATUS | read only |
| 0x1C | DYNPD | 0x3F with dynamic payloads |
| 0x1D | FEATURE | EN_DPL, EN_ACK_PAY, EN_DYN_ACK |

SPI commands used (Table 20): R_REGISTER, W_REGISTER, R_RX_PAYLOAD,
W_TX_PAYLOAD, W_TX_PAYLOAD_NOACK, W_ACK_PAYLOAD, R_RX_PL_WID, FLUSH_TX,
FLUSH_RX and NOP. REUSE_TX_PL is defined but not used: after MAX_RT the
payload is still in the TX FIFO (section 7.8.7), so `retry_last()` only has
to clear MAX_RT and pulse CE.

## Roles in this project

```
 node (PTX)                                      gateway (PRX)
 ----------                                      -------------
 TX_ADDR = RX_ADDR_P0 = node address             pipe 1 = node address
 sends link frames (telemetry, polls) ------->   RX FIFO -> link layer -> USB serial
 RX FIFO <- ACK payload  <---------------------  W_ACK_PAYLOAD (commands, link acks)
 HopMaster: owns hop timing                      HopFollower: follows, resyncs
```

The node is always the primary transmitter: it decides when the radio is
on, which suits a battery node, and it never has to listen for unsolicited
packets. Everything the gateway sends travels in ACK payloads, so the node
sends an empty "idle" link frame every `NTL_POLL_INTERVAL_MS` (10 ms) when it
has nothing else to send. Command latency is therefore bounded below by the
poll interval.

## Hardware notes

* **Supply.** VDD is 1.9 to 3.6 V (section 4). Never connect the module to
  5 V. The digital inputs are 5 V tolerant (key features list), so a 5 V
  Arduino Nano can drive SCK, MOSI, CSN and CE directly; MISO and IRQ are
  3.3 V outputs, which an ATmega328P at 5 V reads as high (V_IH = 0.6 VCC =
  3.0 V), with little margin.
* **Peak current.** The specification lists 11.3 mA in TX at 0 dBm and
  13.5 mA in RX at 2 Mbps. Modules with a PA/LNA draw far more. The Nano's
  3.3 V pin comes from the USB-serial chip or a small regulator and often
  cannot supply PA/LNA modules; use a separate 3.3 V regulator.
* **Decoupling.** Put a 10 uF (or larger) electrolytic or tantalum capacitor
  plus 100 nF ceramic directly across VCC and GND at the module's pins. Brown
  outs during transmission bursts are the most common cause of
  "it sends a few packets then stops" with these modules.
* **Clones.** Many modules are not genuine Nordic parts. Some do not support
  250 kbps (the driver's read-back check catches that), and some behave
  differently with ACK payloads. Nothing in this repository has been tested
  on any module (see the README's hardware validation status).
