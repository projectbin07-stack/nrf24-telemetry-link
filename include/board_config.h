// Pin maps and radio settings shared by the node and the gateway firmware.
//
// Every value can be overridden with a -D build flag. The nRF24L01+ module
// must be powered from 3.3 V (1.9 - 3.6 V, product specification Table 6);
// its SPI and CE inputs are 5 V tolerant, so the Nano can drive them
// directly. See README.md "Wiring" for the decoupling capacitor note.
#pragma once

#include <stdint.h>

#if defined(NTL_BOARD_NANO)
// Hardware SPI: D13 SCK, D12 MISO, D11 MOSI. D10 must stay an output for the
// AVR to remain SPI master, so it is used as CSN.
#ifndef NTL_PIN_CE
#define NTL_PIN_CE 9
#endif
#ifndef NTL_PIN_CSN
#define NTL_PIN_CSN 10
#endif
#ifndef NTL_PIN_IRQ
#define NTL_PIN_IRQ 2
#endif
#ifndef NTL_PIN_LED
#define NTL_PIN_LED -1  // the on-board LED is on D13 = SCK, so it is not used
#endif
#ifndef NTL_PIN_VBAT
#define NTL_PIN_VBAT A0
#endif
#define NTL_ADC_REF_MV 5000
#define NTL_ADC_MAX 1023
#define NTL_SPI_HZ 8000000UL  // F_CPU / 2

#elif defined(NTL_BOARD_ESP32DEV)
// VSPI default pins: GPIO18 SCK, GPIO19 MISO, GPIO23 MOSI.
#ifndef NTL_PIN_CE
#define NTL_PIN_CE 4
#endif
#ifndef NTL_PIN_CSN
#define NTL_PIN_CSN 5
#endif
#ifndef NTL_PIN_IRQ
#define NTL_PIN_IRQ 27
#endif
#ifndef NTL_PIN_LED
#define NTL_PIN_LED 2
#endif
#ifndef NTL_PIN_VBAT
#define NTL_PIN_VBAT 34  // ADC1 input-only pin
#endif
#define NTL_ADC_REF_MV 3300  // nominal; the ESP32 ADC is non-linear near the rails
#define NTL_ADC_MAX 4095
#define NTL_SPI_HZ 8000000UL

#elif defined(NTL_BOARD_NUCLEO_F401RE)
// Arduino header SPI: D13 (PA5) SCK, D12 (PA6) MISO, D11 (PA7) MOSI.
#ifndef NTL_PIN_CE
#define NTL_PIN_CE D9  // PC7
#endif
#ifndef NTL_PIN_CSN
#define NTL_PIN_CSN D10  // PB6
#endif
#ifndef NTL_PIN_IRQ
#define NTL_PIN_IRQ D2  // PA10
#endif
#ifndef NTL_PIN_LED
#define NTL_PIN_LED -1  // LD2 is on PA5 = D13 = SCK, so it is not used
#endif
#ifndef NTL_PIN_VBAT
#define NTL_PIN_VBAT A0  // PA0
#endif
#define NTL_ADC_REF_MV 3300
#define NTL_ADC_MAX 1023  // Arduino default 10-bit analogRead resolution
#define NTL_SPI_HZ 8000000UL

#else
#error "Select a board with -D NTL_BOARD_NANO, NTL_BOARD_ESP32DEV or NTL_BOARD_NUCLEO_F401RE"
#endif

// --- Radio and link settings (must match on node and gateway) ---
#ifndef NTL_SERIAL_BAUD
#define NTL_SERIAL_BAUD 115200
#endif
#ifndef NTL_DATA_RATE
#define NTL_DATA_RATE 1  // 0 = 250 kbps, 1 = 1 Mbps, 2 = 2 Mbps
#endif
#ifndef NTL_PA_LEVEL
#define NTL_PA_LEVEL 2  // 0..3 = -18, -12, -6, 0 dBm
#endif
#ifndef NTL_ARD_CODE
#define NTL_ARD_CODE 1  // 500 us: enough for any ACK payload at 1 and 2 Mbps
#endif
#ifndef NTL_ARC
#define NTL_ARC 15
#endif
#ifndef NTL_HOPPING
#define NTL_HOPPING 1  // 0: stay on NTL_FIXED_CHANNEL
#endif
#ifndef NTL_FIXED_CHANNEL
#define NTL_FIXED_CHANNEL 76
#endif
#ifndef NTL_HOP_SEED
#define NTL_HOP_SEED 0x4E524634UL  // shared secret-free network id ("NRF4")
#endif
#ifndef NTL_HOP_FIRST
#define NTL_HOP_FIRST 2
#endif
#ifndef NTL_HOP_LAST
#define NTL_HOP_LAST 80
#endif
#ifndef NTL_HOP_SPACING
#define NTL_HOP_SPACING 2  // 2 MHz keeps 2 Mbps channels from overlapping
#endif
#ifndef NTL_HOP_COUNT
#define NTL_HOP_COUNT 8
#endif
#ifndef NTL_HOP_DWELL_MS
#define NTL_HOP_DWELL_MS 50
#endif
#ifndef NTL_HOP_LOST_SLOTS
#define NTL_HOP_LOST_SLOTS 4
#endif
#ifndef NTL_TELEMETRY_PERIOD_MS
#define NTL_TELEMETRY_PERIOD_MS 50
#endif
#ifndef NTL_POLL_INTERVAL_MS
#define NTL_POLL_INTERVAL_MS 10
#endif
#ifndef NTL_RELIABLE_TIMEOUT_MS
#define NTL_RELIABLE_TIMEOUT_MS 100
#endif
#ifndef NTL_RELIABLE_ATTEMPTS
#define NTL_RELIABLE_ATTEMPTS 8
#endif
#ifndef NTL_DEMO_SENSORS
#define NTL_DEMO_SENSORS 1  // synthetic odometry / IMU samples, see README
#endif
#ifndef NTL_VBAT_DIVIDER_X100
#define NTL_VBAT_DIVIDER_X100 200  // (R1 + R2) / R2 * 100; 2:1 divider by default
#endif

// 5-byte pipe address of the node (LSByte first on the wire).
static const uint8_t kNodeAddress[5] = {0x4C, 0x4E, 0x4B, 0x30, 0x31};
