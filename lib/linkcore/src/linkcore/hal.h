// Abstract hardware interfaces used by the nRF24L01+ driver.
//
// The firmware implements these over the Arduino SPI and digital pin APIs
// (src/arduino_hal.h); the native unit tests implement them with a
// register-level fake chip (test/fakes/fake_nrf24.h).
//
// Destructors are protected and non-virtual: objects are never deleted
// through these interfaces, and this avoids pulling operator delete into AVR
// builds.
#pragma once

#include <stdint.h>

namespace linkcore {

/// SPI mode 0, MSB first, at most 10 MHz (nRF24L01+ spec section 8.2).
/// Chip select is driven separately through an OutputPin so the driver
/// controls exactly when a command starts and ends.
class SpiBus {
 public:
  virtual void begin_transaction() = 0;
  virtual uint8_t transfer(uint8_t byte) = 0;
  virtual void end_transaction() = 0;

 protected:
  ~SpiBus() = default;
};

class OutputPin {
 public:
  virtual void write(bool high) = 0;

 protected:
  ~OutputPin() = default;
};

class Clock {
 public:
  virtual uint32_t micros() = 0;
  virtual void delay_us(uint32_t us) = 0;

 protected:
  ~Clock() = default;
};

}  // namespace linkcore
