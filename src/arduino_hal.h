// linkcore hardware interfaces implemented with the Arduino SPI and pin APIs.
#pragma once

#include <Arduino.h>
#include <SPI.h>

#include <linkcore/hal.h>

class ArduinoSpi final : public linkcore::SpiBus {
 public:
  explicit ArduinoSpi(uint32_t hz) : settings_(hz, MSBFIRST, SPI_MODE0) {}
  void begin_transaction() override { SPI.beginTransaction(settings_); }
  uint8_t transfer(uint8_t byte) override { return SPI.transfer(byte); }
  void end_transaction() override { SPI.endTransaction(); }

 private:
  SPISettings settings_;
};

class ArduinoPin final : public linkcore::OutputPin {
 public:
  explicit ArduinoPin(uint8_t pin) : pin_(pin) {}
  void begin(bool initial) {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, initial ? HIGH : LOW);
  }
  void write(bool high) override { digitalWrite(pin_, high ? HIGH : LOW); }

 private:
  uint8_t pin_;
};

/// A few bits of start-up entropy for initial sequence numbers. Not
/// cryptographic: on AVR and STM32 it mixes ADC noise with micros().
inline uint8_t startup_entropy8(uint8_t analog_pin) {
#if defined(ESP32)
  (void)analog_pin;
  return static_cast<uint8_t>(esp_random());
#else
  uint8_t v = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    const uint8_t bit = static_cast<uint8_t>((analogRead(analog_pin) ^ micros()) & 1u);
    v = static_cast<uint8_t>((v << 1) | bit);
    delayMicroseconds(37);
  }
  return v;
#endif
}

class ArduinoClock final : public linkcore::Clock {
 public:
  uint32_t micros() override { return ::micros(); }
  void delay_us(uint32_t us) override {
    if (us >= 1000) {
      delay(us / 1000);
      us %= 1000;
    }
    delayMicroseconds(static_cast<unsigned int>(us));
  }
};
