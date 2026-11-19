# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [0.1.0] - 2026-11-19

First public version. Compiles for all six firmware environments in CI;
**not tested on hardware or with real radios**.

### Added

- `lib/linkcore` (C++17, native-tested): nRF24L01+ driver over abstract
  SPI / GPIO / clock interfaces, written from the Nordic product
  specification; link layer with fragmentation (up to 160-byte messages),
  end-to-end CRC-16, a stop-and-wait reliable channel and a best-effort
  channel; deterministic channel hopping with resynchronisation; link-quality
  metrics from ACK success, ARC_CNT and RPD; versioned message schema
  (battery, odometry, IMU, key/value, command); COBS + CRC-16 gateway serial
  framing.
- Register-level fake nRF24L01+ for driver tests; 60 Unity test cases.
- Arduino node and gateway firmware for Arduino Nano, ESP32 DevKit and
  NUCLEO-F401RE.
- `nrflink` host package: gateway stream decoder, CSV logging, command
  sender, link simulator (ESB timing, ACK payload FIFO semantics, Bernoulli
  and Gilbert-Elliott loss, Wi-Fi interference) and a dependency-free SVG
  plot writer.
- Shared JSON test vectors, including scripted link scenarios replayed by
  both the C++ and the Python implementation.
- CI: native tests, six firmware builds with a size report, host checks on
  Ubuntu and Windows with Python 3.10 and 3.13.

[0.1.0]: https://github.com/projectbin07-stack/nrf24-telemetry-link/releases/tag/v0.1.0
