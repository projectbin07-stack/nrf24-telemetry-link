# Security policy

## Supported versions

Only the latest release receives fixes.

## Reporting a vulnerability

Please report security problems privately through GitHub's
[private vulnerability reporting](https://github.com/projectbin07-stack/nrf24-telemetry-link/security/advisories/new)
rather than in a public issue. Include the affected version or commit, what
you observed and, if possible, a way to reproduce it. You should get a reply
within two weeks.

## Scope notes

- **The radio link has no encryption and no authentication.** The pipe
  address and hop seed are not secrets: anyone with an nRF24L01+ in range can
  sniff telemetry and, by transmitting as the node or answering as the
  gateway, inject commands. Do not use this link to control anything where an
  injected or replayed command could hurt someone or damage equipment, and
  keep a physical emergency stop on robots.
- The gateway's USB serial port accepts commands without authentication. Do
  not expose it (for example through a network serial bridge) to untrusted
  parties.
- Received radio frames and serial packets are treated as untrusted input:
  length, type, fragment index, COBS structure and CRCs are checked before
  anything is copied, and malformed input is counted and dropped. Reports of
  inputs that crash or corrupt either side are in scope.
