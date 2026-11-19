# nrflink

Host tools for [nrf24-telemetry-link](https://github.com/projectbin07-stack/nrf24-telemetry-link):
a decoder for the gateway's COBS + CRC-16 serial frames, a CLI to log telemetry
and send commands, and a link simulator that runs a Python port of the
firmware's link layer over modelled nRF24L01+ radios.

See the repository README for the protocol, wiring and simulator results.

```sh
pip install -e ".[dev]"
nrflink --help
```
