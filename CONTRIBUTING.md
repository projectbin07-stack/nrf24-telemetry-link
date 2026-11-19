# Contributing

Bug reports, results from real hardware and small focused pull requests are
all welcome.

## Before you open a pull request

1. **C++ unit tests** (no hardware needed):

   ```sh
   pio test -e native
   ```

2. **Firmware builds** for the environments you touched (CI builds all six):

   ```sh
   pio run -e nanoatmega328 -e nanoatmega328_gateway
   pio run -e esp32dev -e esp32dev_gateway
   pio run -e nucleo_f401re -e nucleo_f401re_gateway
   ```

   If your change touches firmware code, include the RAM / Flash lines
   PlatformIO prints, especially for the Nano (2 KB RAM).

3. **Host tool**:

   ```sh
   cd host
   pip install -e ".[dev]"
   ruff check . && ruff format --check . && mypy && pytest
   nrflink vectors
   ```

CI runs all of the above on every push and pull request.

## Guidelines

- `lib/linkcore` must stay free of Arduino and vendor headers so it keeps
  building and testing on the host, and must compile with avr-gcc: no `std::`
  containers, no exceptions, no heap. Hardware access belongs in `src/`.
- **Link-layer changes** touch `lib/linkcore/src/linkcore/link.*` *and*
  `host/src/nrflink/link.py` (the simulator runs the Python port). Regenerate
  the shared vectors with `nrflink vectors --write` and commit
  `test/vectors/link_vectors.json`; the C++ `test_vectors` suite replays the
  scripted scenarios and fails if the two implementations diverge. The same
  applies to `hopping`, `quality`, `messages` and `serial_frame`.
- **Driver changes** need a test against `test/fakes/fake_nrf24.h` and a
  section or table reference to the nRF24L01+ product specification, in the
  code or in `docs/radio.md`. If the fake chip has to change, cite the
  specification for the behaviour you model.
- **Protocol changes** also update `docs/protocol.md`. Changing an existing
  field's layout or meaning needs a new version byte or frame type.
- **Simulator results** in the README must be regenerated with the commands
  given there and stay labelled as simulated.
- Keep commits small, with an imperative subject line that says what changed.

## Reporting hardware results

The README states that nothing has been tested with real radios. If you flash
it, please open an issue with the boards, the nRF24L01+ modules (genuine or
clone, PA/LNA or not), how they were powered, the build flags you changed and
what you observed (the gateway's `stats` lines are the most useful),
including anything that did not work.
