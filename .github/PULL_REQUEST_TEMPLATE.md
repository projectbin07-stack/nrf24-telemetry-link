## What this changes

## Why

## How it was checked

- [ ] `pio test -e native`
- [ ] `pio run` for the board environments affected
- [ ] `cd host && ruff check . && ruff format --check . && mypy && pytest` (if `host/` changed)
- [ ] `nrflink vectors` (if the link layer, hopping, schema or framing changed; vectors regenerated and committed)
- [ ] C++ and Python sides changed together (link, hopping, quality, messages, serial framing)
- [ ] Tried with real radios (say which boards and modules, or "not tested on hardware")

## Firmware size (if firmware code changed)

Paste the RAM / Flash lines PlatformIO prints.
