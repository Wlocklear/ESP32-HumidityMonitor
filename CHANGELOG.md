# HumidityMonitor — Changelog

## History

### 2026-10-18
- Code comments and long docstrings moved out of the `.py` files into **Design notes** below. No geometry or behavior change.

## Design notes

Moved verbatim from code comments and docstrings on 2026-10-18.

### `release.py`

```text
release.py — bump FIRMWARE_VERSION in src/main.cpp, commit, tag, and push.

Usage:
    python release.py 1.4
    python release.py v1.4   (leading 'v' is stripped automatically)
```

### Moved from README / code header (2026-10-23)

- Two I2C buses because the AHT30 is fixed at 0x38 with no address pin. S3-only: the C6's `Wire1` maps to the pin-restricted LP_I2C. A C6 port needs a TCA9548A mux or a different outside sensor (SHT31 @ 0x44, BME280 @ 0x76).
- `/history` is separate from `/status` because 4032 points would mean serializing several hundred KB on the heap every 15 s poll.
- History is stored as tenths in `int16_t` (~48 KB instead of ~96 KB); the AHT30 only resolves ~0.024 %RH / 0.01 °C, so nothing is lost. Dew point uses Magnus-Tetens.

### Moved from code comments, 2026-10-23

**`src/main.cpp`** — before `#define HISTORY_SIZE  4032`

```text
 SENSOR HISTORY
   4032 x 5 min = 2 weeks. Readings are stored as tenths in
   int16_t (12 bytes/point, ~48 KB total) rather than floats,
   which would double the static footprint for no added
   precision — the AHT30 resolves ~0.024 %RH / 0.01 degC.
```
