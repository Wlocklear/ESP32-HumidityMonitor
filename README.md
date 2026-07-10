# Humidity Monitor

Reads temperature and relative humidity **inside** and **outside** a container
with two AHT30 sensors and serves them on a web dashboard.

Monitor only — no relay, no heater, no control loop. If you want the drying
state machine and heater control, see the sibling
[HumidityController](../HumidityController) project.

## Hardware

| | Sensor | Bus | SDA | SCL |
|---|---|---|---|---|
| Inside  | AHT30 @ 0x38 | `Wire`  (I2C0) | GPIO 8  | GPIO 9  |
| Outside | AHT30 @ 0x38 | `Wire1` (I2C1) | GPIO 10 | GPIO 11 |

Both sensors take `VIN → 3V3` and `GND → GND`. Each bus needs its own pull-up
resistors; most AHT30 breakout boards already have them.

### Why two buses, and why ESP32-S3 only

The AHT30's I2C address is hard-wired to `0x38` and it has no address-select
pin, so **two AHT30s can never share one bus.** The ESP32-S3 has two full
hardware I2C controllers (`SOC_HP_I2C_NUM == 2`), so each sensor gets its own
bus and both keep address `0x38`.

The ESP32-C6 has only one high-power I2C controller. Its `Wire1` maps to the
low-power `LP_I2C` peripheral, which is restricted to specific LP GPIO pins.
To run this on a C6 you would need either a TCA9548A I2C multiplexer, or a
different chip for the outside sensor so the two addresses differ (SHT31 at
`0x44`, BME280 at `0x76`).

## Build

```
pio run -e esp32s3 -t upload        # first flash, over USB
pio run -e esp32s3-ota -t upload    # subsequent flashes, over WiFi
```

On first boot the device has no WiFi credentials and brings up an access point
named `ESP32-Setup`. Connect to it, pick your network, and the device reboots
onto your LAN at `http://humiditymonitor.local`.

## HTTP API

| Route | Purpose |
|---|---|
| `GET /` | Dashboard |
| `GET /status` | Current readings, network, settings (small JSON) |
| `GET /history` | Downsampled history for the charts (JSON) |
| `GET /history.csv` | Full history, streamed as CSV |
| `GET /sensor/refresh` | Force a read now; returns the `/status` payload |
| `GET /health` | `OK` |
| `GET /update` | ElegantOTA firmware upload |

State-changing routes (`/settings`, `/device/rename`, `/wifi/*`,
`/sensor/refresh`) require the CSRF token handed out in `/status`, passed as
either `?csrf=` or an `X-CSRF-Token` header.

`/status` and `/history` are deliberately separate endpoints. History holds
4032 points, so folding it into every `/status` poll would mean serializing a
several-hundred-KB JSON string on the heap every 15 seconds.

## Readings

Sensors are polled every 5 minutes. History is a 4032-point ring buffer
(4032 × 5 min = **2 weeks**), stored as tenths in `int16_t` rather than floats
to halve the static footprint to ~48 KB — the AHT30 only resolves about
0.024 %RH and 0.01 °C, so nothing is lost.

Alongside raw temperature and humidity the dashboard shows:

- **Dew point** for each side, via the Magnus-Tetens approximation. When the
  outside dew point rises above the inside temperature, opening the container
  will condense moisture on its inner walls.
- **Difference (inside − outside)** for both RH and temperature. A positive
  humidity delta means the container is wetter than the room, which is the
  condition that matters for a dry box.

The inside RH alert threshold (default 20%) only colors the inside reading.
Outside is ambient air and has no "good" or "bad" value.

## Release

`python release.py 1.1` bumps `FIRMWARE_VERSION`, commits, tags, and pushes.
