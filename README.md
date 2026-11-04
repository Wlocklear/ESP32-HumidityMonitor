# Humidity Monitor

Reads temperature and relative humidity **inside** and **outside** a container with two AHT30s
and shows them on a web dashboard. Monitor only — for heater control see
[HumidityController](https://github.com/Wlocklear/ESP32-HumidityController).

## Hardware

ESP32-S3-DevKitC-1 only (needs two hardware I2C controllers — both AHT30s are fixed at 0x38).

| | Bus | SDA | SCL |
|---|---|---|---|
| Inside AHT30 | `Wire` (I2C0) | GPIO 8 | GPIO 9 |
| Outside AHT30 | `Wire1` (I2C1) | GPIO 10 | GPIO 11 |

VIN → 3V3, GND → GND. Each bus needs pull-ups (most breakouts have them).

## Readings

- Polled every 5 minutes; 2 weeks of history (4032 points).
- Dashboard: temperature, RH and dew point per side, plus inside − outside differences.
  When the outside dew point rises above the inside temperature, opening the box will condense moisture inside.
- The inside RH alert threshold (default 20%) colors the inside reading only.

## HTTP API

| Route | Purpose |
|---|---|
| `GET /` | Dashboard |
| `GET /status` | Readings, network, settings (JSON) |
| `GET /history` | Downsampled history (JSON) |
| `GET /history.csv` | Full history (CSV) |
| `GET /sensor/refresh` | Read now; returns `/status` |
| `GET /health` | `OK` |
| `GET /update` | ElegantOTA firmware upload |

`/settings`, `/device/rename`, `/wifi/*` and `/sensor/refresh` need the CSRF token from
`/status` (`?csrf=` or `X-CSRF-Token` header).

## Wi-Fi

First boot opens the access point `ESP32-Setup-XXXX`. Join it, choose your network; the device
then runs at `http://humiditymonitor.local`.

## Build and upload

```bash
pio run -e esp32s3 -t upload        # USB
pio run -e esp32s3-ota -t upload    # over Wi-Fi
```

## Releasing

`python release.py <X.Y>` — bumps `FW_VERSION` in `include/version.h`, commits, tags and pushes.
