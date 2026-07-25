# ESP32-PowerMonitoring

My take on a power measurement device using a ESP32 & PZEM-004T-v30.

Reads three PZEM-004T v3.0 meters once a second and POSTs each sample as JSON to
an HTTP endpoint, buffering in RAM when the network is unavailable.

## Layout

- `PowerBox-3Phase/` — electrical panel sketch
- `SolarMonitoring/` — solar sketch

The two sketches are **byte-identical except for two lines** — `phase_names[]`
and `metric`. Apply every change to both.

`phase_names[]` is matched to meters **by array position**: `phase_names[i]`
labels the readings from `pzem[i]`. Reordering it silently relabels which
meter's data goes where, rather than just changing a string.

## Hardware required

- ESP32 dev board
- 3x PZEM-004T-v30 sensors
- An HTTP endpoint to receive the JSON samples

### Wiring and meter addressing

| Setting | Value |
| --- | --- |
| UART | `Serial2` |
| RX pin | 16 |
| TX pin | 17 |
| Modbus addresses | `0x01`, `0x02`, `0x03` |

All three meters share one bus, so each must be **programmed to its own Modbus
address before use** — PZEM-004T v3.0 units ship with the default address
`0xF8` and three unconfigured meters will collide. Use the library's
`setAddress()` with one meter connected at a time.

## Toolchain

**Arduino-ESP32 core 3.x (ESP-IDF 5.x) is required.** The sketch uses
`esp_task_wdt_config_t` / `esp_task_wdt_reconfigure()` and the `esp_sntp_*` API,
none of which exist in core 2.x.

## Libraries used

| Library | Purpose |
| --- | --- |
| [mandulaj/PZEM-004T-v30](https://github.com/mandulaj/PZEM-004T-v30) | sensor measuring |
| [bblanchon/ArduinoJson](https://github.com/bblanchon/ArduinoJson) | building the JSON payload |
| [ayushsharma82/ElegantOTA](https://github.com/ayushsharma82/ElegantOTA) | OTA updates (v3) |
| [asjdf/WebSerialLite](https://github.com/asjdf/WebSerialLite) | web serial output used for debugging |
| [ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer) | required by WebSerialLite & ElegantOTA |
| [AsyncTCP](https://github.com/ESP32Async/AsyncTCP) | required by ESPAsyncWebServer |

WiFi, FreeRTOS, SNTP and the task watchdog all come from the ESP32 Arduino core
— nothing extra to install. HTTP is hand-rolled over `WiFiClient` (keep-alive,
single-write requests) rather than using `HTTPClient`, so that is not a
dependency either.

Two notes on the async stack:

- **ElegantOTA must be built in async mode.** Set
  `#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1` in the library's `src/ElegantOTA.h`
  (or via a build flag), otherwise `ElegantOTA.begin(&server)` will not compile
  against an `AsyncWebServer`.
- The original `me-no-dev` AsyncTCP / ESPAsyncWebServer repositories are no
  longer maintained; the `ESP32Async` org hosts the current forks. Either works,
  but check which one your library manager installs if you hit build errors.

## Configuration

Copy `config_example.h` to `config.h` in the sketch folder and fill in:

```c
const char *SSID = "";
const char *WiFiPassword = "";
const char *ENDPOINT = "http://host:port/path";
```

`config.h` is gitignored — do not commit credentials. `ENDPOINT` must be plain
`http://`; TLS is not implemented.

## Runtime behaviour

- **Sampling**: one reading per second, paced to an absolute schedule so
  timestamps do not drift with the cost of the work in between.
- **Timestamps**: UTC ISO-8601 with milliseconds, from NTP
  (`pool.ntp.org`, `time.google.com`, `time.cloudflare.com`).
- **Backlog**: samples queue in RAM (up to 1200, roughly 20 minutes) while the
  network or the endpoint is unavailable. The oldest are dropped once the cap or
  the free-heap floor is reached. **It does not survive a reboot.**
- **Retries**: only `5xx` and transport errors are retried. A `3xx`/`4xx` is
  logged and dropped — a permanently-rejected sample left at the head of the
  queue would block every newer one behind it.
- **Bad readings**: a phase that fails to read is omitted from the payload
  rather than sent as `null`; the cycle is skipped only if all three fail.
  A reading of `0` is legitimate (an idle circuit reports 0 W / 0.00 power
  factor) and is never treated as missing.
- **Before NTP syncs**: samples are sent immediately without a timestamp so the
  receiver can stamp them on arrival, and dropped if they cannot be sent. They
  are never queued, since a queued sample would get the wrong time.
- **Recovery**: a 15 s task watchdog, WiFi reconnect every 30 s while down, and
  a reboot only after 25 minutes of downtime — past the backlog window, so a
  short outage never costs buffered data.

## Web endpoints

| Path | Purpose |
| --- | --- |
| `GET /status` | JSON health: uptime, reset reason, free/minimum heap, backlog depth, dropped & sent counters, clock sync, RSSI, acquisition time, last HTTP code, seconds since last successful POST |
| `/update` | ElegantOTA firmware upload |
| `/webserial` | live log output |

`/status` is the one to alert on — `last_ok_s_ago` climbing (or `-1`) means the
device is sampling but not delivering. Note that neither OTA nor WebSerial is
authenticated, so keep these devices on a trusted network.

WebSerial accepts two commands: `restart` and `version`.
