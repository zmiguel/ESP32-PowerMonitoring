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
- **Retry spacing**: the PZEM library serves all six values from one Modbus
  transaction and caches the result for 200 ms, stamping the cache *before* the
  transaction. A retry inside that window would be answered from the cache and
  re-record the previous cycle's values as if they were fresh, so retries are
  spaced past it.
- **Before NTP syncs**: samples are sent immediately without a timestamp so the
  receiver can stamp them on arrival, and dropped if they cannot be sent. They
  are never queued, since a queued sample would get the wrong time.
- **Recovery**: a 15 s task watchdog, WiFi reconnect every 30 s while down, and
  a reboot only after 25 minutes of downtime — past the backlog window, so a
  short outage never costs buffered data.

## Web endpoints

| Path | Purpose |
| --- | --- |
| `GET /status` | JSON health: uptime, reset reason, free/minimum heap, backlog depth, dropped & sent counters, clock sync, RSSI, acquisition time, last HTTP code, seconds since last successful POST, and a per-meter `sensors[]` block |
| `/update` | ElegantOTA firmware upload |
| `/webserial` | live log output |

`/status` is the one to alert on — `last_ok_s_ago` climbing (or `-1`) means the
device is sampling but not delivering, and any `sensors[].ok` false means a
meter has stopped answering. Note that neither OTA nor WebSerial is
authenticated, so keep these devices on a trusted network.

WebSerial commands:

| Command | Purpose |
| --- | --- |
| `restart` | reboot the device |
| `version` | print the firmware version |
| `sensors` | print the per-meter health counters |
| `diag` | probe every meter address and print the raw Modbus reply |

## Diagnosing a failing meter

A failed read is never a single bad data point. The library reads all ten
registers in one Modbus transaction, so either the whole meter is present for
that second or none of it is — and the other two meters are unaffected, since
each is a separate transaction.

Failures are reported as:

```text
SENSOR FAIL 2 (Loja) addr=0x03: no Modbus reply after 2 attempts in 212ms (last txn 100ms) | V=nan I=nan P=nan E=nan Hz=nan PF=nan | consecutive=1 fails=17/48213 reads | last-good=1s ago
```

- `no Modbus reply` — nothing came back, or what came back failed CRC or was
  the wrong length. The values shown are stale cache and mean nothing; they are
  printed only to make that visible. `last txn` tells the two apart: 100 ms is
  the library's read timeout running out in full, so nothing arrived at all,
  while ~30 ms means a frame did arrive and was rejected.
- `implausible values [Hz,PF]` — a frame *did* decode, but the named fields are
  outside a plausible range. Points at bus corruption rather than a dead meter.
- `consecutive` against `fails/reads` is the pair that matters: `consecutive=1`
  with `fails` far below `reads` is an occasional glitch, while `consecutive`
  climbing and `last-good` going stale is a meter that is gone.

Logging is rate limited — the detail goes out when a fault starts and then once
every 30 s while it lasts, because a dead meter fails once a second and
WebSerial queues every message on the async server's heap. `/status` and the
`sensors` command carry the exact counts regardless.

`WARNING: No valid sensor data in this reading cycle` means all three failed at
once, which points at the bus or the ESP32 side rather than at any one meter.

For the byte level, `diag` probes each address directly and prints the raw
reply — whether *anything* answered is the one thing the library never exposes:

```text
--- PZEM bus probe ---
  addr 0x01: 25 bytes | OK - full 25-byte frame, CRC good | 01 04 14 09 0B ...
  addr 0x03: 0 bytes | SILENT - meter unpowered, miswired, or not at this address
```

`SILENT` is a meter that is not there (unpowered, wiring, or a reassigned
address); `GARBLED` is a meter answering into a bus problem. The probe also
checks `0xF8`, the factory default address every meter answers — a garbled reply
there is the healthy result with more than one live meter, while a clean frame
means only one meter is still talking.
