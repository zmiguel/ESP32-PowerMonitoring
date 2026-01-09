# Copilot instructions (PowerBox-3Phase)

## Project snapshot
- Arduino sketch for ESP32 that reads **3x PZEM-004T v3.0** power meters and ships JSON samples to an HTTP endpoint.
- Main runtime pieces are all in the sketch; there is no multi-module architecture.

## Key files to read first
- [PowerBox-3Phase.ino](../PowerBox-3Phase.ino): core logic (WiFi, NTP, watchdog, sensor reads, backlog queue, HTTP POST).
- [structures.h](../structures.h): `Payload` / `Data` / `Esp` structs used across producer + sender.
- [config.h](../config.h) + [config_example.h](../config_example.h): WiFi creds + `ENDPOINT` (treat as secrets).
- [debug_custom.json](../debug_custom.json), [debug.cfg](../debug.cfg), [debug.svd](../debug.svd): JTAG/OpenOCD debug setup.

## Data flow (important when changing schemas)
- `loop()` samples sensors once per `READING_INTERVAL` (default 1000ms), populates `Payload`, then enqueues it into a **linked-list backlog** protected by `backlogMutex`.
- `send_backlog()` is a FreeRTOS task pinned to core 0 that dequeues (oldest-first) and `HTTPClient::POST()`s JSON to `ENDPOINT`.
- POST success criteria are currently `201` or `406` → remove from backlog; otherwise keep and retry later.
- If you add/rename payload fields: update both `structures.h` AND the JSON serialization in `send_backlog()`.

## Conventions and gotchas (project-specific)
- Logging: use `print_log(...)` so messages go to both Serial and WebSerial.
- Time: `get_time()` returns UTC ISO-8601 with milliseconds (e.g. `YYYY-MM-DDTHH:MM:SS.mmmZ`) and is protected by `timeMutex`.
- Reliability features are intentional: watchdog (`esp_task_wdt_*`), periodic WiFi reconnect, sensor read retries + validation, and aggressive backlog trimming when heap is low.
- Memory: backlog nodes are `malloc`’d and `free`’d; always hold `backlogMutex` when touching `backlogHead/backlogTail/backlogSize`.
- Sensor validation: invalid sensor reads are written as `NAN` and the whole cycle is skipped if **all 3** sensors are invalid.

## Typical workflows
- Configure secrets by copying [config_example.h](../config_example.h) → `config.h` (do not commit real credentials/endpoints).
- Build/flash with Arduino tooling for ESP32 (Arduino IDE/Arduino CLI) targeting `PowerBox-3Phase/PowerBox-3Phase.ino`.
- Debug (JTAG/OpenOCD): start OpenOCD with [debug.cfg](../debug.cfg) and attach using the settings in [debug_custom.json](../debug_custom.json) (breakpoint is placed at `setup`).
