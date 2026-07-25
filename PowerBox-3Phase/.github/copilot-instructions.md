# Copilot instructions (PowerBox-3Phase)

## Project snapshot
- Arduino sketch for ESP32 that reads **3x PZEM-004T v3.0** power meters and ships JSON samples to an HTTP endpoint.
- Main runtime pieces are all in the sketch; there is no multi-module architecture.

## Key files to read first
- [PowerBox-3Phase.ino](../PowerBox-3Phase.ino): core logic (WiFi, NTP, watchdog, sensor reads, backlog queue, HTTP POST).
- [structures.h](../structures.h): `Payload` / `Data` / `Esp` structs used across producer + sender. Nothing constant is stored per sample — the phase name comes from `phase_names[]` **indexed by position in `data`**, the metric from `metric`, and the timestamp is epoch-ms. Node size sets backlog depth, so do not add per-sample copies of constants.
- [config.h](../config.h) + [config_example.h](../config_example.h): WiFi creds + `ENDPOINT` (treat as secrets).
- [debug_custom.json](../debug_custom.json), [debug.cfg](../debug.cfg), [debug.svd](../debug.svd): JTAG/OpenOCD debug setup.

## Data flow (important when changing schemas)
- `loop()` samples sensors once per `READING_INTERVAL` (default 1000ms), populates `Payload`, then enqueues it into a **linked-list backlog** protected by `backlogMutex`.
- `send_backlog()` is a FreeRTOS task pinned to core 0 that dequeues (oldest-first) and POSTs JSON to `ENDPOINT` over a hand-rolled keep-alive `WiFiClient` (`postJsonIngest()`).
- Retry policy: `2xx` and `3xx/4xx` → remove from backlog; only `5xx` and transport errors (`-1`) are retried. A permanently-rejected sample must never stay at the head, or it blocks every newer sample behind it.
- If you add/rename payload fields: update both `structures.h` AND `buildIngestBody()`, and keep the field-count check next to it in sync.
- A sample taken before NTP sync has `time_ms == 0`. It is POSTed **immediately** with `backlog=false` so the server stamps it on arrival, and is dropped if that fails — never queued, because queueing would attach the drain time rather than the sample time.
- `ingestMutex` guards the shared request buffers (`ingestDoc`/`ingestBody`/`ingestHeader`/`ingestReq`) and `ingestClient`; both the sampling task and the backlog task send. Everything below `sendPayload()` assumes the caller holds it.
- `GET /status` returns JSON health (uptime, reset reason, heap, backlog depth, last successful POST). Keep it cheap and lock-free — it runs on the async_tcp task.

## Conventions and gotchas (project-specific)
- Logging: use `print_log(...)` so messages go to both Serial and WebSerial.
- Time: `format_time(buf, len)` writes UTC ISO-8601 with milliseconds (e.g. `YYYY-MM-DDTHH:MM:SS.mmmZ`) into a caller-supplied buffer, and returns `false` while the clock is still unsynced. Never hand out a shared static buffer here — the sampling task and the SNTP callback both call it.
- Reliability features are intentional: watchdog (`esp_task_wdt_*`), periodic WiFi reconnect, sensor read retries + validation, and aggressive backlog trimming when heap is low.
- Memory: backlog nodes are `malloc`’d and `free`’d; always hold `backlogMutex` when touching `backlogHead/backlogTail/backlogSize`.
- Sensor validation: a phase that fails to read is flagged `valid = false` and **omitted from the JSON**; the whole cycle is skipped if all 3 are invalid. Never serialize `NAN` — ArduinoJson emits it as `null` and the ingest API rejects the entire payload.
- A reading of `0` is legitimate (an idle circuit reports 0 W / 0.00 power factor), so validation checks `isfinite()` and ranges, never "non-zero".

## Typical workflows
- Configure secrets by copying [config_example.h](../config_example.h) → `config.h` (do not commit real credentials/endpoints).
- Build/flash with Arduino tooling for ESP32 (Arduino IDE/Arduino CLI) targeting `PowerBox-3Phase/PowerBox-3Phase.ino`.
- Debug (JTAG/OpenOCD): start OpenOCD with [debug.cfg](../debug.cfg) and attach using the settings in [debug_custom.json](../debug_custom.json) (breakpoint is placed at `setup`).
