#include <WiFiMulti.h>
#include <PZEM004Tv30.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WebSerialLite.h>
#include <ArduinoJson.h>
#include <strings.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_sntp.h"
#include "esp_system.h"
#include "time.h"
#include "esp_task_wdt.h"
#include "config.h"
#include "structures.h"

#define FW_VERSION "2.5.0"

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define PZEM_SERIAL Serial2

#define TZ_INFO "WET-0WEST-1,M3.5.0/01:00:00,M10.5.0/02:00:00"
#define NTP_SERVER1 "pool.ntp.org"
#define NTP_SERVER2 "time.google.com"
#define NTP_SERVER3 "time.cloudflare.com"

#define READING_INTERVAL 1000
// ~20 minutes at 1 Hz. A backlog node is ~112 bytes, so this ceiling costs
// roughly the same worst-case heap the old 540-deep backlog did with its
// much larger nodes. MIN_FREE_HEAP is still the real limiter.
#define BACKLOG_MAX_SIZE 1200
#define MIN_FREE_HEAP 24576
#define SENSOR_READ_ATTEMPTS 2
// Generous enough to survive a blocking WiFiMulti scan (which does not feed the
// WDT internally) while still catching a genuinely stuck 1 Hz sampling loop.
#define WATCHDOG_TIMEOUT_SEC 15
#define WIFI_RECONNECT_INTERVAL 30000
#define WIFI_CONNECT_TIMEOUT_MS 5000
// Only reboot once the outage has outlasted the backlog window; before that a
// reboot would throw away buffered samples that could still be delivered.
#define WIFI_MAX_DOWNTIME_MS 1500000UL
// 2024-01-01T00:00:00Z. Anything below this means NTP has not synced yet.
#define MIN_VALID_EPOCH 1704067200LL

WiFiMulti WiFiMulti;
WiFiClient ingestClient;

String ingestHost;
uint16_t ingestPort = 80;
String ingestPath = "/";
bool ingestEndpointParsed = false;

static const uint16_t HTTP_TIMEOUT_MS = 1500;
AsyncWebServer server(80);
PZEM004Tv30 pzem[3] = {
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x01),
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x02),
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x03)
};

uint32_t last_time = 1;
SemaphoreHandle_t backlogMutex;
SemaphoreHandle_t logMutex;
SemaphoreHandle_t ingestMutex;
TaskHandle_t BacklogTaskHandle = NULL;
const char* phase_names[] = {"Casa", "Cozinha", "Loja"};
const char* metric = "Solar";

// Request scratch space. Kept out of the task stacks because both the sampling
// task and the backlog task can send; ingestMutex serialises access, so a
// single shared set of buffers is enough and saves ~3.7 kB of stack on each.
static StaticJsonDocument<1024> ingestDoc;
static char ingestBody[1024];
static char ingestHeader[320];
static uint8_t ingestReq[1400];

// Define the linked list node structure
struct PayloadNode {
  Payload payload;
  PayloadNode* next;
  uint32_t seq;  // identifies a node across an unlocked window without dereferencing it
};

PayloadNode* backlogHead = nullptr;
PayloadNode* backlogTail = nullptr;
// Read without the mutex by the pacing/trim checks, so it must not be cached in
// a register.
volatile uint16_t backlogSize = 0;
uint32_t backlogSeq = 0;
unsigned long lastWiFiCheck = 0;
unsigned long wifiDownSince = 0;
uint32_t nextSampleAt = 0;

// Diagnostics, surfaced by /status.
const char* bootReason = "unknown";
volatile uint32_t lastPostOkMs = 0;
volatile int lastHttpCode = 0;
volatile uint32_t droppedSamples = 0;
volatile uint32_t sentSamples = 0;

void print_log(const char* msg);

// ---------------------------------------------------------------------------
// Watchdog helpers
// ---------------------------------------------------------------------------

// The SDK already brings the TWDT up at boot with the idle tasks subscribed, so
// esp_task_wdt_deinit()/init() both fail with ESP_ERR_INVALID_STATE and the
// requested timeout is silently never applied. Reconfigure in place instead.
void configureWatchdog(uint32_t timeout_ms) {
  esp_task_wdt_config_t cfg = {
    .timeout_ms = timeout_ms,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_err_t err = esp_task_wdt_reconfigure(&cfg);
  if (err == ESP_ERR_INVALID_STATE) {
    err = esp_task_wdt_init(&cfg);  // TWDT not running yet
  }
  if (err != ESP_OK) {
    Serial.printf("WARNING: watchdog config failed: %s\n", esp_err_to_name(err));
  }
}

void wdtSubscribe(TaskHandle_t handle) {
  if (esp_task_wdt_status(handle) == ESP_ERR_NOT_FOUND) {
    esp_task_wdt_add(handle);
  }
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

// Current UTC time as Unix epoch milliseconds. Returns false while the clock is
// unsynced, so a sample is never stamped 1970 and written into the database.
bool get_epoch_ms(int64_t* out) {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  if ((int64_t)tv_now.tv_sec < MIN_VALID_EPOCH) return false;
  *out = (int64_t)tv_now.tv_sec * 1000LL + (int64_t)(tv_now.tv_usec / 1000);
  return true;
}

bool clockIsSynced() {
  int64_t ignored;
  return get_epoch_ms(&ignored);
}

// Renders epoch milliseconds as ISO-8601 UTC, e.g. 2026-07-25T22:08:31.123Z.
void format_epoch_ms(int64_t ms, char* out, size_t outSize) {
  time_t secs = (time_t)(ms / 1000LL);
  int millis_part = (int)(ms % 1000LL);
  struct tm timeinfo;
  gmtime_r(&secs, &timeinfo);

  char strftime_buf[24];
  strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  snprintf(out, outSize, "%s.%03dZ", strftime_buf, millis_part);
}

// ---------------------------------------------------------------------------
// HTTP ingest
// ---------------------------------------------------------------------------

void resetIngestClient() {
  ingestClient.stop();
}

bool parseIngestEndpointOnce() {
  if (ingestEndpointParsed) return true;

  String url = ENDPOINT;
  url.trim();
  if (url.startsWith("http://")) {
    url = url.substring(7);
  }

  int slashIdx = url.indexOf('/');
  String hostPort = (slashIdx >= 0) ? url.substring(0, slashIdx) : url;
  ingestPath = (slashIdx >= 0) ? url.substring(slashIdx) : "/";
  if (ingestPath.length() == 0) ingestPath = "/";

  int colonIdx = hostPort.indexOf(':');
  if (colonIdx >= 0) {
    ingestHost = hostPort.substring(0, colonIdx);
    ingestPort = (uint16_t)hostPort.substring(colonIdx + 1).toInt();
  } else {
    ingestHost = hostPort;
    ingestPort = 80;
  }

  ingestHost.trim();
  ingestEndpointParsed = (ingestHost.length() > 0 && ingestPort > 0);
  return ingestEndpointParsed;
}

// Returns true if the connection was already open (i.e. reused keep-alive).
bool ensureIngestConnected(bool* reused) {
  *reused = false;
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!parseIngestEndpointOnce()) return false;

  if (ingestClient.connected()) {
    *reused = true;
    return true;
  }

  ingestClient.stop();
  bool ok = ingestClient.connect(ingestHost.c_str(), ingestPort, HTTP_TIMEOUT_MS);
  if (ok) {
    ingestClient.setNoDelay(true);
  }
  return ok;
}

// Case-insensitive substring search. strcasestr() is a GNU extension that is
// not reliably visible in the ESP32 newlib headers, so roll our own.
bool containsCI(const char* haystack, const char* needle) {
  size_t nlen = strlen(needle);
  if (nlen == 0) return true;
  for (const char* p = haystack; *p != '\0'; p++) {
    if (strncasecmp(p, needle, nlen) == 0) return true;
  }
  return false;
}

// Reads one CRLF-terminated line into `out` (without the line terminator).
// Bounded by an absolute deadline: WiFiClient::setTimeout() takes seconds on
// some cores and milliseconds on others, so Stream::readBytesUntil() could
// otherwise block for many minutes and trip the watchdog.
bool readLineBounded(char* out, size_t outSize, uint32_t deadline) {
  size_t idx = 0;
  while (true) {
    if ((int32_t)(millis() - deadline) >= 0) return false;
    if (ingestClient.available() <= 0) {
      if (!ingestClient.connected()) return false;
      delay(1);
      continue;
    }
    int ch = ingestClient.read();
    if (ch < 0) continue;
    if (ch == '\n') {
      out[idx] = '\0';
      return true;
    }
    if (ch != '\r' && idx + 1 < outSize) {
      out[idx++] = (char)ch;
    }
  }
}

// Discards exactly `length` body bytes. Returns false if it could not.
bool drainBody(long length, uint32_t deadline) {
  uint8_t sink[64];
  while (length > 0) {
    if ((int32_t)(millis() - deadline) >= 0) return false;
    int available = ingestClient.available();
    if (available <= 0) {
      if (!ingestClient.connected()) return false;
      delay(1);
      continue;
    }
    int want = (int)((long)sizeof(sink) < length ? (long)sizeof(sink) : length);
    if (want > available) want = available;
    int got = ingestClient.read(sink, want);
    if (got <= 0) {
      delay(1);
      continue;
    }
    length -= got;
  }
  return true;
}

// Sends one request on the current connection. Returns the HTTP status code,
// or -1 on a transport failure. Caller must hold ingestMutex.
int postJsonOnce(const uint8_t* body, size_t bodyLen, bool* reused) {
  if (!ensureIngestConnected(reused)) {
    resetIngestClient();
    return -1;
  }

  int headerLen = snprintf(
    ingestHeader,
    sizeof(ingestHeader),
    "POST %s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Content-Type: application/json\r\n"
    "Content-Length: %u\r\n"
    "Connection: keep-alive\r\n"
    "\r\n",
    ingestPath.c_str(),
    ingestHost.c_str(),
    (unsigned)bodyLen
  );

  if (headerLen <= 0 || headerLen >= (int)sizeof(ingestHeader)) {
    resetIngestClient();
    return -1;
  }

  // Send in a *single* TCP write to avoid Nagle+delayed-ACK latency gaps.
  // (Two back-to-back writes can otherwise produce ~40-60ms jitter on WiFi.)
  const size_t totalLen = (size_t)headerLen + bodyLen;
  if (totalLen > sizeof(ingestReq)) {
    // Fallback (shouldn't happen with current payload sizes).
    size_t w1 = ingestClient.write((const uint8_t*)ingestHeader, (size_t)headerLen);
    size_t w2 = ingestClient.write(body, bodyLen);
    if (w1 != (size_t)headerLen || w2 != bodyLen) {
      resetIngestClient();
      return -1;
    }
  } else {
    memcpy(ingestReq, ingestHeader, (size_t)headerLen);
    memcpy(ingestReq + (size_t)headerLen, body, bodyLen);
    size_t w = ingestClient.write(ingestReq, totalLen);
    if (w != totalLen) {
      resetIngestClient();
      return -1;
    }
  }

  const uint32_t deadline = millis() + HTTP_TIMEOUT_MS;
  char line[128];

  // Status line: "HTTP/1.1 201 Created"
  if (!readLineBounded(line, sizeof(line), deadline)) {
    resetIngestClient();
    return -1;
  }
  int code = -1;
  char* sp = strchr(line, ' ');
  if (sp != nullptr) code = atoi(sp + 1);
  if (code <= 0) {
    resetIngestClient();
    return -1;
  }

  // Headers. Content-Length is required to know where the body ends: draining
  // only what has already arrived leaves the tail of an error body in the
  // socket, which the next response then parses as its status line.
  long contentLength = -1;
  bool chunked = false;
  bool closeConn = false;
  bool headersComplete = false;
  while (readLineBounded(line, sizeof(line), deadline)) {
    if (line[0] == '\0') {
      headersComplete = true;
      break;
    }
    if (strncasecmp(line, "Content-Length:", 15) == 0) {
      contentLength = strtol(line + 15, nullptr, 10);
    } else if (strncasecmp(line, "Transfer-Encoding:", 18) == 0) {
      chunked = true;
    } else if (strncasecmp(line, "Connection:", 11) == 0 &&
               containsCI(line + 11, "close")) {
      closeConn = true;
    }
  }

  bool bodiless = (code == 204 || code == 304);
  if (!headersComplete) {
    resetIngestClient();
  } else if (bodiless || contentLength == 0) {
    // Nothing to drain.
  } else if (chunked || contentLength < 0) {
    // Body length is not knowable cheaply; the socket can no longer be reused.
    resetIngestClient();
  } else if (!drainBody(contentLength, deadline)) {
    resetIngestClient();
  }

  if (closeConn) resetIngestClient();

  return code;
}

// Caller must hold ingestMutex.
int postJsonIngest(const uint8_t* body, size_t bodyLen) {
  bool reused = false;
  int code = postJsonOnce(body, bodyLen, &reused);
  // A keep-alive socket the server closed while idle fails exactly once; the
  // retry runs on a guaranteed-fresh connection.
  if (code < 0 && reused) {
    resetIngestClient();
    code = postJsonOnce(body, bodyLen, &reused);
  }
  return code;
}

// Builds the request body into ingestBody. Returns its length, or 0 if the
// document could not be built completely. Caller must hold ingestMutex.
//
// `backlog` tells the server whether it may stamp the row on arrival: false
// means "this is live, use your own clock", which is only correct for a payload
// being sent the instant it was sampled.
size_t buildIngestBody(const Payload& pl, bool backlog) {
  ingestDoc.clear();

  char timebuf[32];
  if (pl.time_ms > 0) {
    format_epoch_ms(pl.time_ms, timebuf, sizeof(timebuf));
    ingestDoc["time"] = timebuf;  // char* -> ArduinoJson copies it
  }
  // const char* is stored by reference, so these cost no pool memory.
  ingestDoc["metric"] = metric;
  ingestDoc["backlog"] = backlog;
  ingestDoc["esp"]["rssi"] = pl.esp.rssi;
  ingestDoc["esp"]["acq_time"] = pl.esp.acq_time;

  // Phases that failed to read are omitted entirely. Sending them as NaN makes
  // ArduinoJson emit `null`, which the ingest API rejects as a missing required
  // field - taking the two good phases down with the bad one.
  int emitted = 0;
  for (int i = 0; i < 3; i++) {
    if (!pl.data[i].valid) continue;
    ingestDoc["data"][emitted]["phase"] = phase_names[i];
    ingestDoc["data"][emitted]["voltage"] = pl.data[i].voltage;
    ingestDoc["data"][emitted]["current"] = pl.data[i].current;
    ingestDoc["data"][emitted]["power"] = pl.data[i].power;
    ingestDoc["data"][emitted]["energy"] = pl.data[i].energy;
    ingestDoc["data"][emitted]["frequency"] = pl.data[i].frequency;
    ingestDoc["data"][emitted]["power_factor"] = pl.data[i].power_factor;
    emitted++;
  }

  // If the document pool ever runs out, ArduinoJson drops members silently and
  // we would ship a payload with missing fields. Verify before sending.
  JsonArrayConst builtData = ingestDoc["data"].as<JsonArrayConst>();
  bool docOk = (emitted > 0) && ((int)builtData.size() == emitted);
  for (int k = 0; docOk && k < emitted; k++) {
    if (builtData[k].as<JsonObjectConst>().size() != 7) docOk = false;
  }
  if (!docOk) return 0;

  size_t len = serializeJson(ingestDoc, ingestBody, sizeof(ingestBody));
  if (len == 0 || len >= sizeof(ingestBody)) return 0;
  return len;
}

// Serializes and POSTs one payload. Returns the HTTP status code, or:
//   -1  transport failure          (retry)
//   -2  body could not be built    (do not retry - it never will build)
//   -3  another task holds the client (retry)
int sendPayload(const Payload& pl, bool backlog, uint32_t waitMs) {
  if (xSemaphoreTake(ingestMutex, pdMS_TO_TICKS(waitMs)) != pdTRUE) return -3;

  size_t bodyLen = buildIngestBody(pl, backlog);
  if (bodyLen == 0) {
    xSemaphoreGive(ingestMutex);
    return -2;
  }

  int code = postJsonIngest((const uint8_t*)ingestBody, bodyLen);
  xSemaphoreGive(ingestMutex);

  lastHttpCode = code;
  if (code >= 200 && code < 300) {
    lastPostOkMs = millis();
    sentSamples++;
  }
  return code;
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------

bool connectToWiFi(uint32_t maxWaitMs) {
  uint32_t start = millis();
  while (millis() - start < maxWaitMs) {
    esp_task_wdt_reset();
    if (WiFiMulti.run() == WL_CONNECTED) {
      // Reduce latency/jitter: disable WiFi power save (modem sleep).
      WiFi.setSleep(false);
      ingestClient.setNoDelay(true);
      wifiDownSince = 0;
      return true;
    }
    delay(100);
  }
  return false;
}

void checkAndReconnectWiFi() {
  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    wifiDownSince = 0;
    return;
  }

  if (wifiDownSince == 0) wifiDownSince = now;
  if (now - lastWiFiCheck < WIFI_RECONNECT_INTERVAL) return;
  lastWiFiCheck = now;

  print_log("WiFi disconnected! Attempting reconnection...");
  resetIngestClient();  // old keep-alive socket is now stale
  WiFi.disconnect();
  delay(100);

  if (connectToWiFi(WIFI_CONNECT_TIMEOUT_MS)) {
    print_log("WiFi reconnected successfully!");
    resetIngestClient();
    return;
  }

  // Keep sampling into the backlog rather than rebooting: a reboot loses every
  // buffered sample. Only give up once the outage outlasts the backlog window.
  if (now - wifiDownSince > WIFI_MAX_DOWNTIME_MS) {
    print_log("WiFi down beyond backlog window, restarting...");
    delay(1000);
    ESP.restart();
  }
}

// ---------------------------------------------------------------------------
// Backlog
// ---------------------------------------------------------------------------

bool backlog_remove_oldest() {
  if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    print_log("ERROR: Failed to take semaphore REMOVE");
    return false;
  }
  if (backlogHead != nullptr) {
    PayloadNode* temp = backlogHead;
    backlogHead = backlogHead->next;
    backlogSize--;
    droppedSamples++;
    if (backlogHead == nullptr) {
      backlogTail = nullptr;
    }
    free(temp);
  }
  xSemaphoreGive(backlogMutex);
  return true;
}

void checkFreeAndMax(void) {
  // Trim the oldest samples until we are back under the size cap and above the
  // heap floor. The empty-backlog break is what stops this spinning forever
  // when the heap pressure comes from somewhere else (WiFi, the async server).
  while (backlogSize >= BACKLOG_MAX_SIZE ||
         heap_caps_get_free_size(MALLOC_CAP_8BIT) < MIN_FREE_HEAP) {
    esp_task_wdt_reset();
    if (backlogSize == 0) break;
    if (!backlog_remove_oldest()) break;
  }
}

void send_backlog(void* pvParameters) {
  wdtSubscribe(NULL);
  uint32_t lastSizeLog = 0;

  while (true) {
    esp_task_wdt_reset();

    if (backlogSize == 0) {
      delay(50);
      continue;
    }

    // Rate-limited: WebSerial queues every message on the async server's heap,
    // so logging once per POST while draining the backlog is itself a way to
    // run out of memory.
    if (backlogSize > 1 && millis() - lastSizeLog > 5000) {
      lastSizeLog = millis();
      char msg[80];
      snprintf(msg, sizeof(msg), "[C0] Backlog size: %u/%d | Free: %.2f kB",
               (unsigned)backlogSize, BACKLOG_MAX_SIZE,
               heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024.0);
      print_log(msg);
    }

    if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
      print_log("ERROR: Failed to take semaphore BACKLOG");
      continue;
    }
    if (backlogHead == nullptr) {
      xSemaphoreGive(backlogMutex);
      delay(50);
      continue;
    }
    // Copy the payload while holding the mutex.
    Payload pl = backlogHead->payload;
    uint32_t sentSeq = backlogHead->seq;
    xSemaphoreGive(backlogMutex);

    int httpCode = sendPayload(pl, true, 3000);

    // Only transport errors and 5xx are worth retrying. A 3xx/4xx will fail
    // identically forever, and keeping it at the head of the queue blocks every
    // newer sample behind it - which is how one bad sample stalls all ingest.
    bool drop;
    if (httpCode >= 200 && httpCode < 300) {
      drop = true;
    } else if (httpCode == -2) {
      // Could not build the body at all; retrying cannot help.
      print_log("ERROR: could not build a complete payload, dropping sample");
      drop = true;
    } else if (httpCode < 0 || (httpCode >= 500 && httpCode < 600)) {
      drop = false;
    } else {
      char msg[80];
      snprintf(msg, sizeof(msg), "ERROR: server rejected sample (HTTP %d), dropping", httpCode);
      print_log(msg);
      drop = true;
    }

    if (drop) {
      if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // The trimmer may have dropped this node while we were posting; the
        // sequence number makes sure we never discard a different, unsent one.
        if (backlogHead != nullptr && backlogHead->seq == sentSeq) {
          PayloadNode* temp = backlogHead;
          backlogHead = backlogHead->next;
          backlogSize--;
          if (backlogHead == nullptr) {
            backlogTail = nullptr;
          }
          free(temp);
        }
        xSemaphoreGive(backlogMutex);
      } else {
        print_log("ERROR: Failed to take semaphore BACKLOG");
      }
    } else {
      delay(READING_INTERVAL);
    }
    yield();
  }
}

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

void print_log(const char* msg) {
  // WebSerialLite writes through AsyncWebSocket, which is not safe to call from
  // the sampling task and the backlog task at the same time.
  bool locked = (logMutex != NULL) && (xSemaphoreTake(logMutex, pdMS_TO_TICKS(200)) == pdTRUE);
  Serial.println(msg);
  WebSerial.println(msg);
  if (locked) xSemaphoreGive(logMutex);
}

void syncTimeCallBack(struct timeval* tv) {  // re-sync callback
  int64_t now_ms;
  print_log("===== Time Updated =====");
  if (get_epoch_ms(&now_ms)) {
    char buf[32];
    format_epoch_ms(now_ms, buf, sizeof(buf));
    print_log(buf);
  }
}

void timeSync() {
  print_log("Syncing Time...");
  setenv("TZ", TZ_INFO, 1);
  tzset();
  // Three servers: a single unreachable pool (or a DNS failure) would otherwise
  // leave the clock unsynced, and unsynced samples cannot be backlogged.
  configTime(0, 0, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
  esp_sntp_init();
  sntp_set_sync_interval(1 * 60 * 60 * 1000);
  sntp_set_time_sync_notification_cb(syncTimeCallBack);
  sntp_restart();
  // Wait for NTP sync with watchdog feeding
  int sync_attempts = 0;
  while (esp_sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED && sync_attempts < 100) {
    esp_task_wdt_reset();  // Feed watchdog while waiting for NTP
    delay(100);
    sync_attempts++;
    yield();
  }
  if (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
    print_log("Time sync completed successfully");
  } else {
    print_log("WARNING: Time sync timeout, sending live samples server-stamped");
  }
}

// ---------------------------------------------------------------------------
// Web endpoints
// ---------------------------------------------------------------------------

void recvMsg(uint8_t* data, size_t len) {
  // Runs on the async_tcp task; the message length is remote-controlled, so it
  // must not size a stack buffer.
  char d[32];
  size_t n = (len < sizeof(d) - 1) ? len : sizeof(d) - 1;
  memcpy(d, data, n);
  d[n] = '\0';

  if (strcmp(d, "restart") == 0) {
    print_log("Restarting...");
    delay(100);
    ESP.restart();
  }
  if (strcmp(d, "version") == 0) {
    print_log("ESP32 FW " FW_VERSION);
  }
}

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external-pin";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt-watchdog";
    case ESP_RST_TASK_WDT:  return "task-watchdog";
    case ESP_RST_WDT:       return "other-watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep-wake";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
  }
}

// Machine-readable health, so a stalled device is alertable instead of
// something you have to notice on WebSerial.
void handleStatus(AsyncWebServerRequest* request) {
  StaticJsonDocument<512> doc;
  doc["fw"] = FW_VERSION;
  doc["metric"] = metric;
  doc["uptime_s"] = (uint32_t)(millis() / 1000);
  doc["reset_reason"] = bootReason;
  doc["free_heap"] = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT);
  doc["min_free_heap"] = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  doc["backlog"] = (uint32_t)backlogSize;
  doc["backlog_max"] = BACKLOG_MAX_SIZE;
  doc["dropped"] = droppedSamples;
  doc["sent"] = sentSamples;
  doc["clock_synced"] = clockIsSynced();
  doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  doc["rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  doc["acq_ms"] = last_time;
  doc["last_http_code"] = lastHttpCode;
  // -1 until the first successful POST, so an alert can fire on "never" too.
  doc["last_ok_s_ago"] = lastPostOkMs == 0 ? -1 : (int32_t)((millis() - lastPostOkMs) / 1000);

  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(100);

  bootReason = resetReasonName(esp_reset_reason());
  Serial.printf("\n%s FW %s | reset reason: %s\n", metric, FW_VERSION, bootReason);

  // Longer timeout during setup: WiFi association and NTP both stall for a
  // while here.
  configureWatchdog(20000);
  wdtSubscribe(NULL);

  WiFi.mode(WIFI_STA);
  // Reduce latency/jitter: disable WiFi power save (modem sleep).
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.print("Connecting to WiFi: ");
  Serial.println(SSID);
  WiFiMulti.addAP(SSID, WiFiPassword);

  backlogMutex = xSemaphoreCreateMutex();
  logMutex = xSemaphoreCreateMutex();
  ingestMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    send_backlog,
    "BacklogTask",
    8192,
    NULL,
    1,
    &BacklogTaskHandle,
    0
  );

  // Serial, not print_log: WebSerial has not been started yet.
  if (!connectToWiFi(20000)) {
    Serial.println("WiFi connection failed at boot, restarting...");
    delay(1000);
    ESP.restart();
  }
  Serial.print("WiFi Connected! IP: ");
  Serial.println(WiFi.localIP());

  // Start web services
  server.on("/status", HTTP_GET, handleStatus);
  WebSerial.begin(&server);
  ElegantOTA.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();

  // Give async_tcp task time to initialize
  delay(500);

  // Now sync time
  timeSync();

  configureWatchdog(WATCHDOG_TIMEOUT_SEC * 1000);
  wdtSubscribe(NULL);
  if (BacklogTaskHandle != NULL) {
    wdtSubscribe(BacklogTaskHandle);
  }

  print_log("Setup complete, starting main loop...");
}

// Paces the loop to an absolute 1 Hz schedule so sample timestamps do not drift
// with the cost of the work in between.
void waitForNextSample() {
  uint32_t now = millis();
  if (nextSampleAt == 0 || (int32_t)(now - nextSampleAt) > (int32_t)(5 * READING_INTERVAL)) {
    nextSampleAt = now;  // first run, or we fell far behind
  }
  while (true) {
    esp_task_wdt_reset();
    int32_t remaining = (int32_t)(nextSampleAt - millis());
    if (remaining <= 0) break;
    delay(remaining > 50 ? 50 : remaining);
  }
  nextSampleAt += READING_INTERVAL;
}

void loop() {
  // Feed the watchdog at the start of each loop
  esp_task_wdt_reset();

  ElegantOTA.loop();
  waitForNextSample();
  uint32_t start_time = millis();

  // Periodic WiFi connection check
  checkAndReconnectWiFi();

  // Monitor heap health
  size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  if (freeHeap < (MIN_FREE_HEAP / 2)) {
    char msg[80];
    snprintf(msg, sizeof(msg), "WARNING: Low heap memory: %.2f kB - Consider restart", freeHeap / 1024.0);
    print_log(msg);
    if (freeHeap < (MIN_FREE_HEAP / 4)) {
      print_log("CRITICAL: Extremely low memory, clearing backlog before restart...");
      // Aggressively clear backlog to free memory
      while (backlogSize > 0) {
        esp_task_wdt_reset();
        if (!backlog_remove_oldest()) break;
      }
      delay(1000);
      ESP.restart();
    }
  }

  if (backlogSize > 0) {
    checkFreeAndMax();
  }

  // init a payload
  Payload pl = {};

  // add base values
  int64_t now_ms = 0;
  bool haveClock = get_epoch_ms(&now_ms);
  pl.time_ms = haveClock ? now_ms : 0;
  // Check WiFi connection before getting RSSI
  pl.esp.rssi = (WiFi.status() == WL_CONNECTED) ? (int16_t)WiFi.RSSI() : -100;
  pl.esp.acq_time = last_time;

  //read sensors with retry logic and validation
  for (int i = 0; i < 3; i++) {
    // Try to get valid readings with multiple attempts
    for (int attempt = 0; attempt < SENSOR_READ_ATTEMPTS && !pl.data[i].valid; attempt++) {
      // Feed watchdog during attempts
      esp_task_wdt_reset();

      if (attempt > 0) {
        char msg[80];
        snprintf(msg, sizeof(msg), "Attempt %d for sensor %d (%s)", attempt + 1, i, phase_names[i]);
        print_log(msg);
        delay(100);  // Small delay between attempts
      }

      // All three meters share one RS485 bus. A late reply from the previous
      // address is still sitting in the RX FIFO and would be consumed as the
      // head of this meter's response, so start each transaction clean.
      while (PZEM_SERIAL.available()) PZEM_SERIAL.read();

      float voltage = pzem[i].voltage();
      float current = pzem[i].current();
      float power = pzem[i].power();
      float energy = pzem[i].energy();
      float frequency = pzem[i].frequency();
      float pf = pzem[i].pf();

      // Check if all critical readings are valid (allow zero readings: an idle
      // inverter legitimately reports 0 W / 0.00 pf).
      // isfinite() also rejects the infinities that a corrupted frame can decode to.
      if (isfinite(voltage) && isfinite(current) && isfinite(power) &&
          isfinite(energy) && isfinite(frequency) && isfinite(pf) &&
          voltage >= 0 && current >= 0 && power >= 0 && energy >= 0 &&
          frequency > 0 && pf >= -0.01 && pf <= 1.01) {
        pl.data[i].voltage = voltage;
        pl.data[i].current = current;
        pl.data[i].power = power;
        pl.data[i].energy = energy;
        pl.data[i].frequency = frequency;
        pl.data[i].power_factor = pf;
        pl.data[i].valid = true;
      }
    }

    if (!pl.data[i].valid) {
      char msg[128];
      snprintf(msg, sizeof(msg),
               "ERROR: Failed to get valid reading from sensor %d (%s) after %d attempts",
               i, phase_names[i], SENSOR_READ_ATTEMPTS);
      print_log(msg);
    }
  }

  // Check if at least one sensor has valid data before adding to backlog
  bool hasValidData = false;
  for (int i = 0; i < 3; i++) {
    if (pl.data[i].valid) {
      hasValidData = true;
      break;
    }
  }

  if (!hasValidData) {
    print_log("WARNING: No valid sensor data in this reading cycle, skipping...");
    last_time = millis() - start_time;
    return;  // Skip adding to backlog if no valid data
  }

  if (!haveClock) {
    // No timestamp of our own, so the server has to stamp it on arrival. That
    // is only correct if it goes out *now* - queueing it would attach the time
    // it was eventually drained, not the time it was sampled. If it cannot be
    // sent immediately it is dropped rather than stored with a wrong time.
    int code = sendPayload(pl, false, 2000);
    static uint32_t lastClockWarn = 0;
    if (lastClockWarn == 0 || millis() - lastClockWarn > 30000) {
      lastClockWarn = millis();
      char msg[96];
      snprintf(msg, sizeof(msg),
               "Clock unsynced: sending live server-stamped (last result %d)", code);
      print_log(msg);
    }
    if (!(code >= 200 && code < 300)) droppedSamples++;
    last_time = millis() - start_time;
    return;
  }

  // add payload to backlog
  if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    PayloadNode* newNode = (PayloadNode*)malloc(sizeof(PayloadNode));
    if (newNode == nullptr) {
      // Out of heap: free the oldest sample and take the slot for this one.
      xSemaphoreGive(backlogMutex);
      backlog_remove_oldest();
      if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        print_log("ERROR: Failed to retake semaphore after removing oldest");
        return;
      }
      newNode = (PayloadNode*)malloc(sizeof(PayloadNode));
    }

    if (newNode == nullptr) {
      print_log("ERROR: out of memory, dropping sample");
      droppedSamples++;
    } else {
      newNode->payload = pl;
      newNode->next = nullptr;
      newNode->seq = ++backlogSeq;
      if (backlogTail) {
        backlogTail->next = newNode;
        backlogTail = newNode;
      } else {
        backlogHead = newNode;
        backlogTail = newNode;
      }
      backlogSize++;
    }
    xSemaphoreGive(backlogMutex);
  } else {
    print_log("ERROR: Failed to take semaphore MAIN");
  }

  last_time = millis() - start_time;
  char msg[64];
  snprintf(msg, sizeof(msg), "[Acq Time] %lums", (unsigned long)last_time);
  print_log(msg);
}
