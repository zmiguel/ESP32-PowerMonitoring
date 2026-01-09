#include <WiFiMulti.h>
#include <PZEM004Tv30.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WebSerialLite.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_sntp.h"
#include "time.h"
#include "esp_task_wdt.h"
#include "config.h"
#include "structures.h"

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define PZEM_SERIAL Serial2

#define TZ_INFO "WET-0WEST-1,M3.5.0/01:00:00,M10.5.0/02:00:00"
#define NTP_SERVER1 "pool.ntp.org"

#define READING_INTERVAL 1000
#define BACKLOG_MAX_SIZE 540
#define MIN_FREE_HEAP 24576
#define SENSOR_READ_ATTEMPTS 2
#define WATCHDOG_TIMEOUT_SEC 5
#define WIFI_RECONNECT_INTERVAL 30000

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
SemaphoreHandle_t timeMutex;
TaskHandle_t BacklogTaskHandle = NULL;
const char* phase_names[] = {"Loja", "Cozinha", "Casa"};
char *metric = "PowerBox";

// Define the linked list node structure
struct PayloadNode {
  Payload payload;
  PayloadNode* next;
};

PayloadNode* backlogHead = nullptr;
PayloadNode* backlogTail = nullptr;
uint16_t backlogSize = 0;
unsigned long lastWiFiCheck = 0;

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

bool ensureIngestConnected() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!parseIngestEndpointOnce()) return false;

  if (ingestClient.connected()) return true;

  ingestClient.stop();
  ingestClient.setNoDelay(true);
  ingestClient.setTimeout(HTTP_TIMEOUT_MS);
  bool ok = ingestClient.connect(ingestHost.c_str(), ingestPort);
  if (ok) {
    // Some stacks require setting this after connect as well.
    ingestClient.setNoDelay(true);
  }
  return ok;
}

int postJsonIngest(const uint8_t* body, size_t bodyLen) {
  if (!ensureIngestConnected()) {
    resetIngestClient();
    return -1;
  }

  char header[320];
  int headerLen = snprintf(
    header,
    sizeof(header),
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

  if (headerLen <= 0 || headerLen >= (int)sizeof(header)) {
    resetIngestClient();
    return -1;
  }

  // Send in a *single* TCP write to avoid Nagle+delayed-ACK latency gaps.
  // (Two back-to-back writes can otherwise produce ~40-60ms jitter on WiFi.)
  const size_t totalLen = (size_t)headerLen + bodyLen;
  if (totalLen > 1500) {
    // Fallback (shouldn't happen with current payload sizes).
    size_t w1 = ingestClient.write((const uint8_t*)header, (size_t)headerLen);
    size_t w2 = ingestClient.write(body, bodyLen);
    if (w1 != (size_t)headerLen || w2 != bodyLen) {
      resetIngestClient();
      return -1;
    }
  } else {
    uint8_t req[1500];
    memcpy(req, header, (size_t)headerLen);
    memcpy(req + (size_t)headerLen, body, bodyLen);
    size_t w = ingestClient.write(req, totalLen);
    if (w != totalLen) {
      resetIngestClient();
      return -1;
    }
  }

  if (!ingestClient.connected()) {
    resetIngestClient();
    return -1;
  }

  // Read status line.
  uint32_t start = millis();
  while (!ingestClient.available()) {
    if (!ingestClient.connected()) {
      resetIngestClient();
      return -1;
    }
    if (millis() - start > HTTP_TIMEOUT_MS) {
      resetIngestClient();
      return -1;
    }
    delay(1);
  }

  char line[96];
  size_t n = ingestClient.readBytesUntil('\n', line, sizeof(line) - 1);
  if (n == 0) {
    resetIngestClient();
    return -1;
  }
  line[n] = '\0';

  // Parse HTTP status code: "HTTP/1.1 201 ..."
  int code = -1;
  char* sp = strchr(line, ' ');
  if (sp != nullptr) {
    code = atoi(sp + 1);
  }

  // Drain headers.
  while (true) {
    n = ingestClient.readBytesUntil('\n', line, sizeof(line) - 1);
    if (n == 0) {
      break;
    }
    line[n] = '\0';
    if (strcmp(line, "\r") == 0) {
      break;
    }
  }

  // Drain any response body quickly (keeps keep-alive clean).
  while (ingestClient.available()) {
    ingestClient.read();
  }

  if (code <= 0) {
    resetIngestClient();
  }

  return code;
}

void ConnectToWiFiMulti() {
  int attempts = 0;
  while (WiFiMulti.run() != WL_CONNECTED && attempts < 50) {
    Serial.print(".");
    delay(100);
    attempts++;
    esp_task_wdt_reset(); // Feed the watchdog while connecting
  }
  if (WiFiMulti.run() != WL_CONNECTED) {
    print_log("WiFi connection failed after 50 attempts, restarting...");
    delay(1000);
    ESP.restart();
  }

  // Reduce latency/jitter: disable WiFi power save (modem sleep).
  WiFi.setSleep(false);

  // Keep the client configured after (re)connect.
  ingestClient.setNoDelay(true);
}

void checkAndReconnectWiFi() {
  unsigned long currentTime = millis();
  if (currentTime - lastWiFiCheck >= WIFI_RECONNECT_INTERVAL) {
    lastWiFiCheck = currentTime;
    if (WiFi.status() != WL_CONNECTED) {
      print_log("WiFi disconnected! Attempting reconnection...");
      WiFi.disconnect();
      delay(100);
      ConnectToWiFiMulti();
      if (WiFi.status() == WL_CONNECTED) {
        print_log("WiFi reconnected successfully!");
		resetIngestClient(); // old keep-alive socket is now stale
      }
    }
  }
}

void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  char d[len+1];
  // Direct assignment - O(n) instead of O(n²)
  for(uint16_t i = 0; i < len; i++){
    d[i] = char(data[i]);
  }
  d[len] = '\0';

  if(strcmp(d, "restart") == 0){
    print_log("Restarting...");
    delay(100);
    ESP.restart();
  }
  if(strcmp(d, "version") == 0){
    print_log("ESP32 FW 2.3.0");
  }
}

void checkFreeAndMax(void) {
  // Check the free memory and remove nodes if necessary
  while (backlogSize >= BACKLOG_MAX_SIZE || heap_caps_get_free_size(MALLOC_CAP_8BIT) < MIN_FREE_HEAP) {
    if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (backlogHead != nullptr) {
        PayloadNode* temp = backlogHead;
        backlogHead = backlogHead->next;
        free(temp);
        temp = nullptr;
        backlogSize--;
        if (backlogHead == nullptr) {
          backlogTail = nullptr;
        }
      }
      xSemaphoreGive(backlogMutex);
    } else {
      print_log("ERROR: Failed to take semaphore FREE");
    }
  }
}

void backlog_remove_oldest() {
  if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    if (backlogHead != nullptr) {
      PayloadNode* temp = backlogHead;
      backlogHead = backlogHead->next;
      free(temp);
      temp = nullptr;
      backlogSize--;
      if (backlogHead == nullptr) {
        backlogTail = nullptr;
      }
    }
    xSemaphoreGive(backlogMutex);
  } else {
    print_log("ERROR: Failed to take semaphore REMOVE");
  }
}

void send_backlog(void * pvParameters){
  // Add this task to the watchdog
  esp_task_wdt_add(NULL);

  while(true){
    // Feed the watchdog
    esp_task_wdt_reset();

    try {
      if (backlogSize == 0) {
        delay(50);
        continue;
      }

      if (backlogSize > 1) {
        char msg[64];
        sprintf(msg, "[C0] Backlog size: %d/%d | Free: %.2f kB", backlogSize, BACKLOG_MAX_SIZE, heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024.0);
        print_log(msg);
      }

      if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (backlogHead == nullptr) {
          print_log("ERROR: backlogHead is null");
          xSemaphoreGive(backlogMutex);
          continue;
        }

        // Copy the payload while holding the mutex
        Payload pl = backlogHead->payload;
        xSemaphoreGive(backlogMutex);

        StaticJsonDocument<1024> doc;
        doc["time"] = pl.time;
        doc["metric"] = pl.metric;
        doc["backlog"] = true;
        doc["esp"]["rssi"] = pl.esp.rssi;
        doc["esp"]["acq_time"] = pl.esp.acq_time;
        for(int i=0; i<3; i++){
          doc["data"][i]["phase"] = pl.data[i].phase;
          doc["data"][i]["voltage"] = pl.data[i].voltage;
          doc["data"][i]["current"] = pl.data[i].current;
          doc["data"][i]["power"] = pl.data[i].power;
          doc["data"][i]["energy"] = pl.data[i].energy;
          doc["data"][i]["frequency"] = pl.data[i].frequency;
          doc["data"][i]["power_factor"] = pl.data[i].power_factor;
        }

        char body[1024];
        size_t bodyLen = serializeJson(doc, body, sizeof(body));
        if (bodyLen == 0 || bodyLen >= sizeof(body)) {
          print_log("ERROR: JSON serialization overflow");
          delay(READING_INTERVAL);
          yield();
          continue;
        }

        int httpCode = postJsonIngest((const uint8_t*)body, bodyLen);

        if(httpCode == 201 || httpCode == 406) {
          // Successfully sent or not acceptable, remove from backlog
          if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (backlogHead != nullptr) {
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
      } else {
        print_log("ERROR: Failed to take semaphore BACKLOG");
      }
    } catch(const std::exception& e) {
      print_log(e.what());
    }
  }
}

void print_log(const char* msg){
  Serial.println(msg);
  WebSerial.println(msg);
}

char* get_time(){
  static char buffer[32];

  // Protect static buffer with mutex for thread-safety
  if (xSemaphoreTake(timeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Get the current time
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    // Convert to struct tm
    struct tm timeinfo;
    gmtime_r(&tv_now.tv_sec, &timeinfo);

    // Format the time into a string
    char strftime_buf[20];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);

    // Add milliseconds
    snprintf(buffer, sizeof(buffer), "%s.%03ldZ", strftime_buf, tv_now.tv_usec / 1000);

    xSemaphoreGive(timeMutex);
  }

  return buffer;
}

void syncTimeCallBack(struct timeval *tv) { // re-sync callback
  print_log("===== Time Updated =====");
  print_log(get_time());
}

void timeSync(){
  print_log("Syncing Time...");
  // sync NTP
  time_t now;
  time(&now);
  setenv("TZ", TZ_INFO, 1);
  tzset();
  configTime(0, 0, NTP_SERVER1);
  esp_sntp_init();
  sntp_set_sync_interval(1 * 60 * 60 * 1000);
  sntp_set_time_sync_notification_cb(syncTimeCallBack);
  sntp_restart();
  // Wait for NTP sync with watchdog feeding
  int sync_attempts = 0;
  while(esp_sntp_get_sync_status()!=SNTP_SYNC_STATUS_COMPLETED && sync_attempts < 100){
    esp_task_wdt_reset(); // Feed watchdog while waiting for NTP
    delay(100);
    sync_attempts++;
    yield();
  }
  if(esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
    print_log("Time sync completed successfully");
  } else {
    print_log("WARNING: Time sync timeout, will sync in background");
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Temporarily disable watchdog during setup
  esp_task_wdt_deinit();

  // Reconfigure watchdog timer with longer timeout for setup
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 20000, // 20 seconds during setup
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); // Add current task to watchdog

  WiFi.mode(WIFI_STA);
  // Reduce latency/jitter: disable WiFi power save (modem sleep).
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.println("Connecting to WiFi");
  Serial.print(SSID);
  Serial.print(" ");
  Serial.println(WiFiPassword);
  WiFiMulti.addAP(SSID, WiFiPassword);

  backlogMutex = xSemaphoreCreateMutex();
  timeMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    send_backlog,
    "BacklogTask",
    10000,
    NULL,
    1,
    &BacklogTaskHandle,
    0
  );

  ConnectToWiFiMulti();
  Serial.print("WiFi Connected! IP: ");
  Serial.println(WiFi.localIP());

  // Start web services
  WebSerial.begin(&server);
  ElegantOTA.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();

  // Give async_tcp task time to initialize
  delay(500);

  // Now sync time
  timeSync();

  // Reconfigure watchdog with shorter timeout for normal operation
  esp_task_wdt_deinit();
  esp_task_wdt_config_t runtime_config = {
    .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&runtime_config);
  esp_task_wdt_add(NULL);

  // Re-add BacklogTask to the new watchdog instance
  if (BacklogTaskHandle != NULL) {
    esp_task_wdt_add(BacklogTaskHandle);
  }

  print_log("Setup complete, starting main loop...");
}

void loop() {
  // Feed the watchdog at the start of each loop
  esp_task_wdt_reset();

  ElegantOTA.loop();
  unsigned long start_time = millis();

  // Periodic WiFi connection check
  checkAndReconnectWiFi();

  // Monitor heap health
  size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  if (freeHeap < (MIN_FREE_HEAP / 2)) {
    char msg[80];
    sprintf(msg, "WARNING: Low heap memory: %.2f kB - Consider restart", freeHeap / 1024.0);
    print_log(msg);
    if (freeHeap < (MIN_FREE_HEAP / 4)) {
      print_log("CRITICAL: Extremely low memory, clearing backlog before restart...");
      // Aggressively clear backlog to free memory
      while (backlogSize > 0) {
        backlog_remove_oldest();
      }
      delay(1000);
      ESP.restart();
    }
  }

  if(backlogSize > 0){
    checkFreeAndMax();
  }

  // init a payload
  Payload pl = {};

  // add base values
  strcpy(pl.time, get_time());
  strcpy(pl.metric, metric);
  // Check WiFi connection before getting RSSI
  pl.esp.rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -100;
  pl.esp.acq_time = last_time;

  //read sensors with retry logic and validation
  for(int i=0;i<3;i++){
    strcpy(pl.data[i].phase, phase_names[i]);
    bool validReading = false;

    // Try to get valid readings with multiple attempts
    for(int attempt = 0; attempt < SENSOR_READ_ATTEMPTS && !validReading; attempt++) {
      // Feed watchdog during attempts
      esp_task_wdt_reset();

      if(attempt > 0) {
        char msg[64];
        sprintf(msg, "Attempt %d for sensor %d (%s)", attempt + 1, i, phase_names[i]);
        print_log(msg);
        delay(100); // Small delay between attempts
      }

      float voltage = pzem[i].voltage();
      float current = pzem[i].current();
      float power = pzem[i].power();
      float energy = pzem[i].energy();
      float frequency = pzem[i].frequency();
      float pf = pzem[i].pf();

      // Check if all critical readings are valid (allow energy to be 0)
      // Relaxed power factor validation for floating point precision
      if(!isnan(voltage) && !isnan(current) && !isnan(power) &&
         !isnan(energy) && !isnan(frequency) && !isnan(pf) &&
         voltage >= 0 && current >= 0 && power >= 0 && energy >= 0 &&
         frequency > 0 && pf >= -0.01 && pf <= 1.01) {
        pl.data[i].voltage = voltage;
        pl.data[i].current = current;
        pl.data[i].power = power;
        pl.data[i].energy = energy;
        pl.data[i].frequency = frequency;
        pl.data[i].power_factor = pf;
        validReading = true;
      }
    }

    if(!validReading) {
      // After all retries failed, mark this sensor's data as invalid
      char msg[80];
      sprintf(msg, "ERROR: Failed to get valid reading from sensor %d (%s) after %d attempts",
              i, phase_names[i], SENSOR_READ_ATTEMPTS);
      print_log(msg);
      // Set all values to NaN to indicate invalid data
      pl.data[i].voltage = NAN;
      pl.data[i].current = NAN;
      pl.data[i].power = NAN;
      pl.data[i].energy = NAN;
      pl.data[i].frequency = NAN;
      pl.data[i].power_factor = NAN;
    }
  }
  // Check if at least one sensor has valid data before adding to backlog
  bool hasValidData = false;
  for(int i=0; i<3; i++) {
    if(!isnan(pl.data[i].voltage) && !isnan(pl.data[i].current) && !isnan(pl.data[i].power)) {
      hasValidData = true;
      break;
    }
  }

  if(!hasValidData) {
    print_log("WARNING: No valid sensor data in this reading cycle, skipping...");
    last_time = millis() - start_time;

    // Ensure we maintain consistent 1-second intervals even when skipping
    unsigned long elapsed = millis() - start_time;
    if(elapsed < READING_INTERVAL) {
      delay(READING_INTERVAL - elapsed);
    } else {
      delay(10);
    }

    // Feed watchdog before returning to ensure it doesn't timeout
    esp_task_wdt_reset();
    return; // Skip adding to backlog if no valid data
  }

  // check connection to wifi
  if (WiFi.status() != WL_CONNECTED) {
    ConnectToWiFiMulti();
  }
  // add payload to backlog
  if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    PayloadNode* newNode = nullptr;
    while (newNode == nullptr) {
      newNode = (PayloadNode*)malloc(sizeof(PayloadNode));
      if (newNode == nullptr) {
        xSemaphoreGive(backlogMutex);
        backlog_remove_oldest();
        if (xSemaphoreTake(backlogMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            print_log("ERROR: Failed to retake semaphore after removing oldest");
            // Maintain timing and feed watchdog before returning
            unsigned long elapsed = millis() - start_time;
            if(elapsed < READING_INTERVAL) {
              delay(READING_INTERVAL - elapsed);
            }
            esp_task_wdt_reset();
            return;
        }
      }
    }
    newNode->payload = pl;
    newNode->next = nullptr;
    if (backlogTail) {
      backlogTail->next = newNode;
      backlogTail = newNode;
    } else {
      backlogHead = newNode;
      backlogTail = newNode;
    }
    backlogSize++;
    xSemaphoreGive(backlogMutex);
  } else {
    print_log("ERROR: Failed to take semaphore MAIN");
  }

  last_time = millis() - start_time;
  char msg[64];
  sprintf(msg, "[Acq Time] %ldms", last_time);
  print_log(msg);

  // Ensure we maintain consistent 1-second intervals
  unsigned long elapsed = millis() - start_time;
  if(elapsed < READING_INTERVAL) {
    delay(READING_INTERVAL - elapsed);
  } else {
    // If we took longer than READING_INTERVAL, yield briefly
    delay(10);
  }
}