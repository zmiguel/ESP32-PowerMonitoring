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

WiFiMulti WiFiMulti;
HTTPClient http;
AsyncWebServer server(80);
PZEM004Tv30 pzem[3] = {
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x01),
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x02),
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x03)
};

uint32_t last_time = 1;
SemaphoreHandle_t backlogMutex;
const char* phase_names[] = {"Casa", "Cozinha", "Loja"};
char *metric = "Solar";

bool ntp_synced = false;
time_t now;
char strftime_buf[64];
struct tm timeinfo;

// Define the linked list node structure
struct PayloadNode {
  Payload payload;
  PayloadNode* next;
};

PayloadNode* backlogHead = nullptr;
PayloadNode* backlogTail = nullptr;
uint16_t backlogSize = 0;

void ConnectToWiFiMulti() {
  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
}

void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  char d[len+1] = "";
  for(uint16_t i = 0; i < len; i++){
    char temp[2] = {char(data[i]), '\0'};
    strcat(d, temp);
  }
  if(strcmp(d, "restart") == 0){
    print_log("Restarting...");
    delay(100);
    ESP.restart();
  }
}

void checkFreeAndMax(void) {
  // Check the free memory and remove nodes if necessary
  while (backlogSize >= BACKLOG_MAX_SIZE || heap_caps_get_free_size(MALLOC_CAP_8BIT) < MIN_FREE_HEAP) {
    if (xSemaphoreTake(backlogMutex, portMAX_DELAY) == pdTRUE) {
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
  if (xSemaphoreTake(backlogMutex, portMAX_DELAY) == pdTRUE) {
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
  while(true){
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

      if (xSemaphoreTake(backlogMutex, portMAX_DELAY) == pdTRUE) {
        if (backlogHead == nullptr) {
          print_log("ERROR: backlogHead is null");
          xSemaphoreGive(backlogMutex);
          continue;
        }

        PayloadNode* node = backlogHead;
        Payload pl = node->payload;
        xSemaphoreGive(backlogMutex);

        http.begin(ENDPOINT);
        http.addHeader("Content-Type", "application/json");
        DynamicJsonDocument doc(1024);
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

        String output;
        serializeJson(doc, output);

        int httpCode = 0;

        try {
          httpCode = http.POST(output);
        } catch(const std::exception& e) {
          print_log(e.what());
        }

        if(httpCode == 201 || httpCode == 406) {
          // Successfully sent or not acceptable, remove from backlog
          if (xSemaphoreTake(backlogMutex, portMAX_DELAY) == pdTRUE) {
            if (backlogHead == nullptr) {
              print_log("ERROR: backlogHead is null before removal");
              xSemaphoreGive(backlogMutex);
              continue;
            }
            if (backlogHead != node) {
              print_log("ERROR: backlogHead is not equal to current node");
              xSemaphoreGive(backlogMutex);
              continue;
            }

            backlogHead = backlogHead->next;
            backlogSize--;
            if (backlogHead == nullptr) {
              backlogTail = nullptr;
            }
            free(node);
            node = nullptr;
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

  return buffer;
}

void syncTimeCallBack(struct timeval *tv) { // re-sync callback
  print_log("===== Time Updated =====");
  print_log(get_time());
}

void timeSync(){
  print_log("Syncing Time...");
  // sync NTP
  time(&now);
  setenv("TZ", TZ_INFO, 1);
  tzset();
  configTime(0, 0, NTP_SERVER1);
  esp_sntp_init();
  sntp_set_sync_interval(1 * 60 * 60 * 1000);
  sntp_set_time_sync_notification_cb(syncTimeCallBack);
  sntp_restart();
  while(esp_sntp_get_sync_status()!=SNTP_SYNC_STATUS_COMPLETED){
    yield();
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  WiFi.mode(WIFI_STA);
  Serial.println("Connecting to WiFi");
  Serial.print(SSID);
  Serial.print(" ");
  Serial.println(WiFiPassword);
  WiFiMulti.addAP(SSID, WiFiPassword);

  backlogMutex = xSemaphoreCreateMutex();
  TaskHandle_t BacklogTask;
  xTaskCreatePinnedToCore(
    send_backlog,
    "BacklogTask",
    10000,
    NULL,
    1,
    &BacklogTask,
    0
  );

  ConnectToWiFiMulti();
  Serial.print("WiFi Connected! IP: ");
  Serial.println(WiFi.localIP());
  WebSerial.begin(&server);
  ElegantOTA.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();
  timeSync();
}

void loop() {
  ElegantOTA.loop();
  unsigned long start_time = millis();

  if(backlogSize > 0){
    checkFreeAndMax();
  }

  // init a payload
  Payload pl = {};

  // add base values
  strcpy(pl.time, get_time());
  strcpy(pl.metric, metric);
  pl.esp.rssi = WiFi.RSSI();
  pl.esp.acq_time = last_time;

  //read sensors
  for(int i=0;i<3;i++){
    strcpy(pl.data[i].phase, phase_names[i]);
    float voltage = pzem[i].voltage();
    if(!isnan(voltage)){
        pl.data[i].voltage = voltage;
    } else {
        pl.data[i].voltage = NAN;
        print_log("ERROR: No Voltage");
    }
    float current = pzem[i].current();
    if(!isnan(current)){
        pl.data[i].current = current;
    } else {
        pl.data[i].current = NAN;
        print_log("ERROR: No Current");
    }
    float power = pzem[i].power();
    if(!isnan(power)){
        pl.data[i].power = power;
    } else {
        pl.data[i].power = NAN;
        print_log("ERROR: No Power");
    }
    float energy = pzem[i].energy();
    if(!isnan(energy)){
        pl.data[i].energy = energy;
    } else {
        pl.data[i].energy = NAN;
        print_log("ERROR: No Energy");
    }
    float frequency = pzem[i].frequency();
    if(!isnan(frequency)){
        pl.data[i].frequency = frequency;
    } else {
        pl.data[i].frequency = NAN;
        print_log("ERROR: No Frequency");
    }
    float pf = pzem[i].pf();
    if(!isnan(pf)){
        pl.data[i].power_factor = pf;
    } else {
        pl.data[i].power_factor = NAN;
        print_log("ERROR: No PowerFactor");
    }
  }
  // check connection to wifi
  ConnectToWiFiMulti();
  // add payload to backlog
  if (xSemaphoreTake(backlogMutex, portMAX_DELAY) == pdTRUE) {
    PayloadNode* newNode = nullptr;
    while (newNode == nullptr) {
      newNode = (PayloadNode*)malloc(sizeof(PayloadNode));
      if (newNode == nullptr) {
        xSemaphoreGive(backlogMutex);
        backlog_remove_oldest();
        if (xSemaphoreTake(backlogMutex, portMAX_DELAY) != pdTRUE) {
            print_log("ERROR: Failed to retake semaphore after removing oldest");
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
  int16_t wait_time = READING_INTERVAL - (millis() - start_time);
  if(wait_time > READING_INTERVAL) wait_time = READING_INTERVAL / 10;
  delay(wait_time);
}