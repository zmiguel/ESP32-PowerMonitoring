#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <PZEM004Tv30.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <WebSerialLite.h>
#include "config.h"

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define PZEM_SERIAL Serial2

#define TZ_INFO "WET-0WEST-1,M3.5.0/01:00:00,M10.5.0/02:00:00"
#define NTP_SERVER1  "pt.pool.ntp.org"
#define WRITE_PRECISION WritePrecision::MS
#define MAX_BATCH_SIZE 4
#define WRITE_BUFFER_SIZE 64
#define MAX_FAILED_READS 30

WiFiMulti WiFiMulti;
AsyncWebServer server(80);
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
PZEM004Tv30 pzem[3] = {
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x11),
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x12),
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x13)
};

// Data point
Point sensor[4] = {
  Point("ESP32"),
  Point("PowerBox"),
  Point("PowerBox"),
  Point("PowerBox")
};

uint16_t last_time = 0;
uint8_t failed = 0;

void ConnectToWiFiMulti() {
  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
}

void ConnectToInflux() {
  // Add constant tags - only once
  sensor[0].addTag("Device", "PowerBox");
  sensor[1].addTag("Phase", "Cozinha");
  sensor[2].addTag("Phase", "Loja");
  sensor[3].addTag("Phase", "Casa");

  // Check server connection
  if (!client.validateConnection()) {
    Serial.println("Connection to InfluxDB failed.");
    WebSerial.println("Connection to InfluxDB failed.");
    delay(100);
    ESP.restart();
  }

  Serial.println("Connection to InfluxDB successful!");
  WebSerial.println("Connection to InfluxDB successful!");

  client.setWriteOptions(WriteOptions().writePrecision(WRITE_PRECISION).batchSize(MAX_BATCH_SIZE).bufferSize(WRITE_BUFFER_SIZE));
  Serial.println("Write options set!");
  WebSerial.println("Write options set!");
}

void InfuxStatus(){
  // Check server connection
  if (!client.validateConnection()) {
    Serial.println("Connection to InfluxDB failed.");
    WebSerial.println("Connection to InfluxDB failed.");
    delay(100);
    ESP.restart();
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

  ConnectToWiFiMulti();
  WebSerial.begin(&server);
  AsyncElegantOTA.begin(&server);
  server.begin();
  timeSync(TZ_INFO, NTP_SERVER1);
  ConnectToInflux();
}

void loop() {
  unsigned long start_time = millis();
  sensor[0].clearFields();
  sensor[0].addField("wifi-rssi", WiFi.RSSI());
  sensor[0].addField("acq-time", last_time);
  //read sensors
  for(int i=0;i<3;i++){
    sensor[i+1].clearFields();
    float voltage = pzem[i].voltage();
    if(!isnan(voltage)){
        sensor[i+1].addField("voltage", voltage);
    } else {
        Serial.println("ERROR: No Voltage");
        WebSerial.println("ERROR: No Voltage");
    }
    float current = pzem[i].current();
    if(!isnan(current)){
        sensor[i+1].addField("current", current);
    } else {
        Serial.println("ERROR: No Current");
        WebSerial.println("ERROR: No Current");
    }
    float power = pzem[i].power();
    if(!isnan(power)){
        sensor[i+1].addField("power", power);
    } else {
        Serial.println("ERROR: No Power");
        WebSerial.println("ERROR: No Power");
    }
    float energy = pzem[i].energy();
    if(!isnan(energy)){
        sensor[i+1].addField("energy", energy);
    } else {
        Serial.println("ERROR: No Energy");
        WebSerial.println("ERROR: No Energy");
    }
    float frequency = pzem[i].frequency();
    if(!isnan(frequency)){
        sensor[i+1].addField("freq", frequency);
    } else {
        Serial.println("ERROR: No Frequency");
        WebSerial.println("ERROR: No Frequency");
    }
    float pf = pzem[i].pf();
    if(!isnan(pf)){
        sensor[i+1].addField("power-factor", pf);
    } else {
        Serial.println("ERROR: No PowerFactor");
        WebSerial.println("ERROR: No PowerFactor");
    }
  }
  // check conection to wifi
  ConnectToWiFiMulti();
  // check connection to influx
  InfuxStatus();
  for(int i=0;i<4;i++){
    if(!client.writePoint(sensor[i])){
      failed++;
      Serial.print(failed);
      WebSerial.print(failed);
      Serial.print(" Write failed, error: ");
      WebSerial.print(" Write failed, error: ");
      Serial.println(client.getLastErrorMessage());
      WebSerial.println(client.getLastErrorMessage());
    }else{
      failed = 0;  
    }
  }
  if(failed > MAX_FAILED_READS){
    Serial.println("Too many failed reads, restarting...");
    WebSerial.println("Too many failed reads, restarting...");
    delay(50);
    ESP.restart();
  }
  // check if buffer is full
  if(client.isBufferFull()){
    Serial.println("Buffer full");
    WebSerial.println("Buffer full");
    client.flushBuffer();
  }
  last_time = millis() - start_time;
  Serial.println(last_time);
  WebSerial.println(last_time);
  unsigned long wait_time = 1000 - (millis() - start_time);
  if(wait_time > 1000) wait_time = 100;
  delay(wait_time);
}
