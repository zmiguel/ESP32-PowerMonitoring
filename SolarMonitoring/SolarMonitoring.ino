#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <PZEM004Tv30.h>
#include "config.h"

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define PZEM_SERIAL Serial2

#define TZ_INFO "WET-0WEST-1,M3.5.0/01:00:00,M10.5.0/02:00:00"
#define NTP_SERVER1  "pt.pool.ntp.org"
#define WRITE_PRECISION WritePrecision::S
#define MAX_BATCH_SIZE 50
#define WRITE_BUFFER_SIZE 500

WiFiMulti WiFiMulti;
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
PZEM004Tv30 pzem[3] = {
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x01),
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x02),
  PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x03)
};

// Data point
Point sensor[4] = {
  Point("ESP32"),
  Point("Solar"),
  Point("Solar"),
  Point("Solar")
};

unsigned long last_time = 0;

void ConnectToWiFiMulti() {
  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
  }
}

void ConnectToInflux() {
  // Add constant tags - only once
  sensor[0].addTag("Device", "Solar");
  sensor[1].addTag("Phase", "Casa");
  sensor[2].addTag("Phase", "Cozinha");
  sensor[3].addTag("Phase", "Loja");

  // Check server connection
  if (!client.validateConnection()) {
    delay(100);
    ESP.restart();
  }

  client.setWriteOptions(WriteOptions().writePrecision(WRITE_PRECISION).batchSize(MAX_BATCH_SIZE).bufferSize(WRITE_BUFFER_SIZE));
}

void setup() {
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(SSID, WiFiPassword);

  ConnectToWiFiMulti();
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
    }
    float current = pzem[i].current();
    if(!isnan(current)){
        sensor[i+1].addField("current", current);
    } else {
    }
    float power = pzem[i].power();
    if(!isnan(power)){
        sensor[i+1].addField("power", power);
    } else {
    }
    float energy = pzem[i].energy();
    if(!isnan(energy)){
        sensor[i+1].addField("energy", energy);
    } else {
    }
    float frequency = pzem[i].frequency();
    if(!isnan(frequency)){
        sensor[i+1].addField("freq", frequency);
    } else {
    }
    float pf = pzem[i].pf();
    if(!isnan(pf)){
        sensor[i+1].addField("power-factor", pf);
    } else {
    }
  }
  ConnectToWiFiMulti();
  for(int i=0;i<4;i++){
    client.writePoint(sensor[i]);
  }
  client.flushBuffer();
  last_time = millis() - start_time;
  unsigned long wait_time = 1000 - (millis() - start_time);
  if(wait_time > 1000) wait_time = 100;
  delay(wait_time);
}
