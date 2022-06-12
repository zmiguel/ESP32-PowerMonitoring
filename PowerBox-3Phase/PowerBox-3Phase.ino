
#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <PZEM004Tv30.h>
#include "config.h"

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define PZEM_SERIAL Serial2

WiFiMulti WiFiMulti;
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

unsigned long last_time = 0;

void ConnectToWiFiMulti() {
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(SSID, WiFiPassword);
  while (WiFiMulti.run() != WL_CONNECTED) {
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
    ESP.restart();
  }
}

void setup() {
  ConnectToWiFiMulti();
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
  // If no Wifi signal, try to reconnect it
  if (WiFiMulti.run() != WL_CONNECTED) {
    ESP.restart();
  }
  // Write point
  for(int i=0;i<4;i++){
    if (!client.writePoint(sensor[i])) {
    }
  }
  last_time = millis() - start_time;
  unsigned long wait_time = 1000 - (millis() - start_time);
  if(wait_time > 1000) wait_time = 100;
  delay(wait_time);
}
