#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <PZEM004Tv30.h>
#include "config.h"

WiFiMulti WiFiMulti;
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);
PZEM004Tv30 pzem[3] = {
  PZEM004Tv30(&Serial2, 0x11),
  PZEM004Tv30(&Serial2, 0x12),
  PZEM004Tv30(&Serial2, 0x13)
};

// Data point
Point sensor[4] = {
  Point("PowerBox"),
  Point("PowerBox"),
  Point("PowerBox"),
  Point("PowerBox")
};

void ConnectToWiFiMulti() {
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(SSID, WiFiPassword);
  Serial.println("Connecting Wifi...");
  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void ConnectToInflux() {
  // Add constant tags - only once
  sensor[0].addTag("Device", "ESP32");
  sensor[1].addTag("Power_Phase", "Phase_1");
  sensor[2].addTag("Power_Phase", "Phase_2");
  sensor[3].addTag("Power_Phase", "Phase_3");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}

void setup() {
  Serial.begin(115200);
  ConnectToWiFiMulti();
  ConnectToInflux();
}

void loop() {
  sensor[0].clearFields();
  sensor[0].addField("wifi-rssi", WiFi.RSSI());
  //read sensors
  for(int i=0;i<3;i++){
    sensor[i+1].clearFields();
    float voltage = pzem[i].voltage();
    if(!isnan(voltage)){
        sensor[i+1].addField("voltage", voltage);
    } else {
        Serial.println("Error reading voltage");
    }
    float current = pzem[i].current();
    if(!isnan(current)){
        sensor[i+1].addField("current", current);
    } else {
        Serial.println("Error reading current");
    }
    float power = pzem[i].power();
    if(!isnan(power)){
        sensor[i+1].addField("power", power);
    } else {
        Serial.println("Error reading power");
    }
    float energy = pzem[i].energy();
    if(!isnan(energy)){
        sensor[i+1].addField("energy", energy);
    } else {
        Serial.println("Error reading energy");
    }
    float frequency = pzem[i].frequency();
    if(!isnan(frequency)){
        sensor[i+1].addField("freq", frequency);
    } else {
        Serial.println("Error reading frequency");
    }
    float pf = pzem[i].pf();
    if(!isnan(pf)){
        sensor[i+1].addField("power-factor", pf);
    } else {
        Serial.println("Error reading power factor");
    }
  }
     // If no Wifi signal, try to reconnect it
  if ((WiFi.RSSI() == 0) && (WiFiMulti.run() != WL_CONNECTED))
  Serial.print("Writing: ");
  Serial.println(sensor[0].toLineProtocol());
  Serial.println(sensor[1].toLineProtocol());
  Serial.println(sensor[2].toLineProtocol());
  Serial.println(sensor[3].toLineProtocol());
  // Write point
  for(int i=0;i<4;i++){
    if (!client.writePoint(sensor[i])) {
      Serial.print(i);
      Serial.print(": InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
    }
  }
  //Wait 5s
  Serial.println("Wait ~5s");
  delay(4000);
}
