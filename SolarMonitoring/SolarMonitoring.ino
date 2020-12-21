#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <PZEM004Tv30.h>
#include "config.h"

WiFiMulti WiFiMulti;
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);
PZEM004Tv30 pzem(&Serial2);

// Data point
Point sensor("Solar_1");

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
  sensor.addTag("device", "ESP32-Solar");

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
  sensor.clearFields();
  sensor.addField("wifi-rssi", WiFi.RSSI());
   float voltage = pzem.voltage();
    if(!isnan(voltage)){
        sensor.addField("voltage", voltage);
    } else {
        Serial.println("Error reading voltage");
    }
    float current = pzem.current();
    if(!isnan(current)){
        sensor.addField("current", current);
    } else {
        Serial.println("Error reading current");
    }
    float power = pzem.power();
    if(!isnan(power)){
        sensor.addField("power", power);
    } else {
        Serial.println("Error reading power");
    }
    float energy = pzem.energy();
    if(!isnan(energy)){
        sensor.addField("energy", energy);
    } else {
        Serial.println("Error reading energy");
    }
    float frequency = pzem.frequency();
    if(!isnan(frequency)){
        sensor.addField("freq", frequency);
    } else {
        Serial.println("Error reading frequency");
    }
    float pf = pzem.pf();
    if(!isnan(pf)){
        sensor.addField("power-factor", pf);
    } else {
        Serial.println("Error reading power factor");
    }

     // If no Wifi signal, try to reconnect it
  if ((WiFi.RSSI() == 0) && (WiFiMulti.run() != WL_CONNECTED))
    Serial.println("Wifi connection lost");
  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  //Wait 5s
  Serial.println("Wait ~5s");
  delay(4000);
}
