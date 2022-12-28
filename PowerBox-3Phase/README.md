# ESP32-PowerMonitoring (PowerBox/3Phase)

My take on a power measurement device using a ESP32 & PZEM-004T-v30

## Hardware requiered
- ESP32 Dev Board
- PZEM-004T-v30 Sensor
- InfluxDB server

### Libraries used
- [mandulaj/PZEM-004T-v30](https://github.com/mandulaj/PZEM-004T-v30) - for sensor measuring
- [tobiasschuerg/InfluxDB-Client-for-Arduino](https://github.com/tobiasschuerg/InfluxDB-Client-for-Arduino) - for influxdb connection
- [asjdf/WebSerialLite](https://github.com/asjdf/WebSerialLite) - for web serial output used for debugging
- [me-no-dev/ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) - required by `asjdf/WebSerialLite`
- [me-no-dev/AsyncTCP](https://github.com/me-no-dev/AsyncTCP) - required by `me-no-dev/ESPAsyncWebServer`