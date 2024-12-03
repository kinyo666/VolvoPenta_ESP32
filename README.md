# Dashboard TAMD40 for Volvo Penta

This is the ESP32 part of the project to digitize Volvo Penta TAMD40 marine engine.
It is sized to fit with 2 TAMD engines (portside and starboard) but can easily be restricted to only one.

## FEATURES
  Digital dashboard :
  - Coolant temperature
  - Oil pressure
  - Voltage
  - RPM gauge
  - Fuel consumption  
  - Windlass chain counter (persistent)
  - Fuel tank level
  - Fresh water tank level
  - Exhaust temperature
  - Bilge temperature

## WHAT YOU NEED
### MUST-HAVE
- ESP32 Wroom32
- INA3221 volt sensors (x3)
- DS18B20 temperature sensors (x3)
- PC817 rpm sensors (x2)
- Wifi Access Point (AP) where your ESP32 will connect to
- [Signal K](http://signalk.org) Server running on you local network with plugins (KIP, ...)
- [Visual Code Studio](https://code.visualstudio.com/) with [PlateformIO](https://platformio.org/) plugin

### NICE TO HAVE
- [Raspberry Pi](https://www.raspberrypi.com/products/) (4B or 5) - Signal K server, InfluxDB and Grafana
- [InfluxDB OSS](https://www.influxdata.com/downloads/) (>= 2.7.10) - Store Signal K values
- [Grafana OSS](https://grafana.com/grafana/download?pg=get&plcmt=selfmanaged-box1-cta1&edition=oss) (>= 11.3.0) - Visualize the InfluxDB data

## HOW TO INSTALL
_TO DO_
`overview.md`
`{{site.path}}`
