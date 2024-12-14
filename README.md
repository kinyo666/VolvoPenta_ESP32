# Dashboard TAMD40 for Volvo Penta

This is the ESP32 part of the project to digitize Volvo Penta TAMD40 marine engine :motor_boat:
It is sized to fit with 2 TAMD engines (portside and starboard) but can easily be restricted to only one.
> [!NOTE]
> This code is also suitable for others Volvo Penta TAMD/TMD/AQAD engines but haven't been tested.
## FEATURES
  Digital dashboard :
  - [x] Coolant temperature
  - [x] Oil pressure
  - [x] Voltage
  - [x] RPM gauge
  - [x] Fuel consumption  
  - [x] Windlass chain counter (persistent)
  - [x] Fuel tank level
  - [x] Fresh water tank level
  - [x] Exhaust temperature
  - [x] Bilge temperature
  - [x] Motion sensor (roll, pitch, yaw)
  - [ ] Autopilot

## WHAT YOU NEED
### MUST-HAVE
- ESP32 Wroom32
- INA3221 volt sensors (x3)
- DS18B20 temperature sensors (x3)
- PC817 rpm sensors (x2)
- MPU6050 or MPU9250 motion sensor (x1)
- Wifi Access Point (AP) where your ESP32 will connect to
- [Signal K](http://signalk.org) Server running on you local network with plugins (KIP, ...)
- [Visual Code Studio](https://code.visualstudio.com/) with [PlatformIO](https://platformio.org/) plugin
- [SensESP SDK](https://github.com/SignalK/SensESP) (>= 3.0.0)

### NICE TO HAVE
- [Raspberry Pi](https://www.raspberrypi.com/products/) (4B or 5) - Signal K server, InfluxDB and Grafana
- [InfluxDB OSS](https://www.influxdata.com/downloads/) (>= 2.7.10) - Store Signal K values
- [Grafana OSS](https://grafana.com/grafana/download?pg=get&plcmt=selfmanaged-box1-cta1&edition=oss) (>= 11.3.0) - Visualize the InfluxDB data

## HOW TO INSTALL
1. Edit the library dependencies in [platformio.ini](/platformio.ini) :
  ```lib_deps = SignalK/SensESP@^3.0.0
        SensESP/OneWire@^3.0.2
        tinyu-zhao/INA3221@^0.0.1
        electroniccats/MPU6050@^1.4.1
  ```
2. Adjust the build_flags in [platformio.ini](/platformio.ini) :
  ```build_flags = 
    -D LED_BUILTIN=2
    -D CORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_VERBOSE
    -D TAG='"Arduino"'
    -D USE_ESP_IDF_LOG```

3. Download this code or fetch the repository :

_TO DO_
`overview.md`
`{{site.path}}`
> [!NOTE]
> Useful information that users should know, even when skimming content.

> [!TIP]
> Helpful advice for doing things better or more easily.

> [!IMPORTANT]
> Key information users need to know to achieve their goal.

> [!WARNING]
> Urgent info that needs immediate user attention to avoid problems.

> [!CAUTION]
> Advises about risks or negative outcomes of certain actions.