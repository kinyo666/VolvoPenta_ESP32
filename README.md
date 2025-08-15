# Dashboard TAMD40 for Volvo Penta

This is the ESP32 part of the project to digitize Volvo Penta TAMD40 marine engine :motor_boat:
It is sized to fit with 2 TAMD engines (portside and starboard) but can easily be restricted to only one.
> [!NOTE]
> This code is also suitable for others Volvo Penta TAMD/TMD/AQAD engines but haven't been tested.
> The hardware configuration allows you to continue using the analog dashboard without any disruption
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
  - [ ] Bilge water level
  - [x] Motion sensor (roll, pitch, yaw)
  - [ ] Autopilot
  - [X] Rudder Angle
  
  ESP32 special features :
  - [x] INA3221 auto sleep mode (power down / energy saving)
  - [x] MPU6050 auto calibration (persistent offset)
  - [x] Global features enabling / disabling by checkbox
  - [x] Run-time configuration (Wifi AP / webserver)

  > [!WARNING]
  > - The run-time configuration is stored in the ESP32's flash memory and overrides the compile-time values.
  > - This feature is very useful because it allows you to change any value and restart the ESP32 to apply the new configuration without any coding.
  > - Since there is no 'ls -all' or 'dir' command, be careful when adding a new configuration path or using an existing one as it can create conflicts and unexpected behaviour.
  > - Values MUST be calibrated before using at sea.

## WHAT YOU NEED
### MUST-HAVE
- ESP32 Wroom32
- INA3221 volt sensors (x4)
- DS18B20 temperature sensors (x3)
- PC817 rpm sensors (x2)
- MPU6050 or MPU9250 motion sensor (x1)
- Reed switch with magnet from a cycling computer wired (e.g Sigma) or burglar alarm (x1)
- HSTS016L Hall effect current sensor 3.3V / 100A (x1)
- Wifi Access Point (AP) where your ESP32 will connect to
- [Visual Code Studio](https://code.visualstudio.com/) with [PlatformIO](https://platformio.org/) plugin
- [Git Cli](https://cli.github.com/) (>= 2.74.2)
- [Signal K](http://signalk.org) Server running on you local network with plugins (KIP, ...)
- [SensESP SDK](https://github.com/SignalK/SensESP) (>= 3.1.0)

### NICE TO HAVE
- [Raspberry Pi](https://www.raspberrypi.com/products/) (4B or 5) - Signal K server, InfluxDB and Grafana
- [InfluxDB OSS](https://www.influxdata.com/downloads/) (>= 2.7.10) - Store Signal K values
- [Grafana OSS](https://grafana.com/grafana/download?pg=get&plcmt=selfmanaged-box1-cta1&edition=oss) (>= 11.3.0) - Visualize the InfluxDB data

## HARDWARE SETUP
![Schéma_v7](https://github.com/user-attachments/assets/6006ebc8-a7bc-456a-aaca-7ce9d4a54592)
> [!CAUTION]
> - The ESP32 MUST be connected to the same 12V electrical circuit as your boat.
> - If not, the sensors HAVE TO to be connected to ground on your boat's 12V circuit otherwise you will get inaccurate and irrelevant readings.

## HOW TO INSTALL
1. Edit the library dependencies in [platformio.ini](/platformio.ini) :
  ```
  lib_deps = SignalK/SensESP@^3.1.0
              SensESP/OneWire@^3.0.2
              tinyu-zhao/INA3221@^0.0.1
              electroniccats/MPU6050@^1.4.1
              wh1terabbithu/ADS1115-Driver@^1.0.2
  ```
2. Adjust the build_flags in [platformio.ini](/platformio.ini) :
  ```
  build_flags = 
    -D LED_BUILTIN=2
    -D CORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_VERBOSE
    -D TAG='"Arduino"'
    -D USE_ESP_IDF_LOG
  ```

3. Download this code or fetch the repository [GitHub > kinyo666/Capteurs_ESP32](https://github.com/kinyo666/Capteurs_ESP32)

> [!IMPORTANT]
> Some key elements need to be customized to suit your configuration :
> - Replace the 1-wire addresses with those of your DS18B20 sensors in the source code
> - Check that the pin number matches with your physical setup
> - Global constants with the _NB suffix are used to define the number of elements in each array, change them carefully to suit your needs (e.g 1 or 2 engines)
> - Values MUST be calibrated before using at sea

4. Build the firmware and upload it to your ESP32 :
- Visual Studio > Terminal > Run Build Task PlatformIO: Build (or use the checkmark icon at the bottom)
- Visual Studio > Terminal > Run Task > PlatformIO: Upload (or use the right arrow icon at the bottom)

5. Configure your ESP32 module
- Connect to the ESP32 Wifi Access Point (default : SSID = “SensESP”, Password = “thisisfine”) with your smartphone or laptop
- Configure the SSID / Password for the Raspberry Pi Wifi connection (or your router for testing purpose)
- Configure the IP address / port of Signal K server (default : mDNS, port 3000)
- Save the configuration
- Launch SignalK on your Raspberry Pi (or PC for testing purpose), go to Security > Access Request to modify the Authentication Timeout (set it to NEVER) of your ESP32 and click on Approve
- Check whether your ESP32 is now showing up in the connected devices list

> [!TIP]
> If your ESP32 is up but unable to connect to the Signal K server, check if a firewall or antivirus program is blocking the connection. 
> On Windows, you may need to disable Microsoft Defender on your local private network to allow the ESP32 to use mDNS / server discovery
