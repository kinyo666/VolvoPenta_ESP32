/*
  Temperature sensors :
  - Use 3 DS18B20 sensors to measure the temperature of the engine room and exhausts
  - DS18B20_BABORD_0 : Port engine exhaust temperature
  - DS18B20_TRIBORD_1 : Starboard engine exhaust temperature
  - DS18B20_COMMON_2 : Engine room temperature
  - Temperature values are converted to Kelvin and filtered to avoid out of range values

  @author kinyo666
  @version 1.0.18
  @date 04/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

// Temperature sensors DS18B20
#define DS18B20_PIN 33                                    // Use ADC1 pin (GPIO32-35) rather than ADC2 pin to avoid Wifi interference
#define DS18B20_BABORD_0 0
#define DS18B20_TRIBORD_1 1
#define DS18B20_COMMON_2 2
#define DS18B20_NB 3
#define DS18B20_BABORD_ADDR "28:61:64:34:35:a9:96:d9"     // Replace your 1-Wire addresses here
#define DS18B20_TRIBORD_ADDR "28:ff:64:1f:74:6f:1f:0b"
#define DS18B20_COMMON_ADDR "28:ff:64:1f:75:83:99:c9"

//#include <sensesp.h>
#include <sensesp/signalk/signalk_output.h>
#include <sensesp_onewire/onewire_temperature.h>
#include "customClasses.h"

const String sk_path_temp[DS18B20_NB] = {
        "propulsion.babord.exhaustTemperature",
        "propulsion.tribord.exhaustTemperature",
        "environment.inside.engineroom.temperature"};

void setupTemperatureSensors(ConfigSensESP*);

#endif // TEMPERATURE_SENSOR_H