/* 
  Voltage Sensor :
    - Use INA3221 Voltage Sensors with 3 channels each for oil pressure, coolant temperature and voltage 12V
    - INA3221_BABORD_0 : Port engine (0x40 GND)
    - INA3221_CUVES_1 : Fuel and water tanks (0x41 VS)
    - INA3221_TRIBORD_2 : Starboard engine (0x42 SDA
    - INA3221_OTHERS_3 : Other sensors (0x43 SCL)
    - Engines INA3221 sensors are powered down after 60 seconds if the engine is off

  @author kinyo666
  @version 1.0.25
  @date 04/09/2025
  @link GitHub source code : https://github.com/kinyo666/VolvoPenta_ESP32
*/
#ifndef VOLTAGE_SENSOR_H
#define VOLTAGE_SENSOR_H

#include <sensesp/sensors/digital_input.h>
#include <sensesp/signalk/signalk_output.h>
#include <sensesp/transforms/moving_average.h>
#include <INA3221.h>
#include "customClasses.h"

// Volt / Current sensors INA3221
#define INA3221_BABORD_0 0                                // I2C address = 0x40 GND
#define INA3221_CUVES_1 1                                 // I2C address = 0x41 VS
#define INA3221_TRIBORD_2 2                               // I2C address = 0x42 SDA
#define INA3221_OTHERS_3 3                                // I2C address = 0x43 SCL
#define INA3221_NB 4

// Binary mask for engine state
#define ENGINE_STATE_OFF          0x000
#define ENGINE_STATE_ON           0x001
#define ENGINE_STATE_NOT_RUNNING  0x101
#define ENGINE_STATE_RUNNING      0x011
#define ENGINE_STATE_IS_RUNNING   0x010

const String sk_path_volt[INA3221_NB][INA3221_CH_NUM] = {
        {"propulsion.babord.coolantTemperature",  "propulsion.babord.oilPressure",    "propulsion.babord.alternatorVoltage"},
        {"tanks.fuel.babord.currentVolume",       "tanks.fuel.tribord.currentVolume", "tanks.freshWater.eaudouce.currentVolume"},
        {"propulsion.tribord.coolantTemperature", "propulsion.tribord.oilPressure",   "propulsion.tribord.alternatorVoltage"}};

const String conf_path_volt[INA3221_NB][INA3221_CH_NUM] = {
        {"/CONFIG/BABORD/INA3221/0/LINEAR_CH1",   "/CONFIG/BABORD/INA3221/0/LINEAR_CH2",  "/CONFIG/BABORD/INA3221/0/LINEAR_CH3"},
        {"/CONFIG/CUVES/INA3221/1/LINEAR_CH1",    "/CONFIG/CUVES/INA3221/1/LINEAR_CH2",   "/CONFIG/CUVES/INA3221/1/LINEAR_CH3"},
        {"/CONFIG/TRIBORD/INA3221/2/LINEAR_CH1",  "/CONFIG/TRIBORD/INA3221/2/LINEAR_CH2", "/CONFIG/TRIBORD/INA3221/2/LINEAR_CH3"}};

const String sk_path_rudder = "propulsion.rudderAngle";
const String conf_path_rudder = "/CONFIG/RUDDER";

bool sleepModeINA3221(bool, u_int8_t, unsigned int);
void setupVoltageSensors(ConfigSensESP*);

#endif // VOLTAGE_SENSOR_H