/*
  Extended classes for custom usage :
  - CurveInterpolator for Fuel Consumption, Coolant Temperature, Oil Pressure
  - Transform for Positive Linear transform and Engine Data
  - Integrator for Persistent Integrator
  - ConfigSchema for ConfigSensESP and Persistent Integrator

  @author kinyo666
  @version 1.0.16
  @date 03/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#include "customClasses.h"

namespace sensesp {

// Callback for Linear Positive (Lambda Transform returns 0.0 if not positive)
//auto linearPositive = [](float input, float multiplier, float offset) -> float {
float linearPositive(float input, float multiplier, float offset) {
  if (input > 0)
    return ((multiplier * input) + offset);
  else
    return (0.0f);
}
const ParamInfo* linearPositive_ParamInfo = new ParamInfo[2]{{"multiplier", "Multiplier"}, {"offset", "Offset"}};

// Callback for Engine State (Lambda Transform returns true if engine is running, false otherwise)
boolean runningState(float input) {
  return (input > 0);
}
//const ParamInfo* runningState_ParamInfo = new ParamInfo[1]{{"engine_id", "Engine ID"}, {"sensesp_config", "SensESP Configuration"}};

const String ConfigSchema(const MotionSensorOffsets& obj) {
  return R"({
    "type": "object",
    "properties": {
        "ax_offset": { "title": "Accel X Offset", "type": "number" },
        "ay_offset": { "title": "Accel Y Offset", "type": "number" },
        "az_offset": { "title": "Accel Z Offset", "type": "number" },
        "gx_offset": { "title": "Gyro X Offset", "type": "number" },
        "gy_offset": { "title": "Gyro Y Offset", "type": "number" },
        "gz_offset": { "title": "Gyro Z Offset", "type": "number" }
    }
  })";
  }
  
bool ConfigRequiresRestart(const MotionSensorOffsets& obj) {
  return true;
}

// Set default configuration schema for ConfigSensESP
const String ConfigSchema(const ConfigSensESP& obj) {
  return R"({
    "type": "object",
    "properties": {
      "DS18B20_FEATURE": { "type": "boolean", "title": "DS18B20 Feature" },
      "DS18B20_BABORD_0": { "type": "boolean", "title": "- DS18B20 Babord" },
      "DS18B20_TRIBORD_1": { "type": "boolean", "title": "- DS18B20 Tribord" },
      "DS18B20_COMMON_2": { "type": "boolean", "title": "- DS18B20 Common" },
      "INA3221_FEATURE": { "type": "boolean", "title": "INA3221 Feature" },
      "INA3221_BABORD_0": { "type": "boolean", "title": "- INA3221 Babord" },
      "INA3221_CUVES_1": { "type": "boolean", "title": "- INA3221 Cuves" },
      "INA3221_TRIBORD_2": { "type": "boolean", "title": "- INA3221 Tribord" },
      "INA3221_OTHERS_3": { "type": "boolean", "title": "- INA3221 Autres" },
      "INA3221_POWERDOWN": { "type": "boolean", "title": "- INA3221 Power Down" },
      "PC817_FEATURE": { "type": "boolean", "title": "PC817 Feature" },
      "PC817_BABORD": { "type": "boolean", "title": "- PC817 Babord" },
      "PC817_TRIBORD": { "type": "boolean", "title": "- PC817 Tribord" },
      "CHAIN_COUNTER_FEATURE": { "type": "boolean", "title": "ADS1115 Chain Counter Feature" },
      "MOTION_SENSOR_FEATURE": { "type": "boolean", "title": "Motion Sensor Feature" },
      "MOTION_SENSOR_CALIBRATE": { "type": "boolean", "title": "Calibrate Motion Sensor" },
      "RUDDER_ANGLE_FEATURE": { "type": "boolean", "title": "Rudder Angle Feature" }
    }
  })";
}

// Set requires restart to true for ConfigSensESP
bool ConfigRequiresRestart(const ConfigSensESP& obj) {
  return true;
}
}