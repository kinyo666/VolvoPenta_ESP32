/*
  Motion sensor :
  - MPU6050 6-axis motion sensor
  - Load and save offsets for the motion sensor
  - Provide Yaw, Pitch, Roll values in radians

  @author kinyo666
  @version 1.0.17
  @date 04/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32

  TODO : find a workaround for the offsets config conflict with constant_sensor.h
*/
#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

//#include <sensesp/sensors/constant_sensor.h>
#include <sensesp/sensors/sensor.h>
#include <sensesp/signalk/signalk_output.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "customClasses.h"
//https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU9250
//https://registry.platformio.org/libraries/mbed-anything-connected/MPU9250/installation
//#include <MPU9250.h>

// Motion sensor MPU6050
#define MPU6050_ATTITUDE 0
#define MPU6050_HEADING 1
#define MPU6050_OFFSETS 2

const String sk_path_motion[3] = { "navigation.attitude", "navigation.headingMagnetic", "navigation.offsets" };
const String conf_path_motion = "/CONFIG/MOTION/OFFSETS";

// Class to handle motion sensor offsets for MPU6050
class MotionSensorOffsets : public sensesp::StringTransform { //, public sensesp::SensorConfig {
  public:
  MotionSensorOffsets(const String& config_path = "")
     : sensesp::StringTransform(config_path) { //, SensorConfig(config_path) {
      conf_motionsensor = jdoc_conf_motionsensor.to<JsonObject>();
      valid_offset = sensesp::StringTransform::load();
  }

  inline void set(const String input) {
    // Convert the input string "ax|ay|..." to float values
    int16_t values[6];
    char *input_cstr = const_cast<char*>(input.c_str());
    char* token = strtok(input_cstr, "|");
    int idx = 0;
    while (token != NULL && idx < 6) {
      values[idx++] = atof(token);
      token = strtok(NULL, "|");
    }

    // JSON object to store the motion sensor offsets
    conf_motionsensor["ax_offset"] = values[0];
    conf_motionsensor["ay_offset"] = values[1];
    conf_motionsensor["az_offset"] = values[2];
    conf_motionsensor["gx_offset"] = values[3];
    conf_motionsensor["gy_offset"] = values[4];
    conf_motionsensor["gz_offset"] = values[5];

    // Update the VectorFloat objects with the new offsets
    accel_offset.x = values[0];
    accel_offset.y = values[1];
    accel_offset.z = values[2];
    gyro_offset.x = values[3];
    gyro_offset.y = values[4];
    gyro_offset.z = values[5];
    
    // Save the updated configuration to the file system
    serializeJsonPretty(conf_motionsensor, output_);
    notify();
    valid_offset = sensesp::StringTransform::save();
    #ifdef DEBUG_MODE_CUSTOM_CLASSES_H
    Serial.printf("MOTION SENSOR OFFSET SAVED : Acceleration X = %i\tY = %i\tZ = %i | ",
                  values[0], values[1], values[2]);
    Serial.printf("Gyroscope X = %i\tY = %i\tZ = %i\n", values[3], values[4], values[5]);
    #endif
  }

  inline void reset() { 
    accel_offset.x = 0;
    accel_offset.y = 0;
    accel_offset.z = 0;
    gyro_offset.x = 0;
    gyro_offset.y = 0;
    gyro_offset.z = 0;
    valid_offset = false;
  }

  // Convert offset values to a JSON file
  inline bool to_json(JsonObject& doc) {
    doc["ax_offset"] = accel_offset.x;
    doc["ay_offset"] = accel_offset.y;
    doc["az_offset"] = accel_offset.z;
    doc["gx_offset"] = gyro_offset.x;
    doc["gy_offset"] = gyro_offset.y;
    doc["gz_offset"] = gyro_offset.z;
    return true;
  }

  // Load offset values from a JSON file
  inline bool from_json(const JsonObject& config) {
    if (!config["ax_offset"].is<int16_t>() || !config["ay_offset"].is<int16_t>() || !config["az_offset"].is<int16_t>() ||
        !config["gx_offset"].is<int16_t>() || !config["gy_offset"].is<int16_t>() || !config["gz_offset"].is<int16_t>()) 
      return false;

    accel_offset.x = config["ax_offset"];
    accel_offset.y = config["ay_offset"];
    accel_offset.z = config["az_offset"];
    gyro_offset.x = config["gx_offset"];
    gyro_offset.y = config["gy_offset"];
    gyro_offset.z = config["gz_offset"];

    #ifdef DEBUG_MODE_CUSTOM_CLASSES_H
    Serial.printf("MOTION SENSOR OFFSET FROM JSON : Acceleration X = %i\tY = %i\tZ = %i | ",
                  accel_offset.x, accel_offset.y, accel_offset.z);
    Serial.printf("Gyroscope X = %i\tY = %i\tZ = %i\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);
    #endif
    return true;
  }

  // Return if the offsets are valid (loaded successfully and int16_t format)
  inline bool is_valid() const {
    return valid_offset;
  }

  // Get the acceleration offsets as a VectorFloat object
  inline VectorInt16 getAccelOffset() const {
    return accel_offset;
  }

  // Get the gyroscope offsets as a VectorFloat object
  inline VectorInt16 getGyroOffset() const {
    return gyro_offset;
  }

  private:
  JsonDocument jdoc_conf_motionsensor;
  JsonObject conf_motionsensor;                                 // MPU X/Y/Z offsets values if exists
  VectorInt16 accel_offset;                                     // Acceleration X/Y/Z offset values
  VectorInt16 gyro_offset;                                      // Gyroscope X/Y/Z offset values
  bool valid_offset;                                            // Flag to indicate if the offsets are valid
};

void setupMotionSensor(sensesp::ConfigSensESP*);
//namespace sensesp {
bool ConfigRequiresRestart(const MotionSensorOffsets& obj);
const String ConfigSchema(const MotionSensorOffsets& obj);
//}
#endif // MOTION_SENSOR_H