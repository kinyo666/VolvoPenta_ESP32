/*
  Motion sensor :
  - MPU6050 6-axis motion sensor
  - Load and save offsets for the motion sensor
  - Provide Yaw, Pitch, Roll values in radians

  @author kinyo666
  @version 1.0.18
  @date 04/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#include "sensors/motion_sensor.h"

// Motion and compass sensor
MPU6050 sensor_mpu;                                           // 1 MPU-6050 Motion sensor
#ifndef FAKE_MODE
sensesp::RepeatSensor<float> *sensor_compass;                          // MPU Compass values
#else
FloatConstantSensor *sensor_compass;                          // Fake Compass values
#endif

// Callback for motion sensor Yaw/Pitch/Roll
// @param q Quaternion with X, Y, Z values
// @return JSON String with Yaw, Pitch, Roll values in radians
String getMotionSensorYPR(Quaternion q) {
  VectorFloat gravity;    // [x, y, z]            Gravity vector
  float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container
  uint8_t FIFOBuffer[64]; // FIFO storage buffer
  JsonDocument json_doc;  // JSON radian values
  String json;
  JsonObject value = json_doc.to<JsonObject>();
  
  // Read a packet from FIFO
  if ((q.x != 0) && (q.y != 0) && (q.z != 0)) {
    sensor_mpu.dmpGetGravity(&gravity, &q);
    sensor_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    value["roll"] = ypr[2];       // Roll / Roulis
    value["pitch"] = ypr[1];      // Pitch / Tangage
    value["yaw"] = ypr[0];        // Yaw / Lacet

    #ifdef DEBUG_MODE
      Serial.printf("MOTION SENSOR : Yaw = %f°\t| Pitch = %f°\t| Roll = %f°\n", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
    #endif
  }
  else {
    value["roll"] = NULL;
    value["pitch"] = NULL;
    value["yaw"] = NULL;
  }

  serializeJson(json_doc, json);
  return json;
}

// Callback for motion sensor quaternion
// @return Quaternion with X, Y, Z values
Quaternion getMotionSensorQuaternion() {
  Quaternion q;
  uint8_t FIFOBuffer[64]; // FIFO storage buffer
  
  // Read a packet from FIFO
  if (sensor_mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) // Get the Latest packet 
    sensor_mpu.dmpGetQuaternion(&q, FIFOBuffer);

  #ifdef DEBUG_MODE
    Serial.printf("MOTION SENSOR : X = %f°\t| Y = %f°\t| Z = %f°\n", q.x, q.y, q.z);
  #endif

  return q;
}

// Callback for motion sensor
// @return Attitude JSON Vector with Yaw, Pitch, Roll
// @see https://youtu.be/kCS-wmnhlvQ
String getMotionSensorValues() {
  Quaternion q;           // [w, x, y, z]         Quaternion container
  VectorFloat gravity;    // [x, y, z]            Gravity vector
  float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container
  uint8_t FIFOBuffer[64]; // FIFO storage buffer
  JsonDocument json_doc;  // JSON radian values
  String json;
  JsonObject value = json_doc.to<JsonObject>();

  // Read a packet from FIFO
  if (sensor_mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    sensor_mpu.dmpGetQuaternion(&q, FIFOBuffer);
    sensor_mpu.dmpGetGravity(&gravity, &q);
    sensor_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    value["roll"] = ypr[2];       // Roll / Roulis
    value["pitch"] = ypr[1];      // Pitch / Tangage
    value["yaw"] = ypr[0];        // Yaw / Lacet

    #ifdef DEBUG_MODE
      Serial.printf("MOTION SENSOR : Yaw = %f°\t| Pitch = %f°\t| Roll = %f°\n", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
    #endif
  }
  else {
    value["roll"] = NULL;
    value["pitch"] = NULL;
    value["yaw"] = NULL;
  }

  serializeJson(json_doc, json);
  return json;
}

// Callback for compass sensor
// @return Fake heading value in radian
float getCompassSensorValue() {
  return 2.094395;      // 120°
}

// Set offsets for the motion sensor
void setMotionSensorOffsets(MotionSensorOffsets* sensor_motion_offsets) {
  VectorInt16 accel_offset = sensor_motion_offsets->getAccelOffset();
  VectorInt16 gyro_offset = sensor_motion_offsets->getGyroOffset();

  // Set offsets
  sensor_mpu.setXAccelOffset(accel_offset.x);
  sensor_mpu.setYAccelOffset(accel_offset.y);
  sensor_mpu.setZAccelOffset(accel_offset.z);
  sensor_mpu.setXGyroOffset(gyro_offset.x);
  sensor_mpu.setYGyroOffset(gyro_offset.y);
  sensor_mpu.setZGyroOffset(gyro_offset.z);

  #ifdef DEBUG_MODE
    Serial.printf("MOTION SENSOR OFFSET LOADED : Acceleration X = %i\tY = %i\tZ = %i | ",
                  accel_offset.x, accel_offset.y, accel_offset.z);
    Serial.printf("Gyroscope X = %i\tY = %i\tZ = %i\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);
  #endif
}

// Calibrate the motion sensor MPU6050
void calibrateMotionSensor(MotionSensorOffsets* sensor_motion_offsets) {
  int16_t *motion_sensor_offsets;     // Gyro offsets for the MPU6050
  
  // Reset the MPU6050 offsets to 0 before calibration
  sensor_mpu.setXGyroOffset(0);
  sensor_mpu.setYGyroOffset(0);
  sensor_mpu.setZGyroOffset(0);
  sensor_mpu.setXAccelOffset(0);
  sensor_mpu.setYAccelOffset(0);
  sensor_mpu.setZAccelOffset(0);

  sensor_mpu.CalibrateAccel(6);                           // Calibration Time: generate offsets and calibrate our MPU6050
  sensor_mpu.CalibrateGyro(6);

  motion_sensor_offsets = sensor_mpu.GetActiveOffsets();  // Get the active offsets from the MPU6050

  #ifdef DEBUG_MODE
  Serial.println("Active offsets : ");
  //	A_OFFSET_H_READ_A_OFFS(Data);
  Serial.printf("Acceleration X = %i\tY = %i\tZ = %i | ", 
        motion_sensor_offsets[0],
        motion_sensor_offsets[1],
        motion_sensor_offsets[2]);
  //	XG_OFFSET_H_READ_OFFS_USR(Data);
  Serial.printf("Gyroscope X = %i\tY = %i\tZ = %i\n", 
        motion_sensor_offsets[3],
        motion_sensor_offsets[4],
        motion_sensor_offsets[5]);
  #endif

  // Convert the offsets to a string
  String motion_string_offsets = String(motion_sensor_offsets[0]) + "|" +
    String(motion_sensor_offsets[1]) + "|" +
    String(motion_sensor_offsets[2]) + "|" +
    String(motion_sensor_offsets[3]) + "|" +
    String(motion_sensor_offsets[4]) + "|" +
    String(motion_sensor_offsets[5]);

  // TODO : resolve the conflict "multiple definition of `String const sensesp::ConfigSchemaSensorType<String>(String const&)';" 
  
  // Sends the offsets to the Signal K server and save the motion sensor offsets
  //sensesp::StringConstantSensor *sensor_string_offsets = new sensesp::StringConstantSensor(motion_string_offsets, 600);
  //sensor_string_offsets->connect_to(sensor_motion_offsets);
  //sensor_motion_offsets->connect_to(new sensesp::SKOutputRawJson(sk_path_motion[MPU6050_OFFSETS]));
}

// Setup the motion sensor MPU6050
// This function initializes the MPU6050 motion sensor, calibrates it if needed, and sets the offsets.
// @link https://registry.platformio.org/libraries/electroniccats/MPU6050
void setupMotionSensor(ConfigSensESP* sensesp_config) {
  uint8_t devStatus;                                            // Return status after each device operation (0 = success, !0 = error)
  unsigned int read_delay = sensesp_config->get_read_delay();

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.setPins(21, 22);
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // Initialize device
  Serial.println(F("Initializing I2C devices..."));
  sensor_mpu.initialize();

  // Verify connection
  Serial.println(F("Testing MPU6050 connection..."));
  if (sensor_mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    return;
  }
  else
    Serial.println("MPU6050 connection successful");

  // Initializate and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = sensor_mpu.dmpInitialize();
  
  // Making sure it worked (returns 0 if so)
  if (devStatus == 0) {
    MotionSensorOffsets *sensor_motion_offsets = new MotionSensorOffsets(conf_path_motion); // MPU Offsets values

    if (sensesp_config->is_enabled("MOTION_SENSOR_CALIBRATE") || !sensor_motion_offsets->is_valid())
      calibrateMotionSensor(sensor_motion_offsets);
    setMotionSensorOffsets(sensor_motion_offsets);         // Set offsets using the saved values or the calibrated values

    // Make the offsets editable in the SensESP UI
    sensesp::ConfigItem(sensor_motion_offsets)
      ->set_title("Gyroscope - Offsets")
      ->set_description("Offsets - MPU6050 - MotionSensorOffsets")
      ->set_sort_order(UI_ORDER_MOTION);

    Serial.print(F("Enabling DMP..."));   // Turning ON DMP
    sensor_mpu.setDMPEnabled(true);
    Serial.println(F("DMP ready !"));
    uint16_t packetSize = sensor_mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison

    // Read motion sensor values every read_delay milliseconds
    sensesp::RepeatSensor<String> *sensor_motion = new sensesp::RepeatSensor<String>(read_delay, getMotionSensorValues);
    sensor_motion->connect_to((new sensesp::SKOutputRawJson(sk_path_motion[MPU6050_ATTITUDE])));

    // TEST
    //RepeatSensor<Quaternion> *sensor_motion_quaternion = new RepeatSensor<Quaternion>(read_delay, getMotionSensorQuaternion);
    //LambdaTransform<Quaternion, String> *sensor_motion_ypr = new LambdaTransform<Quaternion, String>(getMotionSensorYPR);
    //sensor_motion_quaternion->connect_to(sensor_motion_ypr);
    //sensor_motion_ypr->connect_to(new SKOutputRawJson(sk_path_motion[MPU6050_ATTITUDE]));

    #ifndef FAKE_MODE
    sensor_compass = new sensesp::RepeatSensor<float>(read_delay, getCompassSensorValue);
    #else
    sensor_compass = new sensesp::FloatConstantSensor(1.0, read_delay);
    #endif
    sensor_compass->connect_to(new sensesp::SKOutputFloat(sk_path_motion[MPU6050_HEADING],
                  new sensesp::SKMetadata("rad", "Cap compas", "Current magnetic heading received from the compass", "Cap compas")));
  }
  #ifdef DEBUG_MODE
  else {
    Serial.print(F("DMP Initialization failed (code ")); // Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  #endif
}

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