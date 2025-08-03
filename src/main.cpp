/*
  @TODO :
  P1
  - Ajouter une moyenne de consommation de carburant réelle
  - Intégrer la lib MPU9250
  P2
  - Surcharger la classe SKMetaData pour envoyer les données de zones

  Volvo Penta digital dashboard :
  - Coolant temperature (°C, F)
  - Oil pressure (Bar, PSI)
  - Voltage (V)
  - Tachometer (RPM)
  - Fuel consumption (L/hr)
  - Windlass chain counter (Meter, persistent)
  - Fuel tank level (L)
  - Fresh water tank level (L)
  - Exhaust temperature (°C, F)
  - Engine room temperature (°C, F)
  - Motion sensor (Roll / Pitch / Yaw)

  @author kinyo666
  @version 1.0.16
  @date 03/08/2025
  @ref SensESP v3.1.0
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
  @link SensESP Documentation : https://signalk.org/SensESP/
  @link Thanks to Mat Baileys (Boating with the Baileys) : https://github.com/Boatingwiththebaileys/ESP32-code
  @link Thanks to Jason Greenwood (Après) : https://github.com/Techstyleuk
  @link https://www.arduino.cc/reference/en/libraries/ina3221/
  @link https://signalk.org/specification/1.7.0/schemas/definitions.json
  */
#include <sensesp.h>
#include <sensesp_app_builder.h>
//#include <sensesp/transforms/repeat.h>
#include <sensesp/sensors/constant_sensor.h>
#include <sensesp/sensors/digital_input.h>
#include <sensesp/signalk/signalk_output.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
//https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU9250
//https://registry.platformio.org/libraries/mbed-anything-connected/MPU9250/installation
//#include <MPU9250.h>
#include "customClasses.h"
#include "sensors/chain_counter_sensor.h"
#include "sensors/temperature_sensor.h"
#include "sensors/voltage_sensor.h"
//#include "locals.h"

// RPM sensors PC817
#define PC817_BABORD_PIN 35
#define PC817_TRIBORD_PIN 34

// Use Debug Mode for verbose logs and Fake Mode to simulate some data
#define DEBUG_MODE 1
//#define FAKE_MODE 1
//#define SIMULATE_RPM 1
//#define SIMULATE_RPM_PIN 13

// Motion sensor MPU6050
#define MPU6050_ATTITUDE 0
#define MPU6050_HEADING 1
#define MPU6050_OFFSETS 2

// Local languages
#define LANG_EN 0
#define LANG_FR 1

using namespace sensesp;
using namespace sensesp::onewire;

// Constants for Signal-K path
const String sk_path_engines[ENGINE_NB][ENGINE_SK_PATH_NB] = {
        {"propulsion.babord.revolutions",   "propulsion.babord.fuel.rate",  "propulsion.babord.state"},
        {"propulsion.tribord.revolutions",  "propulsion.tribord.fuel.rate", "propulsion.tribord.state"}};

const String sk_path_motion[3] = { "navigation.attitude", "navigation.headingMagnetic", "navigation.offsets" };

// SensESP Configuration
const String conf_path_global = "/CONFIG/SENSORS_CONFIG";
const String conf_path_engines[ENGINE_NB][2] = {{"/CONFIG/BABORD/PC817/FREQUENCY_RPM", "/CONFIG/BABORD/PC817/MOVING_AVG"},
                                                {"/CONFIG/TRIBORD/PC817/FREQUENCY_RPM", "/CONFIG/TRIBORD/PC817/MOVING_AVG"}}; 

const String conf_path_motion = "/CONFIG/MOTION/OFFSETS";

const unsigned int sensor_read_delay = 1000;                         // Sensors read delay = 1s
ConfigSensESP *sensesp_config;                                // Sensors activation

// Engines state and timer (in seconds) to power-down sensors
enum engine_sk_path_t { REVOLUTIONS = 0, FUELRATE, STATE };


#ifndef FAKE_MODE
DigitalInputCounter *sensor_engine[ENGINE_NB];                // 2 PC817 RPM Sensors values
#else
FloatConstantSensor *sensor_engine[ENGINE_NB];                // Fake RPM Sensor values
#endif

// Motion and compass sensor
MPU6050 sensor_mpu;                                           // 1 MPU-6050 Motion sensor
#ifndef FAKE_MODE
RepeatSensor<float> *sensor_compass;                          // MPU Compass values
#else
FloatConstantSensor *sensor_compass;                          // Fake Compass values
#endif

// SensESP builds upon the ReactESP framework. Every ReactESP application must instantiate the "app" object.
reactesp::EventLoop app;

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

/* https://signalk.org/specification/1.7.0/doc/data_model_metadata.html
  {
    "displayName": "Port Tachometer",
    "longName": "Engine 2 Tachometer",
    "shortName": "Tacho",
    "description": "Engine revolutions (x60 for RPM)",
    "units": "Hz",
    "timeout": 1,
    "displayScale": {"lower": 0, "upper": 75, "type": "linear"},
    "alertMethod": ["visual"],
    "warnMethod": ["visual"],
    "alarmMethod": ["sound", "visual"],
    "emergencyMethod": ["sound", "visual"],
    "zones": [
      {"upper": 4, "state": "alarm", "message": "Stopped or very slow"},
      {"lower": 4, "upper": 60, "state": "normal"},
      {"lower": 60, "upper": 65, "state": "warn", "message": "Approaching maximum"},
      {"lower": 65, "state": "alarm", "message": "Exceeding maximum"}
    ]
  }

| State/Zone | Description |
|------------|--------|--------| 
| nominal | this is a special type of normal state/zone (see below) | 
| normal | the normal operating range for the value in question (default) | 
| alert | Indicates a safe or normal condition which is brought to the operators attention to impart information for routine action purposes | 
| warn | Indicates a condition that requires immediate attention but not immediate action | 
| alarm | Indicates a condition which is outside the specified acceptable range. Immediate action is required to prevent loss of life or equipment damage | 
| emergency | the value indicates a life-threatening condition |
*/
SKMetadata getEnginesSKMetadata() {
  SKMetadata *engine_skmeta = new SKMetadata("Hz", "Moteur (RPM)", "Engine revolutions (x60 for RPM)", "RPM");
  JsonDocument jsondoc;
  JsonArray jsondocarray = jsondoc.to<JsonArray>();

  engine_skmeta->add_entry(sk_path_engines[ENGINE_BABORD][REVOLUTIONS], jsondocarray);

  if (jsondocarray[0].is<JsonObject>() && jsondocarray[0]["value"].is<JsonObject>()) {
    JsonObject jsondocvalue = jsondocarray[0]["value"];
    JsonArray jsondoczones = jsondocvalue["zones"].to<JsonArray>();
    JsonObject jsondoczones1 = jsondoczones.add<JsonObject>();
    jsondoczones1["upper"] = 8;
    jsondoczones1["state"] = "warn";
    jsondoczones1["message"] = "Moteur calé ou trop lent";
    JsonObject jsondoczones2 = jsondoczones.add<JsonObject>();
    jsondoczones2["lower"] = 8;
    jsondoczones2["upper"] = 42;
    jsondoczones2["state"] = "normal";
    JsonObject jsondoczones3 = jsondoczones.add<JsonObject>();
    jsondoczones3["lower"] = 42;
    jsondoczones3["upper"] = 50;
    jsondoczones3["state"] = "nominal";
    JsonObject jsondoczones4 = jsondoczones.add<JsonObject>();
    jsondoczones4["lower"] = 50;
    jsondoczones4["upper"] = 68;
    jsondoczones4["state"] = "alert";
    JsonObject jsondoczones5 = jsondoczones.add<JsonObject>();
    jsondoczones5["lower"] = 50;
    jsondoczones5["upper"] = 68;
    jsondoczones5["state"] = "alarm";
    jsondoczones5["message"] = "Régime moteur excessif !";
  }
  #ifdef DEBUG_MODE
  else
      Serial.println("JSON : Condition KO");
      String jsonify;
      serializeJsonPretty(jsondoc, jsonify);
      Serial.print("JSON :");
      Serial.println(jsonify);
  #endif

  return *engine_skmeta;
}

// Simulate RPM for testing purposes
// This function simulates a pulse on the PC817 sensor to generate a constant RPM value
#ifdef SIMULATE_RPM
float simulateRPM() {
  digitalWrite(SIMULATE_RPM_PIN, HIGH);  // Simulate a pulse on the PC817 sensor
  delay(10);                             // Wait for 10ms = 50 Hz
  digitalWrite(SIMULATE_RPM_PIN, LOW);   // Stop the pulse
  delay(10);
  return 50.0; // Return a constant value for testing purposes
}
#endif

// Setup the PC817 sensor for engine RPM
void setupPC817Sensor(u_int8_t engine_id, String engine, u_int8_t PC817_pin) {
  // Step 1 : calibrate the multiplier with known values (CurveInterpolator class) from analog gauge
  //const float multiplier = 1.0 / 97.0;
  const float multiplier = 1.0;
  EngineDataTransform *engine_data = new EngineDataTransform(multiplier, conf_path_engines[engine_id], engine_id);

  // Step 2 : instanciates DigitalInputCounter to read PC817 sensor values
  #ifndef FAKE_MODE
  const unsigned int read_rpm_delay = 500;
  sensor_engine[engine_id] = new DigitalInputCounter(PC817_pin, INPUT, RISING, read_rpm_delay);   // INPUT for GPIO 32-35 or INPUT_PULLUP for others pins
  #else
  sensor_engine[engine_id] = new FloatConstantSensor(50.0, 1, "/config/engine/EL817/CONSTANT");   // Fake sensor for test purpose
  #endif
  #ifdef DEBUG_MODE
  sensor_engine[engine_id]->connect_to(new SKOutputFloat(sk_path_engines[engine_id][REVOLUTIONS] + ".raw"));
  #endif

  // Step 3 : connects the output of sensor to the input of Frequency() and the Signal K output as a float in Hz
  sensor_engine[engine_id]
    ->connect_to(engine_data->freq)
    ->connect_to(engine_data->moving_avg)
    ->connect_to(new SKOutputFloat(sk_path_engines[engine_id][REVOLUTIONS], 
                  new SKMetadata("Hz", "Moteur " + engine, "Engine revolutions (x60 for RPM)", "RPM")));

  // Step 4 : transforms the output of Frequency to RPM and Liter per hour from FuelConsumption
  engine_data->freq
    ->connect_to(engine_data->hz_to_rpm)
    ->connect_to(engine_data->rpm_to_lhr)
    ->connect_to(engine_data->lhr_to_m3s)
    ->connect_to(new SKOutputFloat(sk_path_engines[engine_id][FUELRATE],
                  new SKMetadata("m3/s", "Conso " + engine, "Fuel rate of consumption", "L/hr")));

  // Step 5 : changes the engine state if needed
  engine_data->freq
    ->connect_to(engine_data->running_state)
    ->connect_to(new SKOutputBool(sk_path_engines[engine_id][STATE]));

    // TODO : faire un connect_to vers un LambdaTransform pour envoyer l'état du moteur
    /*
      if (sensesp_config->is_enabled("INA3221_POWERDOWN"))
    sleepModeINA3221(engine_id, ((input > 1) ? ENGINE_STATE_RUNNING : ENGINE_STATE_NOT_RUNNING));*/
}

/* Measure Engines RPM

  multiplier_port and multiplier_starboard needs to be adjusted based on flywheel to Alternator W ratio.
  Example: 1 turn on an AQAD41 makes 2.45 turns on the Alternator, and the Alternator has 6 poles which makes a total of 2.45 x 6 = 14.7 Hz per turn.
  SignalK needs info in Hz so we need to divide the incoming value with 14.7, or as in our case multiply with (1/14.7) = 0,06803
  If Ratio is unknown, the original Tachometer might have a max Impulses/min written on it, divide that with max rpm on the meter and you'll get the ratio.
  Tachometer 860420 is marked with 73500Imp/min and has a scale to 5000rpm. 73500 divided with 5000 equals 14,7, Tada!
  If above multiplier needs to be recalculated, the FuelMultipliers needs to be recalculated as well, they're both based on AQAD41 values right now.
  AQAD41 example: 60 divided with FlyWheel To W Ratio (60 / 14,7 = 4,08)
*/
void setupRPMSensors() {
    if (sensesp_config->is_enabled("PC817_BABORD"))
      setupPC817Sensor(ENGINE_BABORD, "Babord", PC817_BABORD_PIN);
    if (sensesp_config->is_enabled("PC817_TRIBORD"))
      setupPC817Sensor(ENGINE_TRIBORD, "Tribord", PC817_TRIBORD_PIN);
    
    #ifdef SIMULATE_RPM
    pinMode(SIMULATE_RPM_PIN, OUTPUT); // Simulate RPM for testing purposes
    RepeatSensor<float> *sensor_simulate_rpm = new RepeatSensor<float>(100, simulateRPM);
    #endif
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

  // Sends the offsets to the Signal K server and save the motion sensor offsets
  StringConstantSensor *sensor_string_offsets = new StringConstantSensor(motion_string_offsets, 600);
  //sensor_motion_offsets = new MotionSensorOffsets(conf_path_motion);
  sensor_string_offsets->connect_to(sensor_motion_offsets);
  sensor_motion_offsets->connect_to(new SKOutputRawJson(sk_path_motion[MPU6050_OFFSETS]));
}

// Setup the motion sensor MPU6050
// This function initializes the MPU6050 motion sensor, calibrates it if needed, and sets the offsets.
// @link https://registry.platformio.org/libraries/electroniccats/MPU6050
void setupMotionSensor() {
  uint8_t devStatus;                  // Return status after each device operation (0 = success, !0 = error)

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
    ConfigItem(sensor_motion_offsets)
      ->set_title("Gyroscope - Offsets")
      ->set_description("Offsets - MPU6050 - MotionSensorOffsets")
      ->set_sort_order(UI_ORDER_MOTION);

    Serial.print(F("Enabling DMP..."));   // Turning ON DMP
    sensor_mpu.setDMPEnabled(true);
    Serial.println(F("DMP ready !"));
    uint16_t packetSize = sensor_mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison

    // Read motion sensor values every read_delay milliseconds
    RepeatSensor<String> *sensor_motion = new RepeatSensor<String>(sensor_read_delay, getMotionSensorValues);
    sensor_motion->connect_to((new SKOutputRawJson(sk_path_motion[MPU6050_ATTITUDE])));

    // TEST
    //RepeatSensor<Quaternion> *sensor_motion_quaternion = new RepeatSensor<Quaternion>(read_delay, getMotionSensorQuaternion);
    //LambdaTransform<Quaternion, String> *sensor_motion_ypr = new LambdaTransform<Quaternion, String>(getMotionSensorYPR);
    //sensor_motion_quaternion->connect_to(sensor_motion_ypr);
    //sensor_motion_ypr->connect_to(new SKOutputRawJson(sk_path_motion[MPU6050_ATTITUDE]));

    #ifndef FAKE_MODE
    sensor_compass = new RepeatSensor<float>(sensor_read_delay, getCompassSensorValue);
    #else
    sensor_compass = new FloatConstantSensor(1.0, sensor_read_delay);
    #endif
    sensor_compass->connect_to(new SKOutputFloat(sk_path_motion[MPU6050_HEADING],
                  new SKMetadata("rad", "Cap compas", "Current magnetic heading received from the compass", "Cap compas")));
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

// The setup function performs one-time application initialization.
void setup() {
  #ifdef DEBUG_MODE
  SetupLogging(ESP_LOG_DEBUG); //ESP_LOG_VERBOSE); 
  #else
  SetupLogging(ESP_LOG_INFO);
  #endif
  Serial.begin(115200);

  // Create the global SensESPApp() object.
  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("birchwood-ESP32")->get_app();
  sensesp_config = new ConfigSensESP(conf_path_global);

  if (sensesp_config->is_enabled("DS18B20_FEATURE"))
    setupTemperatureSensors(sensesp_config);
  if (sensesp_config->is_enabled("INA3221_FEATURE"))
    setupVoltageSensors(sensesp_config);
  if (sensesp_config->is_enabled("PC817_FEATURE"))
    setupRPMSensors();
  if (sensesp_config->is_enabled("MOTION_SENSOR_FEATURE"))
    setupMotionSensor();
  if (sensesp_config->is_enabled("CHAIN_COUNTER_FEATURE"))
    setupWindlassSensor();
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop() { 
  static auto event_loop = SensESPBaseApp::get_event_loop();
  event_loop->tick();
}