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
  @version 1.0.14
  @date 31/07/2025
  @ref SensESP v3.1.0
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
  @link SensESP Documentation : https://signalk.org/SensESP/
  @link Thanks to Mat Baileys (Boating with the Baileys) : https://github.com/Boatingwiththebaileys/ESP32-code
  @link Thanks to Jason Greenwood (Après) : https://github.com/Techstyleuk
  @link https://www.arduino.cc/reference/en/libraries/ina3221/
  @link https://signalk.org/specification/1.7.0/schemas/definitions.json
  */
#include <INA3221.h>
#include <sensesp.h>
#include <sensesp_app_builder.h>
#include <sensesp/transforms/repeat.h>
#include <sensesp/sensors/constant_sensor.h>
#include <sensesp/sensors/digital_input.h>
#include <sensesp/signalk/signalk_output.h>
#include <sensesp_onewire/onewire_temperature.h>
#include "MPU6050_6Axis_MotionApps20.h"
//https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU9250
//https://registry.platformio.org/libraries/mbed-anything-connected/MPU9250/installation
//#include <MPU9250.h>
#include "ADS1115-Driver.h"
#include "customClasses.h"
//#include "locals.h"
#include "Wire.h"

// Temperature sensors DS18B20
#define DS18B20_PIN 33                                    // Use ADC1 pin (GPIO32-35) rather than ADC2 pin to avoid Wifi interference
#define DS18B20_BABORD_0 0
#define DS18B20_TRIBORD_1 1
#define DS18B20_COMMON_2 2
#define DS18B20_NB 3
#define DS18B20_BABORD_ADDR "28:61:64:34:35:a9:96:d9"     // Replace your 1-Wire addresses here
#define DS18B20_TRIBORD_ADDR "28:ff:64:1f:74:6f:1f:0b"
#define DS18B20_COMMON_ADDR "28:ff:64:1f:75:83:99:c9"

// Volt / Current sensors INA3221
#define INA3221_BABORD_0 0                                // I2C address = 0x40 GND
#define INA3221_CUVES_1 1                                 // I2C address = 0x41 VS
#define INA3221_TRIBORD_2 2                               // I2C address = 0x42 SDA
#define INA3221_OTHERS_3 3                                // I2C address = 0x43 SCL
#define INA3221_NB 4

// RPM sensors PC817
#define PC817_BABORD_PIN 35
#define PC817_TRIBORD_PIN 34

// Engines
#define ENGINE_BABORD 0
#define ENGINE_TRIBORD 1
#define ENGINE_NB 2
#define ENGINE_SK_PATH_NB 3
#define ENGINE_SLEEP_TIMER 60

// Windlass Chain counter
#define CHAIN_COUNTER_PIN 32
#define CHAIN_COUNTER_SAVE_TIMER 20                       // Save the chain counter value every 20 cycles (20 * read_delay_windlass = 10 seconds)
#define CHAIN_COUNTER_IGNORE_DELAY 400                    // Delay to ignore multiples chain counter readings. Must be less than read_delay_windlass
#define CHAIN_COUNTER_PATH 0
#define CHAIN_COUNTER_DELAY 1
#define CHAIN_COUNTER_NB 2
#define CHAIN_COUNTER_UI_CYCLE 2                          // Number of cycles to wait before refreshing the UI symbol direction to IDLE
#define ADS1115_WINDLASS_ADDR ADS1115_I2C_ADDR_VDD        // I2C address = 0x49 VDD
#define ADS1115_WINDLASS_THRESHOLD 26400                  // Windlass threshold (depending on the HSTS016L sensor and PGA used)

// Use Debug Mode for verbose logs and Fake Mode to simulate some data
#define DEBUG_MODE 1
//#define FAKE_MODE 1
//#define SIMULATE_RPM 1
//#define SIMULATE_RPM_PIN 13

// SensESP UI Config order
#define UI_ORDER_TEMP 10
#define UI_ORDER_ENGINE 20
#define UI_ORDER_TANK 30
#define UI_ORDER_CHAIN 40
#define UI_ORDER_MOTION 50

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
const String sk_path_temp[DS18B20_NB] = {
        "propulsion.babord.exhaustTemperature",
        "propulsion.tribord.exhaustTemperature",
        "environment.inside.engineroom.temperature"};
const String sk_path_volt[INA3221_NB][INA3221_CH_NUM] = {
        {"propulsion.babord.coolantTemperature",  "propulsion.babord.oilPressure",    "propulsion.babord.alternatorVoltage"},
        {"tanks.fuel.babord.currentVolume",       "tanks.fuel.tribord.currentVolume", "tanks.freshWater.eaudouce.currentVolume"},
        {"propulsion.tribord.coolantTemperature", "propulsion.tribord.oilPressure",   "propulsion.tribord.alternatorVoltage"}};
const String sk_path_engines[ENGINE_NB][ENGINE_SK_PATH_NB] = {
        {"propulsion.babord.revolutions",   "propulsion.babord.fuel.rate",  "propulsion.babord.state"},
        {"propulsion.tribord.revolutions",  "propulsion.tribord.fuel.rate", "propulsion.tribord.state"}};
const String sk_path_windlass = "navigation.anchor.rodeDeployed";
const String sk_path_motion[3] = { "navigation.attitude", "navigation.headingMagnetic", "navigation.offsets" };
const String sk_path_rudder = "propulsion.rudderAngle";

// SensESP Configuration
const String conf_path_global = "/CONFIG/SENSORS_CONFIG";
const String conf_path_volt[INA3221_NB][INA3221_CH_NUM] = {
        {"/CONFIG/BABORD/INA3221/0/LINEAR_CH1",   "/CONFIG/BABORD/INA3221/0/LINEAR_CH2",  "/CONFIG/BABORD/INA3221/0/LINEAR_CH3"},
        {"/CONFIG/CUVES/INA3221/1/LINEAR_CH1",    "/CONFIG/CUVES/INA3221/1/LINEAR_CH2",   "/CONFIG/CUVES/INA3221/1/LINEAR_CH3"},
        {"/CONFIG/TRIBORD/INA3221/2/LINEAR_CH1",  "/CONFIG/TRIBORD/INA3221/2/LINEAR_CH2", "/CONFIG/TRIBORD/INA3221/2/LINEAR_CH3"}};
const String conf_path_engines[ENGINE_NB][2] = {{"/CONFIG/BABORD/PC817/FREQUENCY_RPM", "/CONFIG/BABORD/PC817/MOVING_AVG"},
                                                {"/CONFIG/TRIBORD/PC817/FREQUENCY_RPM", "/CONFIG/TRIBORD/PC817/MOVING_AVG"}}; 
const String conf_path_chain[CHAIN_COUNTER_NB] = { "/CONFIG/CHAINE/COUNTER", "/CONFIG/CHAINE/DELAY" };
const String conf_path_motion = "/CONFIG/MOTION/OFFSETS";
const String conf_path_rudder = "/CONFIG/RUDDER";
const float gipsy_circum = 0.43982;                           // Windlass gipsy circumference (meter) - r = 85 mm ; c = 0.534071 m
const unsigned int read_delay_windlass = 500;                 // Windlass read delay = 0.5s
const unsigned int read_delay = 1000;                         // Sensors read delay = 1s
ConfigSensESP *sensesp_config;                                // Sensors activation

// Engines state and timer (in seconds) to power-down sensors
enum engine_sk_path_t { REVOLUTIONS = 0, FUELRATE, STATE };
u_int8_t engine_state[ENGINE_NB] = { 0, 0 };
int engine_timer[ENGINE_NB] = { ENGINE_SLEEP_TIMER * 1000 / read_delay, ENGINE_SLEEP_TIMER * 1000 / read_delay };

// Temperature & Voltage sensors
OneWireTemperature* sensor_temp[DS18B20_NB];                  // 3 DS18B20 Temperature values
INA3221 *sensor_INA3221[INA3221_NB];                          // 4 INA3221 Voltage Sensors with 3 channels each
RepeatSensor<float> *sensor_volt[INA3221_NB][INA3221_CH_NUM]; // 4 Voltage values for each channel
#ifndef FAKE_MODE
DigitalInputCounter *sensor_engine[ENGINE_NB];                // 2 PC817 RPM Sensors values
#else
FloatConstantSensor *sensor_engine[ENGINE_NB];                // Fake RPM Sensor values
#endif

// Windlass sensor
ADS1115 *sensor_ADS1115;                                      // ADS1115 sensor for windlass motor
Transform<int, int> *sensor_windlass_debounce;                // Windlass sensor (DigitalInputCounter + Debounce)
PersistentIntegrator *chain_counter;                          // Windlass chain counter
JsonDocument jdoc_conf_windlass;
JsonObject conf_windlass;                                     // Windlass direction of rotation (UP or DOWN) and last value if exists
unsigned int chain_counter_timer = 0;                         // Timer to save chain counter value
boolean chain_counter_saved = true;                           // Trigger to save chain counter new value
enum { WINDLASS_IDLE = 0, WINDLASS_UP = 1, WINDLASS_DOWN = -1 };
int windlass_state = WINDLASS_IDLE;

// Motion and compass sensor
MPU6050 sensor_mpu;                                           // 1 MPU-6050 Motion sensor
#ifndef FAKE_MODE
RepeatSensor<float> *sensor_compass;                          // MPU Compass values
#else
FloatConstantSensor *sensor_compass;                          // Fake Compass values
#endif

// SensESP builds upon the ReactESP framework. Every ReactESP application must instantiate the "app" object.
reactesp::EventLoop app;

// Filter Kelvin values out of range
auto filterKelvinValues = [](float input, float offset = 273.15, float max = 413.1) -> float {
    return ((((input - offset) > 0) && (input < max)) ? input : offset);
};
const ParamInfo* filterKelvinValues_ParamInfo = new ParamInfo[2]{{"offset", "Offset"}, {"max", "Max value"}};

// Callback for LambdaTransform to add a UP/DOWN/IDLE symbol to the chain counter
auto chainCounterToString = [](float input) -> String {
  String chainCounterString = String(input) + "m ";
  switch (windlass_state) {
    case WINDLASS_UP:   { chainCounterString += "▲"; break; }
    case WINDLASS_DOWN: { chainCounterString += "▼"; break; }
    default:            { chainCounterString += "▬"; }
  }
  return chainCounterString;
};

/* Setup the INA3221 volt sensors
  The INA3221 must have an I²C address on A0 pin, solder the right pin GND (0x40) / SDA (0x41) / SCL (0x42) / VS (0x43)
  - On a Raspberry Pi, the I²C bus is required to be enabled
  - On an ESP32, default SDA pin = 21 & SCL pin = 22
  The Configuration Register is set to continious bus-voltage measurement with 4 values averaged (to avoid noise)
  @see https://www.ti.com/product/INA3221
  @param index Slot for INA3221 < INA3221_NB
  @param addr Physical A0 address of the INA3221 sensor
  @return boolean True if the sensor has been found, False otherwise
*/
boolean setupSensorINA3221(u_int8_t index, ina3221_addr_t addr = INA3221_ADDR40_GND) {
  sensor_INA3221[index] = new INA3221(addr);                        // Set I2C address to addr (default = 0x40 GND)
  sensor_INA3221[index]->begin(&Wire);                              // Default shunt resistors = 10 mOhm (R100)
  sensor_INA3221[index]->reset();                                   // Reset the INA3221 to default settings
  sensor_INA3221[index]->setShuntMeasDisable();                     // Disable shunt-voltage measurement
  sensor_INA3221[index]->setAveragingMode(INA3221_REG_CONF_AVG_4);  // Set averaging mode to 4 samples
  sensor_INA3221[index]->setModeContinious();                       // Set mode to continious measurement (110)
  
  delay(10);                                                        // Wait 10 ms for the next sensor setup
  #ifdef DEBUG_MODE
  Serial.printf("Setup Sensor INA3221 0x%X completed : ManufID = %i ; DieID = %i\n", addr, sensor_INA3221[index]->getManufID(), sensor_INA3221[index]->getDieID());
  #endif
  return (sensor_INA3221[index]->getDieID() != 0);  // True if the sensor has been found
}

// Callbacks for Sensor Volt
float getVoltageINA3221_BABORD0_CH1() { return sensor_INA3221[INA3221_BABORD_0]->getVoltage(INA3221_CH1); }    // Coolant temperature
float getVoltageINA3221_BABORD0_CH2() { return sensor_INA3221[INA3221_BABORD_0]->getVoltage(INA3221_CH2); }    // Oil pressure
float getVoltageINA3221_BABORD0_CH3() {                                                                        // Voltage
  float volt = sensor_INA3221[INA3221_BABORD_0]->getVoltage(INA3221_CH3);
  if (volt < 1)
    engine_state[ENGINE_BABORD] = ENGINE_STATE_OFF;    // Engine state is OFF and not running
  else
    engine_state[ENGINE_BABORD] |= ENGINE_STATE_ON;    // Engine state is ON (and maybe running or not)
  return volt;
}
float getVoltageINA3221_CUVES1_CH1() { return sensor_INA3221[INA3221_CUVES_1]->getVoltage(INA3221_CH1); }      // Fuel tank volume
float getVoltageINA3221_CUVES1_CH2() { return sensor_INA3221[INA3221_CUVES_1]->getVoltage(INA3221_CH2); }      // Fuel tank volume
float getVoltageINA3221_CUVES1_CH3() { return sensor_INA3221[INA3221_CUVES_1]->getVoltage(INA3221_CH3); }      // Water tank volume
float getVoltageINA3221_TRIBORD2_CH1() { return sensor_INA3221[INA3221_TRIBORD_2]->getVoltage(INA3221_CH1); }  // Coolant temperature
float getVoltageINA3221_TRIBORD2_CH2() { return sensor_INA3221[INA3221_TRIBORD_2]->getVoltage(INA3221_CH2); }  // Oil pressure
float getVoltageINA3221_TRIBORD2_CH3() {                                                                       // Voltage
  float volt = sensor_INA3221[INA3221_TRIBORD_2]->getVoltage(INA3221_CH3);
  if (volt < 1)
    engine_state[ENGINE_TRIBORD] = ENGINE_STATE_OFF;    // Engine state is OFF and not running
  else
    engine_state[ENGINE_TRIBORD] |= ENGINE_STATE_ON;    // Engine state is ON (and maybe running or not)
  return volt;
}
float getVoltageINA3221_OTHERS3_CH3() { return sensor_INA3221[INA3221_OTHERS_3]->getVoltage(INA3221_CH3); }    // Radius Angle

// Read the value from the ADS1115 sensor (A0 or A1 input pin)
uint16_t readValue(uint8_t input) {
	sensor_ADS1115->setMultiplexer(input);
	sensor_ADS1115->startSingleConvertion();

	delayMicroseconds(25); // The ADS1115 needs to wake up from sleep mode and usually it takes 25 uS to do that

	while (sensor_ADS1115->getOperationalStatus() == 0);

	return sensor_ADS1115->readConvertedValue();
}

/* Callbacks for Windlass UP/DOWN buttons sensor
   This function determines :
   - In which direction the windlass turns (UP or DOWN)
   - Whether the last value should be saved (persistent storage)

   The direction UP (or DOWN) is derived from the current of the UP cable to the windlass motor.
   - If the current is above a certain threshold, it means the windlass is turning UP (-k)
   - If the current is below and a signal is received from the gypsy inductive sensor, it means the windlass is turning DOWN (+k)
   Once we know the direction, we can change the sign (+/-) of the chain_counter accumulator k
   It is not necessary to change it each time the button is pressed : only when the direction is reversed.
   In order to show the direction on the UI a few seconds, the windlass state is set to IDLE after CHAIN_COUNTER_UI_CYCLE cycles without UP/DOWN event.
   
   We use the Repeat Sensor to arm a timer that saves the last chain_counter value after CHAIN_COUNTER_SAVE_TIMER cycles.
   To avoid infinite increment of the timer when there is no UP/DOWN event and multiples saves, the timer is set to 0 
   when the last value has already been saved.

   NOTE(1) : If the global read_delay_windlass is set to more or less than one second, the number of cycles before saving SHOULD be modified
   NOTE(2) : A cycle is defined as one revolution of the gypsy, so the read_delay_windlass MUST have a lower value than the revolution time
   NOTE(3) : DOWN = chain_counter + gypsy_circum (addition) ; UP = chain_counter - gypsy_circum (substraction)

   @returns float - Current measured for the channel ADS1115_A0
*/
float getVoltageADS1115_A0() { 
  float direction = conf_windlass["k"].as<float>();
  float threshold = conf_windlass["threshold"].as<float>();
  float up = readValue(ADS1115_MUX_AIN0_GND);

  if (up > threshold) {                                                                     // Windlass IDLE/DOWN -> UP
    windlass_state = WINDLASS_UP;                                                           // Set the windlass state to UP    
    if (direction > 0) {                                                                    // If the windlass direction was DOWN
      conf_windlass["k"] = -1.0 * direction;                                                // Reverse direction
      chain_counter->set_direction(conf_windlass["k"]);                                     // Set the new config direction
    }
    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : UP -> up = %f \t| direction = %f\n", up, conf_windlass["k"].as<float>());
    #endif
  }
  else if ((windlass_state == WINDLASS_DOWN) && (up <= threshold)) {                         // Windlass IDLE/UP -> DOWN
    if (direction < 0) {                                                                     // If the windlass direction was UP
      conf_windlass["k"] = abs(direction);                                                   // Reverse direction
      chain_counter->set_direction(conf_windlass["k"]);                                      // Set the new config direction
    }
    chain_counter_timer++;
    if (chain_counter_timer >= CHAIN_COUNTER_UI_CYCLE)                                       // Wait N cycles before resetting the windlass state on the UI
      windlass_state = WINDLASS_IDLE;                                                        // Return to the windlass state IDLE

    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : DOWN -> up = %f \t| direction = %f | timer = %i | windlass_state = %s\n", up, 
                  conf_windlass["k"].as<float>(), chain_counter_timer, 
                  (windlass_state == WINDLASS_DOWN) ? "DOWN" : "IDLE");
    #endif
  }
  else if ((chain_counter_saved == false) && (chain_counter_timer == CHAIN_COUNTER_SAVE_TIMER)) {
    windlass_state = WINDLASS_IDLE;                                                          // Set the windlass state to IDLE
    conf_windlass["k"] = abs(direction);                                                     // Reset the default direction to DOWN
    chain_counter->set_direction(conf_windlass["k"]);                                        // Set the new config direction
    chain_counter_saved = chain_counter->save();                                             // Save last config and value to local file system
    if (chain_counter_saved)
      chain_counter_timer = 0;                                                               // Reset timer to 0
    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : IDLE -> saved = %s\n", chain_counter_saved ? "true" : "false");
    #endif
  }
  else {
    chain_counter_timer = ((chain_counter_saved == true) ? 0 : (chain_counter_timer + 1));   // Avoid infinite increment when no changes
    if (chain_counter_timer >= CHAIN_COUNTER_UI_CYCLE)                                       // Wait N cycles before resetting the windlass state on the UI
      windlass_state = WINDLASS_IDLE;                                                        // Return to the windlass state IDLE
    if (direction < 0) {
      conf_windlass["k"] = abs(direction);                                                   // Reset the default direction to DOWN
      chain_counter->set_direction(conf_windlass["k"]);                                      // Set the new config direction
    }
    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : IDLE -> up = %f \t| direction = %f | timer = %i | saved = %s\n", up, 
                  conf_windlass["k"].as<float>(), chain_counter_timer, chain_counter_saved ? "true" : "false");
    #endif
  }

  return up;
}

// Callback for the gypsy inductive sensor
// We use a 5 kΩ pull-up resistor on the ADS1115 A1 input pin to read the gypsy inductive sensor (+3.3V = open, 0 = closed)
float getVoltageADS1115_A1() {
  return readValue(ADS1115_MUX_AIN1_GND);
}

// Callback for chain counter
// This function is called after each sensor_windlass value changes (after debounce)
void handleChainCounterChange() {
  int value = sensor_windlass_debounce->get();  // Get the current value from the debounced sensor
  if (value == 0) {
    chain_counter_saved = false;                // Set the last value status to 'not saved'
    chain_counter_timer = 0;                    // Arm a timer to save chain counter last value after CHAIN_COUNTER_SAVE_TIMER cycles
    if (windlass_state == WINDLASS_IDLE) 
      windlass_state = WINDLASS_DOWN;           // Set the windlass state to DOWN if it was IDLE (gypsy freewheel) 

    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : NEW VALUE TO SAVE = %i | windlass_state = %s\n", value, 
                  (windlass_state == WINDLASS_UP) ? "UP" : ((windlass_state == WINDLASS_DOWN) ? "DOWN" : "IDLE"));
    #endif
  }
}

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

/* Callback for setting INA3221 device in active or power-down mode when engine is off
   - if RPM value equals to 0, it means the engine is not running
   - if Voltage value equals to 0, it means the engine is not turned on
  The INA3221 device goes into power-down mode when the engine is not runnning AND is turned off after ENGINE_SLEEP_TIMER period
  It returns to continious mode when the engine is on (but not necessarilly running)
  State machine :
    TIMER--
    ┌─┐
    OFF ────> ON ────> RUNNING
     ↑        |↑          ↓
     └────────┘└───── NOT RUNNING
  [POWERDOWN]   [ACTIVE]
*/
void sleepModeINA3221(u_int8_t engine_id, u_int8_t powerdown_mode) {
  u_int8_t INA3221_id = ((engine_id == ENGINE_BABORD) ? INA3221_BABORD_0 : INA3221_TRIBORD_2);

  if (!sensesp_config->is_enabled("INA3221_POWERDOWN"))
    return;                                                   // Do not power-down the INA3221 if the feature is disabled

  if (engine_state[engine_id] == ENGINE_STATE_OFF) {          // Engine is off
    if (powerdown_mode == ENGINE_STATE_RUNNING) {
      engine_state[engine_id] |= powerdown_mode;
      engine_timer[engine_id] = ENGINE_SLEEP_TIMER * 1000 / read_delay; // Reset the timer
      if (sensor_INA3221[INA3221_id] != NULL)                 // Check if the sensor is initialized
        sensor_INA3221[INA3221_id]->setModeContinious();      // INA3221 is now active
      #ifdef DEBUG_MODE
      Serial.printf("ENGINE : id = %i | state = %#05x -> WAKE UP\n", engine_id, engine_state[engine_id]);
      #endif
    }
    else if (engine_timer[engine_id] == 0) {                  // INA3221 is going to power-down
        engine_timer[engine_id] = -1;                         // Disable the timer
        if (sensor_INA3221[INA3221_id] != NULL)               // Check if the sensor is initialized
          sensor_INA3221[INA3221_id]->setModePowerDown();     // INA3221 is now in power-down mode
        #ifdef DEBUG_MODE
        Serial.printf("ENGINE : id = %i | state = %#05x -> POWER DOWN\n", engine_id, engine_state[engine_id]);
        #endif
    }
    else if (engine_timer[engine_id] > 0)                     // Wait for ENGINE_SLEEP_TIMER before going to power-down
      engine_timer[engine_id]--;                              // Decrease the timer
  }
  else if (powerdown_mode == ENGINE_STATE_NOT_RUNNING) {
      engine_state[engine_id] &= powerdown_mode;              // Engine is on but not running
      engine_timer[engine_id] = ENGINE_SLEEP_TIMER * 1000 / read_delay; // Reset the timer
  }
  #ifdef DEBUG_MODE
  Serial.printf("ENGINE : id = %i | timer = %i\t| ", engine_id, engine_timer[engine_id]); 
  Serial.printf("state = %#05x -> %s - %s\n", engine_state[engine_id],
                        ((engine_state[engine_id] & ENGINE_STATE_ON) ? "ON" : "OFF"),
                        ((engine_state[engine_id] & ENGINE_STATE_IS_RUNNING) ? "RUNNING" : "NOT RUNNING"));
  #endif
}

// Returns the temperature sensor location by its 1-Wire address
int getTemperatureSensorLocation(String sensor_addr) {
  if (sensor_addr == DS18B20_BABORD_ADDR)
    return DS18B20_BABORD_0;
  else if (sensor_addr == DS18B20_TRIBORD_ADDR)
    return DS18B20_TRIBORD_1;
  else if (sensor_addr == DS18B20_COMMON_ADDR)
    return DS18B20_COMMON_2;
  else
    return -1;
}

// Setup engine temperature sensors
void setupTemperatureSensors() {
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(DS18B20_PIN);
  JsonDocument jsondoc;
  JsonObject jsonsensor = jsondoc.add<JsonObject>();
  
  for (u_int8_t i = 0; i < DS18B20_NB; i++) {
    sensor_temp[i] = new OneWireTemperature(dts, read_delay);
    sensor_temp[i]->to_json(jsonsensor);

    if (jsonsensor["found"].is<bool>() && (jsonsensor["found"] == true))
      switch (getTemperatureSensorLocation(jsonsensor["address"])) {
        case DS18B20_BABORD_0 : {
          if (sensesp_config->is_enabled("DS18B20_BABORD_0"))
            sensor_temp[DS18B20_BABORD_0]
              ->connect_to(new SKOutputFloat(sk_path_temp[DS18B20_BABORD_0], "",
                          new SKMetadata("K", "Echappt Babord", sk_path_temp[DS18B20_BABORD_0], "T° Echappement Moteur Babord")));
          break;
        }
        case DS18B20_TRIBORD_1 : {
          if (sensesp_config->is_enabled("DS18B20_TRIBORD_1"))
            sensor_temp[DS18B20_TRIBORD_1]
              ->connect_to(new SKOutputFloat(sk_path_temp[DS18B20_TRIBORD_1], "",
                          new SKMetadata("K", "Echappt Tribord", sk_path_temp[DS18B20_TRIBORD_1], "T° Echappement Moteur Tribord")));
          break;
        }
        case DS18B20_COMMON_2 : {
          if (sensesp_config->is_enabled("DS18B20_COMMON_2"))
            sensor_temp[DS18B20_COMMON_2]
              ->connect_to(new SKOutputFloat(sk_path_temp[DS18B20_COMMON_2], "",
                          new SKMetadata("K", "Compartiment Moteur", sk_path_temp[DS18B20_COMMON_2], "T° Compartiment Moteur")));
          break;
        }
        default : {
          #ifdef DEBUG_MODE
          String output;
          serializeJsonPretty(jsonsensor, output);
          Serial.println("DS18B20 not declared. Constant DS18B20_xxx_ADDR needs to be modified :\n" + output);
          sleep(5);
          #endif
        }
      };
  }
}

// Setup the INA3221 Voltage sensor for engine gauges
// @param INA3221_Id Id / position in the table sensor_volt
// @param engine Engine name ("Port" or "Starboad") as it will appear under KIP Plugin
void setupVoltageEngineSensors(u_int8_t INA3221_Id, String engine) {
  LinearPos *sensor_volt_linearpos_volt2k = new LinearPos(linearPositive, 1.0, 0.0, linearPositive_ParamInfo,
                                                      conf_path_volt[INA3221_Id][INA3221_CH1] + "/LINEAR_POSITIVE");
  LinearPos *sensor_volt_linearpos_volt2pa = new LinearPos(linearPositive, pow(10, 5) * 1.4793, -0.776, linearPositive_ParamInfo,
                                                       conf_path_volt[INA3221_Id][INA3221_CH2] + "/LINEAR_POSITIVE");

  // Coolant temperature
  sensor_volt[INA3221_Id][INA3221_CH1]
    ->connect_to(sensor_volt_linearpos_volt2k)    
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH1],
                    new SKMetadata("K", "T° Eau " + engine, "Coolant temperature", "Temp " + engine)));

  ConfigItem(sensor_volt_linearpos_volt2k)
    ->set_title("Température Eau - " + engine)
    ->set_description("T° Eau - INA3221_CH1 - LinearPositive")
    ->set_sort_order(UI_ORDER_ENGINE+2);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_Id][INA3221_CH1]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH1] + ".raw"));
  #endif

  // Oil pressure
  sensor_volt[INA3221_Id][INA3221_CH2]
    ->connect_to(sensor_volt_linearpos_volt2pa)    
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH2],
                    new SKMetadata("Pa", "Pression Huile " + engine, "Oil pressure", "Huile " + engine)));

  ConfigItem(sensor_volt_linearpos_volt2pa)
    ->set_title("Pression Huile - " + engine)
    ->set_description("Pression Huile - INA3221_CH2 - LinearPositive")
    ->set_sort_order(UI_ORDER_ENGINE+5);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_Id][INA3221_CH2]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH2] + ".raw"));
  #endif

  // Voltage for Volts gauge
  sensor_volt[INA3221_Id][INA3221_CH3]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH3],
                    new SKMetadata("V", "Volt " + engine, "Alternator voltage", "Volt " + engine)));
}

// Setup the INA3221 Voltage sensors for fuel / water tanks
// @param INA3221_channel Channel number (0-2) for the INA3221 sensor
// @param location Tank location (Portside, Starboard, etc.)
// @param type Tank type (Fuel, Water, etc.)
// @param order Tank order (0-2) for the UI sort order
void setupVoltageTankSensor(u_int8_t INA3221_channel, String location, String type, u_int8_t order) {
  LinearPos *sensor_volt_linearpos = new LinearPos(linearPositive, 1.0, 0.0, linearPositive_ParamInfo,
                                                  conf_path_volt[INA3221_CUVES_1][INA3221_channel] + "/LINEAR_POSITIVE");

  sensor_volt[INA3221_CUVES_1][INA3221_channel]
    ->connect_to(sensor_volt_linearpos)
    ->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_CUVES_1][INA3221_channel] + "/MOVING_AVERAGE"))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_channel],
                  new SKMetadata("m3", "Cuve " + location, "Niveau " + type + " " + location, "Cuve " + location)));

  ConfigItem(sensor_volt_linearpos)
    ->set_title("Cuve " + type + " " + location)
    ->set_description("Cuve " + type + " " + location + " - INA3221_CH" + String(INA3221_channel) + " - LinearPositive")
    ->set_sort_order(UI_ORDER_TANK+order+1);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_CUVES_1][INA3221_channel]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_channel] + ".raw"));
  #endif
}

// Setup the INA3221 Voltage sensor for rudder angle
void setupVoltageRudderAngleSensor() {
  LinearPos *sensor_volt_linearpos = new LinearPos(linearPositive, 1.0, 0.0, linearPositive_ParamInfo,
                                                  conf_path_rudder + "/LINEAR_POSITIVE");
  sensor_volt[INA3221_OTHERS_3][INA3221_CH3]
    ->connect_to(sensor_volt_linearpos)
    ->connect_to(new SKOutputFloat(sk_path_rudder,
                  new SKMetadata("m", "Angle de barre", "Angle de barre", "Angle de barre")));
  ConfigItem(sensor_volt_linearpos)
    ->set_title("Angle de barre")
    ->set_description("Angle de barre - INA3221_CH3 - LinearPositive")
    ->set_sort_order(UI_ORDER_TANK+6);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_OTHERS_3][INA3221_CH3]
    ->connect_to(new SKOutputFloat(sk_path_rudder + ".raw"));
  #endif
}

// Check if the ADS1115 sensor is present on the I²C bus
bool isADS1115Present(uint8_t i2c_addr) {
  Wire.beginTransmission(i2c_addr);
  uint8_t error = Wire.endTransmission();
  return (error == 0); // 0 = périphérique présent
}

/* Setup the ADS1115 sensor for windlass chain counter
   The ADS1115 returns a value between 0 and 32767. We set PGA to 2.048V so when the ADS1115 reads a value :
   - on the A1 pin (+3,3V), it returns a deterministic max value equals to 38 (out of PGA range) or 0 if the reed sensor is closed (gypsy signal)
   - on the A0 pin (+/- 1,65V), it returns a value > 26400 if the HSTS016L Hall effect sensor measures a current (UP signal) 
                                or <= 26400 if the windlass is not running in UP direction
   If you want to use a higher PGA (i.e 4,096V), you MUST :
   - change the ADS1115_WINDLASS_THRESHOLD value
   - use a LambdaTransform to ignore the range of nominal values (+3,3V) before the DebounceInt sensor

   ADS1115 Formula : pin_value = pin_volt / PGA_volt * 32768
   HSTS016L Formula : volt_HSTS = 1.65V +（(I / Ipn）* 0.625V) (I = current measured and Ipn = 200A, depending on your HSTS016L model)
    
   Example 1 : I = 50A (HSTS016L current)
    volt_HSTS = 1.65V + ((50 / 200) * 0.625V) = 1.65V + 0.15625V = 1.80625V
    A0_pin_value = (1.80625 / 2.048) * 32768 = 28900

   Example 2 : I = 0A (HSTS016L current)
    volt_HSTS = 1.65V + (0 / 200) * 0.625V = 1.65V + 0.0V = 1.65V
    A0_pin_value = (1.65 / 2.048) * 32768 = 26400
*/
bool setupADS1115Sensor() {
  // Initialize the I²C bus if not already done
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.setPins(21, 22);
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // Check if the ADS1115 sensor is present
  if (!isADS1115Present(ADS1115_WINDLASS_ADDR)) {
    #ifdef DEBUG_MODE
    Serial.println("ADS1115 sensor NOT FOUND at address " + String(ADS1115_WINDLASS_ADDR, HEX));
    #endif
    return false; // Exit if the sensor is not present
  }
  else {
    // Initialize the ADS1115 sensor
    sensor_ADS1115 = new ADS1115(ADS1115_WINDLASS_ADDR);
    sensor_ADS1115->reset();
    sensor_ADS1115->setDeviceMode(ADS1115_MODE_SINGLE);
    sensor_ADS1115->setDataRate(ADS1115_DR_128_SPS);
    sensor_ADS1115->setPga(ADS1115_PGA_2_048); // Set PGA to 2.048V (default) 1 bit = 1mV @see https://forums.adafruit.com/viewtopic.php?t=186225
    #ifdef DEBUG_MODE
    Serial.println("ADS1115 sensor found at address " + String(ADS1115_WINDLASS_ADDR, HEX));
    #endif
    return true;
  }
}

// Windlass chain counter with ADS1115 and HSTS016L sensors
void setupWindlassSensor() {
  if (!setupADS1115Sensor()) {
    #ifdef DEBUG_MODE
    Serial.println("Failed to initialize ADS1115 sensor. Windlass sensor setup aborted.");
    #endif
    return; // Exit if the ADS1115 sensor is not initialized
  }

  RepeatSensor<float> *sensor_windlass_A0 = new RepeatSensor<float>(read_delay_windlass, 
                                                                 getVoltageADS1115_A0);              // Read the A0 value (windlass) from the ADS1115 sensor
  conf_windlass = jdoc_conf_windlass.to<JsonObject>();                                               // Store Windlass configuration
  conf_windlass["k"] = gipsy_circum;                                                                 // Default direction = DOWN
  conf_windlass["threshold"] = ADS1115_WINDLASS_THRESHOLD;                                           // Default threshold = 26400
  chain_counter = new PersistentIntegrator(gipsy_circum, 0.0, ADS1115_WINDLASS_THRESHOLD,
                                            conf_path_chain[CHAIN_COUNTER_PATH]);                    // Chain counter in meter
  chain_counter->to_json(conf_windlass);                                                             // Retrieve last saved value if exists
  delay(10);                                                                                         // Wait to avoid RepeatSensor synchronisation issues
  RepeatSensor<float> *sensor_gypsy_A1 = new RepeatSensor<float>(read_delay_windlass, 
                                                                    getVoltageADS1115_A1);           // Read the A1 value (gypsy) with a delay
  DebounceInt *chain_debounce = new DebounceInt(CHAIN_COUNTER_IGNORE_DELAY, conf_path_chain[CHAIN_COUNTER_DELAY]);
  sensor_windlass_debounce = sensor_gypsy_A1->connect_to(chain_debounce);                            // Avoid multiples counts
  auto *sensor_windlass_counter = sensor_windlass_debounce->connect_to(chain_counter);               // Add +/- gipsy_circum to the counter
  sensor_windlass_counter->attach(handleChainCounterChange);                                         // Set a callback for each value read
  sensor_windlass_counter
    ->connect_to(new SKOutputFloat(sk_path_windlass, 
                new SKMetadata("m", "Compteur Chaine", "Chain Counter", "Mètre")));                  // Output the last float value to SignalK

  LambdaTransform<float, String> *sensor_windlass_string = new LambdaTransform<float, String>(chainCounterToString);
  sensor_windlass_counter
    ->connect_to(sensor_windlass_string)
    ->connect_to(new SKOutputString(sk_path_windlass + ".direction"));                               // Output the last value + direction to SignalK

  /* Set SensESP Configuration UI for chain_counter and chain_debounce
     If you want something to appear in the web UI, you first define overloaded ConfigSchema and ConfigRequiresRestart functions for that class.
     Then, you call ConfigItem to actually instantiate the ConfigItemT object
  */
  ConfigItem(chain_counter)
    ->set_title("Compteur Chaine")
    ->set_description("Compteur Chaine - PersistentIntegrator")
    ->set_sort_order(UI_ORDER_CHAIN);

  ConfigItem(chain_debounce)
    ->set_title("Compteur Chaine Debounce")
    ->set_description("Compteur Chaine - DebounceInt")
    ->set_sort_order(UI_ORDER_CHAIN+1);

  #ifdef DEBUG_MODE
  sensor_windlass_debounce->connect_to(new SKOutputInt(sk_path_windlass + ".debounce.raw"));
  sensor_windlass_A0->connect_to(new SKOutputFloat(sk_path_windlass + ".up.raw"));
  //sensor_gypsy_A1->connect_to(new SKOutputInt(sk_path_windlass + ".raw"));
  #endif
}

// Measure instruments voltage
void setupVoltageSensors() {
  /*
    The INA219 or INA3221 needs to be connected to the same ground reference as the voltage source being measured.
    This is crucial for maintaining a common reference point and avoiding measurement errors
    - Step 1 : measure the voltage at start up (engine is off), startVoltage = 0.728
    - Step 2 : measure the voltage and pressure when engine is running (engine is idling), runningVoltage = 3.836V & runningBar = 5.0 bars
    - Step 3 : calculate the offset & multiplier ((voltage - startVoltage) * runningBar) / (runningVoltage - startVoltage)
                offset = -startVoltage = -0.728
                multiplier = runningBar / (runningVoltage - startVoltage) = 5.0 / (4.152 - 0.728) = 1.46
    - Step 4 : set the linar function with offset & multiplier
    
    VoltageDividerR1
    Uses the voltage divider formula to calculate (and output) the resistance of R1 in the circuit.

    Vout = (Vin x R2) / (R1 + R2) is the voltage divider formula. We know:

    Vout - that's the input to this transform, probably coming from an AnalogVoltage transform, or directly from an AnalogInput sensor.
    Vin - that's one of the input parameters to this transform. It's a fixed voltage source that you know from your physical voltage divider circuit.
    R2 - also a parameter to this transform, and also from your physical voltage divider.
    Knowing Vin, Vout, and R2, we can calculate R1 (which is what this transform does).

    The purpose of this transform is to help determine the resistance value of a physical sensor of the "variable resistor" type, such as
    a temperature sensor, or an oil pressure sensor. If we know the resistance of the sensor, we can then determine the temperature 
    (or pressure, etc.) that the sensor is reading, by connecting this transform's output to an instance of the CurveInterpolator transform.

    @link https://github.com/SignalK/SensESP/blob/main/examples/temperature_sender.cpp
  */

  // Enable each INA3221 according to the SensESP configuration
  boolean sensorFound[INA3221_NB] = {false, false, false, false};

  if (sensesp_config->is_enabled("INA3221_BABORD_0"))
    sensorFound[INA3221_BABORD_0] = setupSensorINA3221(INA3221_BABORD_0, INA3221_ADDR40_GND);
  if (sensesp_config->is_enabled("INA3221_CUVES_1"))
    sensorFound[INA3221_CUVES_1] = setupSensorINA3221(INA3221_CUVES_1, INA3221_ADDR41_VCC);
  if (sensesp_config->is_enabled("INA3221_TRIBORD_2"))
    sensorFound[INA3221_TRIBORD_2] = setupSensorINA3221(INA3221_TRIBORD_2, INA3221_ADDR42_SDA);
  if (sensesp_config->is_enabled("INA3221_OTHERS_3"))
    sensorFound[INA3221_OTHERS_3] = setupSensorINA3221(INA3221_OTHERS_3, INA3221_ADDR43_SCL);

  // Setup portside voltage sensors
  if (sensorFound[INA3221_BABORD_0]) {
    sensor_volt[INA3221_BABORD_0][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH1);
    sensor_volt[INA3221_BABORD_0][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH2);
    sensor_volt[INA3221_BABORD_0][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH3);
    setupVoltageEngineSensors(INA3221_BABORD_0, "Babord");
  }

  // Setup tank voltage sensors
  if (sensorFound[INA3221_CUVES_1]) {
    sensor_volt[INA3221_CUVES_1][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH1);
    sensor_volt[INA3221_CUVES_1][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH2);
    sensor_volt[INA3221_CUVES_1][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH3);
    setupVoltageTankSensor(INA3221_CH1, "Babord", "Carburant", 0);    // Fuel tank volume on portside
    setupVoltageTankSensor(INA3221_CH2, "Tribord", "Carburant", 2);   // Fuel tank volume on starboard
    setupVoltageTankSensor(INA3221_CH3, "Eau", "Inox", 4);            // Water tank volume
  }

  // Setup starboard voltage sensors
  if (sensorFound[INA3221_TRIBORD_2]) {
    sensor_volt[INA3221_TRIBORD_2][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH1);
    sensor_volt[INA3221_TRIBORD_2][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH2);
    sensor_volt[INA3221_TRIBORD_2][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH3);
    setupVoltageEngineSensors(INA3221_TRIBORD_2, "Tribord");
  }

  // Setup others voltage sensors like rudder angle sensor
  if (sensorFound[INA3221_OTHERS_3]) { 
    if (sensesp_config->is_enabled("RUDDER_ANGLE_FEATURE")) {
      sensor_volt[INA3221_OTHERS_3][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_OTHERS3_CH3);
      setupVoltageRudderAngleSensor();
    }
  }
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
    RepeatSensor<String> *sensor_motion = new RepeatSensor<String>(read_delay, getMotionSensorValues);
    sensor_motion->connect_to((new SKOutputRawJson(sk_path_motion[MPU6050_ATTITUDE])));

    // TEST
    //RepeatSensor<Quaternion> *sensor_motion_quaternion = new RepeatSensor<Quaternion>(read_delay, getMotionSensorQuaternion);
    //LambdaTransform<Quaternion, String> *sensor_motion_ypr = new LambdaTransform<Quaternion, String>(getMotionSensorYPR);
    //sensor_motion_quaternion->connect_to(sensor_motion_ypr);
    //sensor_motion_ypr->connect_to(new SKOutputRawJson(sk_path_motion[MPU6050_ATTITUDE]));

    #ifndef FAKE_MODE
    sensor_compass = new RepeatSensor<float>(read_delay, getCompassSensorValue);
    #else
    sensor_compass = new FloatConstantSensor(1.0, read_delay);
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
    setupTemperatureSensors();
  if (sensesp_config->is_enabled("INA3221_FEATURE"))
    setupVoltageSensors();
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