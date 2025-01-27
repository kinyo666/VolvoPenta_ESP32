/*
  @TODO :
  P0
  - Utiliser un MovingAverage sur la valeur RPM
  - Intégrer la lib MPU9250

  P1
  - Ajouter une moyenne de consommation de carburant réelle

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
  @version 1.0.9
  @date 28/01/2025
  @ref SensESP v3.0.0
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
  @link SensESP Documentation : https://signalk.org/SensESP/
  @link Based on https://github.com/Boatingwiththebaileys/ESP32-code
  @link https://www.arduino.cc/reference/en/libraries/ina3221/
  @link https://signalk.org/specification/1.7.0/schemas/definitions.json
  */
#include <INA3221.h>
#include <sensesp.h>
#include <sensesp_app_builder.h>
#include <sensesp/sensors/constant_sensor.h>
#include <sensesp/sensors/digital_input.h>
#include <sensesp/signalk/signalk_output.h>
#include <sensesp_onewire/onewire_temperature.h>
//#include <string>
#include "MPU6050_6Axis_MotionApps20.h"
//https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU9250
//https://registry.platformio.org/libraries/mbed-anything-connected/MPU9250/installation
//#include <MPU9250.h>
#include "customClasses.h"
#include "debounce.h"

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
#define INA3221_CUVES_1 1                                 // I2C address = 0x41 SDA
#define INA3221_TRIBORD_2 2                               // I2C address = 0x42 SCL
#define INA3221_OTHERS_3 3                                // I2C address = 0x43 VS
#define INA3221_NB 4

// RPM sensors PC817
#define PC817_BABORD_PIN 34
#define PC817_TRIBORD_PIN 35

// Engines
#define ENGINE_BABORD 0
#define ENGINE_TRIBORD 1
#define ENGINE_NB 2
#define ENGINE_SK_PATH_NB 3
#define ENGINE_SLEEP_TIMER 60

// Windlass Chain counter
#define CHAIN_COUNTER_PIN 32
#define CHAIN_COUNTER_SAVE_TIMER 10
#define CHAIN_COUNTER_IGNORE_DELAY 900
#define CHAIN_COUNTER_PATH 0
#define CHAIN_COUNTER_DELAY 1
#define CHAIN_COUNTER_NB 2

// Use Debug Mode for verbose logs and Fake Mode to simulate some data
#define DEBUG_MODE 1
//#define FAKE_MODE 1

// SensESP UI Config order
#define UI_ORDER_TEMP 10
#define UI_ORDER_ENGINE 20
#define UI_ORDER_TANK 30
#define UI_ORDER_CHAIN 40
#define UI_ORDER_MOTION 50

// Motion sensor MPU9250
#define MPU9250_ATTITUDE 0
#define MPU9250_HEADING 1

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
const String sk_path_motion[2] = { "navigation.attitude", "navigation.headingMagnetic" };

// SensESP Configuration
const String conf_path_global = "/CONFIG/SENSORS_CONFIG";
const String conf_path_volt[INA3221_NB][INA3221_CH_NUM] = {
        {"/CONFIG/BABORD/INA3221/0/LINEAR_CH1",   "/CONFIG/BABORD/INA3221/0/LINEAR_CH2",  "/CONFIG/BABORD/INA3221/0/LINEAR_CH3"},
        {"/CONFIG/CUVES/INA3221/1/LINEAR_CH1",    "/CONFIG/CUVES/INA3221/1/LINEAR_CH2",   "/CONFIG/CUVES/INA3221/1/LINEAR_CH3"},
        {"/CONFIG/TRIBORD/INA3221/2/LINEAR_CH1",  "/CONFIG/TRIBORD/INA3221/2/LINEAR_CH2", "/CONFIG/TRIBORD/INA3221/2/LINEAR_CH3"}};
const String conf_path_engines[ENGINE_NB] = {"/CONFIG/BABORD/PC817/FREQUENCY_RPM", 
                                            "/CONFIG/TRIBORD/PC817/FREQUENCY_RPM"};
const String conf_path_chain[CHAIN_COUNTER_NB] = { "/CONFIG/CHAINE/COUNTER", "/CONFIG/CHAINE/DELAY" };
const float gipsy_circum = 0.43982;                           // Windlass gipsy circumference (meter)
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
Transform<int, int> *sensor_windlass_debounce;                // Windlass sensor (DigitalInputCounter + Debounce)
PersistentIntegrator *chain_counter;                          // Windlass chain counter
JsonDocument jdoc_conf_windlass;
JsonObject conf_windlass;                                     // Windlass direction of rotation (UP or DOWN) and last value if exists
unsigned int chain_counter_timer = 0;                         // Timer to save chain counter value
boolean chain_counter_saved = true;                           // Trigger to save chain counter new value

// Motion and compass sensor
MPU6050 sensor_mpu;                                           // 1 MPU-6050 Motion sensor
RepeatSensor<String> *sensor_motion;                          // MPU Motion values
#ifndef FAKE_MODE
RepeatSensor<float> *sensor_compass;                          // MPU Compass values
#else
FloatConstantSensor *sensor_compass;                          // Fake Compass values
#endif

// SensESP builds upon the ReactESP framework. Every ReactESP application must instantiate the "app" object.
reactesp::EventLoop app;

/*
// Callback for Linear Positive (Linear Transform which returns 0.0 if not positive)
float (*LinearPositive::function_)(float, float, float) =
    [](float input, float multiplier, float offset) {
      if (input > 0)
        return multiplier * input + offset;
      else
        return (0.0f);
    };
const ParamInfo LinearPositive::param_info_[] = {{"multiplier", "Multiplier"}, {"offset", "Offset"}};
*/

// Filter Kelvin values out of range
auto filterKelvinValues = [](float input, float offset = 273.15, float max = 413.1) -> float {
    return ((((input - offset) > 0) && (input < max)) ? input : offset);
};
const ParamInfo* filterKelvinValues_ParamInfo = new ParamInfo[2]{{"offset", "Offset"}, {"max", "Max value"}};

// Callback for LambdaTransform to add a UP/DOWN/IDLE symbol to the chain counter
auto chainCounterToString = [](float input) -> String {
  String chainCounterString = String(input) + "m ";
  if (conf_windlass["k"].as<float>() > 0)
    chainCounterString += "▲";
  else if (conf_windlass["k"].as<float>() < 0)
    chainCounterString += "▼";
  else
    chainCounterString += "▬";
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

/* Callbacks for Windlass UP/DOWN buttons sensor
   This function determines :
   - In which direction the windlass turns (UP or DOWN)
   - Whether the last value should be saved (persistent storage)

   The direction UP (or DOWN) is derived from the voltage of each push button / relay
   Once we know the direction, we can change the sign (+/-) of the chain_counter accumulator
   It is not necessary to change it each time the button is pressed : only when the direction is reversed.
   
   We use the Repeat Sensor to arm a timer that saves the last chain_counter value after CHAIN_COUNTER_SAVE_TIMER cycles.
   To avoid infinite increment of the timer when there is no UP/DOWN event and multiples saves, the timer is set to 0 
   when the last value has already been saved.

   NOTE(1) : if the global read_delay is set to more or less than one second, the number of cycles may need to be modified.
   NOTE(2) : only one function needs to trigger the timer and the save_configuration() funcion

   @returns float - Voltage measured for the channel INA3221_CHx
*/
float getVoltageINA3221_OTHERS3_CH1() { 
  float up = sensor_INA3221[INA3221_OTHERS_3]->getVoltage(INA3221_CH1);                     // Windlass UP

  if ((up > 0.0) && (conf_windlass["k"] > 0)) {
    conf_windlass["k"] = -1.0 * conf_windlass["k"].as<float>();                             // Reverse direction
    bool set_config = chain_counter->from_json(conf_windlass);
    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : UP ; SET_CONFIG = %s\n", set_config ? "true" : "false");
    #endif
  }
  else if ((chain_counter_saved == false) && (chain_counter_timer == CHAIN_COUNTER_SAVE_TIMER)) {
      chain_counter_saved = chain_counter->save();                                          // Save last value to local file system
      if (chain_counter_saved)
        chain_counter_timer = 0;                                                            // Reset timer to 0

      #ifdef DEBUG_MODE
      Serial.printf("WINDLASS : UP NOK -> saved = %s\n", chain_counter_saved ? "true" : "false");
      #endif
    }
  else {
    chain_counter_timer = ((chain_counter_saved == true) ? 0 : (chain_counter_timer + 1));  // Avoid infinite increment when no changes

    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : UP NOK -> up = %f direction = %f timer = %i saved = %s\n", up, 
                  conf_windlass["k"].as<float>(), chain_counter_timer, chain_counter_saved ? "true" : "false");
    #endif
  }

  return up;
}

float getVoltageINA3221_OTHERS3_CH2() { 
  float down = sensor_INA3221[INA3221_OTHERS_3]->getVoltage(INA3221_CH2);                   // Windlass DOWN
  
  if ((down > 0.0) && (conf_windlass["k"] < 0)) {
    conf_windlass["k"] = -1.0 * conf_windlass["k"].as<float>();                             // Reverse direction
    bool set_config = chain_counter->from_json(conf_windlass);
    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : DOWN ; SET_CONFIG = %s\n", set_config ? "true" : "false");
    #endif
  }
  else {
    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : DOWN NOK -> down = %f direction = %f\n", down, conf_windlass["k"].as<float>());
    #endif
  }

  return down;
}

// Callback for chain counter
// This function is called after each sensor_windlass value changes
// TODO : Ajouter un contrôle de la valeur avec CHAIN_LIMIT_LOW et CHAIN_LIMIT HIGH ?
void handleChainCounterChange() {
  if (sensor_windlass_debounce->get() > 0) {
    chain_counter_saved = false;          // Set the last value status to 'not saved'
    chain_counter_timer = 0;              // Arm a timer to save chain counter last value after CHAIN_COUNTER_SAVE_TIMER cycles

    #ifdef DEBUG_MODE
    Serial.println("WINDLASS : NEW VALUE TO SAVE");
    #endif
  }
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
// @return Heading value in radian
float getCompassSensorValue() {
  return 100.0;
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
      sensor_INA3221[INA3221_id]->setModeContinious();        // INA3221 is now active
      #ifdef DEBUG_MODE
      Serial.printf("ENGINE : id = %i | state = %#05x -> WAKE UP\n", engine_id, engine_state[engine_id]);
      #endif
    }
    else if (engine_timer[engine_id] == 0) {                  // INA3221 is going to power-down
        engine_timer[engine_id] = -1;                         // Disable the timer
        sensor_INA3221[INA3221_id]->setModePowerDown();       // INA3221 is now in power-down mode
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
// @param Vin Volt input (3.5V)
// @param R1 Known resistance in ohm (0.1 ohm for INA3221)
void setupVoltageEngineSensors(u_int8_t INA3221_Id, String engine, float Vin, float R1) {
  VoltageDividerR2 *sensor_volt_divider_CH1 = new VoltageDividerR2(R1, Vin, conf_path_volt[INA3221_Id][INA3221_CH1] + "/VOLTAGE_DIVIDER");
  VoltageDividerR2 *sensor_volt_divider_CH2 = new VoltageDividerR2(R1, Vin, conf_path_volt[INA3221_Id][INA3221_CH2] + "/VOLTAGE_DIVIDER");
  LinearPos *sensor_volt_linearpos_CH1 = new LinearPos(linearPositive, 1.0, 0.0, linerPositive_ParamInfo,
                                                      conf_path_volt[INA3221_CUVES_1][INA3221_CH1] + "/LINEAR_POSITIVE");
  LinearPos *sensor_volt_linearpos_CH2 = new LinearPos(linearPositive, pow(10, 5), 0.0, linerPositive_ParamInfo,
                                                       conf_path_volt[INA3221_CUVES_1][INA3221_CH2] + "/LINEAR_BAR_TO_PA");
  LinearPos *sensor_volt_linearpos_CH3 = new LinearPos(linearPositive, 1.0, 0.0, linerPositive_ParamInfo,
                                                       conf_path_volt[INA3221_CUVES_1][INA3221_CH3] + "/LINEAR_POSITIVE");
  CoolantTemperature *sensor_volt_coolanttemp = new CoolantTemperature(conf_path_volt[INA3221_Id][INA3221_CH1] + "/COOLANT_TEMPERATURE");
  OilPressure *sensor_volt_oilpressure = new OilPressure(conf_path_volt[INA3221_Id][INA3221_CH2] + "/OIL_PRESSURE");

    // Voltage for Coolant Temperature gauge
  sensor_volt[INA3221_Id][INA3221_CH1]
    ->connect_to(sensor_volt_divider_CH1)
    ->connect_to(sensor_volt_coolanttemp)
    ->connect_to(new LambdaTransform<float, float, float, float>(filterKelvinValues, 273.15, 413.1, filterKelvinValues_ParamInfo, 
                                                                conf_path_volt[INA3221_Id][INA3221_CH1] + "/TRANSFORM"))
    ->connect_to(sensor_volt_linearpos_CH1)
    //->connect_to(new LinearPositive(1.0, 0.0, conf_path_volt[INA3221_Id][INA3221_CH1] + "/LINEAR_POSITIVE"))
    //->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_Id][INA3221_CH1] + "/MOVING_AVERAGE"))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH1],
                    new SKMetadata("K", "T° Eau " + engine, "Coolant temperature", "Temp " + engine)));
  ConfigItem(sensor_volt_divider_CH1)
    ->set_title("Température Eau - " + engine)
    ->set_description("T° Eau - INA3221_CH1 - VoltageDividerR2")
    ->set_sort_order(UI_ORDER_ENGINE);
  ConfigItem(sensor_volt_coolanttemp)
    ->set_title("Température Eau - " + engine)
    ->set_description("T° Eau - INA3221_CH1 - CoolantTemperature")
    ->set_sort_order(UI_ORDER_ENGINE+1);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_Id][INA3221_CH1]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH1] + ".raw"));
  #endif

  // Voltage for Oil Pressure gauge
  sensor_volt[INA3221_Id][INA3221_CH2]
    ->connect_to(sensor_volt_divider_CH2)
    ->connect_to(sensor_volt_oilpressure)
    ->connect_to(sensor_volt_linearpos_CH2)    
    //->connect_to(new LinearPositive(pow(10, 5), 0.0, conf_path_volt[INA3221_Id][INA3221_CH2] + "/LINEAR_BAR_TO_PA"))
    //->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_Id][INA3221_CH2]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH2],
                    new SKMetadata("Pa", "Pression Huile " + engine, "Oil pressure", "Huile " + engine)));
  ConfigItem(sensor_volt_divider_CH2)
    ->set_title("Pression Huile - " + engine)
    ->set_description("Pression Huile - INA3221_CH2 - VoltageDividerR2")
    ->set_sort_order(UI_ORDER_ENGINE+2);
  ConfigItem(sensor_volt_oilpressure)
    ->set_title("Pression Huile - " + engine)
    ->set_description("Pression Huile - INA3221_CH2 - OilPressure")
    ->set_sort_order(UI_ORDER_ENGINE+3);
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
// @param Vin Volt input (3.5V)
// @param R1 Known resistance in ohm (0.1 ohm for INA3221)
void setupVoltageTankSensors(float Vin, float R1) {
  // 1. Take the maximum analog input value (i.e. value when sensor is at the high end)
  // 2. Divide 1 by max value. In my case: 1 / 870 = 0.001149425
  // Assuming the resistance is 3-180 ohms for VDO / Veritron gauge
  VoltageDividerR2 *sensor_volt_divider_CH1 = new VoltageDividerR2(R1, Vin, conf_path_volt[INA3221_CUVES_1][INA3221_CH1] + "/VOLTAGE_DIVIDER");
  VoltageDividerR2 *sensor_volt_divider_CH2 = new VoltageDividerR2(R1, Vin, conf_path_volt[INA3221_CUVES_1][INA3221_CH2] + "/VOLTAGE_DIVIDER");
  VoltageDividerR2 *sensor_volt_divider_CH3 = new VoltageDividerR2(R1, Vin, conf_path_volt[INA3221_CUVES_1][INA3221_CH3] + "/VOLTAGE_DIVIDER");
  LinearPos *sensor_volt_linearpos_CH1 = new LinearPos(linearPositive, 1.0, 0.0, linerPositive_ParamInfo,
                                                       conf_path_volt[INA3221_CUVES_1][INA3221_CH1] + "/LINEAR_POSITIVE");
  LinearPos *sensor_volt_linearpos_CH2 = new LinearPos(linearPositive, 1.0, 0.0, linerPositive_ParamInfo,
                                                       conf_path_volt[INA3221_CUVES_1][INA3221_CH2] + "/LINEAR_POSITIVE");
  LinearPos *sensor_volt_linearpos_CH3 = new LinearPos(linearPositive, 1.0, 0.0, linerPositive_ParamInfo,
                                                       conf_path_volt[INA3221_CUVES_1][INA3221_CH3] + "/LINEAR_POSITIVE");

  // Fuel tank volume on portside
  sensor_volt[INA3221_CUVES_1][INA3221_CH1]
    ->connect_to(sensor_volt_divider_CH1)
    ->connect_to(sensor_volt_linearpos_CH1)
    //->connect_to(new LinearPositive(1.0, 0.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH1] + "/LINEAR_POSITIVE"))
    ->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH1] + "/MOVING_AVERAGE"))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH1],
                  new SKMetadata("m3", "Cuve Babord", "Niveau carburant Babord", "Cuve Babord")));
  ConfigItem(sensor_volt_divider_CH1)
    ->set_title("Cuve Babord")
    ->set_description("Cuve Babord - INA3221_CH1 - VoltageDividerR2")
    ->set_sort_order(UI_ORDER_TANK);
  ConfigItem(sensor_volt_linearpos_CH1)
    ->set_title("Cuve Babord")
    ->set_description("Cuve Babord - INA3221_CH1 - LinearPositive")
    ->set_sort_order(UI_ORDER_TANK+1);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_CUVES_1][INA3221_CH1]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH1] + ".raw"));
  #endif

  // Fuel tank volume on starboard
  sensor_volt[INA3221_CUVES_1][INA3221_CH2]
    ->connect_to(sensor_volt_divider_CH2)
    ->connect_to(sensor_volt_linearpos_CH2)
    //->connect_to(new LinearPositive(1.0, 0.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH2] + "/LINEAR_POSITIVE"))
    ->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH2] + "/MOVING_AVERAGE"))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH2],
                  new SKMetadata("m3", "Cuve Tribord", "Niveau carburant Tribord", "Cuve Tribord")));
  ConfigItem(sensor_volt_divider_CH2)
    ->set_title("Cuve Tribord")
    ->set_description("Cuve Tribord - INA3221_CH2 - VoltageDividerR2")
    ->set_sort_order(UI_ORDER_TANK+2);
  ConfigItem(sensor_volt_linearpos_CH2)
    ->set_title("Cuve Tribord")
    ->set_description("Cuve Tribord - INA3221_CH2 - LinearPositive")
    ->set_sort_order(UI_ORDER_TANK+3);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_CUVES_1][INA3221_CH2]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH2] + ".raw"));
  #endif

  // Water tank volume
  sensor_volt[INA3221_CUVES_1][INA3221_CH3]
    ->connect_to(sensor_volt_divider_CH3)
    ->connect_to(sensor_volt_linearpos_CH3)
    //->connect_to(new LinearPositive(1.0, 0.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH3] + "/LINEAR_POSITIVE"))
    ->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH3] + "/MOVING_AVERAGE"))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH3],
                  new SKMetadata("m3", "Cuve Eau douce", "Niveau eau douce", "Cuve Eau")));
  ConfigItem(sensor_volt_divider_CH3)
    ->set_title("Cuve Eau douce")
    ->set_description("Cuve Eau douce - INA3221_CH3 - VoltageDividerR2")
    ->set_sort_order(UI_ORDER_TANK+4);
  ConfigItem(sensor_volt_linearpos_CH3)
    ->set_title("Cuve Eau douce")
    ->set_description("Cuve Eau douce - INA3221_CH3 - LinearPositive")
    ->set_sort_order(UI_ORDER_TANK+5);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_CUVES_1][INA3221_CH3]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH3] + ".raw"));
  #endif
}

// Windlass chain counter
void setupVoltageChainSensors() {
  conf_windlass = jdoc_conf_windlass.to<JsonObject>();                                               // Store Windlass configuration
  conf_windlass["k"] = gipsy_circum;                                                                 // Default direction = UP
  chain_counter = new PersistentIntegrator(gipsy_circum, 0.0, conf_path_chain[CHAIN_COUNTER_PATH]);  // Chain counter in meter

  // Do not use DigitalInputDebounceCounter as it is unfinished in SensESP 3.0.0
  DigitalInputCounter *sensor_windlass = new DigitalInputCounter(CHAIN_COUNTER_PIN, INPUT_PULLUP, 
                                                                RISING, read_delay);                 // Count pulses per revolution
  DebounceInt *chain_debounce = new DebounceInt(CHAIN_COUNTER_IGNORE_DELAY, conf_path_chain[CHAIN_COUNTER_DELAY]);

  chain_counter->to_json(conf_windlass);                                                             // Retrieve last saved value if exists
  sensor_windlass_debounce = sensor_windlass->connect_to(chain_debounce);                            // Avoid multiples counts
  auto *sensor_windlass_counter = sensor_windlass_debounce->connect_to(chain_counter);               // Add +/- gipsy_circum to the counter
  sensor_windlass_counter->attach(handleChainCounterChange);                                         // Set a callback for each value read
  sensor_windlass_counter
    ->connect_to(new SKOutputFloat(sk_path_windlass, 
                new SKMetadata("m", "Compteur Chaine", "Chain Counter", "Mètre")));                  // Output the float value to SignalK

  sensor_windlass_counter
    ->connect_to(new LambdaTransform<float, String>(chainCounterToString))
    ->connect_to(new SKOutputString(sk_path_windlass + ".direction"));                               // Output the value + direction to SignalK

  /* Set SensUP Configuration UI for chain_counter and sensor_windlass_debounce
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
  sensor_windlass->connect_to(new SKOutputInt(sk_path_windlass + ".raw"));
  sensor_volt[INA3221_OTHERS_3][INA3221_CH1]
    ->connect_to(new SKOutputFloat(sk_path_windlass + ".CH1.raw"));
  sensor_volt[INA3221_OTHERS_3][INA3221_CH2]
    ->connect_to(new SKOutputFloat(sk_path_windlass + ".CH2.raw"));
  #endif
}

// Measure instruments voltage
void setupVoltageSensors() {
  /*
    The INA219 or INA3221 needs to be connected to the same ground reference as the voltage source being measured.
    This is crucial for maintaining a common reference point and avoiding measurement errors
    - Step 1 : measure the voltage at start up (engine is off), startVoltage = 0.728
    - Step 2 : measure the voltage and pressure when engine is running (engine is idling), runningVoltage = 4.152V & runningBar = 5.0 bars
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
  
  // Input values for VoltageDivider ; INA3221 is 0.1 ohm (R100) and fuel/water sensors are ~180 ohms max
  const float Vin = 3.32; // 3.5
  const float R1 = 1.0; // 120
  boolean sensorFound[INA3221_NB] = {false, false, false, false};

  if (sensesp_config->is_enabled("INA3221_BABORD_0"))
    sensorFound[INA3221_BABORD_0] = setupSensorINA3221(INA3221_BABORD_0, INA3221_ADDR40_GND);
  if (sensesp_config->is_enabled("INA3221_CUVES_1"))
    sensorFound[INA3221_CUVES_1] = setupSensorINA3221(INA3221_CUVES_1, INA3221_ADDR41_VCC);
  if (sensesp_config->is_enabled("INA3221_TRIBORD_2"))
    sensorFound[INA3221_TRIBORD_2] = setupSensorINA3221(INA3221_TRIBORD_2, INA3221_ADDR42_SDA);
  if (sensesp_config->is_enabled("INA3221_OTHERS_3"))
    sensorFound[INA3221_OTHERS_3] = setupSensorINA3221(INA3221_OTHERS_3, INA3221_ADDR43_SCL);

  if (sensorFound[INA3221_BABORD_0]) {
    sensor_volt[INA3221_BABORD_0][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH1);
    sensor_volt[INA3221_BABORD_0][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH2);
    sensor_volt[INA3221_BABORD_0][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH3);

    setupVoltageEngineSensors(INA3221_BABORD_0, "Babord", Vin, R1);
  }

  if (sensorFound[INA3221_CUVES_1]) {
    sensor_volt[INA3221_CUVES_1][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH1);
    sensor_volt[INA3221_CUVES_1][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH2);
    sensor_volt[INA3221_CUVES_1][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH3);

    setupVoltageTankSensors(Vin, R1);
  }

  if (sensorFound[INA3221_TRIBORD_2]) {
    sensor_volt[INA3221_TRIBORD_2][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH1);
    sensor_volt[INA3221_TRIBORD_2][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH2);
    sensor_volt[INA3221_TRIBORD_2][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH3);

    setupVoltageEngineSensors(INA3221_TRIBORD_2, "Tribord", Vin, R1);
  }

  if (sensorFound[INA3221_OTHERS_3]) {
    sensor_volt[INA3221_OTHERS_3][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_OTHERS3_CH1);
    sensor_volt[INA3221_OTHERS_3][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_OTHERS3_CH2);
    sensor_volt[INA3221_OTHERS_3][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_OTHERS3_CH3);
    
    if (sensesp_config->is_enabled("CHAIN_COUNTER_FEATURE"))
      setupVoltageChainSensors();
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

// Setup the PC817 sensor for engine RPM
void setupPC817Sensor(u_int8_t engine_id, String engine, u_int8_t PC817_pin) {
  // Step 1 : calibrate the multiplier with known values (CurveInterpolator class) from analog gauge
  //const float multiplier = 1.0 / 97.0;
  const float multiplier = 1.0;
  const unsigned int read_rpm_delay = 1000;
  EngineDataTransform *engine_data = new EngineDataTransform(multiplier, conf_path_engines[engine_id], engine_id);

  // Step 2 : instanciates DigitalInputCounter to read PC817 sensor values
  #ifndef FAKE_MODE
  sensor_engine[engine_id] = new DigitalInputCounter(PC817_pin, INPUT, RISING, read_rpm_delay);    // INPUT for GPIO 34-35 or INPUT_PULLUP for others pins
  #else
  sensor_engine[engine_id] = new FloatConstantSensor(50.0, 1, "/config/engine/EL817/CONSTANT");   // Fake sensor for test purpose
  #endif

  // Step 3 : connects the output of sensor to the input of Frequency() and the Signal K output as a float in Hz
  sensor_engine[engine_id]
    ->connect_to(engine_data->freq)
    //->connect_to(engine_data->moving_avg)
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
  Example: 1 turn on an AQAD41 makes 2.45 turns on the Alternator, and the Alternator has 6 poles which makes a total of 2.45 x 6 = 14.7 Hz per turn. SignalK needs info in Hz so we need to divide the incoming value with 14.7, or as in our case multiply with (1/14.7) = 0,06803
  If Ratio is unknown, the original Tachometer might have a max Impulses/min written on it, divide that with max rpm on the meter and you'll get the ratio. Tachometer 860420 is marked with 73500Imp/min and has a scale to 5000rpm. 73500 divided with 5000 equals 14,7, Tada!
  If above multiplier needs to be recalculated, the FuelMultipliers needs to be recalculated as well, they're both based on AQAD41 values right now.
  AQAD41 example: 60 divided with FlyWheel To W Ratio (60 / 14,7 = 4,08)
*/
void setupRPMSensors() {
    if (sensesp_config->is_enabled("PC817_BABORD"))
      setupPC817Sensor(ENGINE_BABORD, "Babord", PC817_BABORD_PIN);
    if (sensesp_config->is_enabled("PC817_TRIBORD"))
      setupPC817Sensor(ENGINE_TRIBORD, "Tribord", PC817_TRIBORD_PIN);
}

// Motion Sensor
// @link https://registry.platformio.org/libraries/electroniccats/MPU6050
void setupMotionSensor() {
  uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // Initialize device
  Serial.println(F("Initializing I2C devices..."));
  sensor_mpu.initialize();
  //pinMode(INTERRUPT_PIN, INPUT);

  // Verify connection
  Serial.println(F("Testing MPU6050 connection..."));
  if(sensor_mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    return;
  }
  else
    Serial.println("MPU6050 connection successful");

  // Initializate and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = sensor_mpu.dmpInitialize();
  
  // Supply your gyro offsets here, scaled for min sensitivity
  sensor_mpu.setXGyroOffset(0);
  sensor_mpu.setYGyroOffset(0);
  sensor_mpu.setZGyroOffset(0);
  sensor_mpu.setXAccelOffset(0);
  sensor_mpu.setYAccelOffset(0);
  sensor_mpu.setZAccelOffset(0);

  // Making sure it worked (returns 0 if so)
  if (devStatus == 0) {
    sensor_mpu.CalibrateAccel(6);         // Calibration Time: generate offsets and calibrate our MPU6050
    sensor_mpu.CalibrateGyro(6);
    Serial.println("Active offsets : ");
    sensor_mpu.PrintActiveOffsets();
    Serial.print(F("Enabling DMP..."));   // Turning ON DMP
    sensor_mpu.setDMPEnabled(true);
    Serial.println(F("DMP ready !"));
    uint16_t packetSize = sensor_mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison

    sensor_motion = new RepeatSensor<String>(read_delay, getMotionSensorValues);
    sensor_motion->connect_to((new SKOutputRawJson(sk_path_motion[MPU9250_ATTITUDE])));

    #ifndef FAKE_MODE
    sensor_compass = new RepeatSensor<float>(read_delay, getCompassSensorValue);
    #else
    sensor_compass = new FloatConstantSensor(1.0, read_delay);
    #endif
    sensor_compass->connect_to(new SKOutputFloat(sk_path_motion[MPU9250_HEADING],
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
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop() { 
  static auto event_loop = SensESPBaseApp::get_event_loop();
  event_loop->tick();
}