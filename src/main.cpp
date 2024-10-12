/*
  TODO :
  - Tester si les sondes sont présentes et ne pas les attacher à un path signal K le cas échéant (comment le tester ??)
  - Ajouter un calcul de consommation
  - Ajouter un compteur de chaine (à partir du signal 12V?)

  @author B. DIVERCHY
  @link SensESP Documentation : https://signalk.org/SensESP/
  @link Based on https://github.com/Techstyleuk/SensESP_3_Battery_Monitor
  @link https://www.arduino.cc/reference/en/libraries/ina3221/
  @link https://signalk.org/specification/1.7.0/schemas/definitions.json
  */

//#include <Arduino.h>
//#include <WiFi.h>
#include <Wire.h>
#include <INA3221.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include "sensesp.h"
//#include "sensesp_app.h"
#include "sensesp_app_builder.h"
#include <sensesp/sensors/sensor.h>
//#include <sensesp/sensors/constant_sensor.h>
#include "sensesp/sensors/digital_input.h"
//#include "sensesp/sensors/analog_input.h"
#include "sensesp/signalk/signalk_output.h"
//#include "sensesp/signalk/signalk_put_request.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp_onewire/onewire_temperature.h"

// Temperature sensors DS18B20
#define DS18B20_PIN 33            // Use ADC1 pin (G32-35) rather than ADC2 pin to avoid Wifi interference
#define DS18B20_BABORD_0 0
#define DS18B20_TRIBORD_1 1
#define DS18B20_COMMON_2 2
#define DS18B20_NB 3

// Volt / Current sensors INA3221
#define INA3221_BABORD_0 0
#define INA3221_CUVES_1 1
#define INA3221_TRIBORD_2 2
//#define INA3221_TRIBORD_3 3
#define INA3221_NB 3

// RPM sensors EL817
#define EL817_BABORD_PIN 34
#define EL817_TRIBORD_PIN 35

// Engines
#define ENGINE_BABORD 0
#define ENGINE_TRIBORD 1
#define ENGINE_NB 2
#define ENGINE_SK_PATH_NB 3

#define DEBUG_MODE 1

using namespace sensesp;
using namespace sensesp::onewire;

// Fuel Consumption for TAMD40B based on https://www.volvopenta.com/your-engine/manuals-and-handbooks/ (see Product Leaflet)
class FuelConsumption : public CurveInterpolator {
 public:
  FuelConsumption(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the RPM values to L/h (1 L/h = 0,001 m3/h)
    clear_samples();
    // addSample(CurveInterpolator::Sample(rpmValue, litersPerHour));
    add_sample(CurveInterpolator::Sample(2000, 23.80));
    add_sample(CurveInterpolator::Sample(2100, 25.37));
    add_sample(CurveInterpolator::Sample(2200, 26.19));
    add_sample(CurveInterpolator::Sample(2300, 27.80));
    add_sample(CurveInterpolator::Sample(2400, 28.91));
    add_sample(CurveInterpolator::Sample(2500, 30.01));
    add_sample(CurveInterpolator::Sample(2600, 31.09));
    add_sample(CurveInterpolator::Sample(2700, 32.38));
    add_sample(CurveInterpolator::Sample(2800, 33.60));
    add_sample(CurveInterpolator::Sample(2900, 34.38)); 
    add_sample(CurveInterpolator::Sample(3000, 35.16));
    add_sample(CurveInterpolator::Sample(3100, 36.07)); 
    add_sample(CurveInterpolator::Sample(3200, 37.00));
    add_sample(CurveInterpolator::Sample(3300, 37.60));
    add_sample(CurveInterpolator::Sample(3400, 39.09));
    add_sample(CurveInterpolator::Sample(3500, 39.70));
    add_sample(CurveInterpolator::Sample(3600, 40.74));
  }
};

enum engine_sk_path_t { REVOLUTIONS = 0, FUELRATE };

// Constants for Signal-K path & ESP config
const String ssid = "Bbox-75CDA064"; // "BirchwoodTS33"
const String password = "5DDF31AF4C1FD577";
//const String signalkip = "192.168.1.57";
//const int signalkport = 3000;
const char* sk_path_temp[DS18B20_NB];
const char* sk_path_volt[INA3221_NB][INA3221_CH_NUM];
const char* sk_path_engines[ENGINE_NB][ENGINE_SK_PATH_NB];
const char* conf_path_temp[DS18B20_NB];
const char* conf_path_volt[INA3221_NB][INA3221_CH_NUM];
const char* conf_path_engines[ENGINE_NB];
const unsigned int read_delay = 2000;

// Temperature & Voltage sensors
OneWireTemperature* sensor_temp[DS18B20_NB];                  // 3 DS18B20 Temperature Sensors values
INA3221 *sensor_INA3221[INA3221_NB];                          // 4 INA3221 Voltage Sensors with 3 channels each
RepeatSensor<float> *sensor_volt[INA3221_NB][INA3221_CH_NUM]; // 4 Voltage values for each channels
DigitalInputCounter *sensor_engine[ENGINE_NB];                // 2 EL817 RPM Sensors values

// SensESP builds upon the ReactESP framework. Every ReactESP application must instantiate the "app" object.
//reactesp::ReactESP app;
reactesp::EventLoop app;

// Setup the INA3221 volt sensors
// The INA3221 must have an address on A0 pin, solder the right pin GND (0x40) / SDA (0x41) / SCL (0x42) / VS (0x43)
// - On a Raspberry Pi, the I²C bus is required to be enabled
// - On an ESP32, default SDA pin = 21 & SCL pin = 22
// @param index Slot for INA3221 < INA3221_NB
// @param addr Physical A0 address of the INA3221 sensor
void setupSensorINA3221(u_int8_t index, ina3221_addr_t addr = INA3221_ADDR40_GND) {
  sensor_INA3221[index] = new INA3221(addr);  // Set I2C default address to 0x40 GND or addr
  //delay(800);
  sensor_INA3221[index]->begin(&Wire);        // Default shunt resistors = 10 mOhm (R100)
  sensor_INA3221[index]->reset();
  #ifdef DEBUG_MODE
  Serial.printf("Setup Sensor INA3221 completed : ManufID = %i ; DieID = %i\n", sensor_INA3221[index]->getManufID(), sensor_INA3221[index]->getDieID());
  #endif
}

// Callbacks for Sensor Volt
float getVoltageINA3221_BABORD0_CH1() { return sensor_INA3221[INA3221_BABORD_0]->getVoltage(INA3221_CH1); }    // Coolant temperature
float getVoltageINA3221_BABORD0_CH2() { return sensor_INA3221[INA3221_BABORD_0]->getVoltage(INA3221_CH2); }    // Oil pressure
float getVoltageINA3221_BABORD0_CH3() { return sensor_INA3221[INA3221_BABORD_0]->getVoltage(INA3221_CH3); }    // Voltage
float getVoltageINA3221_CUVES1_CH1() { return sensor_INA3221[INA3221_CUVES_1]->getVoltage(INA3221_CH1); }      // Fuel tank volume
float getVoltageINA3221_CUVES1_CH2() { return sensor_INA3221[INA3221_CUVES_1]->getVoltage(INA3221_CH2); }      // Fuel tank volume
float getVoltageINA3221_CUVES1_CH3() { return sensor_INA3221[INA3221_CUVES_1]->getVoltage(INA3221_CH3); }      // Water tank volume
float getVoltageINA3221_TRIBORD2_CH1() { return sensor_INA3221[INA3221_TRIBORD_2]->getVoltage(INA3221_CH1); }  // Coolant temperature
float getVoltageINA3221_TRIBORD2_CH2() { return sensor_INA3221[INA3221_TRIBORD_2]->getVoltage(INA3221_CH2); }  // Oil pressure
float getVoltageINA3221_TRIBORD2_CH3() { return sensor_INA3221[INA3221_TRIBORD_2]->getVoltage(INA3221_CH3); }  // Voltage

// Setup Signal-K paths
void setupSignalKPath() {
  // The "Signal K path" identifies this sensor to the Signal K server. Leaving
  // this blank would indicate this particular sensor (or transform) does not
  // broadcast Signal K data.
  // To find valid Signal K Paths that fits your need you look at this link:
  // @link https://signalk.org/specification/1.7.0/doc/vesselsBranch.html
  sk_path_temp[DS18B20_BABORD_0] = "propulsion.babord.exhaustTemperature";
  sk_path_temp[DS18B20_TRIBORD_1] = "propulsion.tribord.exhaustTemperature";
  sk_path_temp[DS18B20_COMMON_2] = "environment.inside.engineroom.temperature";
  sk_path_volt[INA3221_BABORD_0][INA3221_CH1] = "propulsion.babord.coolantTemperature";
  sk_path_volt[INA3221_BABORD_0][INA3221_CH2] = "propulsion.babord.oilPressure";
  sk_path_volt[INA3221_BABORD_0][INA3221_CH3] = "propulsion.babord.alternatorVoltage";
  sk_path_volt[INA3221_CUVES_1][INA3221_CH1] = "tanks.fuel.babord.currentVolume";
  sk_path_volt[INA3221_CUVES_1][INA3221_CH2] = "tanks.fuel.tribord.currentVolume";
  sk_path_volt[INA3221_CUVES_1][INA3221_CH3] = "tanks.freshWater.eaudouce.currentVolume";
  sk_path_volt[INA3221_TRIBORD_2][INA3221_CH1] = "propulsion.tribord.coolantTemperature";
  sk_path_volt[INA3221_TRIBORD_2][INA3221_CH2] = "propulsion.tribord.oilPressure";
  sk_path_volt[INA3221_TRIBORD_2][INA3221_CH3] = "propulsion.tribord.alternatorVoltage"; 
  sk_path_engines[ENGINE_BABORD][REVOLUTIONS] = "propulsion.babord.revolutions";
  sk_path_engines[ENGINE_BABORD][FUELRATE] = "propulsion.babord.fuel.rate";
  sk_path_engines[ENGINE_TRIBORD][REVOLUTIONS] = "propulsion.tribord.revolutions";
}

// Setup SensESP Run-time Config for each Sensor / Linear Class
void setupSensESPConfig() {
  // The "Configuration path" is combined with "/config" to formulate a URL
  // used by the RESTful API for retrieving or setting configuration data.
  // It is ALSO used to specify a path to the file system
  // where configuration data is saved on the MCU board. It should
  // ALWAYS start with a forward slash if specified. If left blank,
  // that indicates this sensor or transform does not have any
  // configuration to save, or that you're not interested in doing
  // run-time configuration.
  conf_path_volt[INA3221_BABORD_0][INA3221_CH1] = "/config/BABORD/INA3221/0/LINEAR_CH1";
  conf_path_volt[INA3221_BABORD_0][INA3221_CH2] = "/config/BABORD/INA3221/0/LINEAR_CH2";
  conf_path_volt[INA3221_BABORD_0][INA3221_CH3] = "/config/BABORD/INA3221/0/LINEAR_CH3";
  conf_path_volt[INA3221_CUVES_1][INA3221_CH1] = "/config/CUVES/INA3221/1/LINEAR_CH1";
  conf_path_volt[INA3221_CUVES_1][INA3221_CH2] = "/config/CUVES/INA3221/1/LINEAR_CH2";
  conf_path_volt[INA3221_CUVES_1][INA3221_CH3] = "/config/CUVES/INA3221/1/LINEAR_CH3";
  conf_path_volt[INA3221_TRIBORD_2][INA3221_CH1] = "/config/TRIBORD/INA3221/2/LINEAR_CH1";
  conf_path_volt[INA3221_TRIBORD_2][INA3221_CH2] = "/config/TRIBORD/INA3221/2/LINEAR_CH2";
  conf_path_volt[INA3221_TRIBORD_2][INA3221_CH3] = "/config/TRIBORD/INA3221/2/LINEAR_CH3";
  conf_path_engines[ENGINE_BABORD] = "/config/BABORD/EL817/FREQUENCY_RPM";
  conf_path_engines[ENGINE_TRIBORD] = "/config/TRIBORD/EL817/FREQUENCY_RPM";
  conf_path_temp[DS18B20_BABORD_0] = "/config/BABORD/DS18B20/TEMP_BABORD"; 
  conf_path_temp[DS18B20_TRIBORD_1] = "/config/TRIBORD/DS18B20/TEMP_TRIBORD";
  conf_path_temp[DS18B20_COMMON_2] = "/config/COMMUN/DS18B20/TEMP_CALE";
}

// Setup engine temperature sensors
void setupTemperatureSensors() {
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(DS18B20_PIN);
  
  for (u_int8_t i = 0; i < DS18B20_NB; i++) {
    String config_path_temp = "/config/DS18DB20/" + i;
    sensor_temp[i] = new OneWireTemperature(dts, read_delay, config_path_temp);
  }
  
  sensor_temp[DS18B20_BABORD_0]
      ->connect_to(new SKOutputFloat(sk_path_temp[DS18B20_BABORD_0], conf_path_temp[DS18B20_BABORD_0],
      new SKMetadata("K", "°C", sk_path_temp[DS18B20_BABORD_0], "T° Echappement Moteur Babord")));

  sensor_temp[DS18B20_TRIBORD_1]
      ->connect_to(new SKOutputFloat(sk_path_temp[DS18B20_TRIBORD_1], conf_path_temp[DS18B20_TRIBORD_1],
      new SKMetadata("K", "°C", sk_path_temp[DS18B20_TRIBORD_1], "T° Echappement Moteur Tribord")));

  sensor_temp[DS18B20_COMMON_2]
      ->connect_to(new SKOutputFloat(sk_path_temp[DS18B20_COMMON_2], conf_path_temp[DS18B20_COMMON_2],
      new SKMetadata("K", "°C", sk_path_temp[DS18B20_COMMON_2], "T° Compartiment Moteur")));
}

// Measure instruments voltage
void setupVoltageSensors() {
  /*
    The INA219 or INA3221 needs to be connected to the same ground reference as the voltage source being measured.
    This is crucial for maintaining a common reference point and avoiding measurement errors
    - Step 1 : measure the voltage at start up (engine is off), startVoltage = 0.72
    - Step 2 : measure the voltage and pressure when engine is running (engine is idling), runningVoltage = 5.4V & runningBar = 3.8
    - Step 3 : calculate the offset & multiplier ((voltage - startVoltage) * runningBar) / (runningVoltage - startVoltage)
                offset = -startVoltage
                multiplier = runningBar) / (runningVoltage - startVoltage)
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
  setupSensorINA3221(INA3221_BABORD_0, INA3221_ADDR40_GND);
  //setupSensorINA3221(INA3221_CUVES_1, INA3221_ADDR41_VCC);
  //setupSensorINA3221(INA3221_TRIBORD_2, INA3221_ADDR42_SDA);
  //setupSensorINA3221(INA3221_OTHERS_3, INA3221_ADDR43_SCL);

  sensor_volt[INA3221_BABORD_0][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH1);
  sensor_volt[INA3221_BABORD_0][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH2);
  sensor_volt[INA3221_BABORD_0][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH3);

  //sensor_volt[INA3221_CUVES_1][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH1);
  //sensor_volt[INA3221_CUVES_1][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH2);
  //sensor_volt[INA3221_CUVES_1][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH3);

  //sensor_volt[INA3221_TRIBORD_2][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH1);
  //sensor_volt[INA3221_TRIBORD_2][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH2);
  //sensor_volt[INA3221_TRIBORD_2][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH3);
    
  // Voltage for Coolant Temperature gauge
  sensor_volt[INA3221_BABORD_0][INA3221_CH1]
    ->connect_to(new Linear(1.0, 0.0, conf_path_volt[INA3221_BABORD_0][INA3221_CH1]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_BABORD_0][INA3221_CH1]));
  // Voltage for Oil Pressure gauge
  sensor_volt[INA3221_BABORD_0][INA3221_CH2]
    ->connect_to(new Linear(1.0, 0.0, conf_path_volt[INA3221_BABORD_0][INA3221_CH2]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_BABORD_0][INA3221_CH2]));
  // Voltage for Volts gauge
  sensor_volt[INA3221_BABORD_0][INA3221_CH3]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_BABORD_0][INA3221_CH3]));
  
  // Same on the other engine
  /*
  sensor_volt[INA3221_TRIBORD_2][INA3221_CH1]
    ->connect_to(new Linear(1.0, 0.0, conf_path_volt[INA3221_TRIBORD_2][INA3221_CH1]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_TRIBORD_2][INA3221_CH1]));
  sensor_volt[INA3221_TRIBORD_2][INA3221_CH2]
    ->connect_to(new Linear(1.0, 0.0, conf_path_volt[INA3221_TRIBORD_2][INA3221_CH2]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_TRIBORD_2][INA3221_CH2]));
  sensor_volt[INA3221_TRIBORD_2][INA3221_CH3]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_TRIBORD_2][INA3221_CH3]));
  */

  /*
   // Fuel tank volume on portside
  sensor_volt[INA3221_CUVES_1][INA3221_CH1]
    ->connect_to(new Linear(1.0, 0.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH1]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH1]));
  // Fuel tank volume on starboard
  sensor_volt[INA3221_CUVES_1][INA3221_CH2]
    ->connect_to(new Linear(1.0, 0.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH2]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH2]));
  // Water tank volume
  sensor_volt[INA3221_CUVES_1][INA3221_CH3]
    ->connect_to(new Linear(1.0, 0.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH3]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH3]));
  */
}

// Measure Engines RPM
void setupRPMSensors() {
  // Step 1 : calibrate the multiplier with known values (CurveInterpolator class) from analog gauge
  const float multiplier = 1.0 / 97.0;
  //const float hz_to_rpm = 59.999999999999;
  const unsigned int read_rpm_delay = 1000;
  Frequency *freq_babord = new Frequency(multiplier, conf_path_engines[ENGINE_BABORD]);
  Frequency *freq_tribord = new Frequency(multiplier, conf_path_engines[ENGINE_TRIBORD]);
  Linear *hz_to_rpm = new Linear(59.999999999999, 0.0);
  FuelConsumption *rpm_to_lph = new FuelConsumption();

  // Step 2 : instanciate DigitalInputCounter from EL817 sensor
  sensor_engine[ENGINE_BABORD] = new DigitalInputCounter(EL817_BABORD_PIN, INPUT, RISING, read_rpm_delay);    // INPUT or INPUT_PULLUP ? (depends on pin)
  sensor_engine[ENGINE_TRIBORD] = new DigitalInputCounter(EL817_TRIBORD_PIN, INPUT, RISING, read_rpm_delay);  // INPUT or INPUT_PULLUP ? (depends on pin)

  // Step 3 : connect the output of sensor to the input of Frequency() and the Signal K output as a float
  sensor_engine[ENGINE_BABORD]
    ->connect_to(freq_babord)
    ->connect_to(new SKOutputFloat(sk_path_engines[ENGINE_BABORD][REVOLUTIONS]));

  sensor_engine[ENGINE_TRIBORD]
    ->connect_to(freq_tribord)
    ->connect_to(new SKOutputFloat(sk_path_engines[ENGINE_TRIBORD][REVOLUTIONS]));

// cf time_counter.cpp
  freq_babord
    ->connect_to(hz_to_rpm)
    ->connect_to(rpm_to_lph)
    ->connect_to(new SKOutputFloat(sk_path_engines[ENGINE_TRIBORD][FUELRATE]));

}

// The setup function performs one-time application initialization.
void setup() {
  // Some initialization boilerplate when in debug mode...
  //#ifndef SERIAL_DEBUG_DISABLED
  //  SetupSerialDebug(115200);
  //#endif  
  SetupLogging(ESP_LOG_DEBUG); //ESP_LOG_INFO
  Serial.begin(115200);

  // Create the global SensESPApp() object.
  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("birchwood-ESP32")
                    ->get_app();
//                ->set_sk_server(signalkip, signalkport)
//                  ->set_wifi(ssid, password)

  setupSignalKPath();
  setupSensESPConfig();
  setupTemperatureSensors();
  setupVoltageSensors();
  //setupRPMSensors();
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop() { 
  static auto event_loop = SensESPBaseApp::get_event_loop();
  event_loop->tick();

  //JsonDocument jsondoc_engine_1;
  //String output = setupJsonEngine(jsondoc_engine_1, ENGINE_BABORD);
  //Serial.println(output);

  //const char *sk_path_engine = "environment.json.pin15";
  //JsonDocument jsondoc_engine_1;
  //JsonDocument output = setupJsonEngine(jsondoc_engine_1, ENGINE_BABORD);
  //String jsonify;
  //serializeJsonPretty(output, jsonify);
  //Serial.print("JSON :");
  //Serial.println(jsonify);
  //ConstantSensor<JsonDocument> *sensor_engine = new ConstantSensor<JsonDocument>(jsondoc_engine_1);
  //sensor_engine->connect_to(new SKOutputRawJson(sk_path_engine));
  //SKRequest *sk_request = new SKRequest();
  //sk_request->send_request(jsondoc_engine_1, skRequestCallback);

  delay(2000);
}
