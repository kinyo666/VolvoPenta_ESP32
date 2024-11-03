/*
  TODO :
  P0
  - Mettre des valeurs à 0.0 quand le moteur ne tourne pas ? (CoolantTemperature avec K <= 273.15)
  
  P1
  - Ajouter une moyenne de consommation de carburant réelle

  P2
  - Ajouter un compteur de chaine (à partir du signal 12V?)

  @author kinyo666
  @version 1.0.0
  @date 03/11/2024
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
#include <sensesp.h>
//#include "sensesp_app.h"
#include <sensesp_app_builder.h>
#include <sensesp/sensors/sensor.h>
#include <sensesp/sensors/constant_sensor.h>
#include <sensesp/sensors/digital_input.h>
//#include "sensesp/sensors/analog_input.h"
#include <sensesp/signalk/signalk_output.h>
//#include "sensesp/signalk/signalk_put_request.h"
#include <sensesp/transforms/linear.h>
#include <sensesp/transforms/curveinterpolator.h>
#include <sensesp/transforms/frequency.h>
#include <sensesp/transforms/voltagedivider.h>
#include <sensesp/transforms/moving_average.h>
#include <sensesp_onewire/onewire_temperature.h>
#include <string>

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

// RPM sensors PC817
#define PC817_BABORD_PIN 34
#define PC817_TRIBORD_PIN 35

// Engines
#define ENGINE_BABORD 0
#define ENGINE_TRIBORD 1
#define ENGINE_NB 2
#define ENGINE_SK_PATH_NB 3

//#define DEBUG_MODE 1

using namespace sensesp;
using namespace sensesp::onewire;

enum engine_sk_path_t { REVOLUTIONS = 0, FUELRATE, STATE };

// Constants for Signal-K path & ESP config
const String ssid = "Bbox-75CDA064"; // "BirchwoodTS33"
const String password = "5DDF31AF4C1FD577";
//const String signalkip = "192.168.1.57";
//const int signalkport = 3000;
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
const String conf_path_temp[DS18B20_NB] = {
        "/CONFIG/BABORD/DS18B20/TEMP_BABORD", 
        "/CONFIG/TRIBORD/DS18B20/TEMP_TRIBORD",
        "/CONFIG/COMMUN/DS18B20/TEMP_CALE"};
const String conf_path_volt[INA3221_NB][INA3221_CH_NUM] = {
        {"/CONFIG/BABORD/INA3221/0/LINEAR_CH1",   "/CONFIG/BABORD/INA3221/0/LINEAR_CH2",  "/CONFIG/BABORD/INA3221/0/LINEAR_CH3"},
        {"/CONFIG/CUVES/INA3221/1/LINEAR_CH1",    "/CONFIG/CUVES/INA3221/1/LINEAR_CH2",   "/CONFIG/CUVES/INA3221/1/LINEAR_CH3"},
        {"/CONFIG/TRIBORD/INA3221/2/LINEAR_CH1",  "/CONFIG/TRIBORD/INA3221/2/LINEAR_CH2", "/CONFIG/TRIBORD/INA3221/2/LINEAR_CH3"}};
const String conf_path_engines[ENGINE_NB] = {"/CONFIG/BABORD/EL817/FREQUENCY_RPM", 
                                            "/CONFIG/TRIBORD/EL817/FREQUENCY_RPM"};

const unsigned int read_delay = 2000;

// Temperature & Voltage sensors
OneWireTemperature* sensor_temp[DS18B20_NB];                  // 3 DS18B20 Temperature values
INA3221 *sensor_INA3221[INA3221_NB];                          // 4 INA3221 Voltage Sensors with 3 channels each
RepeatSensor<float> *sensor_volt[INA3221_NB][INA3221_CH_NUM]; // 4 Voltage values for each channel
#ifndef DEBUG_MODE
DigitalInputCounter *sensor_engine[ENGINE_NB];                // 2 PC817 RPM Sensors values
#else
FloatConstantSensor *sensor_engine[ENGINE_NB];                // Fake values
#endif

// SensESP builds upon the ReactESP framework. Every ReactESP application must instantiate the "app" object.
reactesp::EventLoop app;

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

// Temperature Interpreter (MD2030 based)
class CoolantTemperature : public CurveInterpolator {
 public:
  CoolantTemperature(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by the temperature sender to degrees Kelvin
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownKelvin));
    add_sample(CurveInterpolator::Sample(20, 393.15));
    add_sample(CurveInterpolator::Sample(30, 383.15));
    add_sample(CurveInterpolator::Sample(40, 373.15));
    add_sample(CurveInterpolator::Sample(55, 363.15));
    add_sample(CurveInterpolator::Sample(70, 353.15));
    add_sample(CurveInterpolator::Sample(100, 343.15));
    add_sample(CurveInterpolator::Sample(140, 333.15));
    add_sample(CurveInterpolator::Sample(200, 323.15));
    add_sample(CurveInterpolator::Sample(300, 317.15));
    add_sample(CurveInterpolator::Sample(400, 313.15)); 
  }
};

// Oil Pressure Interpreter
class OilPressure : public CurveInterpolator {
 public:
  OilPressure(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by the oil pressure sender to bar
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownBar));
    add_sample(CurveInterpolator::Sample(10, 0.0));
    add_sample(CurveInterpolator::Sample(52, 2.0));
    add_sample(CurveInterpolator::Sample(88, 4.0));
    add_sample(CurveInterpolator::Sample(124, 6.0));
  }
};

// Override the Linear class to return values only when positive (e.g RPM = 0.0 => Fuel rate = 0.0)
class LinearPositive : public LambdaTransform<float, float, float, float> {
 public:
  LinearPositive(float multiplier, float offset, const String& config_path = "")
    : LambdaTransform<float, float, float, float>(function_, multiplier, offset,
                                                  param_info_, config_path) {}
 private:
  static float (*function_)(float, float, float);
  static const ParamInfo param_info_[];
};

float (*LinearPositive::function_)(float, float, float) =
    [](float input, float multiplier, float offset) {
      if (input > 0)
        return multiplier * input + offset;
      else
        return (0.0f);
    };

const ParamInfo LinearPositive::param_info_[] = {{"multiplier", "Multiplier"}, {"offset", "Constant offset"}};

// Define the engines transform to output data to Signal-K in m3/s
class EngineDataTransform {
  public :
    Frequency *freq;
    Linear *hz_to_rpm;
    FuelConsumption *rpm_to_lhr;
    LinearPositive *lhr_to_m3s;

    EngineDataTransform(float multiplier, uint8_t engineID) { 
        freq = new Frequency(multiplier, conf_path_engines[engineID]);
        hz_to_rpm = new Linear(59.999999999999, 0.0);                   // Hertz (Hz) to Revolutions Per Minute (RPM)
        rpm_to_lhr = new FuelConsumption();                             // RPM to Liter per Hour (l/hr)
        lhr_to_m3s = new LinearPositive(1 / (3.6 * pow(10, 6)), 0.0);   // Liter per Hour (l/hr) to Meter cube per second (m3/s)
      }
};

// Setup the INA3221 volt sensors
// The INA3221 must have an address on A0 pin, solder the right pin GND (0x40) / SDA (0x41) / SCL (0x42) / VS (0x43)
// - On a Raspberry Pi, the I²C bus is required to be enabled
// - On an ESP32, default SDA pin = 21 & SCL pin = 22
// @param index Slot for INA3221 < INA3221_NB
// @param addr Physical A0 address of the INA3221 sensor
void setupSensorINA3221(u_int8_t index, ina3221_addr_t addr = INA3221_ADDR40_GND) {
  sensor_INA3221[index] = new INA3221(addr);  // Set I2C default address to 0x40 GND or addr
  sensor_INA3221[index]->begin(&Wire);        // Default shunt resistors = 10 mOhm (R100)
  sensor_INA3221[index]->reset();
  delay(10);                                  // Wait for the next sensor setup
  #ifdef DEBUG_MODE
  Serial.printf("Setup Sensor INA3221 0x%X completed : ManufID = %i ; DieID = %i\n", addr, sensor_INA3221[index]->getManufID(), sensor_INA3221[index]->getDieID());
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

// Setup engine temperature sensors
void setupTemperatureSensors() {
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(DS18B20_PIN);
  JsonDocument jsondoc;
  JsonObject jsonsensor = jsondoc.add<JsonObject>();
  
  for (u_int8_t i = 0; i < DS18B20_NB; i++) {
    String config_path_temp = "/CONFIG/DS18B20/SONDE_" + String(std::to_string(i).c_str());
    sensor_temp[i] = new OneWireTemperature(dts, read_delay, config_path_temp);
    sensor_temp[i]->get_configuration(jsonsensor);

    if (jsonsensor["found"] == true)
      switch (i) {
        case DS18B20_BABORD_0 : {
          sensor_temp[DS18B20_BABORD_0]
            ->connect_to(new SKOutputFloat(sk_path_temp[DS18B20_BABORD_0], conf_path_temp[DS18B20_BABORD_0],
                        new SKMetadata("K", "Echappt Babord", sk_path_temp[DS18B20_BABORD_0], "T° Echappement Moteur Babord")));
          break;
        }
        case DS18B20_TRIBORD_1 : {
          sensor_temp[DS18B20_TRIBORD_1]
            ->connect_to(new SKOutputFloat(sk_path_temp[DS18B20_TRIBORD_1], conf_path_temp[DS18B20_TRIBORD_1],
                        new SKMetadata("K", "Echappt Tribord", sk_path_temp[DS18B20_TRIBORD_1], "T° Echappement Moteur Tribord")));
          break;
        }
        case DS18B20_COMMON_2 : {
          sensor_temp[DS18B20_COMMON_2]
            ->connect_to(new SKOutputFloat(sk_path_temp[DS18B20_COMMON_2], conf_path_temp[DS18B20_COMMON_2],
                        new SKMetadata("K", "Compartiment Moteur", sk_path_temp[DS18B20_COMMON_2], "T° Compartiment Moteur")));
          break;
        }
        default : {
          #ifdef DEBUG_MODE
          String output;
          serializeJsonPretty(jsonsensor, output);
          Serial.println("Config DS18B20 : " + config_path_temp + "\n" + output);
          #endif
        }
      };
  }
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
                multiplier = runningBar / (runningVoltage - startVoltage)
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
  setupSensorINA3221(INA3221_CUVES_1, INA3221_ADDR42_SDA);
  //setupSensorINA3221(INA3221_TRIBORD_2, INA3221_ADDR42_SDA);
  //setupSensorINA3221(INA3221_OTHERS_3, INA3221_ADDR43_SCL);

  sensor_volt[INA3221_BABORD_0][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH1);
  sensor_volt[INA3221_BABORD_0][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH2);
  sensor_volt[INA3221_BABORD_0][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH3);

  sensor_volt[INA3221_CUVES_1][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH1);
  sensor_volt[INA3221_CUVES_1][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH2);
  sensor_volt[INA3221_CUVES_1][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH3);

  //sensor_volt[INA3221_TRIBORD_2][INA3221_CH1] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH1);
  //sensor_volt[INA3221_TRIBORD_2][INA3221_CH2] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH2);
  //sensor_volt[INA3221_TRIBORD_2][INA3221_CH3] = new RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH3);
    
  // Voltage for Coolant Temperature gauge
  const float Vin = 3.3; // 3.5
  const float R1 = 1.0; // 120
  sensor_volt[INA3221_BABORD_0][INA3221_CH1]
    ->connect_to(new VoltageDividerR2(R1, Vin, conf_path_volt[INA3221_BABORD_0][INA3221_CH1] + "/VOLTAGE_DIVIDER"))
    ->connect_to(new CoolantTemperature(conf_path_volt[INA3221_BABORD_0][INA3221_CH1] + "/COOLANT_TEMPERATURE"))
    ->connect_to(new LinearPositive(1.0, 0.0, conf_path_volt[INA3221_BABORD_0][INA3221_CH1] + "/LINEAR_POSITIVE"))
    //->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_BABORD_0][INA3221_CH1] + "/MOVING_AVERAGE"))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_BABORD_0][INA3221_CH1],
                    new SKMetadata("K", "T° Eau Babord", "Coolant temperature", "Temp Babord")));

  sensor_volt[INA3221_BABORD_0][INA3221_CH1]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_BABORD_0][INA3221_CH1] + ".raw"));

  // Voltage for Oil Pressure gauge
  sensor_volt[INA3221_BABORD_0][INA3221_CH2]
    ->connect_to(new VoltageDividerR2(R1, Vin, conf_path_volt[INA3221_BABORD_0][INA3221_CH2] + "/VOLTAGE_DIVIDER"))
    ->connect_to(new OilPressure(conf_path_volt[INA3221_BABORD_0][INA3221_CH2] + "/OIL_PRESSURE"))
    ->connect_to(new LinearPositive(pow(10, 5), 0.0, conf_path_volt[INA3221_BABORD_0][INA3221_CH2] + "/LINEAR_BAR_TO_PA"))
    //->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_BABORD_0][INA3221_CH2]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_BABORD_0][INA3221_CH2],
                    new SKMetadata("Pa", "Pression Huile Babord", "Oil pressure", "Huile Babord")));

  sensor_volt[INA3221_BABORD_0][INA3221_CH2]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_BABORD_0][INA3221_CH2] + ".raw"));

  // Voltage for Volts gauge
  sensor_volt[INA3221_BABORD_0][INA3221_CH3]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_BABORD_0][INA3221_CH3],
                    new SKMetadata("V", "Volt Babord", "Alternator voltage", "Volt Babord")));
  
  // Same on the other engine
  /*
  sensor_volt[INA3221_TRIBORD_2][INA3221_CH1]
    ->connect_to(new Linear(1.0, 0.0, conf_path_volt[INA3221_TRIBORD_2][INA3221_CH1]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_TRIBORD_2][INA3221_CH1],
                        new SKMetadata("K", "T° Eau Babord", "Coolant temperature", "Temp Babord")));
  sensor_volt[INA3221_TRIBORD_2][INA3221_CH2]
    ->connect_to(new Linear(1.0, 0.0, conf_path_volt[INA3221_TRIBORD_2][INA3221_CH2]))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_TRIBORD_2][INA3221_CH2]));
  sensor_volt[INA3221_TRIBORD_2][INA3221_CH3]
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_TRIBORD_2][INA3221_CH3]));
  */
  
  // 1. Take the maximum analog input value (i.e. value when sensor is at the high end)
  // 2. Divide 1 by max value. In my case: 1 / 870 = 0.001149425

  // Fuel tank volume on portside
  sensor_volt[INA3221_CUVES_1][INA3221_CH1]
    ->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH1] + "/MOVING_AVERAGE"))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH1],
                  new SKMetadata("m3", "Cuve Babord", "Niveau carburant Babord", "Cuve Babord")));
  // Fuel tank volume on starboard
  sensor_volt[INA3221_CUVES_1][INA3221_CH2]
    ->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH2] + "/MOVING_AVERAGE"))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH2],
                  new SKMetadata("m3", "Cuve Tribord", "Niveau carburant Tribord", "Cuve Tribord")));
  // Water tank volume
  sensor_volt[INA3221_CUVES_1][INA3221_CH3]
    ->connect_to(new MovingAverage(4, 1.0, conf_path_volt[INA3221_CUVES_1][INA3221_CH3] + "/MOVING_AVERAGE"))
    ->connect_to(new SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_CH3],
                  new SKMetadata("m3", "Cuve Eau douce", "Niveau eau douce", "Cuve Eau")));
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

// Measure Engines RPM
void setupRPMSensors() {
  // Step 1 : calibrate the multiplier with known values (CurveInterpolator class) from analog gauge
  //const float multiplier = 1.0 / 97.0;
  const float multiplier = 1.0;
  const unsigned int read_rpm_delay = 1000;

  EngineDataTransform *engine_babord = new EngineDataTransform(multiplier, ENGINE_BABORD);
  EngineDataTransform *engine_tribord = new EngineDataTransform(multiplier, ENGINE_TRIBORD);

  // Step 2 : instanciate DigitalInputCounter from PC817 sensor
  #ifndef DEBUG_MODE
  sensor_engine[ENGINE_BABORD] = new DigitalInputCounter(PC817_BABORD_PIN, INPUT, RISING, read_rpm_delay);    // INPUT or INPUT_PULLUP ? (depends on pin)
  sensor_engine[ENGINE_TRIBORD] = new DigitalInputCounter(PC817_TRIBORD_PIN, INPUT, RISING, read_rpm_delay);  // INPUT or INPUT_PULLUP ? (depends on pin)
  #else
  sensor_engine[ENGINE_BABORD] = new FloatConstantSensor(50.0, 1, "/config/BABORD/EL817/CONSTANT");   // Fake sensor for test purpose
  sensor_engine[ENGINE_TRIBORD] = new FloatConstantSensor(60.0, 1, "/config/TRIBORD/EL817/CONSTANT"); // Fake sensor for test purpose
  #endif

  // Step 3 : connect the output of sensor to the input of Frequency() and the Signal K output as a float in Hz
  sensor_engine[ENGINE_BABORD]
    ->connect_to(engine_babord->freq)
    ->connect_to(new SKOutputFloat(sk_path_engines[ENGINE_BABORD][REVOLUTIONS], 
                  new SKMetadata("Hz", "Moteur Babord", "Engine revolutions (x60 for RPM)", "RPM")));

  sensor_engine[ENGINE_TRIBORD]
    ->connect_to(engine_tribord->freq)
    ->connect_to(new SKOutputFloat(sk_path_engines[ENGINE_TRIBORD][REVOLUTIONS],
                  new SKMetadata("Hz", "Moteur Tribord", "Engine revolutions (x60 for RPM)", "RPM")));

// Step 4 : transforms the output of Frequency to RPM and Liter per hour from FuelConsomption - cf time_counter.cpp
  engine_babord->freq
    ->connect_to(engine_babord->hz_to_rpm)
    ->connect_to(engine_babord->rpm_to_lhr)
    ->connect_to(engine_babord->lhr_to_m3s)
    ->connect_to(new SKOutputFloat(sk_path_engines[ENGINE_BABORD][FUELRATE],
                  new SKMetadata("m3/s", "Conso Babord", "Fuel rate of consumption", "L/hr")));

  engine_tribord->freq
    ->connect_to(engine_tribord->hz_to_rpm)
    ->connect_to(engine_tribord->rpm_to_lhr)
    ->connect_to(engine_tribord->lhr_to_m3s)
    ->connect_to(new SKOutputFloat(sk_path_engines[ENGINE_TRIBORD][FUELRATE],
                  new SKMetadata("m3/s", "Conso Tribord", "Fuel rate of consumption", "L/hr")));

// Step 5 : change the state if needed
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

  //setupSignalKPath();
  //setupSensESPConfig();
  setupTemperatureSensors();
  setupVoltageSensors();
  setupRPMSensors();
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop() { 
  static auto event_loop = SensESPBaseApp::get_event_loop();
  event_loop->tick();

  delay(2000);
}