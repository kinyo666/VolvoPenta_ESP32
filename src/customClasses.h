/*
  Extended classes for custom usage :
  - CurveInterpolator for Fuel Consumption, Coolant Temperature, Oil Pressure
  - Transform for Positive Linear transform and Engine Data
  - Integrator for Persistent Integrator
  - ConfigSchema for ConfigSensESP and Persistent Integrator

  @author kinyo666
  @version 1.0.20
  @date 06/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#ifndef CUSTOM_CLASSES_H
#define CUSTOM_CLASSES_H

#include <sensesp/sensors/sensor.h>
#include <sensesp/transforms/lambda_transform.h>
#include <sensesp/transforms/curveinterpolator.h>
#include <sensesp/ui/config_item.h>

// SensESP UI Config order
#define UI_ORDER_TEMP 10
#define UI_ORDER_ENGINE 20
#define UI_ORDER_TANK 30
#define UI_ORDER_CHAIN 40
#define UI_ORDER_MOTION 50

// Engines
#define ENGINE_BABORD 0
#define ENGINE_TRIBORD 1
#define ENGINE_NB 2
#define ENGINE_SK_PATH_NB 3
#define ENGINE_SLEEP_TIMER 60

// Config path index for PC817 sensors
#define PC817_FREQUENCY_RPM 0
#define PC817_MOVING_AVG 1

// Debug mode (verbose logs)
#define DEBUG_MODE 1

const String conf_path_global = "/CONFIG/SENSORS_CONFIG";
extern const sensesp::ParamInfo *linearPositive_ParamInfo;

typedef sensesp::LambdaTransform<float, float, float, float> LinearPos;

float linearPositive(float, float, float);

// Load and save SensESP configuration to enable/disable sensors
class ConfigSensESP : public sensesp::SensorConfig {
  public:
  ConfigSensESP(const String& config_path = conf_path_global) : SensorConfig(config_path) {
    // Load current configuration otherwise set default values to true for all sensors
    if (!load()) {
      for (const char* key : sensor_keys)
        config_json[key] = true;
      save();
    }

    // Initialize ConfigItem instance with custom ConfigSchema
    ConfigItem(this)->set_title("Sensors Configuration")
                    ->set_description("Enable or disable sensors by group or individually. Restart to apply any changes.")
                    ->set_config_schema(get_config_schema())
                    ->set_requires_restart(true)
                    ->set_sort_order(0);

    #ifdef DEBUG_MODE
      Serial.println("SENSORS CONFIG :");
      String jsonify;
      serializeJsonPretty(config_json, jsonify);
      Serial.println(jsonify);
    #endif
  }

  // Set configuration to a JSON file
  inline bool to_json(JsonObject& root) override { 
    for (const char* key : sensor_keys)
      if (config_json[key].is<bool>())
        root[key] = config_json[key];
      else
        root[key] = false;

    return true;
  }

  // Get configuration from the JSON file
  inline bool from_json(const JsonObject& root) override {
    for (const char* key : sensor_keys)
      if (root[key].is<bool>())
        config_json[key] = root[key];
      else
        config_json[key] = false;

    return true;
  }

  // Return true if the sensor (key) is enabled
  inline bool is_enabled(String key) {
    if (config_json[key].is<bool>())
      return config_json[key];
    else
      return false;
  }

  // Set the status of a sensor (key) to enabled or disabled
  inline void set_status(String key, bool status) {
    config_json[key] = status;
  }

  // Return the configuration path
  inline const String& get_config_path() {
    return conf_path_global;
  }

  // Return the configuration schema
  inline const String get_config_schema() {
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

  inline unsigned int get_read_delay() {
    return this->read_delay;
  }

  private:
  unsigned int read_delay = 1000; // Default read delay in milliseconds
  JsonDocument config_json;
  // Keys for enable / disable sensors in the configuration UI
  const char* sensor_keys[17] = {
        "DS18B20_FEATURE", "DS18B20_BABORD_0", "DS18B20_TRIBORD_1", "DS18B20_COMMON_2",
        "INA3221_FEATURE", "INA3221_BABORD_0", "INA3221_CUVES_1", "INA3221_TRIBORD_2",
        "INA3221_OTHERS_3", "INA3221_POWERDOWN", "PC817_FEATURE", "PC817_BABORD", "PC817_TRIBORD",
        "CHAIN_COUNTER_FEATURE", "MOTION_SENSOR_FEATURE", "MOTION_SENSOR_CALIBRATE", "RUDDER_ANGLE_FEATURE"
  };
};

// Temperature Interpreter (MD2030 based)
class CoolantTemperature : public sensesp::CurveInterpolator {
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
class OilPressure : public sensesp::CurveInterpolator {
 public:
  OilPressure(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by the oil pressure sender to bar
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownBar));
    /*
    add_sample(CurveInterpolator::Sample(10, 0.0));
    add_sample(CurveInterpolator::Sample(52, 2.0));
    add_sample(CurveInterpolator::Sample(88, 4.0));
    add_sample(CurveInterpolator::Sample(124, 6.0));
    */
    /* @link https://www.14point7.com/products/solid-afr-gauge
      PSI	0	14,5	29	43,5	58	72,5	87	101,5	116	130,5	145
      Ω	3	17	34	51	68	85	102	119	133	147	160
    */
    add_sample(CurveInterpolator::Sample(3, 0.0));    // 0 PSI = 0 bar
    add_sample(CurveInterpolator::Sample(17, 1.0));   // 14.5 PSI = 1 bar
    add_sample(CurveInterpolator::Sample(34, 2.0));   // 29 PSI = 2 bar
    add_sample(CurveInterpolator::Sample(51, 3.0));   // 43.5 PSI = 3 bar
    add_sample(CurveInterpolator::Sample(68, 4.0));   // 58 PSI = 4 bar
    add_sample(CurveInterpolator::Sample(85, 5.0));   // 72.5 PSI = 5 bar
    add_sample(CurveInterpolator::Sample(102, 6.0));  // 87 PSI = 6 bar
    add_sample(CurveInterpolator::Sample(119, 7.0));  // 101.5 PSI = 7 bar
    add_sample(CurveInterpolator::Sample(133, 8.0));  // 116 PSI = 8 bar
    add_sample(CurveInterpolator::Sample(147, 9.0));  // 130.5 PSI = 9 bar
    add_sample(CurveInterpolator::Sample(160, 10.0)); // 145 PSI = 10 bar
  }
};

bool ConfigRequiresRestart(const ConfigSensESP& obj);
const String ConfigSchema(const ConfigSensESP& obj);

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
/*
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
*/

#endif // CUSTOM_CLASSES_H