/*
  Extended classes for custom usage :
  - CurveInterpolator for Fuel Consumption, Coolant Temperature, Oil Pressure
  - Transform for Positive Linear transform and Engine Data
  - Integrator for Persistent Integrator
  - ConfigSchema for Persistent Integrator and DebounceInt (missing in SensESP v3.0.0)

  @author kinyo666
  @version 1.0.10
  @date 19/04/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#include <sensesp/transforms/linear.h>
#include <sensesp/transforms/curveinterpolator.h>
#include <sensesp/transforms/frequency.h>
#include <sensesp/transforms/voltagedivider.h>
#include <sensesp/transforms/moving_average.h>
#include <sensesp/transforms/integrator.h>
#include <sensesp/ui/config_item.h>

// Binary mask for engine state and sleep mode enable(1) / disable(0)
#define ENGINE_STATE_OFF          0x000
#define ENGINE_STATE_ON           0x001
#define ENGINE_STATE_NOT_RUNNING  0x101
#define ENGINE_STATE_RUNNING      0x011
#define ENGINE_STATE_IS_RUNNING   0x010
#define ENGINE_SLEEP_ENABLE 1

#define DEBUG_MODE_CUSTOM_CLASSES_H 1

extern void sleepModeINA3221(u_int8_t, u_int8_t);
extern const String conf_path_global;

using namespace sensesp;

typedef LambdaTransform<float, float, float, float> LinearPos;
typedef LambdaTransform<float, boolean, u_int8_t> EngineState;

// Keys for enable / disable sensors in the configuration UI
const char* sensor_keys[] = {
      "DS18B20_FEATURE", "DS18B20_BABORD_0", "DS18B20_TRIBORD_1", "DS18B20_COMMON_2",
      "INA3221_FEATURE", "INA3221_BABORD_0", "INA3221_CUVES_1", "INA3221_TRIBORD_2",
      "INA3221_OTHERS_3", "INA3221_POWERDOWN", "PC817_FEATURE", "PC817_BABORD", "PC817_TRIBORD",
      "CHAIN_COUNTER_FEATURE", "MOTION_SENSOR_FEATURE", "MOTION_SENSOR_CALIBRATE", "RUDDER_ANGLE_FEATURE"
 };

// Callback for Linear Positive (Lambda Transform returns 0.0 if not positive)
auto linearPositive = [](float input, float multiplier, float offset) -> float {
  if (input > 0)
    return multiplier * input + offset;
  else
    return (0.0f);
};
const ParamInfo* linearPositive_ParamInfo = new ParamInfo[2]{{"multiplier", "Multiplier"}, {"offset", "Offset"}};

// Callback for Engine State (Lambda Transform returns true if engine is running, false otherwise)
auto runningState = [](float input, u_int8_t engine_id) -> boolean {
  #if ENGINE_SLEEP_ENABLE == 1
  sleepModeINA3221(engine_id, ((input > 1) ? ENGINE_STATE_RUNNING : ENGINE_STATE_NOT_RUNNING));
  #endif
  return (input > 0);
};
const ParamInfo* runningState_ParamInfo = new ParamInfo[1]{{"engine_state", "Engine State"}};

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

/*
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
*/

// Define the engines transform to output data to Signal-K
class EngineDataTransform {
  public :
    Frequency *freq;
    Linear *hz_to_rpm;
    FuelConsumption *rpm_to_lhr;
    LinearPos *lhr_to_m3s; //LinearPositive *lhr_to_m3s;
    MovingAverage *moving_avg;
    EngineState *running_state;

    EngineDataTransform(float multiplier, String conf_path_engine, u_int engine_id) { 
        freq = new Frequency(multiplier, conf_path_engine);                 // Pulses to Hertz (Hz)
        hz_to_rpm = new Linear(60, 0.0);                                    // Hertz (Hz) to Revolutions Per Minute (RPM)
        rpm_to_lhr = new FuelConsumption();                                 // RPM to Liter per Hour (l/hr)
        //lhr_to_m3s = new LinearPositive(1 / (3.6 * pow(10, 6)), 0.0);   // Liter per Hour (l/hr) to Meter cube per second (m3/s)
        lhr_to_m3s = new LinearPos(linearPositive, 1 / (3.6 * pow(10, 6)), 
                                  0.0, linearPositive_ParamInfo);           // Liter per Hour (l/hr) to Meter cube per second (m3/s)
        moving_avg = new MovingAverage(4, 1.0);                             // Moving average with 4 samples
        running_state = new EngineState(runningState, engine_id, 
                                        runningState_ParamInfo);            // Hertz (Hz) to running state (true or false)

        ConfigItem(freq)
        ->set_title("Compte-tours - " + conf_path_engine)
        ->set_description("Multiplier needs to be adjusted based on flywheel to Alternator W ratio."\
                          "Example: 1 turn on an AQAD41 makes 2.45 turns on the Alternator, and the Alternator has 6 poles which makes a total of 2.45 x 6 = 14.7 Hz per turn. SignalK needs info in Hz so we need to divide the incoming value with 14.7, or as in our case multiply with (1/14.7) = 0,06803"\
                          "If Ratio is unknown, the original Tachometer might have a max Impulses/min written on it, divide that with max rpm on the meter and you\'ll get the ratio. Tachometer 860420 is marked with 73500Imp/min and has a scale to 5000rpm. 73500 divided with 5000 equals 14,7, Tada!"\
                          "If above multiplier needs to be recalculated, the FuelMultipliers needs to be recalculated as well, they're both based on AQAD41 values right now."\
                          "AQAD41 example: 60 divided with FlyWheel To W Ratio (60 / 14,7 = 4,08)")
        ->set_sort_order(25+(2*engine_id));

        // TODO : add a ConfigItem for moving_avg
        /*
        ConfigItem(moving_avg)
        ->set_title("Compte-tours - MovingAvg " + conf_path_engine)
        ->set_description("Moving average of the engine RPM to smooth out the signal."\
                          "Default value is 4 samples.")
        ->set_sort_order(26+(2*engine_id));
        */
      }
};

// MovingAverage class to calculate the average offset of a quaternion over a number of samples
class MovingAverageOffsetQuaternion : public Transform<Quaternion, String> {
  public:
  MovingAverageOffsetQuaternion(int num_samples = 6, const String& config_path = "")
     : Transform<Quaternion, String>(config_path), sample_size_{num_samples}, initialized_(false), saved_(false) {
      bufX_.resize(sample_size_, 0);
      bufY_.resize(sample_size_, 0);
      bufZ_.resize(sample_size_, 0);
      outputX_ = 0.0;
      outputY_ = 0.0;
      outputZ_ = 0.0;
      conf_motionsensor = jdoc_conf_motionsensor.to<JsonObject>();

      this->load();
  }

  void set(const Quaternion& input) {
    // So the first value to be included in the average doesn't default to 0.0
    if (!initialized_) {
      bufX_.assign(sample_size_, input.x);
      bufY_.assign(sample_size_, input.y);
      bufZ_.assign(sample_size_, input.z);
      outputX_ = input.x;
      outputY_ = input.y;
      outputZ_ = input.z;
      initialized_ = true;
    } else {
      // Subtract 1/nth of the oldest value and add 1/nth of the newest value
      outputX_ += -bufX_[ptr_] / sample_size_;
      outputX_ += input.x / sample_size_;
      outputY_ += -bufY_[ptr_] / sample_size_;
      outputY_ += input.y / sample_size_;
      outputZ_ += -bufZ_[ptr_] / sample_size_;
      outputZ_ += input.z / sample_size_;
  
      // Save the most recent input, then advance to the next storage location.
      // When storage location n is reached, start over again at 0.
      bufX_[ptr_] = input.x;
      bufY_[ptr_] = input.y;
      bufZ_[ptr_] = input.z;
      ptr_ = (ptr_ + 1) % sample_size_;

      // Save the average value to the configuration file
      if ((ptr_ == 0) && (!saved_)) {
        this->save();
        saved_ = true;

        // TODO : remove this line
        #ifdef DEBUG_MODE_CUSTOM_CLASSES_H
        Serial.printf("MOTION SENSOR OFFSET SAVED : X_OFFSET = %f\t| Y_OFFSET = %f\t| Z_OFFSET = %f\n", outputX_, outputY_, outputZ_);
        #endif
      }
    }

    conf_motionsensor["x_offset"] = outputX_;
    conf_motionsensor["y_offset"] = outputY_;
    conf_motionsensor["z_offset"] = outputZ_;    
    serializeJsonPretty(conf_motionsensor, output_);
    notify();
  }

  void reset() { 
    bufX_.assign(sample_size_, 0);
    bufY_.assign(sample_size_, 0);
    bufZ_.assign(sample_size_, 0);
    ptr_ = 0;
    initialized_ = false;
    saved_ = false;
    outputX_ = 0.0;
    outputY_ = 0.0;
    outputZ_ = 0.0;
  }

  bool to_json(JsonObject& doc) {
    doc["sample_size"] = sample_size_;
    doc["x_offset"] = outputX_;
    doc["y_offset"] = outputY_;
    doc["z_offset"] = outputZ_;
    return true;
  }

  bool from_json(const JsonObject& config) {
    if (!config["x_offset"].is<float>())
      return false;

    outputX_ = config["x_offset"].is<float>() ? config["x_offset"] : 0.0;
    outputY_ = config["y_offset"].is<float>() ? config["y_offset"] : 0.0;
    outputZ_ = config["z_offset"].is<float>() ? config["z_offset"] : 0.0;
    sample_size_ = config["sample_size"].is<int>() ? config["sample_size"] : 6;

    return true;
  }

  float getXOffset() const { return outputX_; }
  float getYOffset() const { return outputY_; }
  float getZOffset() const { return outputZ_; }

  private:
  std::vector<float> bufX_{};
  std::vector<float> bufY_{};
  std::vector<float> bufZ_{};
  float outputX_;
  float outputY_; 
  float outputZ_;
  int ptr_ = 0;
  int sample_size_;
  bool saved_;
  bool initialized_;
  JsonDocument jdoc_conf_motionsensor;
  JsonObject conf_motionsensor;                                 // MPU X/Y/Z offsets values if exists
};

const String ConfigSchema(const MovingAverageOffsetQuaternion& obj) {
  return R"({
    "type": "object",
    "properties": {
        "sample_size": { "title": "Sample Size", "type": "number" },
        "x_offset": { "title": "X Offset", "type": "number" },
        "y_offset": { "title": "Y Offset", "type": "number" },
        "z_offset": { "title": "Z Offset", "type": "number" }
    }
  })";
  }
  
  inline bool ConfigRequiresRestart(const MovingAverageOffsetQuaternion& obj) {
    return true;
  }

// Override Integrator class with persistent last value and k multiplier configuration
class PersistentIntegrator : public Transform<int, float> {
  public:
  PersistentIntegrator(float gipsy_circum = 1.0, float value = 0.0, const String& config_path = "")
     : Transform<int, float>(config_path), k{k}, value{value} {
      this->load();
      this->emit(value);
  }

  void set(const int& input) {
    value += input * k;
    this->emit(value);
  }

  void reset() { value = 0.0; k = 1.0; }

  bool to_json(JsonObject& doc) {
    doc["k"] = k;
    doc["value"] = value;
    return true;
  }

  bool from_json(const JsonObject& config) {
    if (!config["k"].is<float>()) {
        return false;
      }
    k = config["k"];
    value = (config["value"].is<float>() ? config["value"] : 0.0);    // May not have a value at the first load

    return true;
  }

 private:
  float k;
  float value = 0.0;
};

const String ConfigSchema(const PersistentIntegrator& obj) {
return R"({
  "type": "object",
  "properties": {
      "k": { "title": "Multiplier", "type": "number" },
      "value": { "title": "Value", "type": "number" }
  }
})";
}

inline bool ConfigRequiresRestart(const PersistentIntegrator& obj) {
  return true;
}

// Load and save SensESP configuration to enable/disable sensors
class ConfigSensESP : public SensorConfig {
  public:
  ConfigSensESP(const String& config_path) : SensorConfig(config_path) {
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

    #ifdef DEBUG_MODE_CUSTOM_CLASSES_H
      Serial.println("SENSORS CONFIG :");
      String jsonify;
      serializeJsonPretty(config_json, jsonify);
      Serial.println(jsonify);
    #endif
  }

  // Set configuration to a JSON file
  bool to_json(JsonObject& root) override { 
    for (const char* key : sensor_keys)
      if (config_json[key].is<bool>())
        root[key] = config_json[key];
      else
        root[key] = false;

    return true;
  }

  // Get configuration from the JSON file
  bool from_json(const JsonObject& root) override {
    for (const char* key : sensor_keys)
      if (root[key].is<bool>())
        config_json[key] = root[key];
      else
        config_json[key] = false;

    return true;
  }

  // Return true if the sensor (key) is enabled
  bool is_enabled(String key) {
    if (config_json[key].is<bool>())
      return config_json[key];
    else
      return false;
  }

  // Set the status of a sensor (key) to enabled or disabled
  void set_status(String key, bool status) {
    config_json[key] = status;
  }

  // Return the configuration path
  const String& get_config_path() {
    return conf_path_global;
  }

  // Return the configuration schema
  const String get_config_schema() {
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
        "INA3221_OTHERS_3": { "type": "boolean", "title": "- INA3221 Windlass + Rudder Angle" },
        "INA3221_POWERDOWN": { "type": "boolean", "title": "- INA3221 Power Down" },
        "PC817_FEATURE": { "type": "boolean", "title": "PC817 Feature" },
        "PC817_BABORD": { "type": "boolean", "title": "- PC817 Babord" },
        "PC817_TRIBORD": { "type": "boolean", "title": "- PC817 Tribord" },
        "CHAIN_COUNTER_FEATURE": { "type": "boolean", "title": "Chain Counter Feature" },
        "MOTION_SENSOR_FEATURE": { "type": "boolean", "title": "Motion Sensor Feature" },
        "MOTION_SENSOR_CALIBRATE": { "type": "boolean", "title": "Calibrate Motion Sensor" },
        "RUDDER_ANGLE_FEATURE": { "type": "boolean", "title": "Rudder Angle Feature" }
      }
    })";
  }

  private:
  JsonDocument config_json;
};

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
      "INA3221_OTHERS_3": { "type": "boolean", "title": "- INA3221 Others" },
      "INA3221_POWERDOWN": { "type": "boolean", "title": "- INA3221 Power Down" },
      "PC817_FEATURE": { "type": "boolean", "title": "PC817 Feature" },
      "PC817_BABORD": { "type": "boolean", "title": "- PC817 Babord" },
      "PC817_TRIBORD": { "type": "boolean", "title": "- PC817 Tribord" },
      "CHAIN_COUNTER_FEATURE": { "type": "boolean", "title": "Chain Counter Feature" },
      "MOTION_SENSOR_FEATURE": { "type": "boolean", "title": "Motion Sensor Feature" },
      "MOTION_SENSOR_CALIBRATE": { "type": "boolean", "title": "Calibrate Motion Sensor" },
      "RUDDER_ANGLE_FEATURE": { "type": "boolean", "title": "Rudder Angle Feature" }
    }
  })";
}

// Set requires restart to true for ConfigSensESP
inline bool ConfigRequiresRestart(const ConfigSensESP& obj) {
  return true;
}

// Patched in custom debounce.h with appropriate template and removed from debounce.cpp.
// from_json() and to_json moved into public section as well
/*
const String ConfigSchema(const Debounce<int>& obj) {
  return R"###({
    "type": "object",
    "properties": {
      "min_delay": {
        "title": "Minimum delay",
        "type": "number",
        "description": "The minimum time in ms between inputs for output to happen"
      }
    }
  })###";
}
*/