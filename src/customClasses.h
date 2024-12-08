/*
  Extended classes for custom usage :
  - CurveInterpolator for Fuel Consumption, Coolant Temperature, Oil Pressure
  - Transform for Positive Linear transform and Engine Data
  - Integrator for Persistent Integrator

  @author kinyo666
  @version 1.0.4
  @date 06/12/2024
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#include <sensesp/transforms/linear.h>
#include <sensesp/transforms/curveinterpolator.h>
#include <sensesp/transforms/frequency.h>
#include <sensesp/transforms/voltagedivider.h>
#include <sensesp/transforms/moving_average.h>
#include <sensesp/transforms/integrator.h>

using namespace sensesp;

// Custom schema of Persistent Integrator for SensESP UI (last value is now saved)
static const char PINTEGRATOR_SCHEMA[] PROGMEM = R"({
    "type": "object",
    "properties": {
        "k": { "title": "Multiplier", "type": "number" },
        "value": { "title": "Value", "type": "number" }
    }
  })";

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

// Define the engines transform to output data to Signal-K
class EngineDataTransform {
  public :
    Frequency *freq;
    Linear *hz_to_rpm;
    FuelConsumption *rpm_to_lhr;
    LinearPositive *lhr_to_m3s;
    MovingAverage *moving_avg;

    EngineDataTransform(float multiplier, String conf_path_engine) { 
        freq = new Frequency(multiplier, conf_path_engine);
        hz_to_rpm = new Linear(60, 0.0);                                // Hertz (Hz) to Revolutions Per Minute (RPM)
        rpm_to_lhr = new FuelConsumption();                             // RPM to Liter per Hour (l/hr)
        lhr_to_m3s = new LinearPositive(1 / (3.6 * pow(10, 6)), 0.0);   // Liter per Hour (l/hr) to Meter cube per second (m3/s)
        moving_avg = new MovingAverage(2, 1.0);                         // Moving average with 2 samples
      }
};

// Override Integrator class with persistent last value and k multiplier configuration
class PersistentIntegrator : public Transform<int, float> {
  public:
  PersistentIntegrator(float gipsy_circum = 1.0, float value = 0.0, const String& config_path = "")
     : Transform<int, float>(config_path), k{k}, value{value} {
      this->load_configuration();
      this->emit(value);
  }

  void set(const int& input) {
    value += input * k;
    this->emit(value);
  }

  void reset() { value = 0.0; k = 1.0; }

  virtual void get_configuration(JsonObject& doc) override final {
    doc["k"] = k;
    doc["value"] = value;
  }

  virtual bool set_configuration(const JsonObject& config) override final {
    if (!config["k"].is<float>()) {
        return false;
      }
    k = config["k"];
    value = (config["value"].is<float>() ? config["value"] : 0.0);    // May not have a value at the first load

    return true;
  }
  virtual String get_config_schema() override {
    return FPSTR(PINTEGRATOR_SCHEMA);
  }

 private:
  float k;
  float value = 0.0;
};