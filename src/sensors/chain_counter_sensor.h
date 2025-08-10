/*
  Chain counter sensor for windlass :
  - Count how many meters of chain are deployed or retrieved
  - Use a gypsy reed sensor to count the number of revolutions
  - Use a Persistent Integrator to accumulate the number of revolutions
  - Use a HSTS016L current sensor to determine the direction of the windlass
  - Use an ADS1115 sensor to read the gypsy reed sensor and the current sensor
  - The ADS1115 is optional and can be replaced with internal ESP32's ADC1

  @author kinyo666
  @version 1.0.20
  @date 10/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#ifndef CHAIN_COUNTER_SENSOR_H
#define CHAIN_COUNTER_SENSOR_H

//#include <sensesp.h>
#include <ADS1115-Driver.h>
#include <sensesp/sensors/sensor.h>
#include <sensesp/transforms/transform.h>
#include <sensesp/transforms/debounce.h>
#include <sensesp/signalk/signalk_output.h>
#include <helper_3dmath.h>
#include "customClasses.h"

// Windlass Chain counter
#define CHAIN_COUNTER_SAVE_TIMER 20                       // Save the chain counter value every 20 cycles (20 * read_delay_windlass = 10 seconds)
#define CHAIN_COUNTER_PATH 0
#define CHAIN_COUNTER_PATH_DEBOUNCE 1
#define CHAIN_COUNTER_DELAY_DEBOUNCE 400                  // Delay to ignore multiples chain counter readings. Must be less than gypsy_delay
#define CHAIN_COUNTER_DELAY_GYPSY 500
#define CHAIN_COUNTER_DELAY_WINDLASS 500
#define CHAIN_COUNTER_NB 2
#define CHAIN_COUNTER_UI_CYCLE 1                          // Number of cycles to wait before refreshing the UI symbol direction to IDLE
#define ADS1115_WINDLASS_ADDR ADS1115_I2C_ADDR_VDD        // I2C address = 0x49 VDD
#define ADS1115_WINDLASS_THRESHOLD 26400                  // Windlass threshold (depending on the HSTS016L sensor model and PGA used)
#define ADS1115_SENSITIVITY 0.0165                        // Sensitivity of the HSTS016L current sensor

const String sk_path_windlass = "navigation.anchor.rodeDeployed";
const String conf_path_chain[CHAIN_COUNTER_NB] = {  "/CONFIG/CHAINE/COUNTER", "/CONFIG/CHAINE/DELAY" };
const float gypsy_circum = 0.43982;                       // Windlass gypsy circumference (meter) - r = 85 mm ; c = 0.534071 m
const unsigned int read_delay_windlass = 500;             // Windlass read delay = 0.5s

// Override Integrator class with persistent last value, gypsy_circum and A0 pin threshold configuration
class PersistentIntegrator : public sensesp::FloatTransform {
  public:
  PersistentIntegrator(float gypsy_circum = 1.0, float rode_deployed = 0.0, float a0_threshold = 26400, const String& config_path = "")
     : sensesp::FloatTransform(config_path), k{gypsy_circum}, value{rode_deployed}, threshold{a0_threshold} {
      this->load();
      this->emit(value);
  }

  inline int get_windlass_delay() const {
    return windlass_delay;
  } 

  inline int get_gypsy_delay() const {
    return gypsy_delay;
  }

  // Set the direction of the integrator (+/- gypsy circumference)
  inline void set_direction(float direction) {
      k = direction;
  }

  // Add a new value to the integrator
  inline void set(const int& input) {
    if (input == 0)
      value += k;   // Increment the counter if the debounced input is zero
    if (value < 0)
      value = 0.0;  // Prevent negative values
    this->emit(value);
  }

  // Reset to default values
  inline void reset() { value = 0.0; k = 1.0; threshold = ADS1115_WINDLASS_THRESHOLD; }

  // Save the current values to JSON
  inline bool to_json(JsonObject& doc) {
    doc["k"] = k;
    doc["value"] = value;
    doc["threshold"] = threshold;
    doc["windlass_delay"] = windlass_delay;
    doc["gypsy_delay"] = gypsy_delay;
    return true;
  }

  // Load the values from JSON
  inline bool from_json(const JsonObject& config) {
    if (!config["k"].is<float>()) {
        return false;
      }
    k = config["k"];
    value = (config["value"].is<float>() ? config["value"] : 0.0);                                      // May not have a value at the first load
    threshold = (config["threshold"].is<float>() ? config["threshold"] : ADS1115_WINDLASS_THRESHOLD);   // Default threshold value for A0 pin
    windlass_delay = (config["windlass_delay"].is<int>() ? config["windlass_delay"] : CHAIN_COUNTER_DELAY_WINDLASS);
    gypsy_delay = (config["gypsy_delay"].is<int>() ? config["gypsy_delay"] : CHAIN_COUNTER_DELAY_GYPSY);

    return true;
  }

 private:
  float k;                                            // Multiplier for the integrator (gypsy circumference)
  float value = 0.0;                                  // Last value of the integrator (rode deployed)
  float threshold = ADS1115_WINDLASS_THRESHOLD;       // Threshold value to determine if windlass UP direction is active (A0 pin, default 26400)
  int windlass_delay = CHAIN_COUNTER_DELAY_WINDLASS;  // Default 500ms
  int gypsy_delay = CHAIN_COUNTER_DELAY_GYPSY;        // Default 500ms
};

void setupWindlassSensor();
bool ConfigRequiresRestart(const PersistentIntegrator& obj);
const String ConfigSchema(const PersistentIntegrator& obj);

#endif // CHAIN_COUNTER_SENSOR_H