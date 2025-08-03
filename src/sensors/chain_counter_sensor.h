/*
  Chain counter sensor for windlass :
  - Count how many meters of chain are deployed or retrieved
  - Use a gypsy inductive sensor to count the number of revolutions
  - Use a Persistent Integrator to accumulate the number of revolutions
  - Use a HSTS016L current sensor to determine the direction of the windlass
  - Use an ADS1115 sensor to read the gypsy inductive sensor and the current sensor

  @author kinyo666
  @version 1.0.15
  @date 03/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#ifndef CHAIN_COUNTER_SENSOR_H
#define CHAIN_COUNTER_SENSOR_H

#include <sensesp.h>
#include <ADS1115-Driver.h>
#include <sensesp/sensors/sensor.h>
#include <sensesp/transforms/transform.h>
#include <sensesp/transforms/debounce.h>
#include <sensesp/signalk/signalk_output.h>
#include <helper_3dmath.h>
#include "customClasses.h"

// Windlass Chain counter
#define CHAIN_COUNTER_PIN 32
#define CHAIN_COUNTER_SAVE_TIMER 20                       // Save the chain counter value every 20 cycles (20 * read_delay_windlass = 10 seconds)
#define CHAIN_COUNTER_IGNORE_DELAY 400                    // Delay to ignore multiples chain counter readings. Must be less than read_delay_windlass
#define CHAIN_COUNTER_PATH 0
#define CHAIN_COUNTER_DELAY 1
#define CHAIN_COUNTER_NB 2
#define CHAIN_COUNTER_UI_CYCLE 1                          // Number of cycles to wait before refreshing the UI symbol direction to IDLE
#define ADS1115_WINDLASS_ADDR ADS1115_I2C_ADDR_VDD        // I2C address = 0x49 VDD
#define ADS1115_WINDLASS_THRESHOLD 26400                  // Windlass threshold (depending on the HSTS016L sensor and PGA used)

namespace sensesp {

const String sk_path_windlass = "navigation.anchor.rodeDeployed";
const String conf_path_chain[CHAIN_COUNTER_NB] = { "/CONFIG/CHAINE/COUNTER", "/CONFIG/CHAINE/DELAY" };
const float gipsy_circum = 0.43982;                           // Windlass gipsy circumference (meter) - r = 85 mm ; c = 0.534071 m
const unsigned int read_delay_windlass = 500;                 // Windlass read delay = 0.5s

// Override Integrator class with persistent last value, gipsy_circum and A0 pin threshold configuration
class PersistentIntegrator : public Transform<int, float> {
  public:
  PersistentIntegrator(float gipsy_circum = 1.0, float rode_deployed = 0.0, float a0_threshold = 26400, const String& config_path = "")
     : Transform<int, float>(config_path), k{gipsy_circum}, value{rode_deployed}, threshold{a0_threshold} {
      this->load();
      this->emit(value);
  }

  inline void set_direction(float direction) {
      k = direction;
  }

  inline void set(const int& input) {
    if (input == 0)
      value += k;   // Increment the counter if the debounced input is zero
    if (value < 0)
      value = 0.0;  // Prevent negative values
    this->emit(value);
  }

  inline void reset() { value = 0.0; k = 1.0; threshold = 26400.0; }  // Reset to default values

  inline bool to_json(JsonObject& doc) {
    doc["k"] = k;
    doc["value"] = value;
    doc["threshold"] = threshold;
    return true;
  }

  inline bool from_json(const JsonObject& config) {
    if (!config["k"].is<float>()) {
        return false;
      }
    k = config["k"];
    value = (config["value"].is<float>() ? config["value"] : 0.0);                  // May not have a value at the first load
    threshold = (config["threshold"].is<float>() ? config["threshold"] : 26400.0);  // Default threshold value for A0 pin

    return true;
  }

 private:
  float k;                  // Multiplier for the integrator (gipsy circumference)
  float value = 0.0;        // Last value of the integrator (rode deployed)
  float threshold;          // Threshold value to determine if windlass UP direction is active (A0 pin, default 26400)
};

void setupWindlassSensor();
bool ConfigRequiresRestart(const PersistentIntegrator& obj);
const String ConfigSchema(const PersistentIntegrator& obj);

}
#endif // CHAIN_COUNTER_SENSOR_H