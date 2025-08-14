/*
  Chain counter sensor for windlass :
  - Count how many meters of chain are deployed or retrieved
  - Use a gypsy reed sensor to count the number of revolutions
  - Use a Persistent Integrator to accumulate the number of revolutions
  - Use a HSTS016L current sensor to determine the direction of the windlass
  - Use an ADS1115 sensor to read the gypsy reed sensor and the current sensor
  - The ADS1115 is optional and can be replaced with internal ESP32's ADC1

  @author kinyo666
  @version 1.0.22
  @date 14/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32

  TODO : revoir la logique d'init / lecture / sauvegarde et types des valeurs
*/
#ifndef CHAIN_COUNTER_SENSOR_H
#define CHAIN_COUNTER_SENSOR_H

#include <ADS1115-Driver.h>
#include <sensesp/sensors/sensor.h>
#include <sensesp/transforms/transform.h>
#include <sensesp/transforms/debounce.h>
#include <sensesp/signalk/signalk_output.h>
#include <helper_3dmath.h>
#include "customClasses.h"

// Windlass Chain counter
#define CHAIN_COUNTER_SAVE_TIMER      20                  // Save the chain counter value every 20 cycles (20 * read_delay_windlass = 10 seconds)
#define CHAIN_COUNTER_PATH            0                   // Index of config_path for PersistentIntegrator
#define CHAIN_COUNTER_PATH_DEBOUNCE   1                   // Index of config_path for Debounce object
#define CHAIN_COUNTER_NB              2                   // Size of conf_path_chain[]
#define CHAIN_COUNTER_DELAY_DEBOUNCE  400                 // Default delay to ignore multiples chain counter readings. Must be less than gypsy_delay
#define CHAIN_COUNTER_DELAY_GYPSY     500                 // Default delay to read gypsy sensor (ms)
#define CHAIN_COUNTER_DELAY_WINDLASS  500                 // Default delay to read windlass sensor (ms)
#define CHAIN_COUNTER_UI_CYCLE        1                   // Number of cycles to wait before refreshing the UI symbol direction to IDLE

// ADS1115 configuration
#define ADS1115_WINDLASS_ADDR ADS1115_I2C_ADDR_GND        // I2C address = 0x48 GND (<-- change here if needed)
#define ADS1115_WINDLASS_THRESHOLD 26400                  // Windlass threshold (depending on the HSTS016L sensor model and PGA used)
#define ADS1115_GYPSY_THRESHOLD    26400                  // Gypsy threshold (depending on the HSTS016L sensor model and PGA used)
#define ADS1115_PGA_6_144_RES      0.1875
#define ADS1115_PGA_4_096_RES      0.125
#define ADS1115_PGA_2_048_RES      0.0625                 // Fix for ADS1115 lib
#define ADS1115_PGA_1_024_RES      0.03125                // Fix for ADS1115 lib
#define ADS1115_PGA_0_512_RES      0.015625
#define ADS1115_PGA_0_256_RES      0.0078125

// HSTS016L characteristics
#define HSTS016L_VREF               1.65f                 // Reference voltage Vref = +1.65V ; Vout = Vref + ±0.625V
#define HSTS016L_SENSITIVITY        0.003125f             // Sensitivity V/A = 0.625V / 200A = 3.125 mV/A

const String sk_path_windlass = "navigation.anchor.rodeDeployed";
const String conf_path_chain[CHAIN_COUNTER_NB] = {  "/CONFIG/CHAINE/COUNTER", "/CONFIG/CHAINE/DELAY" };
const float gypsy_circum = 0.43982;                       // Windlass gypsy circumference (meter) = 2π*r (ex: radius = 85 mm ; circum = 0.534071 m)
const unsigned int read_delay_windlass = 500;             // Windlass read delay = 0.5s

class ADS1115Sensor : public ADS1115 {
  public:
  ADS1115Sensor(i2c_addr_t i2cSlaveAddr = ADS1115_WINDLASS_ADDR) : ADS1115(i2cSlaveAddr) {

    reset();
    delayMicroseconds(50);                                        // Wait for the ADS1115 to reset
    setDeviceMode(ADS1115_MODE_SINGLE);                     // Single conversion
    setDataRate(ADS1115_DR_128_SPS);                        // ADS1115_DR_128_SPS
    setPga(ADS1115_PGA_4_096);                              // Set PGA to 4.096V 1 bit = 2mV @see https://forums.adafruit.com/viewtopic.php?t=186225
    setLatching(false);                                     // Non-latching
    setComparatorQueue(ADS1115_COMP_QUE_DISABLE);           // Disable comparator
  }
  
  inline int16_t readRegister(reg_addr_t dataAddress) {
    Wire.beginTransmission((uint8_t) ADS1115_WINDLASS_ADDR);
    Wire.write(dataAddress);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t) ADS1115_WINDLASS_ADDR, (uint8_t) 2);

    if (Wire.available()) {
      uint8_t firstByte = Wire.read();
      uint8_t secondByte = Wire.read();

      return (firstByte << 8) + secondByte;
    }
    return -1;
  }

  inline int16_t readRawValue() {
	  return readRegister(ADS1115_CONVERSION_REG_ADDR);
  }
};

// Override Integrator class with persistent last value, gypsy_circum and A0 pin threshold configuration
class PersistentIntegrator : public sensesp::FloatTransform {
  public:
  PersistentIntegrator(const String& config_path = "")
     : sensesp::FloatTransform(config_path) {
      this->reset();
      this->load();
      this->emit(value);
  }

  inline unsigned int get_windlass_delay() const {
    return windlass_delay;
  } 

  inline unsigned int get_gypsy_delay() const {
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
  inline void reset() { 
    k = gypsy_circum;
    value = 0.0;
    windlass_threshold = ADS1115_WINDLASS_THRESHOLD;
    windlass_delay = CHAIN_COUNTER_DELAY_WINDLASS;
    gypsy_threshold = ADS1115_GYPSY_THRESHOLD;
    gypsy_delay = CHAIN_COUNTER_DELAY_GYPSY;
  }

  // Save the current values to JSON
  inline bool to_json(JsonObject& doc) {
    doc["k"] = k;
    doc["value"] = value;
    doc["windlass_threshold"] = windlass_threshold;
    doc["windlass_delay"] = windlass_delay;
    doc["gypsy_threshold"] = gypsy_threshold;
    doc["gypsy_delay"] = gypsy_delay;
    return true;
  }

  // Load the values from JSON
  inline bool from_json(const JsonObject& config) {
    if (!config["k"].is<float>()) {
        return false;
      }
    k = config["k"];
    value = (config["value"].is<float>() ? config["value"] : 0.0);    // May not have a value at the first load
    windlass_threshold = (config["windlass_threshold"].is<int>() ? config["windlass_threshold"] : ADS1115_WINDLASS_THRESHOLD);
    windlass_delay = (config["windlass_delay"].is<int>() ? config["windlass_delay"] : CHAIN_COUNTER_DELAY_WINDLASS);
    gypsy_threshold = (config["gypsy_threshold"].is<int>() ? config["gypsy_threshold"] : ADS1115_GYPSY_THRESHOLD);
    gypsy_delay = (config["gypsy_delay"].is<int>() ? config["gypsy_delay"] : CHAIN_COUNTER_DELAY_GYPSY);

    return true;
  }

 private:
  float k;                                                     // Multiplier for the integrator (gypsy circumference)
  float value = 0.0;                                           // Last value of the integrator (rode deployed)
  int windlass_threshold = ADS1115_WINDLASS_THRESHOLD;         // Threshold value to determine if windlass UP direction is active (A0 pin, default 26400)
  int gypsy_threshold = ADS1115_GYPSY_THRESHOLD;               // Threshold value to determine if gypsy DOWN direction is active (A2 pin, default 26400)
  unsigned int windlass_delay = CHAIN_COUNTER_DELAY_WINDLASS;  // Default 500ms
  unsigned int gypsy_delay = CHAIN_COUNTER_DELAY_GYPSY;        // Default 500ms
};

void setupWindlassSensor();
bool ConfigRequiresRestart(const PersistentIntegrator& obj);
const String ConfigSchema(const PersistentIntegrator& obj);

#endif // CHAIN_COUNTER_SENSOR_H