/*
  Chain counter sensor for windlass :
  - Count how many meters of chain are deployed or retrieved
  - Use a gypsy reed sensor to count the number of revolutions
  - Use a Persistent Integrator to accumulate the number of revolutions
  - Use a HSTS016L current sensor to determine the direction of the windlass
  - Use an ADS1115 sensor to read the gypsy reed sensor and the current sensor
  - The ADS1115 is optional and can be replaced with internal ESP32's ADC1

  @author kinyo666
  @version 1.0.23
  @date 15/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#include "chain_counter_sensor.h"

// Windlass sensor
ADS1115Sensor *sensor_ADS1115;                                // ADS1115 sensor for windlass motor
sensesp::FloatTransform *sensor_windlass_debounce;            // Windlass sensor (Debounce)
PersistentIntegrator *chain_counter;                          // Windlass chain counter
JsonDocument jdoc_conf_windlass;
JsonObject conf_windlass;                                     // Windlass direction of rotation (UP or DOWN) and last value if exists
unsigned int chain_counter_timer = 0;                         // Timer to save chain counter value
boolean chain_counter_saved = true;                           // Trigger to save chain counter new value
enum { WINDLASS_IDLE = 0, WINDLASS_UP = 1, WINDLASS_DOWN = -1 };
int windlass_state = WINDLASS_IDLE;

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

// Config scheme for the UI
const String ConfigSchema(const PersistentIntegrator& obj) {
return R"({
  "type": "object",
  "properties": {
      "k": { "title": "Gypsy circumference", "type": "number" },
      "value": { "title": "Rode deployed", "type": "number" },
      "windlass_threshold": { "title": "Windlass threshold", "type": "number" },
      "windlass_delay": { "title": "Windlass delay", "type": "number", "default": 500, 
          "description": "Delay in milliseconds for windlass current sensor" },
      "gypsy_threshold" : { "title": "Gypsy threshold", "type": "number" },
      "gypsy_delay": { "title": "Gypsy delay", "type": "number", "default": 500, 
          "description": "Delay in milliseconds for gypsy sensor. Must be higher than Debounce delay." }
  }
})";
}

// If you change the configuration, the ESP32 must be restarted
bool ConfigRequiresRestart(const PersistentIntegrator& obj) {
  return true;
}

// Convert raw values from ADS1115 to volts (V)
float convertValueToVolts(int16_t rawValue, uint8_t currentPGA) {
	float multiplier;     // Voltage resolution mV/bit

	switch (currentPGA) {
		case ADS1115_PGA_0_256:
			multiplier = ADS1115_PGA_0_256_RES;
			break;
		case ADS1115_PGA_0_512:
			multiplier = ADS1115_PGA_0_512_RES;
			break;
		case ADS1115_PGA_1_024:
			multiplier = ADS1115_PGA_1_024_RES;
			break;
		case ADS1115_PGA_2_048:
			multiplier = ADS1115_PGA_2_048_RES;
			break;
		case ADS1115_PGA_4_096:
			multiplier = ADS1115_PGA_4_096_RES;
			break;
		case ADS1115_PGA_6_144:
			multiplier = ADS1115_PGA_6_144_RES;
			break;
		default:
			multiplier = 1.0;
			break;
	}

	return (multiplier * rawValue) / 1000;  // Returns voltage in volts
}

// Convert voltage (V) to current (A)
float convertVoltsToAmperes(float voltage) {
    // Voltage is relative to Vref
    float vDiff = (voltage > 0) ? voltage - HSTS016L_VREF : 0.0f; // If Vout - Vref is negative, amperes = 0.0
    // Convert to amperes using sensitivity
    return vDiff / HSTS016L_SENSITIVITY;
}

// Read the value from the ADS1115 sensor (A0-A1 or A2 input pin)
int16_t readValue(uint8_t input_mux) {
  uint8_t input_pga; // = (input_mux == ADS1115_MUX_AIN2_GND) ? ADS1115_PGA_4_096 : ADS1115_PGA_2_048; // Set PGA depending on the input pin
  switch (input_mux) {
    case ADS1115_MUX_AIN0_AIN1 :
      input_pga = ADS1115_PGA_1_024;    // Use PGA 1.024V for A0-A1 differential --> ±0.625V
      break;
    case ADS1115_MUX_AIN2_GND :
      input_pga = ADS1115_PGA_4_096;    // Use PGA 4.096V for A2 reed switch --> +3.3V (VCC) or +0.0V (GND)
      break;      
    case ADS1115_MUX_AIN0_GND :
    case ADS1115_MUX_AIN1_GND :
    default :
      input_pga = ADS1115_PGA_2_048;    // Use PGA 2.048V for A1 (Vref) and A0 (Vout) single-ended inputs --> ±1.65V
      break;
  }

	sensor_ADS1115->setMultiplexer(input_mux);
  sensor_ADS1115->setPga(input_pga);  
	sensor_ADS1115->startSingleConvertion();
	delayMicroseconds(25); // The ADS1115 needs to wake up from sleep mode and usually it takes 25 uS to do that

	while (sensor_ADS1115->getOperationalStatus() == 0);

	return sensor_ADS1115->readRawValue(); // Read the raw value from the ADS1115 sensor
}

/* Callback for Windlass UP/DOWN buttons sensor
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

   NOTE(1) : If the read_delay_windlass value is set to more or less than one second, the number of cycles before saving SHOULD be modified
   NOTE(2) : A cycle is defined as one revolution of the gypsy, so the read_delay_windlass MUST have a lower value than the revolution time
   NOTE(3) : DOWN = chain_counter + gypsy_circum (addition) ; UP = chain_counter - gypsy_circum (substraction)

   @returns float - Differential value of current measured on channel ADS1115_A0 and ADS1115_A1
*/
float readValueADS1115_A0_A1() { 
  float direction = conf_windlass["k"].as<float>();
  int16_t windlass_threshold = conf_windlass["windlass_threshold"].as<int16_t>();
  int16_t up = readValue(ADS1115_MUX_AIN0_AIN1);
  float up_volt = convertValueToVolts(up, ADS1115_PGA_1_024);
  float up_amp = convertVoltsToAmperes(up_volt); // Convert voltage to current (A) using the gain of the current sensor

  #ifdef DEBUG_MODE
  int16_t a0 = readValue(ADS1115_MUX_AIN0_GND); // Read the A0 pin (windlass current)
  int16_t a1 = readValue(ADS1115_MUX_AIN1_GND); // Read the A1 pin (gypsy current)
  #endif

  if (up > windlass_threshold) {                                                                     // Windlass IDLE/DOWN -> UP
    windlass_state = WINDLASS_UP;                                                           // Set the windlass state to UP    
    if (direction > 0) {                                                                    // If the windlass direction was DOWN
      conf_windlass["k"] = -1.0 * direction;                                                // Reverse direction
      chain_counter->set_direction(conf_windlass["k"]);                                     // Set the new config direction
    }
    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : UP -> up = %d (%.3fV %.3fA) \t| direction = %f\n", up, up_volt, up_amp, conf_windlass["k"].as<float>());
    #endif
  }
  else if ((windlass_state == WINDLASS_DOWN) && (up <= windlass_threshold)) {                         // Windlass IDLE/UP -> DOWN
    if (direction < 0) {                                                                     // If the windlass direction was UP
      conf_windlass["k"] = abs(direction);                                                   // Reverse direction
      chain_counter->set_direction(conf_windlass["k"]);                                      // Set the new config direction
    }
    chain_counter_timer++;
    if (chain_counter_timer >= CHAIN_COUNTER_UI_CYCLE)                                       // Wait N cycles before resetting the windlass state on the UI
      windlass_state = WINDLASS_IDLE;                                                        // Return to the windlass state IDLE

    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : DOWN -> up = %d \t| direction = %f | timer = %i | windlass_state = %s\n", up, 
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
    Serial.printf("WINDLASS : IDLE -> up = %d (%.3fV %.3fA) \t| direction = %f | timer = %i | saved = %s\n", up,
                  up_volt, up_amp, conf_windlass["k"].as<float>(), chain_counter_timer, chain_counter_saved ? "true" : "false");
    #endif
  }

  #ifdef DEBUG_MODE
  float a1_volt = convertValueToVolts(a1, ADS1115_PGA_2_048);
  float a0_volt = convertValueToVolts(a0, ADS1115_PGA_2_048);
  Serial.printf("WINDLASS : A0-A1 = %d (%.3fV %.3fA)\t A0 = %d (%.3fV %.3fA)\t A1 = %d (%.3fV %.3fA)\n",
                                up, up_volt, up_amp,
                                a0, a0_volt, convertVoltsToAmperes(a0_volt),
                                a1, a1_volt, convertVoltsToAmperes(a1_volt));
  #endif
  return ((float) up);
}

// Callback for the gypsy's reed sensor
// Use a 10 kΩ pull-down resistor on the ADS1115 A2 input pin to read the reed sensor (+3.3V = closed, 0 = open)
float readValueADS1115_A2() {
  int16_t gypsy = readValue(ADS1115_MUX_AIN2_GND);
  #ifdef DEBUG_MODE
  Serial.printf("WINDLASS : A2 = %d (%.3fV)\n", gypsy, convertValueToVolts(gypsy, ADS1115_PGA_4_096));
  #endif

  // If the reed sensor state is open and PGA is set to 4.096, we ignore the values above the threshold
  if (gypsy >= conf_windlass["gypsy_threshold"].as<int16_t>())
    return (float) gypsy;

  return 0.0;
}

// Callback for chain counter
// This function is called after each gypsy sensor value changes (after debounce)
void handleChainCounterChange() {
  float value = sensor_windlass_debounce->get();  // Get the current value from the debounced sensor
  if (value >= conf_windlass["gypsy_threshold"].as<int16_t>()) {
    chain_counter_saved = false;                  // Set the last value status to 'not saved'
    chain_counter_timer = 0;                      // Arm a timer to save chain counter last value after CHAIN_COUNTER_SAVE_TIMER cycles
    if (windlass_state == WINDLASS_IDLE) 
      windlass_state = WINDLASS_DOWN;             // Set the windlass state to DOWN if it was IDLE (gypsy freewheel) 

    #ifdef DEBUG_MODE
    Serial.printf("WINDLASS : NEW VALUE TO SAVE = %f | windlass_state = %s\n", value, 
                  (windlass_state == WINDLASS_UP) ? "UP" : ((windlass_state == WINDLASS_DOWN) ? "DOWN" : "IDLE"));
    #endif
  }
}

// Check if the ADS1115 sensor is present on the I²C bus
bool isADS1115Present(uint8_t i2c_addr) {
  Wire.beginTransmission(i2c_addr);
  uint8_t error = Wire.endTransmission();
  return (error == 0); // 0 = sensor is present
}

/* Setup the ADS1115 sensor for windlass chain counter
   The ADS1115 returns a value between 0 and 32767. We set dynamic PGA from 1.024V to 4.096V so when the ADS1115 reads a value :
   - on the A2 pin (+3.3V), it returns a deterministic max value equals to 0 or 26400 if the reed sensor is closed (gypsy signal)
   - on the A0 pin (±1.65V), it returns a value > 26400 if the HSTS016L Hall effect sensor measures a current (UP signal) 
                                or <= 26400 if the windlass is not running in UP direction
   If you want to use a higher PGA (i.e 4,096V), you MUST :
   - change the ADS1115_WINDLASS_THRESHOLD value
   - use a LambdaTransform to ignore the range of nominal values (+3.3V) before the DebounceInt sensor

   ADS1115 Formula : pin_value = pin_volt / PGA_volt * 32768
   HSTS016L Formula : volt_HSTS = 1.65V +（(I / Ipn）* 0.625V) (I = current measured and Ipn = 200A, depending on your HSTS016L model)
   HSTS_voltage = pin_value * ADS1115_PGA_X_XXX_RES
   HSTS_current = HSTS_voltage / HSTS_sensitivity

   Example 1 : I = 50A (HSTS016L current)
    Vout = 1.65V + ((50 / 200) * 0.625V) = 1.65V + 0.15625V = 1.80625V
    A0_raw_value = (1.80625 / 2.048) * 32768 = 28900

   Example 2 : I = 0A (HSTS016L current)
    Vout = 1.65V + (0 / 200) * 0.625V = 1.65V + 0.0V = 1.65V
    A0_raw_value = (1.65 / 2.048) * 32768 = 26400

  Now 0.625V (625mV) change from 1.65V is equivalent to 200A, implying a scale of 625 (mV) / 200(A) = 3.125 mV/A

  NOTE : Both RepeatSensor have configurable delays from PersistentIntegrator object

  @returns bool - true if the ADS1115 sensor is initialized, false if not
  @see https://forum.dronebotworkshop.com/electronic-components/help-with-high-current-hall-effect-sensor-hsts016l/
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
    Serial.println("ADS1115 : Sensor NOT FOUND at address " + String(ADS1115_WINDLASS_ADDR, HEX));
    #endif
    return false; // Exit if the sensor is not present
  }
  else {
    // Initialize the ADS1115 sensor 
    sensor_ADS1115 = new ADS1115Sensor(ADS1115_WINDLASS_ADDR);

    #ifdef DEBUG_MODE
    Serial.println("ADS1115 : Sensor found at address " + String(ADS1115_WINDLASS_ADDR, HEX));
    #endif
    return true;
  }
}

// Windlass chain counter with ADS1115 and HSTS016L sensors
void setupWindlassSensor() {
  if (!setupADS1115Sensor()) {
    #ifdef DEBUG_MODE
    Serial.println("ADS1115 : Failed to initialize sensor. Windlass sensor setup aborted.");
    #endif
    return; // Exit if the ADS1115 sensor is not initialized
  }
  // Initialize the persistent chain counter
  conf_windlass = jdoc_conf_windlass.to<JsonObject>();                                               // Store Windlass configuration
  conf_windlass["k"] = gypsy_circum;                                                                 // Default direction = DOWN
  conf_windlass["windlass_threshold"] = ADS1115_WINDLASS_THRESHOLD;                                  // Default threshold = 26400
  conf_windlass["gypsy_threshold"] = ADS1115_GYPSY_THRESHOLD;                                        // Default threshold = 26400
  chain_counter = new PersistentIntegrator(conf_path_chain[CHAIN_COUNTER_PATH]);                     // Chain counter in meter
  chain_counter->to_json(conf_windlass);                                                             // Retrieve last saved value if exists

  // Initialize the windlass sensor
  sensesp::RepeatSensor<float> *sensor_windlass_A0 = new sensesp::RepeatSensor<float>(chain_counter->get_windlass_delay(),
                                                                 readValueADS1115_A0_A1);            // Read the A0-A1 value (windlass) from the ADS1115 sensor
  delay(50);                                                                                         // Wait 50 ms to avoid RepeatSensor synchronisation issues

  // Initialize the gypsy sensor
  sensesp::RepeatSensor<float> *sensor_gypsy_A2 = new sensesp::RepeatSensor<float>(chain_counter->get_gypsy_delay(), 
                                                                    readValueADS1115_A2);            // Read the A2 value (gypsy) with a delay
  sensesp::DebounceFloat *chain_debounce = new sensesp::DebounceFloat(CHAIN_COUNTER_DELAY_DEBOUNCE, 
                                                          conf_path_chain[CHAIN_COUNTER_PATH_DEBOUNCE]);
  sensor_windlass_debounce = sensor_gypsy_A2->connect_to(chain_debounce);                            // Avoid multiples counts
  auto *sensor_windlass_counter = sensor_windlass_debounce->connect_to(chain_counter);               // Add +/- gypsy_circum to the counter
  sensor_windlass_counter->attach(handleChainCounterChange);                                         // Set a callback for each value read
  sensor_windlass_counter
    ->connect_to(new sensesp::SKOutputFloat(sk_path_windlass, 
                new sensesp::SKMetadata("m", "Compteur Chaine", "Chain Counter", "Mètre")));         // Output the last float value to SignalK

  sensesp::LambdaTransform<float, String> *sensor_windlass_string = new sensesp::LambdaTransform<float, String>(chainCounterToString);
  sensor_windlass_counter
    ->connect_to(sensor_windlass_string)
    ->connect_to(new sensesp::SKOutputString(sk_path_windlass + ".direction"));                      // Output the last value + direction to SignalK

  /* Set SensESP Configuration UI for chain_counter and chain_debounce
     If you want something to appear in the web UI, you first define overloaded ConfigSchema and ConfigRequiresRestart functions for that class.
     Then, you call ConfigItem to actually instantiate the ConfigItemT object
  */
  ConfigItem(chain_counter)
    ->set_title("Compteur Chaine")
    ->set_description("Compteur Chaine - PersistentIntegrator")
    ->set_requires_restart(true)
    ->set_sort_order(UI_ORDER_CHAIN);

  ConfigItem(chain_debounce)
    ->set_title("Compteur Chaine Debounce")
    ->set_description("Compteur Chaine - DebounceInt")
    ->set_requires_restart(true)
    ->set_sort_order(UI_ORDER_CHAIN+1);

  #ifdef DEBUG_MODE
  sensor_windlass_debounce->connect_to(new sensesp::SKOutputInt(sk_path_windlass + ".debounce.raw"));
  sensor_windlass_A0->connect_to(new sensesp::SKOutputFloat(sk_path_windlass + ".up.raw"));
  #endif
}