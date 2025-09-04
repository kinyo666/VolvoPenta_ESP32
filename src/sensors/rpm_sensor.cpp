/*
    RPM Sensor :
    - Reads the RPM from a PC817 optocoupler
    - Compute the fuel consumption based on the RPM
    - If INA3221_POWERDOWN is enabled, it will power down the INA3221 when the engine is off
  
  @author kinyo666
  @version 1.0.18
  @date 04/08/2025
  @link GitHub source code : https://github.com/kinyo666/VolvoPenta_ESP32
*/
#include "sensors/rpm_sensor.h"

// Simulate RPM for testing purposes
// This function simulates a pulse on the PC817 sensor to generate a constant RPM value
#ifdef SIMULATE_RPM
float simulateRPM() {
  digitalWrite(SIMULATE_RPM_PIN, HIGH);  // Simulate a pulse on the PC817 sensor
  delay(10);                             // Wait for 10ms = 50 Hz
  digitalWrite(SIMULATE_RPM_PIN, LOW);   // Stop the pulse
  delay(10);
  return 50.0; // Return a constant value for testing purposes
}
#endif

#ifndef FAKE_MODE
sensesp::DigitalInputCounter *sensor_engine[ENGINE_NB];                // 2 PC817 RPM Sensors values
#else
sensesp::FloatConstantSensor *sensor_engine[ENGINE_NB];                // Fake RPM Sensor values
#endif

const sensesp::ParamInfo* sleepModeINA3221_ParamInfo = new sensesp::ParamInfo[2]{{"engine_id", "Engine ID"}, {"read_delay", "Read delay"}};

// Callback for Engine State (Lambda Transform returns true if engine is running, false otherwise)
boolean runningState(float input) {
  return (input > 0);
}

// Setup the PC817 sensor for engine RPM
void setupPC817Sensor(u_int8_t engine_id, String engine, u_int8_t PC817_pin, ConfigSensESP* sensesp_config) {
  // Step 1 : calibrate the multiplier with known values (CurveInterpolator class) from analog gauge
  //const float multiplier = 1.0 / 97.0;
  const float multiplier = 1.0;
  const unsigned int read_delay_timer = sensesp_config->get_read_delay();             // Read delay in milliseconds from ConfigSensESP
  EngineDataTransform *engine_data = new EngineDataTransform(multiplier, conf_path_engines[engine_id], engine_id);
  EngineState *engine_running_state;
  EngineSleepMode *engine_sleep_mode;

  // Step 2 : instanciates DigitalInputCounter to read PC817 sensor values
  #ifndef FAKE_MODE
  const unsigned int read_rpm_delay = 500;
  sensor_engine[engine_id] = new sensesp::DigitalInputCounter(PC817_pin, INPUT, RISING, read_rpm_delay);   // INPUT for GPIO 32-35 or INPUT_PULLUP for others pins
  #else
  sensor_engine[engine_id] = new FloatConstantSensor(50.0, 1, "/config/engine/EL817/CONSTANT");   // Fake sensor for test purpose
  #endif
  #ifdef DEBUG_MODE
  sensor_engine[engine_id]->connect_to(new sensesp::SKOutputFloat(sk_path_engines[engine_id][REVOLUTIONS] + ".raw"));
  #endif

  // Step 3 : connects the output of sensor to the input of Frequency() and the Signal K output as a float in Hz
  sensor_engine[engine_id]
    ->connect_to(engine_data->freq)
    ->connect_to(engine_data->moving_avg)
    ->connect_to(new sensesp::SKOutputFloat(sk_path_engines[engine_id][REVOLUTIONS], 
                  new sensesp::SKMetadata("Hz", "Moteur " + engine, "Engine revolutions (x60 for RPM)", "RPM")));

  // Step 4 : transforms the output of Frequency to RPM and Liter per hour from FuelConsumption
  engine_data->freq
    ->connect_to(engine_data->hz_to_rpm)
    ->connect_to(engine_data->rpm_to_lhr)
    ->connect_to(engine_data->lhr_to_m3s)
    ->connect_to(new sensesp::SKOutputFloat(sk_path_engines[engine_id][FUELRATE],
                  new sensesp::SKMetadata("m3/s", "Conso " + engine, "Fuel rate of consumption", "L/hr")));

  // Step 5 : changes the engine state if needed
  engine_running_state = 
  engine_data->freq
    ->connect_to(engine_data->running_state);
  engine_running_state
    ->connect_to(new sensesp::SKOutputBool(sk_path_engines[engine_id][STATE]));

  // Step 6 : forces to power down the INA3221 if the engine is off
  if (sensesp_config->is_enabled("INA3221_POWERDOWN")) {
    engine_sleep_mode = new EngineSleepMode(sleepModeINA3221, engine_id, read_delay_timer, sleepModeINA3221_ParamInfo);
    engine_running_state
        ->connect_to(engine_sleep_mode)
        ->connect_to(new sensesp::SKOutputBool(sk_path_engines[engine_id][STATE] + ".powerdown",
                    new sensesp::SKMetadata("boolean", "Power Down " + engine, "Power down state of INA3221", "Power Down")));
  }
}

/* Measure Engines RPM

  multiplier_port and multiplier_starboard needs to be adjusted based on flywheel to Alternator W ratio.
  Example: 1 turn on an AQAD41 makes 2.45 turns on the Alternator, and the Alternator has 6 poles which makes a total of 2.45 x 6 = 14.7 Hz per turn.
  SignalK needs info in Hz so we need to divide the incoming value with 14.7, or as in our case multiply with (1/14.7) = 0,06803
  If Ratio is unknown, the original Tachometer might have a max Impulses/min written on it, divide that with max rpm on the meter and you'll get the ratio.
  Tachometer 860420 is marked with 73500Imp/min and has a scale to 5000rpm. 73500 divided with 5000 equals 14,7, Tada!
  If above multiplier needs to be recalculated, the FuelMultipliers needs to be recalculated as well, they're both based on AQAD41 values right now.
  AQAD41 example: 60 divided with FlyWheel To W Ratio (60 / 14,7 = 4,08)
*/
void setupRPMSensors(ConfigSensESP* sensesp_config) {
    if (sensesp_config->is_enabled("PC817_BABORD"))
      setupPC817Sensor(ENGINE_BABORD, "Babord", PC817_BABORD_PIN, sensesp_config);
    if (sensesp_config->is_enabled("PC817_TRIBORD"))
      setupPC817Sensor(ENGINE_TRIBORD, "Tribord", PC817_TRIBORD_PIN, sensesp_config);
    
    #ifdef SIMULATE_RPM
    pinMode(SIMULATE_RPM_PIN, OUTPUT); // Simulate RPM for testing purposes
    RepeatSensor<float> *sensor_simulate_rpm = new RepeatSensor<float>(100, simulateRPM);
    #endif
}