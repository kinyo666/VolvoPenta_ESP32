/* 
  Voltage Sensor :
    - Use INA3221 Voltage Sensors with 3 channels each for oil pressure, coolant temperature and voltage 12V
    - INA3221_BABORD_0 : Port engine (0x40 GND)
    - INA3221_CUVES_1 : Fuel and water tanks (0x41 VS)
    - INA3221_TRIBORD_2 : Starboard engine (0x42 SDA
    - INA3221_OTHERS_3 : Other sensors (0x43 SCL)
    - Engines INA3221 sensors are powered down after 60 seconds if the engine is off

  @author kinyo666
  @version 1.0.18
  @date 04/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#include "voltage_sensor.h"

// Voltage sensors
INA3221 *sensor_INA3221[INA3221_NB];                          // 4 INA3221 Voltage Sensors with 3 channels each
sensesp::RepeatSensor<float> *sensor_volt[INA3221_NB][INA3221_CH_NUM]; // 4 Voltage values for each channel
u_int8_t engine_state[ENGINE_NB] = { ENGINE_STATE_OFF, 
                                    ENGINE_STATE_OFF };       // Engine state for each engine (0 = OFF, 1 = ON, 2 = NOT RUNNING, 3 = RUNNING)
int engine_timer[ENGINE_NB] = { 0, 0 };                       // Timer to power-down the INA3221 sensors when the engine is off

/* Setup the INA3221 volt sensors
  The INA3221 must have an I²C address on A0 pin, solder the right pin GND (0x40) / SDA (0x41) / SCL (0x42) / VS (0x43)
  - On a Raspberry Pi, the I²C bus is required to be enabled
  - On an ESP32, default SDA pin = 21 & SCL pin = 22
  The Configuration Register is set to continious bus-voltage measurement with 4 values averaged (to avoid noise)
  @see https://www.ti.com/product/INA3221
  @param index Slot for INA3221 < INA3221_NB
  @param addr Physical A0 address of the INA3221 sensor
  @return boolean True if the sensor has been found, False otherwise
*/
boolean setupSensorINA3221(u_int8_t index, ina3221_addr_t addr = INA3221_ADDR40_GND) {
  sensor_INA3221[index] = new INA3221(addr);                        // Set I2C address to addr (default = 0x40 GND)
  sensor_INA3221[index]->begin(&Wire);                              // Default shunt resistors = 10 mOhm (R100)
  sensor_INA3221[index]->reset();                                   // Reset the INA3221 to default settings
  sensor_INA3221[index]->setShuntMeasDisable();                     // Disable shunt-voltage measurement
  sensor_INA3221[index]->setAveragingMode(INA3221_REG_CONF_AVG_4);  // Set averaging mode to 4 samples
  sensor_INA3221[index]->setModeContinious();                       // Set mode to continious measurement (110)
  
  delay(10);                                                        // Wait 10 ms for the next sensor setup
  #ifdef DEBUG_MODE
  Serial.printf("Setup Sensor INA3221 0x%X completed : ManufID = %i ; DieID = %i\n", addr, sensor_INA3221[index]->getManufID(), sensor_INA3221[index]->getDieID());
  #endif
  return (sensor_INA3221[index]->getDieID() != 0);  // True if the sensor has been found
}

// Callbacks for Sensor Volt
float getVoltageINA3221_BABORD0_CH1() { return sensor_INA3221[INA3221_BABORD_0]->getVoltage(INA3221_CH1); }    // Coolant temperature
float getVoltageINA3221_BABORD0_CH2() { return sensor_INA3221[INA3221_BABORD_0]->getVoltage(INA3221_CH2); }    // Oil pressure
float getVoltageINA3221_BABORD0_CH3() {                                                                        // Voltage
  float volt = sensor_INA3221[INA3221_BABORD_0]->getVoltage(INA3221_CH3);
  if (volt < 1)
    engine_state[ENGINE_BABORD] = ENGINE_STATE_OFF;    // Engine state is OFF and not running
  else
    engine_state[ENGINE_BABORD] |= ENGINE_STATE_ON;    // Engine state is ON (and maybe running or not)
  return volt;
}
float getVoltageINA3221_CUVES1_CH1() { return sensor_INA3221[INA3221_CUVES_1]->getVoltage(INA3221_CH1); }      // Fuel tank volume
float getVoltageINA3221_CUVES1_CH2() { return sensor_INA3221[INA3221_CUVES_1]->getVoltage(INA3221_CH2); }      // Fuel tank volume
float getVoltageINA3221_CUVES1_CH3() { return sensor_INA3221[INA3221_CUVES_1]->getVoltage(INA3221_CH3); }      // Water tank volume
float getVoltageINA3221_TRIBORD2_CH1() { return sensor_INA3221[INA3221_TRIBORD_2]->getVoltage(INA3221_CH1); }  // Coolant temperature
float getVoltageINA3221_TRIBORD2_CH2() { return sensor_INA3221[INA3221_TRIBORD_2]->getVoltage(INA3221_CH2); }  // Oil pressure
float getVoltageINA3221_TRIBORD2_CH3() {                                                                       // Voltage
  float volt = sensor_INA3221[INA3221_TRIBORD_2]->getVoltage(INA3221_CH3);
  if (volt < 1)
    engine_state[ENGINE_TRIBORD] = ENGINE_STATE_OFF;    // Engine state is OFF and not running
  else
    engine_state[ENGINE_TRIBORD] |= ENGINE_STATE_ON;    // Engine state is ON (and maybe running or not)
  return volt;
}
float getVoltageINA3221_OTHERS3_CH3() { return sensor_INA3221[INA3221_OTHERS_3]->getVoltage(INA3221_CH3); }    // Radius Angle

/* Callback for setting INA3221 device in active or power-down mode when engine is off
   - if RPM value equals to 0, it means the engine is not running
   - if Voltage value equals to 0, it means the engine is not turned on
  The INA3221 device goes into power-down mode when the engine is not runnning AND is turned off after ENGINE_SLEEP_TIMER period
  It returns to continious mode when the engine is on (but not necessarilly running)
  State machine :
    TIMER--
    ┌─┐
    OFF ────> ON ────> RUNNING
     ↑        |↑          ↓
     └────────┘└───── NOT RUNNING
  [POWERDOWN]   [ACTIVE]

    @param engine_id Id of the engine (ENGINE_BABORD or ENGINE_TRIBORD)
    @param powerdown_mode Power-down mode to set (ENGINE_STATE_RUNNING or ENGINE_STATE_NOT_RUNNING)
    @param read_delay Read delay in milliseconds from ConfigSensESP
*/
bool sleepModeINA3221(bool running, u_int8_t engine_id, unsigned int read_delay) {
  u_int8_t INA3221_id = ((engine_id == ENGINE_BABORD) ? INA3221_BABORD_0 : INA3221_TRIBORD_2);
  u_int8_t powerdown_mode = ((running == true) ? ENGINE_STATE_RUNNING : ENGINE_STATE_NOT_RUNNING);

  if (engine_state[engine_id] == ENGINE_STATE_OFF) {          // Engine is off
    if (powerdown_mode == ENGINE_STATE_RUNNING) {
      engine_state[engine_id] |= powerdown_mode;
      engine_timer[engine_id] = ENGINE_SLEEP_TIMER * 1000 / read_delay; // Reset the timer
      if (sensor_INA3221[INA3221_id] != NULL)                 // Check if the sensor is initialized
        sensor_INA3221[INA3221_id]->setModeContinious();      // INA3221 is now active
      #ifdef DEBUG_MODE
      Serial.printf("ENGINE : id = %i | state = %#05x -> WAKE UP\n", engine_id, engine_state[engine_id]);
      #endif
    }
    else if (engine_timer[engine_id] == 0) {                  // INA3221 is going to power-down
        engine_timer[engine_id] = -1;                         // Disable the timer
        if (sensor_INA3221[INA3221_id] != NULL)               // Check if the sensor is initialized
          sensor_INA3221[INA3221_id]->setModePowerDown();     // INA3221 is now in power-down mode
        #ifdef DEBUG_MODE
        Serial.printf("ENGINE : id = %i | state = %#05x -> POWER DOWN\n", engine_id, engine_state[engine_id]);
        #endif
    }
    else if (engine_timer[engine_id] > 0)                     // Wait for ENGINE_SLEEP_TIMER before going to power-down
      engine_timer[engine_id]--;                              // Decrease the timer
  }
  else if (powerdown_mode == ENGINE_STATE_NOT_RUNNING) {
      engine_state[engine_id] &= powerdown_mode;              // Engine is on but not running
      engine_timer[engine_id] = ENGINE_SLEEP_TIMER * 1000 / read_delay; // Reset the timer
  }
  #ifdef DEBUG_MODE
  Serial.printf("ENGINE : id = %i | timer = %i\t| ", engine_id, engine_timer[engine_id]); 
  Serial.printf("state = %#05x -> %s - %s\n", engine_state[engine_id],
                        ((engine_state[engine_id] & ENGINE_STATE_ON) ? "ON" : "OFF"),
                        ((engine_state[engine_id] & ENGINE_STATE_IS_RUNNING) ? "RUNNING" : "NOT RUNNING"));
  #endif

  return (engine_state[engine_id] & ENGINE_STATE_ON);
}

// Setup the INA3221 Voltage sensor for engine gauges
// @param INA3221_Id Id / position in the table sensor_volt
// @param engine Engine name ("Port" or "Starboad") as it will appear under KIP Plugin
void setupVoltageEngineSensors(u_int8_t INA3221_Id, String engine) {
  LinearPos *sensor_volt_linearpos_volt2k = new LinearPos(linearPositive, 1.0, 0.0, linearPositive_ParamInfo,
                                                      conf_path_volt[INA3221_Id][INA3221_CH1] + "/LINEAR_POSITIVE");
  LinearPos *sensor_volt_linearpos_volt2pa = new LinearPos(linearPositive, pow(10, 5) * 1.4793, -0.776, linearPositive_ParamInfo,
                                                       conf_path_volt[INA3221_Id][INA3221_CH2] + "/LINEAR_POSITIVE");

  // Coolant temperature
  sensor_volt[INA3221_Id][INA3221_CH1]
    ->connect_to(sensor_volt_linearpos_volt2k)    
    ->connect_to(new sensesp::SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH1],
                    new sensesp::SKMetadata("K", "T° Eau " + engine, "Coolant temperature", "Temp " + engine)));

  ConfigItem(sensor_volt_linearpos_volt2k)
    ->set_title("Température Eau - " + engine)
    ->set_description("T° Eau - INA3221_CH1 - LinearPositive")
    ->set_sort_order(UI_ORDER_ENGINE+2);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_Id][INA3221_CH1]
    ->connect_to(new sensesp::SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH1] + ".raw"));
  #endif

  // Oil pressure
  sensor_volt[INA3221_Id][INA3221_CH2]
    ->connect_to(sensor_volt_linearpos_volt2pa)    
    ->connect_to(new sensesp::SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH2],
                    new sensesp::SKMetadata("Pa", "Pression Huile " + engine, "Oil pressure", "Huile " + engine)));

  ConfigItem(sensor_volt_linearpos_volt2pa)
    ->set_title("Pression Huile - " + engine)
    ->set_description("Pression Huile - INA3221_CH2 - LinearPositive")
    ->set_sort_order(UI_ORDER_ENGINE+5);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_Id][INA3221_CH2]
    ->connect_to(new sensesp::SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH2] + ".raw"));
  #endif

  // Voltage for Volts gauge
  sensor_volt[INA3221_Id][INA3221_CH3]
    ->connect_to(new sensesp::SKOutputFloat(sk_path_volt[INA3221_Id][INA3221_CH3],
                    new sensesp::SKMetadata("V", "Volt " + engine, "Alternator voltage", "Volt " + engine)));
}

// Setup the INA3221 Voltage sensors for fuel / water tanks
// @param INA3221_channel Channel number (0-2) for the INA3221 sensor
// @param location Tank location (Portside, Starboard, etc.)
// @param type Tank type (Fuel, Water, etc.)
// @param order Tank order (0-2) for the UI sort order
void setupVoltageTankSensor(u_int8_t INA3221_channel, String location, String type, u_int8_t order) {
  LinearPos *sensor_volt_linearpos = new LinearPos(linearPositive, 1.0, 0.0, linearPositive_ParamInfo,
                                                  conf_path_volt[INA3221_CUVES_1][INA3221_channel] + "/LINEAR_POSITIVE");

  sensor_volt[INA3221_CUVES_1][INA3221_channel]
    ->connect_to(sensor_volt_linearpos)
    ->connect_to(new sensesp::MovingAverage(4, 1.0, conf_path_volt[INA3221_CUVES_1][INA3221_channel] + "/MOVING_AVERAGE"))
    ->connect_to(new sensesp::SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_channel],
                  new sensesp::SKMetadata("m3", "Cuve " + location, "Niveau " + type + " " + location, "Cuve " + location)));

  ConfigItem(sensor_volt_linearpos)
    ->set_title("Cuve " + type + " " + location)
    ->set_description("Cuve " + type + " " + location + " - INA3221_CH" + String(INA3221_channel) + " - LinearPositive")
    ->set_sort_order(UI_ORDER_TANK+order+1);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_CUVES_1][INA3221_channel]
    ->connect_to(new sensesp::SKOutputFloat(sk_path_volt[INA3221_CUVES_1][INA3221_channel] + ".raw"));
  #endif
}

// Setup the INA3221 Voltage sensor for rudder angle
void setupVoltageRudderAngleSensor() {
  LinearPos *sensor_volt_linearpos = new LinearPos(linearPositive, 1.0, 0.0, linearPositive_ParamInfo,
                                                  conf_path_rudder + "/LINEAR_POSITIVE");
  sensor_volt[INA3221_OTHERS_3][INA3221_CH3]
    ->connect_to(sensor_volt_linearpos)
    ->connect_to(new sensesp::SKOutputFloat(sk_path_rudder,
                  new sensesp::SKMetadata("m", "Angle de barre", "Angle de barre", "Angle de barre")));
  ConfigItem(sensor_volt_linearpos)
    ->set_title("Angle de barre")
    ->set_description("Angle de barre - INA3221_CH3 - LinearPositive")
    ->set_sort_order(UI_ORDER_TANK+6);
  #ifdef DEBUG_MODE
  sensor_volt[INA3221_OTHERS_3][INA3221_CH3]
    ->connect_to(new sensesp::SKOutputFloat(sk_path_rudder + ".raw"));
  #endif
}

// Measure instruments voltage
void setupVoltageSensors(ConfigSensESP* sensesp_config) {
  /*
    The INA219 or INA3221 needs to be connected to the same ground reference as the voltage source being measured.
    This is crucial for maintaining a common reference point and avoiding measurement errors
    - Step 1 : measure the voltage at start up (engine is off), startVoltage = 0.728
    - Step 2 : measure the voltage and pressure when engine is running (engine is idling), runningVoltage = 3.836V & runningBar = 5.0 bars
    - Step 3 : calculate the offset & multiplier ((voltage - startVoltage) * runningBar) / (runningVoltage - startVoltage)
                offset = -startVoltage = -0.728
                multiplier = runningBar / (runningVoltage - startVoltage) = 5.0 / (4.152 - 0.728) = 1.46
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

  // Enable each INA3221 according to the SensESP configuration
  boolean sensorFound[INA3221_NB] = {false, false, false, false};
  unsigned int read_delay = sensesp_config->get_read_delay();

  if (sensesp_config->is_enabled("INA3221_BABORD_0"))
    sensorFound[INA3221_BABORD_0] = setupSensorINA3221(INA3221_BABORD_0, INA3221_ADDR40_GND);
  if (sensesp_config->is_enabled("INA3221_CUVES_1"))
    sensorFound[INA3221_CUVES_1] = setupSensorINA3221(INA3221_CUVES_1, INA3221_ADDR41_VCC);
  if (sensesp_config->is_enabled("INA3221_TRIBORD_2"))
    sensorFound[INA3221_TRIBORD_2] = setupSensorINA3221(INA3221_TRIBORD_2, INA3221_ADDR42_SDA);
  if (sensesp_config->is_enabled("INA3221_OTHERS_3"))
    sensorFound[INA3221_OTHERS_3] = setupSensorINA3221(INA3221_OTHERS_3, INA3221_ADDR43_SCL);

  // Setup port voltage sensors
  if (sensorFound[INA3221_BABORD_0]) {
    sensor_volt[INA3221_BABORD_0][INA3221_CH1] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH1);
    sensor_volt[INA3221_BABORD_0][INA3221_CH2] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH2);
    sensor_volt[INA3221_BABORD_0][INA3221_CH3] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_BABORD0_CH3);
    if (sensesp_config->is_enabled("INA3221_POWERDOWN"))
      engine_timer[ENGINE_BABORD] = ENGINE_SLEEP_TIMER * 1000 / read_delay;
    setupVoltageEngineSensors(INA3221_BABORD_0, "Babord");
  }

  // Setup tank voltage sensors
  if (sensorFound[INA3221_CUVES_1]) {
    sensor_volt[INA3221_CUVES_1][INA3221_CH1] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH1);
    sensor_volt[INA3221_CUVES_1][INA3221_CH2] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH2);
    sensor_volt[INA3221_CUVES_1][INA3221_CH3] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_CUVES1_CH3);
    setupVoltageTankSensor(INA3221_CH1, "Babord", "Carburant", 0);    // Fuel tank volume on portside
    setupVoltageTankSensor(INA3221_CH2, "Tribord", "Carburant", 2);   // Fuel tank volume on starboard
    setupVoltageTankSensor(INA3221_CH3, "Eau", "Inox", 4);            // Water tank volume
  }

  // Setup starboard voltage sensors
  if (sensorFound[INA3221_TRIBORD_2]) {
    sensor_volt[INA3221_TRIBORD_2][INA3221_CH1] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH1);
    sensor_volt[INA3221_TRIBORD_2][INA3221_CH2] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH2);
    sensor_volt[INA3221_TRIBORD_2][INA3221_CH3] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_TRIBORD2_CH3);
    if (sensesp_config->is_enabled("INA3221_POWERDOWN"))
        engine_timer[ENGINE_TRIBORD] = ENGINE_SLEEP_TIMER * 1000 / read_delay;
    setupVoltageEngineSensors(INA3221_TRIBORD_2, "Tribord");
  }

  // Setup others voltage sensors like rudder angle sensor
  if (sensorFound[INA3221_OTHERS_3]) { 
    if (sensesp_config->is_enabled("RUDDER_ANGLE_FEATURE")) {
      sensor_volt[INA3221_OTHERS_3][INA3221_CH3] = new sensesp::RepeatSensor<float>(read_delay, getVoltageINA3221_OTHERS3_CH3);
      setupVoltageRudderAngleSensor();
    }
  }
}