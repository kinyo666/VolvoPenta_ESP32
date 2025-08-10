/*
  @TODO :
  P1
  - Ajouter une moyenne de consommation de carburant réelle
  - Intégrer la lib MPU9250
  P2
  - Surcharger la classe SKMetaData pour envoyer les données de zones

  Volvo Penta digital dashboard :
  - Coolant temperature (°C, F)
  - Oil pressure (Bar, PSI)
  - Voltage (V)
  - Tachometer (RPM)
  - Fuel consumption (L/hr)
  - Windlass chain counter (Meter, persistent)
  - Fuel tank level (L)
  - Fresh water tank level (L)
  - Exhaust temperature (°C, F)
  - Engine room temperature (°C, F)
  - Motion sensor (Roll / Pitch / Yaw)

  @author kinyo666
  @version 1.0.20
  @date 10/08/2025
  @ref SensESP v3.1.0
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32

  Technical documentation :
  @link SensESP Documentation : https://signalk.org/SensESP/
  @link https://www.arduino.cc/reference/en/libraries/ina3221/
  @link https://signalk.org/specification/1.7.0/schemas/definitions.json

  Special thanks to :
  @link Matti Airas (SensESP framework) : https://github.com/SignalK/SensESP
  @link Mat Baileys (Boating with the Baileys) : https://github.com/Boatingwiththebaileys/ESP32-code
  @link Jason Greenwood (Après) : https://github.com/Techstyleuk
  */
#include <sensesp.h>
#include <sensesp_app_builder.h>
#include "customClasses.h"
#include "sensors/chain_counter_sensor.h"
#include "sensors/temperature_sensor.h"
#include "sensors/voltage_sensor.h"
#include "sensors/rpm_sensor.h"
#include "sensors/motion_sensor.h"
//#include "locals.h"

// Use Debug Mode for verbose logs and Fake Mode to simulate some data
#define DEBUG_MODE 1

// Local languages
#define LANG_EN 0
#define LANG_FR 1

// SensESP Configuration
ConfigSensESP *sensesp_config;                                // Sensors activation + read delay = 1s

// SensESP builds upon the ReactESP framework. Every ReactESP application must instantiate the "app" object.
reactesp::EventLoop app;

// The setup function performs one-time application initialization.
void setup() {
  #ifdef DEBUG_MODE
  sensesp::SetupLogging(ESP_LOG_DEBUG); //ESP_LOG_VERBOSE); 
  #else
  SetupLogging(ESP_LOG_INFO);
  #endif
  Serial.begin(115200);

  // Create the global SensESPApp() object.
  sensesp::SensESPAppBuilder builder;
  sensesp::sensesp_app = builder.set_hostname("birchwood-ESP32")->get_app();
  sensesp_config = new ConfigSensESP(conf_path_global);

  if (sensesp_config->is_enabled("DS18B20_FEATURE"))
    setupTemperatureSensors(sensesp_config);
  if (sensesp_config->is_enabled("INA3221_FEATURE"))
    setupVoltageSensors(sensesp_config);
  if (sensesp_config->is_enabled("PC817_FEATURE"))
    setupRPMSensors(sensesp_config);
  if (sensesp_config->is_enabled("MOTION_SENSOR_FEATURE"))
    setupMotionSensor(sensesp_config);
  if (sensesp_config->is_enabled("CHAIN_COUNTER_FEATURE"))
    setupWindlassSensor();
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop() { 
  static auto event_loop = sensesp::SensESPBaseApp::get_event_loop();
  event_loop->tick();
}