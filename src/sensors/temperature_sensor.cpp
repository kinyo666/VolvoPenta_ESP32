/*
  Temperature sensors :
  - Use 3 DS18B20 sensors to measure the temperature of the engine room and exhausts
  - DS18B20_BABORD_0 : Port engine exhaust temperature
  - DS18B20_TRIBORD_1 : Starboard engine exhaust temperature
  - DS18B20_COMMON_2 : Engine room temperature
  - Temperature values are converted to Kelvin and filtered to avoid out of range values

  @author kinyo666
  @version 1.0.18
  @date 04/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#include "temperature_sensor.h"

// Filter Kelvin values out of range
auto filterKelvinValues = [](float input, float offset = 273.15, float max = 413.1) -> float {
    return ((((input - offset) > 0) && (input < max)) ? input : offset);
};
const sensesp::ParamInfo* filterKelvinValues_ParamInfo = new sensesp::ParamInfo[2]{{"offset", "Offset"}, {"max", "Max value"}};

// Returns the temperature sensor location by its 1-Wire address
int getTemperatureSensorLocation(String sensor_addr) {
  if (sensor_addr == DS18B20_BABORD_ADDR)
    return DS18B20_BABORD_0;
  else if (sensor_addr == DS18B20_TRIBORD_ADDR)
    return DS18B20_TRIBORD_1;
  else if (sensor_addr == DS18B20_COMMON_ADDR)
    return DS18B20_COMMON_2;
  else
    return -1;
}

// Setup engine temperature sensors
void setupTemperatureSensors(ConfigSensESP* sensesp_config) {
  sensesp::onewire::OneWireTemperature* sensor_temp[DS18B20_NB];                  // 3 DS18B20 Temperature values
  sensesp::onewire::DallasTemperatureSensors* dts = new sensesp::onewire::DallasTemperatureSensors(DS18B20_PIN);
  JsonDocument jsondoc;
  JsonObject jsonsensor = jsondoc.add<JsonObject>();
  unsigned int read_delay = sensesp_config->get_read_delay();
  
  for (u_int8_t i = 0; i < DS18B20_NB; i++) {
    sensor_temp[i] = new sensesp::onewire::OneWireTemperature(dts, read_delay);
    sensor_temp[i]->to_json(jsonsensor);

    if (jsonsensor["found"].is<bool>() && (jsonsensor["found"] == true))
      switch (getTemperatureSensorLocation(jsonsensor["address"])) {
        case DS18B20_BABORD_0 : {
          if (sensesp_config->is_enabled("DS18B20_BABORD_0"))
            sensor_temp[DS18B20_BABORD_0]
              ->connect_to(new sensesp::SKOutputFloat(sk_path_temp[DS18B20_BABORD_0], "",
                          new sensesp::SKMetadata("K", "Echappt Babord", sk_path_temp[DS18B20_BABORD_0], "T° Echappement Moteur Babord")));
          break;
        }
        case DS18B20_TRIBORD_1 : {
          if (sensesp_config->is_enabled("DS18B20_TRIBORD_1"))
            sensor_temp[DS18B20_TRIBORD_1]
              ->connect_to(new sensesp::SKOutputFloat(sk_path_temp[DS18B20_TRIBORD_1], "",
                          new sensesp::SKMetadata("K", "Echappt Tribord", sk_path_temp[DS18B20_TRIBORD_1], "T° Echappement Moteur Tribord")));
          break;
        }
        case DS18B20_COMMON_2 : {
          if (sensesp_config->is_enabled("DS18B20_COMMON_2"))
            sensor_temp[DS18B20_COMMON_2]
              ->connect_to(new sensesp::SKOutputFloat(sk_path_temp[DS18B20_COMMON_2], "",
                          new sensesp::SKMetadata("K", "Compartiment Moteur", sk_path_temp[DS18B20_COMMON_2], "T° Compartiment Moteur")));
          break;
        }
        default : {
          #ifdef DEBUG_MODE
          String output;
          serializeJsonPretty(jsonsensor, output);
          Serial.println("DS18B20 not declared. Constant DS18B20_xxx_ADDR needs to be modified :\n" + output);
          sleep(5);
          #endif
        }
      };
  }
}