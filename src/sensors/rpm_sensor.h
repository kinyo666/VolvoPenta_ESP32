/*
    RPM Sensor :
    - Reads the RPM from a PC817 optocoupler
    - Compute the fuel consumption based on the RPM
    - If INA3221_POWERDOWN is enabled, it will power down the INA3221 when the engine is off
  
  @author kinyo666
  @version 1.0.18
  @date 04/08/2025
  @link GitHub source code : https://github.com/kinyo666/Capteurs_ESP32
*/
#ifndef RPM_SENSOR_H
#define RPM_SENSOR_H

#include <sensesp/sensors/digital_input.h>
#include <sensesp/signalk/signalk_output.h>
#include "customClasses.h"
#include "sensors/voltage_sensor.h"

// Use for testing purposes
//#define FAKE_MODE 1
//#define SIMULATE_RPM 1
//#define SIMULATE_RPM_PIN 13
#ifdef SIMULATE_RPM
#include <sensesp/sensors/sensor.h>
#endif
#ifdef FAKE_MODE
#include <sensesp/sensors/constant_sensor.h>
#endif

// RPM sensors PC817
#define PC817_BABORD_PIN 35
#define PC817_TRIBORD_PIN 34

// Constants for Signal-K path
const String sk_path_engines[ENGINE_NB][ENGINE_SK_PATH_NB] = {
        {"propulsion.babord.revolutions",   "propulsion.babord.fuel.rate",  "propulsion.babord.state"},
        {"propulsion.tribord.revolutions",  "propulsion.tribord.fuel.rate", "propulsion.tribord.state"}};
const String conf_path_engines[ENGINE_NB][2] = {{"/CONFIG/BABORD/PC817/FREQUENCY_RPM", "/CONFIG/BABORD/PC817/MOVING_AVG"},
                                                {"/CONFIG/TRIBORD/PC817/FREQUENCY_RPM", "/CONFIG/TRIBORD/PC817/MOVING_AVG"}}; 
// Engines state and timer (in seconds) to power-down sensors
enum engine_sk_path_t { REVOLUTIONS = 0, FUELRATE, STATE };

typedef sensesp::LambdaTransform<float, boolean> EngineState;
typedef sensesp::LambdaTransform<bool, bool, u_int8_t, unsigned int> EngineSleepMode;

void setupRPMSensors(ConfigSensESP*);
boolean runningState(float);

// Fuel Consumption for TAMD40B based on https://www.volvopenta.com/your-engine/manuals-and-handbooks/ (see Product Leaflet)
class FuelConsumption : public sensesp::CurveInterpolator {
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

// Define the engines transform to output data to Signal-K
class EngineDataTransform {
  public :
    sensesp::Frequency *freq;
    sensesp::Linear *hz_to_rpm;
    FuelConsumption *rpm_to_lhr;
    LinearPos *lhr_to_m3s;
    sensesp::MovingAverage *moving_avg;
    EngineState *running_state;

    EngineDataTransform(float multiplier, const String *conf_path_engine, u_int engine_id) { 
        freq = new sensesp::Frequency(multiplier, conf_path_engine[PC817_FREQUENCY_RPM]);    // Pulses to Hertz (Hz)
        hz_to_rpm = new sensesp::Linear(60, 0.0);                                            // Hertz (Hz) to Revolutions Per Minute (RPM)
        rpm_to_lhr = new FuelConsumption();                                         // RPM to Liter per Hour (l/hr)
        lhr_to_m3s = new LinearPos(linearPositive, 1 / (3.6 * pow(10, 6)), 
                                  0.0, linearPositive_ParamInfo);                   // Liter per Hour (l/hr) to Meter cube per second (m3/s)
        moving_avg = new sensesp::MovingAverage(4, 1.0, conf_path_engine[PC817_MOVING_AVG]); // Moving average with 4 samples
        running_state = new EngineState(runningState);                              // Hertz (Hz) to running state (true or false)

        ConfigItem(freq)
        ->set_title("Compte-tours - " + conf_path_engine[PC817_FREQUENCY_RPM])
        ->set_description("Multiplier needs to be adjusted based on flywheel to Alternator W ratio."\
                          "Example: 1 turn on an AQAD41 makes 2.45 turns on the Alternator, and the Alternator has 6 poles which makes a total of 2.45 x 6 = 14.7 Hz per turn. SignalK needs info in Hz so we need to divide the incoming value with 14.7, or as in our case multiply with (1/14.7) = 0,06803"\
                          "If Ratio is unknown, the original Tachometer might have a max Impulses/min written on it, divide that with max rpm on the meter and you\'ll get the ratio. Tachometer 860420 is marked with 73500Imp/min and has a scale to 5000rpm. 73500 divided with 5000 equals 14,7, Tada!"\
                          "If above multiplier needs to be recalculated, the FuelMultipliers needs to be recalculated as well, they're both based on AQAD41 values right now."\
                          "AQAD41 example: 60 divided with FlyWheel To W Ratio (60 / 14,7 = 4,08)")
        ->set_sort_order(25+(2*engine_id));

        ConfigItem(moving_avg)
        ->set_title("Compte-tours - MovingAvg " + conf_path_engine[PC817_MOVING_AVG])
        ->set_description("Moving average of the engine RPM to smooth out the signal."\
                          "Default value is 4 samples.")
        ->set_sort_order(26+(2*engine_id));
      }
};

#endif // RPM_SENSOR_H