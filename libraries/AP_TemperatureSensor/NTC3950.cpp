#include "NTC3950.h"

#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

static const uint8_t NTC3950_PIN = 13;
static const uint8_t NOMINAL_TEMPERATURE = 25;      // 25°C
static const uint16_t NOMINAL_RESISTANCE = 100000;  // R=100kΩ at 25°C
static const uint16_t ADC_RESOLUTION = 4096;
static const uint16_t RESISTOR_SERIAL = 100000;     // 100kΩ resistor between the thermistor and ground
static const uint16_t BETA_COEFFICIENT = 3950;      // value taken from ntc3950 calibration sheet

bool NTC3950::init()
{
    _analog_source = std::move(hal.analogin->channel(NTC3950_PIN));
    if (!_analog_source) {
        printf("NTC3950 device is null!");
        return false;
    }

    return true;
}

void NTC3950::update_reading()
{
    float reading = _analog_source->voltage_average();
    
    /// TBD health check over temperature value
    
    // convert a (average) voltage value to resistance
    reading = ADC_RESOLUTION / reading - 1;
    reading = RESISTOR_SERIAL * reading;
 
    // https://en.wikipedia.org/wiki/Steinhart–Hart_equation
    float steinhart;
    steinhart = reading / NOMINAL_RESISTANCE;               // (R/Ro)
    steinhart /= BETA_COEFFICIENT;                          // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);      // + (1/To)
    steinhart = 1.0 / steinhart;                            // Invert
    steinhart -= 273.15;                                    // convert to C
 
    _temperature = steinhart;
}