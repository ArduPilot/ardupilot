#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AnalogIn.h"
#include <stdint.h>

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

ADCSource::ADCSource(SITL_State *sitlState, int16_t pin) :
    _sitlState(sitlState),
    _pin(pin)
{}

float ADCSource::read_average() {
    return read_latest();
}

float ADCSource::voltage_average() {
    return (5.0f/1023.0f) * read_average();
}

float ADCSource::voltage_latest() {
    return (5.0f/1023.0f) * read_latest();
}

float ADCSource::read_latest() {
    switch (_pin) {
    case ANALOG_INPUT_BOARD_VCC:
        return 1023;

    case 0:
        return _sitlState->sonar_pin_value;

    case 1:
        return _sitlState->airspeed_pin_value;

    case 12:
        return _sitlState->current_pin_value;

    case 13:
        return _sitlState->voltage_pin_value;

    case ANALOG_INPUT_NONE:
    default:
        return 0.0f;
    }
}

void ADCSource::set_pin(uint8_t pin) {
    _pin = pin;
}

void AnalogIn::init() {
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t pin) {
    return new ADCSource(_sitlState, pin);
}

#endif
