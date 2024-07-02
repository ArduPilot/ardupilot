#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AnalogIn.h"
#include <stdint.h>

#define VOLTAGE_TO_PIN_VALUE(_v) (constrain_float(_v * (SITL_ADC_MAX_PIN_VALUE/SITL_ADC_FULL_SCALE_VOLTAGE), 0, SITL_ADC_MAX_PIN_VALUE))

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
    return voltage_latest();
}

float ADCSource::voltage_latest() {
    switch (_pin) {
    case ANALOG_INPUT_BOARD_VCC:
        return SITL_ADC_MAX_PIN_VALUE;

    case 0:
        return _sitlState->sonar_pin_voltage;

    case 1:
        return _sitlState->airspeed_pin_voltage[0];
    
    case 2:
        return _sitlState->airspeed_pin_voltage[1];

    case 12:
        return _sitlState->current_pin_voltage;

    case 13:
        return _sitlState->voltage_pin_voltage;

    case 14:
        return _sitlState->current2_pin_voltage;

    case 15:
        return _sitlState->voltage2_pin_voltage;

    case ANALOG_INPUT_NONE:
    default:
        return 0.0f;
    }
}

float ADCSource::read_latest() {
    return VOLTAGE_TO_PIN_VALUE(voltage_latest());
}

bool ADCSource::set_pin(uint8_t pin) {
    _pin = pin;
    return pin != ANALOG_INPUT_NONE;
}

void AnalogIn::init() {
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t pin) {
    return NEW_NOTHROW ADCSource(_sitlState, pin);
}

#endif
