/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "HAL_AVR.h"
#include "AP_HAL_AVR_SITL.h"
#include "AnalogIn.h"
#include <stdint.h>

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;

ADCSource::ADCSource(SITL_State *sitlState, uint8_t pin, float prescale) :
    _sitlState(sitlState),
    _pin(pin),
    _prescale(prescale)
{}

float ADCSource::read_average() {
	return read_latest();
}

float ADCSource::read_latest() {
    switch (_pin) {
    case ANALOG_INPUT_BOARD_VCC:
        return 4900;
        
    case 0:
        return _sitlState->airspeed_pin_value;

    case ANALOG_INPUT_NONE:
    default:
        return 0.0;
    }
}

void ADCSource::set_pin(uint8_t pin) {
    _pin = pin;
}

void SITLAnalogIn::init(void *ap_hal_scheduler) {
}

AP_HAL::AnalogSource* SITLAnalogIn::channel(int16_t pin) {
    return channel(pin, 1.0);
}

AP_HAL::AnalogSource* SITLAnalogIn::channel(int16_t pin, float prescale) {
    return new ADCSource(_sitlState, pin, prescale);	
}

#endif
