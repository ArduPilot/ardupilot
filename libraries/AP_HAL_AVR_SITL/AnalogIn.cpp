/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "HAL_AVR.h"
#include "AP_HAL_AVR_SITL.h"
#include "AnalogIn.h"
#include <stdint.h>

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;

ADCSource::ADCSource(uint8_t pin, float prescale) :
    _prescale(prescale)
{}

float ADCSource::read_average() {
	return 0;
}

float ADCSource::read_latest() {
	return 0;
}

void ADCSource::set_pin(uint8_t pin) {
}

SITLAnalogIn::SITLAnalogIn() {}

void SITLAnalogIn::init(void *ap_hal_scheduler) {
}

AP_HAL::AnalogSource* SITLAnalogIn::channel(int16_t n) {
    return NULL;
}

AP_HAL::AnalogSource* SITLAnalogIn::channel(int16_t n, float prescale) {
    return NULL;	
}

#endif
