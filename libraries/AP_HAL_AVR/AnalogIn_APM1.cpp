
#include <AP_HAL.h>
#include "AnalogIn.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

APM1AnalogIn::APM1AnalogIn () :
    _bat_voltage(ADCSource(0)),
    _bat_current(ADCSource(1)),
    _vdd(ADCSource(AVR_ANALOG_PIN_VCC)),
    AVRAnalogIn()
{ }

void APM1AnalogIn::init(void* machtnichts) {
    /* Register AVRAnalogIn::_timer_event with the scheduler. */
    hal.scheduler->register_timer_process(_timer_event);
    /* Register each private channel with AVRAnalogIn. */
    _register_channel(&_bat_voltage);
    _register_channel(&_bat_current);
    _register_channel(&_vdd);
}

AP_HAL::AnalogSource* APM1AnalogIn::channel(int ch) {
    switch(ch) {
        case 0:
            return &_bat_voltage;
            break;
        case 1:
            return &_bat_current;
            break;
        case 2:
            return &_vdd;
            break;
        default:
            return NULL;
    }
}
