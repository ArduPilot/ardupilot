
#include <AP_HAL.h>

#include "AnalogIn.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL &hal;

APM2AnalogIn::APM2AnalogIn () :
    _pitot(ADCSource(0, 4.0)),
    _bat_voltage(ADCSource(1)),
    _bat_current(ADCSource(2)),
    _rssi(ADCSource(3, 0.25)),
    _vdd(ADCSource(AVR_ANALOG_PIN_VCC)),
    AVRAnalogIn()
{ }

void APM2AnalogIn::init(void * machtichts) {
    /* Register AVRAnalogIn::_timer_event with the scheduler. */
    hal.scheduler->register_timer_process(_timer_event);
    /* Register each private channel with AVRAnalogIn. */
    _register_channel(&_pitot);
    _register_channel(&_bat_voltage);
    _register_channel(&_bat_current);
    _register_channel(&_rssi);
    _register_channel(&_vdd);
}

AP_HAL::AnalogSource* APM2AnalogIn::channel(int ch) {
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
        case 3:
            return &_pitot;
            break;
        case 4:
            return &_rssi;
            break;
        default:
            return NULL;
    }
}

