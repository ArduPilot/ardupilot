#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AnalogIn.h"

using namespace Linux;

AnalogSource::AnalogSource(float v) :
    _v(v)
{}

float AnalogSource::read_average() {
    return _v;
}

float AnalogSource::voltage_average() {
    return 5.0 * _v / 1024.0;
}

float AnalogSource::voltage_latest() {
    return 5.0 * _v / 1024.0;
}

float AnalogSource::read_latest() {
    return _v;
}

void AnalogSource::set_pin(uint8_t p)
{}

void AnalogSource::set_stop_pin(uint8_t p)
{}

void AnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

AnalogIn::AnalogIn()
{}

void AnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n) {
    return new AnalogSource(1.11);
}

#endif // CONFIG_HAL_BOARD
