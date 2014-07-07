#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AnalogIn.h"

using namespace Linux;

LinuxAnalogSource::LinuxAnalogSource(float v) :
    _v(v)
{}

float LinuxAnalogSource::read_average() {
    return _v;
}

float LinuxAnalogSource::voltage_average() {
    return 5.0 * _v / 1024.0;
}

float LinuxAnalogSource::voltage_latest() {
    return 5.0 * _v / 1024.0;
}

float LinuxAnalogSource::read_latest() {
    return _v;
}

void LinuxAnalogSource::set_pin(uint8_t p)
{}

void LinuxAnalogSource::set_stop_pin(uint8_t p)
{}

void LinuxAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

LinuxAnalogIn::LinuxAnalogIn()
{}

void LinuxAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* LinuxAnalogIn::channel(int16_t n) {
    return new LinuxAnalogSource(1.11);
}

#endif // CONFIG_HAL_BOARD
