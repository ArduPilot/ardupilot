#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "AnalogIn.h"

using namespace YUNEEC;

YUNEECAnalogSource::YUNEECAnalogSource(float v) :
    _v(v)
{}

float YUNEECAnalogSource::read_average() {
    return _v;
}

float YUNEECAnalogSource::voltage_average() {
    return 5.0 * _v / 1024.0;
}

float YUNEECAnalogSource::voltage_latest() {
    return 5.0 * _v / 1024.0;
}

float YUNEECAnalogSource::read_latest() {
    return _v;
}

void YUNEECAnalogSource::set_pin(uint8_t p)
{}

void YUNEECAnalogSource::set_stop_pin(uint8_t p)
{}

void YUNEECAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

YUNEECAnalogIn::YUNEECAnalogIn()
{}

void YUNEECAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* YUNEECAnalogIn::channel(int16_t n) {
    return new YUNEECAnalogSource(1.11);
}

float YUNEECAnalogIn::board_voltage(void)
{
    return 0;
}

#endif
