#include "AnalogIn.h"

using namespace Empty;

AnalogSource::AnalogSource(float v) :
    _v(v)
{}

float AnalogSource::read_average() {
    return _v;
}

float AnalogSource::voltage_average() {
    return 5.0f * _v / 1024.0f;
}

float AnalogSource::voltage_latest() {
    return 5.0f * _v / 1024.0f;
}

float AnalogSource::read_latest() {
    return _v;
}

bool AnalogSource::set_pin(uint8_t p) {
    return true;
}

AnalogIn::AnalogIn()
{}

void AnalogIn::init()
{}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n) {
    return new AnalogSource(1.11);
}

float AnalogIn::board_voltage(void)
{
    return 5.0f;
}
