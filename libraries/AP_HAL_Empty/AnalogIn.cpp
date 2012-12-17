
#include "AnalogIn.h"

using namespace Empty;

EmptyAnalogSource::EmptyAnalogSource(float v) :
    _v(v)
{}

float EmptyAnalogSource::read_average() {
    return _v;
}

float EmptyAnalogSource::read_latest() {
    return _v;
}

void EmptyAnalogSource::set_pin(uint8_t p)
{}


EmptyAnalogIn::EmptyAnalogIn()
{}

void EmptyAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* channel(int16_t n) {
    return new EmptyAnalogSource(1.11);
}

AP_HAL::AnalogSource* channel(int16_t n, float scale) {
    return new EmptyAnalogSource(scale/2);
}

