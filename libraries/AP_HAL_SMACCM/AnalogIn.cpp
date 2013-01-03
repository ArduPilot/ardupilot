
#include "AnalogIn.h"

using namespace SMACCM;

SMACCMAnalogSource::SMACCMAnalogSource(float v) :
    _v(v)
{}

float SMACCMAnalogSource::read_average() {
    return _v;
}

float SMACCMAnalogSource::read_latest() {
    return _v;
}

void SMACCMAnalogSource::set_pin(uint8_t p)
{}


SMACCMAnalogIn::SMACCMAnalogIn()
{}

void SMACCMAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* SMACCMAnalogIn::channel(int16_t n) {
    return new SMACCMAnalogSource(1.11);
}

AP_HAL::AnalogSource* SMACCMAnalogIn::channel(int16_t n, float scale) {
    return new SMACCMAnalogSource(scale/2);
}

