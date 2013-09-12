
#include "AnalogIn.h"

using namespace SMACCM;

SMACCMAnalogSource::SMACCMAnalogSource(float v) :
    _v(v)
{}

float SMACCMAnalogSource::read_average() {
    return _v;
}

float SMACCMAnalogSource::voltage_average() {
    // this assumes 5.0V scaling and 1024 range
    return (5.0/1024.0) * read_average();
}

float SMACCMAnalogSource::voltage_latest() {
    // this assumes 5.0V scaling and 1024 range
    return (5.0/1024.0) * read_latest();
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

