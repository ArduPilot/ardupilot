#include <AP_HAL/AP_HAL.h>

#include "GPIO.h"

using namespace Linux;

extern const AP_HAL::HAL& hal;

DigitalSource::DigitalSource(uint8_t v) :
    _v(v)
{

}

void DigitalSource::mode(uint8_t output)
{
    hal.gpio->pinMode(_v, output);
}

uint8_t DigitalSource::read()
{
    return hal.gpio->read(_v);
}

void DigitalSource::write(uint8_t value)
{
    return hal.gpio->write(_v,value);
}

void DigitalSource::toggle()
{
    write(!read());
}
