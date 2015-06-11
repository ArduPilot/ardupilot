#include <AP_HAL.h>
#include "GPIO.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

LinuxDigitalSource::LinuxDigitalSource(uint8_t v) :
    _v(v)
{

}

void LinuxDigitalSource::mode(uint8_t output)
{
    hal.gpio->pinMode(_v, output);
}

uint8_t LinuxDigitalSource::read()
{
    return hal.gpio->read(_v);
}

void LinuxDigitalSource::write(uint8_t value)
{
    return hal.gpio->write(_v,value);
}

void LinuxDigitalSource::toggle()
{
    write(!read());
}

#endif