
#include "GPIO.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{}

int8_t GPIO::analogPinToDigitalPin(uint8_t pin)
{
	return pin;
}


uint8_t GPIO::read(uint8_t pin)
{
    if (!sitlState->_sitl) {
        return 0;
    }
    uint8_t mask = sitlState->_sitl->pin_mask.get();
    return mask & (1U<<pin)? 1: 0;
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    if (!sitlState->_sitl) {
        return;
    }
    uint8_t mask = sitlState->_sitl->pin_mask.get();
    uint8_t new_mask = mask;
    if (value) {
        new_mask |= (1U<<pin);
    } else {
        new_mask &= ~(1U<<pin);
    }
    if (mask != new_mask) {
        sitlState->_sitl->pin_mask.set_and_notify(new_mask);
    }
}

void GPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return new DigitalSource(0);
}

/* Interrupt interface: */
bool GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool GPIO::usb_connected(void)
{
    return false;
}

DigitalSource::DigitalSource(uint8_t v) :
    pin(v)
{}

void DigitalSource::mode(uint8_t output)
{}

uint8_t DigitalSource::read()
{
    return hal.gpio->read(pin);
}

void DigitalSource::write(uint8_t value)
{
    value = value?1:0;
    return hal.gpio->write(pin, value);
}

void DigitalSource::toggle()
{
    return hal.gpio->write(pin, !hal.gpio->read(pin));
}
