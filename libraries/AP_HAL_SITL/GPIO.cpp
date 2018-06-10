
#include "GPIO.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

#define SITL_WOW_PIN 8
#define SITL_WOW_ALTITUDE 0.01

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
    if (!_sitlState->_sitl) {
        return 0;
    }
    
    // Weight on wheels pin
    if (pin == SITL_WOW_PIN) {
        return _sitlState->_sitl->state.altitude < SITL_WOW_ALTITUDE ? 1 : 0;
    }
    
    // pins 0-7 are available for simulation
    if (pin > 7) {
        return 0;
    }
    uint16_t mask = static_cast<uint16_t>(_sitlState->_sitl->pin_mask.get());
    return static_cast<uint16_t>((mask & (1U << pin)) ? 1 : 0);
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    if (!_sitlState->_sitl) {
        return;
    }
    uint16_t mask = static_cast<uint16_t>(_sitlState->_sitl->pin_mask.get());
    uint16_t new_mask = mask;
    
    // pins 0-7 are available for simulation
    if (pin > 7) {
        return;
    }
    
    if (value) {
        new_mask |= (1U << pin);
    } else {
        new_mask &= ~(1U << pin);
    }
    if (mask != new_mask) {
        _sitlState->_sitl->pin_mask.set_and_notify(new_mask);
    }
}

void GPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    if (n < 16) {  // (ie. sizeof(pin_mask)*8)
        return new DigitalSource(static_cast<uint8_t>(n));
    } else {
        return nullptr;
    }

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

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{}

uint8_t DigitalSource::read()
{
    return hal.gpio->read(_pin);
}

void DigitalSource::write(uint8_t value)
{
    value = static_cast<uint8_t>(value ? 1 : 0);
    return hal.gpio->write(_pin, value);
}

void DigitalSource::toggle()
{
    return hal.gpio->write(_pin, !hal.gpio->read(_pin));
}
