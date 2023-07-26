
#include "GPIO.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

#define SITL_WOW_ALTITUDE 0.01

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    if (pin > 7) {
        return;
    }
    if (output) {
        pin_mode_is_write |= (1U<<pin);
    } else {
        pin_mode_is_write &= ~(1U<<pin);
    }
}

uint8_t GPIO::read(uint8_t pin)
{
    if (!_sitlState->_sitl) {
        return 0;
    }
    if (!valid_pin(pin)) {
        return 0;
    }
    
    // weight on wheels pin support
    if (pin == _sitlState->_sitl->wow_pin.get()) {
        return _sitlState->_sitl->state.altitude < SITL_WOW_ALTITUDE ? 1 : 0;
    }
    
    uint16_t mask = static_cast<uint16_t>(_sitlState->_sitl->pin_mask.get());
    return static_cast<uint16_t>((mask & (1U << pin)) ? 1 : 0);
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    if (!_sitlState->_sitl) {
        return;
    }

    if (!valid_pin(pin)) {
        return;
    }
    if (pin < 8) {
        if (!(pin_mode_is_write & (1U<<pin))) {
            // ignore setting of pull-up resistors
            return;
        }
    }
    uint16_t mask = static_cast<uint16_t>(_sitlState->_sitl->pin_mask.get());
    uint16_t new_mask = mask;

    if (pin == _sitlState->_sitl->wow_pin.get()) {
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
#endif
