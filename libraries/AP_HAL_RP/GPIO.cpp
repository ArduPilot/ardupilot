
#include "GPIO.h"

#include "hardware/gpio.h"

using namespace RP;

// RP2350 has GPIOs 0..29 or 0..47 available to the user.
static inline bool rp_valid_pin(uint8_t pin)
{
    return pin < RP_GPIO_COUNT;
}

GPIO::GPIO() = default;

void GPIO::init()
{
    // Nothing to do here – pins are configured on demand in pinMode()
}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    if (!rp_valid_pin(pin)) {
        return;
    }

    gpio_init(pin);
    switch (output) {
    case HAL_GPIO_INPUT:
        gpio_set_dir(pin, false);
        break;
    case HAL_GPIO_OUTPUT:
        gpio_set_dir(pin, true);
        break;
    default:
        // HAL_GPIO_ALT and other modes are not handled here
        break;
    }
}

uint8_t GPIO::read(uint8_t pin)
{
    if (!rp_valid_pin(pin)) {
        return 0;
    }
    return gpio_get(pin) ? 1 : 0;
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    if (!rp_valid_pin(pin)) {
        return;
    }
    gpio_put(pin, value != 0);
}

void GPIO::toggle(uint8_t pin)
{
    if (!rp_valid_pin(pin)) {
        return;
    }
    gpio_put(pin, !gpio_get(pin));
}

bool GPIO::valid_pin(uint8_t pin) const
{
    return rp_valid_pin(pin);
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n)
{
    if (!rp_valid_pin(static_cast<uint8_t>(n))) {
        return nullptr;
    }
    return NEW_NOTHROW DigitalSource(static_cast<uint8_t>(n));
}

bool GPIO::usb_connected(void)
{
    // RP2350 boards may implement this via a dedicated sense pin or USB stack.
    // For now we return false until a board-specific implementation is added.
    return false;
}

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{
    GPIO gpio;
    gpio.pinMode(_pin, output);
}

uint8_t DigitalSource::read()
{
    GPIO gpio;
    return gpio.read(_pin);
}

void DigitalSource::write(uint8_t value)
{
    GPIO gpio;
    gpio.write(_pin, value);
}

void DigitalSource::toggle()
{
    GPIO gpio;
    gpio.toggle(_pin);
}
