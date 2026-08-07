#include "GPIO_Navigator.h"

#if HAL_LINUX_GPIO_NAVIGATOR_ENABLED

uint8_t GPIO_Navigator::read(uint8_t pin)
{
    if (pinAllowed(pin)) {
        return GPIO_RPI::read(pin);
    }
    if (hal.rcout->supports_gpio()) {
        return (uint8_t)hal.rcout->read(pin - 1);
    }
    return 0;
}

void GPIO_Navigator::write(uint8_t pin, uint8_t value)
{
    if (pinAllowed(pin)) {
        GPIO_RPI::write(pin, value);
        return;
    }
    if (hal.rcout->supports_gpio()) {
        hal.rcout->write_gpio(pin - 1, value);
    }
}

void GPIO_Navigator::pinMode(uint8_t pin, uint8_t output)
{
    if (pinAllowed(pin)) {
        GPIO_RPI::pinMode(pin, output);
    }
}

void GPIO_Navigator::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    if (pinAllowed(pin)) {
        GPIO_RPI::pinMode(pin, output, alt);
    }
}

bool GPIO_Navigator::pinAllowed(uint8_t pin)
{
    for (const auto &gpio : AllowedGPIOS) {
        if (pin == gpio)     {
            return true;
        }
    }
    return false;
}

bool GPIO_Navigator::pin_to_servo_channel(uint8_t pin, uint8_t& servo_ch) const
{
    if (1 <= pin && pin <= 16) {
        servo_ch = pin - 1;
        return true;
    }
    return false;
}
#endif  // HAL_LINUX_GPIO_NAVIGATOR_ENABLED
