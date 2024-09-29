
#include "AP_HAL_ESP32.h"
#include "GPIO.h"

#include "driver/gpio.h"

using namespace ESP32;

GPIO::GPIO()
{}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = output ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL<<pin;
    io_conf.pull_down_en = output ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

uint8_t GPIO::read(uint8_t pin) {
    return gpio_get_level((gpio_num_t)pin);
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    gpio_set_level((gpio_num_t)pin, value);
}

void GPIO::toggle(uint8_t pin)
{
    if (read(pin)) {
        write(pin, 0);
    }
    else {
        write(pin, 1);
    }
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return NEW_NOTHROW DigitalSource(0);
}

bool GPIO::usb_connected(void)
{
    return false;
}

DigitalSource::DigitalSource(uint8_t v) :
    _v(v)
{}

void DigitalSource::mode(uint8_t output)
{}

uint8_t DigitalSource::read() {
    return _v;
}

void DigitalSource::write(uint8_t value) {
    _v = value;
}

void DigitalSource::toggle() {
    _v = !_v;
}
