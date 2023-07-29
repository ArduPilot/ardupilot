#include "driver/gpio.h"
#include "GPIO.h"


using namespace ESP32;

GPIO::GPIO()
{}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    gpio_mode_t mode = output?GPIO_MODE_OUTPUT:GPIO_MODE_INPUT;
    gpio_set_direction((gpio_num_t)pin, mode);
}

uint8_t GPIO::read(uint8_t pin)
{
    return gpio_get_level((gpio_num_t)pin);
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    gpio_set_level((gpio_num_t)pin, value);
}

void GPIO::toggle(uint8_t pin)
{
    gpio_set_level((gpio_num_t)pin, !gpio_get_level((gpio_num_t)pin));
}

//below is the remaining undefined function. for now we're borrowing AP_HAL_Empty's definition

AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return new DigitalSource(0);
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
