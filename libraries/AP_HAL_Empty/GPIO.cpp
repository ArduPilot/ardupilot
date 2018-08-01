
#include "GPIO.h"

using namespace Empty;

GPIO::GPIO()
{}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{}

uint8_t GPIO::read(uint8_t pin) {
    return 0;
}

void GPIO::write(uint8_t pin, uint8_t value)
{}

void GPIO::toggle(uint8_t pin)
{}

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
