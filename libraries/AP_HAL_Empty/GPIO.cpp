
#include "GPIO.h"

using namespace Empty;

EmptyGPIO::EmptyGPIO()
{}

void EmptyGPIO::init()
{}

void EmptyGPIO::pinMode(uint8_t pin, uint8_t output)
{}


uint8_t EmptyGPIO::read(uint8_t pin) {
    return 0;
}

void EmptyGPIO::write(uint8_t pin, uint8_t value)
{}

/* Alternative interface: */
AP_HAL::DigitalSource* EmptyGPIO::channel(uint16_t n) {
    return new EmptyDigitalSource(0);
}

/* Interrupt interface: */
bool EmptyGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}


EmptyDigitalSource::EmptyDigitalSource(uint8_t v) :
    _v(v)
{}

void EmptyDigitalSource::mode(uint8_t output)
{}

uint8_t EmptyDigitalSource::read() {
    return _v;
}

void EmptyDigitalSource::write(uint8_t value) {
    _v = value;
}

