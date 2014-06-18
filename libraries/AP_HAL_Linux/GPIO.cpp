#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "GPIO.h"

using namespace Linux;

LinuxGPIO::LinuxGPIO()
{}

void LinuxGPIO::init()
{}

void LinuxGPIO::pinMode(uint8_t pin, uint8_t output)
{}

int8_t LinuxGPIO::analogPinToDigitalPin(uint8_t pin)
{
	return -1;
}


uint8_t LinuxGPIO::read(uint8_t pin) {
    return 0;
}

void LinuxGPIO::write(uint8_t pin, uint8_t value)
{}

void LinuxGPIO::toggle(uint8_t pin)
{}

/* Alternative interface: */
AP_HAL::DigitalSource* LinuxGPIO::channel(uint16_t n) {
    return new LinuxDigitalSource(0);
}

/* Interrupt interface: */
bool LinuxGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool LinuxGPIO::usb_connected(void)
{
    return false;
}

LinuxDigitalSource::LinuxDigitalSource(uint8_t v) :
    _v(v)
{}

void LinuxDigitalSource::mode(uint8_t output)
{}

uint8_t LinuxDigitalSource::read() {
    return _v;
}

void LinuxDigitalSource::write(uint8_t value) {
    _v = value;
}

void LinuxDigitalSource::toggle() {
    _v = !_v;
}

#endif // CONFIG_HAL_BOARD
