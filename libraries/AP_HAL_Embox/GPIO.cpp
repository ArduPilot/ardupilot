#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
// #include <drivers/gpio.h>

#include "GPIO.h"
#include "Util.h"

extern const AP_HAL::HAL &hal;

using namespace Embox;

DigitalSource::DigitalSource(uint8_t v): _v(v) {
}

void DigitalSource::mode(uint8_t output) {
}

uint8_t DigitalSource::read() {
	return _v;
}

void DigitalSource::write(uint8_t value) {
	_v = value;
}

void DigitalSource::toggle() {
	_v = !_v;
}

void GPIO::init() {
}

void GPIO::pinMode(uint8_t pin, uint8_t output) {
	// gpio_setup_mode(GPIO_PORT_A, pin, output ? GPIO_MODE_OUT : GPIO_MODE_IN);
}

uint8_t GPIO::read(uint8_t pin) {
	// return !!gpio_get(GPIO_PORT_A, pin);
	return 0;
}

void GPIO::write(uint8_t pin, uint8_t value) {
	// gpio_set(GPIO_PORT_A, pin, value);
}

void GPIO::toggle(uint8_t pin) {
	// gpio_toggle(GPIO_PORT_A, pin);
}

AP_HAL::DigitalSource *GPIO::channel(uint16_t pin) {
	return NEW_NOTHROW DigitalSource(0);
}

bool GPIO::usb_connected(void) {
	return false;
}
