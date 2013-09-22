#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <stdarg.h>
#include "Console.h"
#include "UARTDriver.h"
#include <stdio.h>

using namespace PX4;

PX4ConsoleDriver::PX4ConsoleDriver() {}

void PX4ConsoleDriver::init(void* uart)
{
	_uart = (PX4UARTDriver *)uart;
}

void PX4ConsoleDriver::backend_open()
{}

void PX4ConsoleDriver::backend_close()
{}

size_t PX4ConsoleDriver::backend_read(uint8_t *data, size_t len) {
    return 0;
}

size_t PX4ConsoleDriver::backend_write(const uint8_t *data, size_t len) {
    return 0;
}

int16_t PX4ConsoleDriver::available() {
	return _uart->available();
}

int16_t PX4ConsoleDriver::txspace() {
	return _uart->txspace();
}

int16_t PX4ConsoleDriver::read() {
	return _uart->read();
}

size_t PX4ConsoleDriver::write(uint8_t c) {
	return _uart->write(c);
}

#endif
