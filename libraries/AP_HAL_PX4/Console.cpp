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

void PX4ConsoleDriver::print_P(const prog_char_t *pstr) {
	print(pstr);
}

void PX4ConsoleDriver::println_P(const prog_char_t *pstr) {
	println(pstr);
}

void PX4ConsoleDriver::printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _uart->vprintf(fmt, ap);
    va_end(ap);
}

void PX4ConsoleDriver::_printf_P(const prog_char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _uart->vprintf(fmt, ap);
    va_end(ap);
}

void PX4ConsoleDriver::vprintf(const char *fmt, va_list ap) {
	_uart->vprintf(fmt, ap);
}

void PX4ConsoleDriver::vprintf_P(const prog_char *fmt, va_list ap) {
	_uart->vprintf(fmt, ap);
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

int16_t PX4ConsoleDriver::peek() {
	return _uart->peek();
}

size_t PX4ConsoleDriver::write(uint8_t c) {
	return _uart->write(c);
}

#endif
