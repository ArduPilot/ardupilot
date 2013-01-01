#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <stdarg.h>
#include "Console.h"
#include <stdio.h>

using namespace PX4;

PX4ConsoleDriver::PX4ConsoleDriver() {}

void PX4ConsoleDriver::init(void* unused)
{
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
    vfprintf(stdout, fmt, ap);
    va_end(ap);
}

void PX4ConsoleDriver::_printf_P(const prog_char *fmt, ...) {
    va_list ap;
    va_start(ap,fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);
}

void PX4ConsoleDriver::vprintf(const char *fmt, va_list ap) {
	vfprintf(stdout, fmt, ap);
}

void PX4ConsoleDriver::vprintf_P(const prog_char *fmt, va_list ap) {
	vfprintf(stdout, fmt, ap);
}

int16_t PX4ConsoleDriver::available() {
	return 0;
}

int16_t PX4ConsoleDriver::txspace() {
	return 0;
}

int16_t PX4ConsoleDriver::read() {
	return 0;
}

int16_t PX4ConsoleDriver::peek() {
	return 0;
}

size_t PX4ConsoleDriver::write(uint8_t c) {
	fputc(c, stdout);
	return 1;
}

#endif
