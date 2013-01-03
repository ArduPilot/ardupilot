#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "UARTDriver.h"
#include <stdio.h>
#include <unistd.h>

using namespace PX4;

PX4UARTDriver::PX4UARTDriver() {}

/*
  this UART driver just maps to fd 0/1, which goes to whatever is
  setup for the PX4 console. Baud rate control is not available.
 */

void PX4UARTDriver::begin(uint32_t b) {}
void PX4UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void PX4UARTDriver::end() {}
void PX4UARTDriver::flush() {}
bool PX4UARTDriver::is_initialized() { return true; }
void PX4UARTDriver::set_blocking_writes(bool blocking) {}
bool PX4UARTDriver::tx_pending() { return false; }

/* PX4 implementations of BetterStream virtual methods */
void PX4UARTDriver::print_P(const prog_char_t *pstr) {
	print(pstr);
}

void PX4UARTDriver::println_P(const prog_char_t *pstr) {
	println(pstr);
}

void PX4UARTDriver::printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _vdprintf(1, fmt, ap);
    va_end(ap);	
}

void PX4UARTDriver::_printf_P(const prog_char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _vdprintf(1, fmt, ap);
    va_end(ap);	
}

void PX4UARTDriver::vprintf(const char *fmt, va_list ap) {
    _vdprintf(1, fmt, ap);
}

void PX4UARTDriver::vprintf_P(const prog_char *fmt, va_list ap) {
    _vdprintf(1, fmt, ap);
}

/* PX4 implementations of Stream virtual methods */
int16_t PX4UARTDriver::available() { return 0; }
int16_t PX4UARTDriver::txspace() { return 128; }

int16_t PX4UARTDriver::read() { 
	uint8_t c;
	if (::read(0, &c, 1) == 1) {
		return c;
	}
	return -1;
}

int16_t PX4UARTDriver::peek() { 
	return -1; 
}

/* PX4 implementations of Print virtual methods */
size_t PX4UARTDriver::write(uint8_t c) { 
	return ::write(1, &c, 1);
}

void PX4UARTDriver::_vdprintf(int fd, const char *fmt, va_list ap) {
	char buf[128];
	int len = vsnprintf(buf, sizeof(buf), fmt, ap);
	if (len > 0) {
		::write(1, buf, len);
	}
}

#endif
