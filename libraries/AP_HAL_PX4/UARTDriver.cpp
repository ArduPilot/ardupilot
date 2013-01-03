#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "UARTDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <poll.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

PX4UARTDriver::PX4UARTDriver(const char *devpath) {
	_devpath = devpath;
}

extern const AP_HAL::HAL& hal;

/*
  this UART driver just maps to fd 0/1, which goes to whatever is
  setup for the PX4 console. Baud rate control is not available.
 */

void PX4UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
	if (!_initialised) {
		_fd = open(_devpath, O_RDWR);
		if (_fd == -1) {
			fprintf(stdout, "Failed to open UART device %s - %s\n",
				_devpath, strerror(errno));
			return;
		}
		_initialised = true;
	}
}

void PX4UARTDriver::begin(uint32_t b) {
	begin(b, 0, 0);
}


void PX4UARTDriver::end() {}
void PX4UARTDriver::flush() {}
bool PX4UARTDriver::is_initialized() { return true; }
void PX4UARTDriver::set_blocking_writes(bool blocking) {
	_nonblocking_writes = !blocking;
}
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
    _vdprintf(_fd, fmt, ap);
    va_end(ap);	
}

void PX4UARTDriver::_printf_P(const prog_char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _vdprintf(_fd, fmt, ap);
    va_end(ap);	
}

void PX4UARTDriver::vprintf(const char *fmt, va_list ap) {
    _vdprintf(_fd, fmt, ap);
}

void PX4UARTDriver::vprintf_P(const prog_char *fmt, va_list ap) {
    _vdprintf(_fd, fmt, ap);
}

/* PX4 implementations of Stream virtual methods */
int16_t PX4UARTDriver::available() { 
	int ret = 0;
	if (!_initialised) {
		return 0;
	}
	if (ioctl(_fd, FIONREAD, (long unsigned int)&ret) == 0 && ret > 0) {
		if (ret > 90) ret = 90;
		return ret;
	}
	return 0;
}

int16_t PX4UARTDriver::txspace() { 
	int ret = 0;
	if (!_initialised) {
		return 0;
	}
	if (ioctl(_fd, FIONWRITE, (long unsigned int)&ret) == 0 && ret > 0) {
		if (ret > 90) ret = 90;
		return ret;
	}
	return 0;
}

int16_t PX4UARTDriver::read() { 
	if (available() > 0) {
		uint8_t c;
		if (::read(_fd, &c, 1) == 1) {
			return c;
		}
	}
	return -1;
}

int16_t PX4UARTDriver::peek() { 
	return -1;
}

/* PX4 implementations of Print virtual methods */
size_t PX4UARTDriver::write(uint8_t c) { 
	if (!_initialised) {
		return 0;
	}
	if (_nonblocking_writes && txspace() == 0) {
		return 0;
	}
	return ::write(_fd, &c, 1);
}

void PX4UARTDriver::_vdprintf(int fd, const char *fmt, va_list ap) {
	char buf[128];
	if (!_initialised) {
		return;
	}
	int len = vsnprintf(buf, sizeof(buf), fmt, ap);
	if (len > 0) {
		if (_nonblocking_writes) {
			int16_t space = txspace();
			if (space < len) {
				len = space;
			}
		}
		if (len > 0) {
			::write(_fd, buf, len);
		}
	}
}

#endif
