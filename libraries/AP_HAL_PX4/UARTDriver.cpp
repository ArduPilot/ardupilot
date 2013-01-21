/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "UARTDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

PX4UARTDriver::PX4UARTDriver(const char *devpath) {
	_devpath = devpath;
}

extern const AP_HAL::HAL& hal;

/*
  this UART driver maps to a serial device in /dev
 */

void PX4UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
	if (!_initialised) {
		_fd = open(_devpath, O_RDWR);
		if (_fd == -1) {
			fprintf(stdout, "Failed to open UART device %s - %s\n",
				_devpath, strerror(errno));
			return;
		}
		_initialised = true;

	}

	if (b != 0) {
		// set the baud rate
		struct termios t;
		tcgetattr(_fd, &t);
		cfsetspeed(&t, b);
		// disable LF -> CR/LF
		t.c_oflag &= ~ONLCR;
		tcsetattr(_fd, TCSANOW, &t);
	}
	if (rxS != 0 && rxS != _readbuf_size) {
		_readbuf_size = rxS;
		if (_readbuf != NULL) {
			free(_readbuf);
		}
		_readbuf = (uint8_t *)malloc(_readbuf_size);
		_readbuf_ofs = 0;
		_readbuf_count = 0;
	}
}

void PX4UARTDriver::begin(uint32_t b) 
{
	begin(b, 0, 0);
}


void PX4UARTDriver::end() {}
void PX4UARTDriver::flush() {}
bool PX4UARTDriver::is_initialized() { return true; }
void PX4UARTDriver::set_blocking_writes(bool blocking) 
{
	unsigned v;
    v = fcntl(_fd, F_GETFL, 0);
    if (blocking) {
        v &= ~O_NONBLOCK;
    } else {
        v |= O_NONBLOCK;
    }
    fcntl(_fd, F_SETFL, v);
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
	if (_readbuf_count != 0) {
		// avoid the cost of the ioctl if possible. This means
		// we are giving a lower response than is real, but
		// this saves a lot of ioctl system calls, which are
		// quite expensive (especially in the GPS driver)
		return (int16_t)_readbuf_count;
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
	uint8_t c;
	if (!_initialised || _readbuf == NULL) {
		return -1;
	}
	if (_readbuf_count == 0) {
		// refill the read buffer
        int n = ::read(_fd, _readbuf, _readbuf_size);
        if (n > 0) {
            _readbuf_count = n;
            _readbuf_ofs = 0;
        }
	}
	if (_readbuf_count == 0) {
		return -1;
	}
	c = _readbuf[_readbuf_ofs];
	_readbuf_ofs++;
	_readbuf_count--;
	return c;
}

/* PX4 implementations of Print virtual methods */
size_t PX4UARTDriver::write(uint8_t c) 
{ 
	if (!_initialised) {
		return 0;
	}
	int ret = ::write(_fd, &c, 1);
    if (ret == -1) {
        ret = 0;
    }
    return ret;
}

size_t PX4UARTDriver::write(const uint8_t *buffer, size_t size)
{
	if (!_initialised) {
		return 0;
	}
	int ret = ::write(_fd, buffer, size);
    if (ret == -1) {
        ret = 0;
    }
    return ret;
}

// handle %S -> %s
void PX4UARTDriver::_vdprintf(int fd, const char *fmt, va_list ap)
{
	if (strstr(fmt, "%S")) {
		char *fmt2 = strdup(fmt);
		if (fmt2 != NULL) {
			for (uint16_t i=0; fmt2[i]; i++) {
				if (fmt2[i] == '%' && fmt2[i+1] == 'S') {
					fmt2[i+1] = 's';
				}
			}
			vdprintf(fd, fmt2, ap);
			free(fmt2);
		}
	} else {
		vdprintf(fd, fmt, ap);
	}	
}

#endif // CONFIG_HAL_BOARD

