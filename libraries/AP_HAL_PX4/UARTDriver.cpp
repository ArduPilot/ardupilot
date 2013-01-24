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

PX4UARTDriver::PX4UARTDriver(const char *devpath, const char *perf_name) :
	_devpath(devpath),
    _perf_uart(perf_alloc(PC_ELAPSED, perf_name))
{}


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

        // always set it non-blocking for the low level IO
        unsigned v;
        v = fcntl(_fd, F_GETFL, 0);
        fcntl(_fd, F_SETFL, v | O_NONBLOCK);

		_initialised = true;
        if (rxS == 0) {
            rxS = 128;
        }
        if (txS == 0) {
            txS = 128;
        }
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

    /*
      allocate the read buffer
     */
	if (rxS != 0 && rxS != _readbuf_size) {
        _initialised = false;
        while (_in_timer) hal.scheduler->delay(1);
		_readbuf_size = rxS;
		if (_readbuf != NULL) {
			free(_readbuf);
		}
		_readbuf = (uint8_t *)malloc(_readbuf_size);
		_readbuf_head = 0;
		_readbuf_tail = 0;
        _initialised = true;
	}

    /*
      allocate the write buffer
     */
	if (txS != 0 && txS != _writebuf_size) {
        _initialised = false;
        while (_in_timer) hal.scheduler->delay(1);
		_writebuf_size = txS;
		if (_writebuf != NULL) {
			free(_writebuf);
		}
		_writebuf = (uint8_t *)malloc(_writebuf_size);
		_writebuf_head = 0;
		_writebuf_tail = 0;
        _initialised = true;
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
    _vprintf(fmt, ap);
    va_end(ap);	
}

void PX4UARTDriver::_printf_P(const prog_char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _vprintf(fmt, ap);
    va_end(ap);	
}

void PX4UARTDriver::vprintf(const char *fmt, va_list ap) {
    _vprintf(fmt, ap);
}

void PX4UARTDriver::vprintf_P(const prog_char *fmt, va_list ap) {
    _vprintf(fmt, ap);
}


void PX4UARTDriver::_internal_vprintf(const char *fmt, va_list ap)
{
    if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
        return;
    }
    char *buf = NULL;
    int n = avsprintf(&buf, fmt, ap);
    if (n > 0) {
        write((const uint8_t *)buf, n);
    }
    if (buf != NULL) {
        free(buf);    
    }
}

// handle %S -> %s
void PX4UARTDriver::_vprintf(const char *fmt, va_list ap)
{
    if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
        return;
    }
    // we don't use vdprintf() as it goes directly to the file descriptor
	if (strstr(fmt, "%S")) {
		char *fmt2 = strdup(fmt);
		if (fmt2 != NULL) {
			for (uint16_t i=0; fmt2[i]; i++) {
				if (fmt2[i] == '%' && fmt2[i+1] == 'S') {
					fmt2[i+1] = 's';
				}
			}
            _internal_vprintf(fmt2, ap);
			free(fmt2);
		}
	} else {
        _internal_vprintf(fmt, ap);
	}	
}


/*
  buffer handling macros
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(buf##_tail - _head) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size

/*
  return number of bytes available to be read from the buffer
 */
int16_t PX4UARTDriver::available() 
{ 
	if (!_initialised) {
		return 0;
	}
    uint16_t _tail;
    return BUF_AVAILABLE(_readbuf);
}

/*
  return number of bytes that can be added to the write buffer
 */
int16_t PX4UARTDriver::txspace() 
{ 
	if (!_initialised) {
		return 0;
	}
    uint16_t _head;
    return BUF_SPACE(_writebuf);
}

/*
  read one byte from the read buffer
 */
int16_t PX4UARTDriver::read() 
{ 
	uint8_t c;
	if (!_initialised || _readbuf == NULL) {
		return -1;
	}
    if (BUF_EMPTY(_readbuf)) {
        return -1;
    }
    c = _readbuf[_readbuf_head];
    BUF_ADVANCEHEAD(_readbuf, 1);
	return c;
}

/* 
   write one byte to the buffer
 */
size_t PX4UARTDriver::write(uint8_t c) 
{ 
    if (!_initialised) {
        return 0;
    }
    if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
        return 0;
    }
    uint16_t _head;
    while (BUF_SPACE(_writebuf) == 0) {
        if (_nonblocking_writes) {
            return 0;
        }
        hal.scheduler->delay(1);
    }
    _writebuf[_writebuf_tail] = c;
    BUF_ADVANCETAIL(_writebuf, 1);
    return 1;
}

/*
  write size bytes to the write buffer
 */
size_t PX4UARTDriver::write(const uint8_t *buffer, size_t size)
{
	if (!_initialised) {
		return 0;
	}
    if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
        return 0;
    }
    uint16_t _head, space;
    space = BUF_SPACE(_writebuf);
    if (space == 0) {
        return 0;
    }
    if (size > space) {
        size = space;
    }
    if (_writebuf_tail < _head) {
        // perform as single memcpy
        memcpy(&_writebuf[_writebuf_tail], buffer, size);
        BUF_ADVANCETAIL(_writebuf, size);
        return size;
    }

    // perform as two memcpy calls
    uint16_t n = _writebuf_size - _writebuf_tail;
    if (n > size) n = size;
    memcpy(&_writebuf[_writebuf_tail], buffer, n);
    BUF_ADVANCETAIL(_writebuf, n);
    buffer += n;
    n = size - n;
    if (n > 0) {
        memcpy(&_writebuf[_writebuf_tail], buffer, n);
        BUF_ADVANCETAIL(_writebuf, n);
    }        
    return size;
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously. 
 */
void PX4UARTDriver::_timer_tick(void)
{
    uint16_t n;

    if (!_initialised) return;

    _in_timer = true;

    // write any pending bytes
    uint16_t _tail;
    n = BUF_AVAILABLE(_writebuf);
    if (n > 0) {
        perf_begin(_perf_uart);
        if (_tail > _writebuf_head) {
            // do as a single write
            int ret = ::write(_fd, &_writebuf[_writebuf_head], n);
            if (ret > 0) {
                BUF_ADVANCEHEAD(_writebuf, ret);
            }
        } else {
            // split into two writes
            uint16_t n1 = _writebuf_size - _writebuf_head;
            int ret = ::write(_fd, &_writebuf[_writebuf_head], n1);
            if (ret > 0) {
                BUF_ADVANCEHEAD(_writebuf, ret);
            }
            if (ret == n1 && n != n1) {
                ret = ::write(_fd, &_writebuf[_writebuf_head], n - n1);                
                if (ret > 0) {
                    BUF_ADVANCEHEAD(_writebuf, ret);
                }
            }
        }
        perf_end(_perf_uart);
    }

    // try to fill the read buffer
    uint16_t _head;
    n = BUF_SPACE(_readbuf);
    if (n > 0) {
        perf_begin(_perf_uart);
        if (_readbuf_tail < _head) {
            // one read will do
            int ret = ::read(_fd, &_readbuf[_readbuf_tail], n);
            if (ret > 0) {
                BUF_ADVANCETAIL(_readbuf, ret);
            }
        } else {
            uint16_t n1 = _readbuf_size - _readbuf_tail;
            int ret = ::read(_fd, &_readbuf[_readbuf_tail], n1);
            if (ret > 0) {
                BUF_ADVANCETAIL(_readbuf, ret);
            }
            if (ret == n1 && n != n1) {
                ret = ::read(_fd, &_readbuf[_readbuf_tail], n - n1);                
                if (ret > 0) {
                    BUF_ADVANCETAIL(_readbuf, ret);
                }
            }
        }
        perf_end(_perf_uart);
    }

    _in_timer = false;
}

#endif // CONFIG_HAL_BOARD

