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
#include <drivers/drv_hrt.h>
#include <assert.h>

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
        uint8_t retries = 0;
        while (retries < 5) {
            _fd = open(_devpath, O_RDWR);
            if (_fd != -1) {
                break;
            }
            // sleep a bit and retry. There seems to be a NuttX bug
            // that can cause ttyACM0 to not be available immediately,
            // but a small delay can fix it
            hal.scheduler->delay(100);
            retries++;
        }
		if (_fd == -1) {
			fprintf(stdout, "Failed to open UART device %s - %s\n",
				_devpath, strerror(errno));
			return;
		}
		if (retries != 0) {
			fprintf(stdout, "WARNING: took %u retries to open UART %s\n", 
                    (unsigned)retries, _devpath);
			return;
		}

        if (rxS == 0) {
            rxS = 128;
        }
        // on PX4 we have enough memory to have a larger transmit
        // buffer for all ports. This means we don't get delays while
        // waiting to write GPS config packets
        if (txS < 512) {
            txS = 512;
        }
	}

    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);

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
		_readbuf_size = rxS;
		if (_readbuf != NULL) {
			free(_readbuf);
		}
		_readbuf = (uint8_t *)malloc(_readbuf_size);
		_readbuf_head = 0;
		_readbuf_tail = 0;
	}

    /*
      allocate the write buffer
     */
	if (txS != 0 && txS != _writebuf_size) {
		_writebuf_size = txS;
		if (_writebuf != NULL) {
			free(_writebuf);
		}
		_writebuf = (uint8_t *)malloc(_writebuf_size+16);
		_writebuf_head = 0;
		_writebuf_tail = 0;
	}

    if (_writebuf_size != 0 && _readbuf_size != 0) {
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

/*
  buffer handling macros
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
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

    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
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
        assert(_writebuf_tail+size <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], buffer, size);
        BUF_ADVANCETAIL(_writebuf, size);
        return size;
    }

    // perform as two memcpy calls
    uint16_t n = _writebuf_size - _writebuf_tail;
    if (n > size) n = size;
    assert(_writebuf_tail+n <= _writebuf_size);
    memcpy(&_writebuf[_writebuf_tail], buffer, n);
    BUF_ADVANCETAIL(_writebuf, n);
    buffer += n;
    n = size - n;
    if (n > 0) {
        assert(_writebuf_tail+n <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], buffer, n);
        BUF_ADVANCETAIL(_writebuf, n);
    }        
    return size;
}

/*
  try writing n bytes, handling an unresponsive port
 */
int PX4UARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    int ret = 0;

    // the FIONWRITE check is to cope with broken O_NONBLOCK behaviour
    // in NuttX on ttyACM0
    int nwrite = 0;
    if (ioctl(_fd, FIONWRITE, (unsigned long)&nwrite) == 0) {
        if (nwrite > n) {
            nwrite = n;
        }
        if (nwrite > 0) {
            ret = ::write(_fd, buf, nwrite);
        }
    }

    if (ret > 0) {
        BUF_ADVANCEHEAD(_writebuf, ret);
        _last_write_time = hrt_absolute_time();
        return ret;
    }

    if (hrt_absolute_time() - _last_write_time > 2000) {
#if 0
        // this trick is disabled for now, as it sometimes blocks on
        // re-opening the ttyACM0 port, which would cause a crash
        if (hrt_absolute_time() - _last_write_time > 2000000) {
            // we haven't done a successful write for 2 seconds - try
            // reopening the port        
            _initialised = false;
            ::close(_fd);
            _fd = ::open(_devpath, O_RDWR);
            if (_fd == -1) {
                fprintf(stdout, "Failed to reopen UART device %s - %s\n",
                        _devpath, strerror(errno));
                // leave it uninitialised
                return n;
            }
            
            _last_write_time = hrt_absolute_time();
            _initialised = true;
        }
#else
        _last_write_time = hrt_absolute_time();
#endif
        // we haven't done a successful write for 2ms, which means the 
        // port is running at less than 500 bytes/sec. Start
        // discarding bytes, even if this is a blocking port. This
        // prevents the ttyACM0 port blocking startup if the endpoint
        // is not connected
        BUF_ADVANCEHEAD(_writebuf, n);
        return n;
    }
    return ret;
}

/*
  try reading n bytes, handling an unresponsive port
 */
int PX4UARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    int ret = 0;

    // the FIONREAD check is to cope with broken O_NONBLOCK behaviour
    // in NuttX on ttyACM0
    int nread = 0;
    if (ioctl(_fd, FIONREAD, (unsigned long)&nread) == 0) {
        if (nread > n) {
            nread = n;
        }
        if (nread > 0) {
            ret = ::read(_fd, buf, nread);
        }
    }
    if (ret > 0) {
        BUF_ADVANCETAIL(_readbuf, ret);
    }
    return ret;
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
            _write_fd(&_writebuf[_writebuf_head], n);
        } else {
            // split into two writes
            uint16_t n1 = _writebuf_size - _writebuf_head;
            int ret = _write_fd(&_writebuf[_writebuf_head], n1);
            if (ret == n1 && n != n1) {
                _write_fd(&_writebuf[_writebuf_head], n - n1);                
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
            assert(_readbuf_tail+n <= _readbuf_size);
            _read_fd(&_readbuf[_readbuf_tail], n);
        } else {
            uint16_t n1 = _readbuf_size - _readbuf_tail;
            assert(_readbuf_tail+n1 <= _readbuf_size);
            int ret = _read_fd(&_readbuf[_readbuf_tail], n1);
            if (ret == n1 && n != n1) {
                assert(_readbuf_tail+(n-n1) <= _readbuf_size);
                _read_fd(&_readbuf[_readbuf_tail], n - n1);                
            }
        }
        perf_end(_perf_uart);
    }

    _in_timer = false;
}

#endif // CONFIG_HAL_BOARD

