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
#include "../AP_HAL/utility/RingBuffer.h"

using namespace PX4;

extern const AP_HAL::HAL& hal;

PX4UARTDriver::PX4UARTDriver(const char *devpath, const char *perf_name) :
	_devpath(devpath),
    _fd(-1),
    _baudrate(57600),
    _perf_uart(perf_alloc(PC_ELAPSED, perf_name)),
    _initialised(false),
    _in_timer(false),
    _flow_control(FLOW_CONTROL_DISABLE)
{
}


extern const AP_HAL::HAL& hal;

/*
  this UART driver maps to a serial device in /dev
 */

void PX4UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
    if (strcmp(_devpath, "/dev/null") == 0) {
        // leave uninitialised
        return;
    }

    uint16_t min_tx_buffer = 1024;
    uint16_t min_rx_buffer = 512;
    if (strcmp(_devpath, "/dev/ttyACM0") == 0) {
        min_tx_buffer = 16384;
        min_rx_buffer = 1024;
    }
    // on PX4 we have enough memory to have a larger transmit and
    // receive buffer for all ports. This means we don't get delays
    // while waiting to write GPS config packets
    if (txS < min_tx_buffer) {
        txS = min_tx_buffer;
    }
    if (rxS < min_rx_buffer) {
        rxS = min_rx_buffer;
    }

    /*
      allocate the read buffer
      we allocate buffers before we successfully open the device as we
      want to allocate in the early stages of boot, and cause minimum
      thrashing of the heap once we are up. The ttyACM0 driver may not
      connect for some time after boot
     */
	if (rxS != 0 && rxS != _readbuf_size) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }
		_readbuf_size = rxS;
		if (_readbuf != NULL) {
			free(_readbuf);
		}
		_readbuf = (uint8_t *)malloc(_readbuf_size);
		_readbuf_head = 0;
		_readbuf_tail = 0;
	}

    if (b != 0) {
        _baudrate = b;
    }

    /*
      allocate the write buffer
     */
	if (txS != 0 && txS != _writebuf_size) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }
		_writebuf_size = txS;
		if (_writebuf != NULL) {
			free(_writebuf);
		}
		_writebuf = (uint8_t *)malloc(_writebuf_size+16);
		_writebuf_head = 0;
		_writebuf_tail = 0;
	}

	if (_fd == -1) {
        _fd = open(_devpath, O_RDWR);
		if (_fd == -1) {
			return;
		}

        // work out the OS write buffer size by looking at how many
        // bytes could be written when we first open the port
        int nwrite = 0;
        if (ioctl(_fd, FIONWRITE, (unsigned long)&nwrite) == 0) {
            _os_write_buffer_size = nwrite;
            if (_os_write_buffer_size & 1) {
                // it is reporting one short
                _os_write_buffer_size += 1;
            }
        }
	}

	if (_baudrate != 0) {
		// set the baud rate
		struct termios t;
		tcgetattr(_fd, &t);
		cfsetspeed(&t, _baudrate);
		// disable LF -> CR/LF
		t.c_oflag &= ~ONLCR;
		tcsetattr(_fd, TCSANOW, &t);

        // separately setup IFLOW if we can. We do this as a 2nd call
        // as if the port has no RTS pin then the tcsetattr() call
        // will fail, and if done as one call then it would fail to
        // set the baudrate.
		tcgetattr(_fd, &t);
		t.c_cflag |= CRTS_IFLOW;
		tcsetattr(_fd, TCSANOW, &t);

		// reset _total_written to reset flow control auto check
		_total_written = 0;
	}

    if (_writebuf_size != 0 && _readbuf_size != 0 && _fd != -1) {
        if (!_initialised) {
            ::printf("initialised %s OK %u %u\n", _devpath, 
                     (unsigned)_writebuf_size, (unsigned)_readbuf_size);
        }
        _initialised = true;
    }
    _uart_owner_pid = getpid();

}

void PX4UARTDriver::set_flow_control(enum flow_control fcontrol)
{
	if (_fd == -1) {
        return;
    }
    struct termios t;
    tcgetattr(_fd, &t);
    // we already enabled CRTS_IFLOW above, just enable output flow control
    if (fcontrol != FLOW_CONTROL_DISABLE) {
        t.c_cflag |= CRTSCTS;
    } else {
        t.c_cflag &= ~CRTSCTS;
    }
    tcsetattr(_fd, TCSANOW, &t);
    _flow_control = fcontrol;
}

void PX4UARTDriver::begin(uint32_t b) 
{
	begin(b, 0, 0);
}


/*
  try to initialise the UART. This is used to cope with the way NuttX
  handles /dev/ttyACM0 (the USB port). The port appears in /dev on
  boot, but cannot be opened until a USB cable is connected and the
  host starts the CDCACM communication.
 */
void PX4UARTDriver::try_initialise(void)
{
    if (_initialised) {
        return;
    }
    if ((hal.scheduler->millis() - _last_initialise_attempt_ms) < 2000) {
        return;
    }
    _last_initialise_attempt_ms = hal.scheduler->millis();
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED) {
        begin(0);
    }
}


void PX4UARTDriver::end() 
{
    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);
    if (_fd != -1) {
        close(_fd);
        _fd = -1;
    }
    if (_readbuf) {
        free(_readbuf);
        _readbuf = NULL;
    }
    if (_writebuf) {
        free(_writebuf);
        _writebuf = NULL;
    }
    _readbuf_size = _writebuf_size = 0;
    _writebuf_head = 0;
    _writebuf_tail = 0;
    _readbuf_head = 0;
    _readbuf_tail = 0;
}

void PX4UARTDriver::flush() {}

bool PX4UARTDriver::is_initialized() 
{ 
    try_initialise();
    return _initialised; 
}

void PX4UARTDriver::set_blocking_writes(bool blocking) 
{
    _nonblocking_writes = !blocking;
}

bool PX4UARTDriver::tx_pending() { return false; }

/*
  return number of bytes available to be read from the buffer
 */
int16_t PX4UARTDriver::available() 
{ 
	if (!_initialised) {
        try_initialise();
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
        try_initialise();
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
    if (_uart_owner_pid != getpid()){
        return -1;
    }
    if (!_initialised) {
        try_initialise();
        return -1;
    }
	if (_readbuf == NULL) {
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
    if (_uart_owner_pid != getpid()){
        return 0;
    }
    if (!_initialised) {
        try_initialise();
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
    if (_uart_owner_pid != getpid()){
        return 0;
    }
	if (!_initialised) {
        try_initialise();
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
        if (nwrite == 0 &&
            _flow_control == FLOW_CONTROL_AUTO &&
            _last_write_time != 0 &&
            _total_written != 0 &&
            _os_write_buffer_size == _total_written &&
            (hal.scheduler->micros64() - _last_write_time) > 500*1000UL) {
            // it doesn't look like hw flow control is working
            ::printf("disabling flow control on %s _total_written=%u\n", 
                     _devpath, (unsigned)_total_written);
            set_flow_control(FLOW_CONTROL_DISABLE);
        }
        if (nwrite > n) {
            nwrite = n;
        }
        if (nwrite > 0) {
            ret = ::write(_fd, buf, nwrite);
        }
    }

    if (ret > 0) {
        BUF_ADVANCEHEAD(_writebuf, ret);
        _last_write_time = hal.scheduler->micros64();
        _total_written += ret;
        return ret;
    }

    if (hal.scheduler->micros64() - _last_write_time > 2000 &&
        _flow_control == FLOW_CONTROL_DISABLE) {
#if 0
        // this trick is disabled for now, as it sometimes blocks on
        // re-opening the ttyACM0 port, which would cause a crash
        if (hal.scheduler->micros64() - _last_write_time > 2000000) {
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
            
            _last_write_time = hal.scheduler->micros64();
            _initialised = true;
        }
#else
        _last_write_time = hal.scheduler->micros64();
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
        _total_read += ret;
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

    // don't try IO on a disconnected USB port
    if (strcmp(_devpath, "/dev/ttyACM0") == 0 && !hal.gpio->usb_connected()) {
        return;
    }

    _in_timer = true;

    // write any pending bytes
    uint16_t _tail;
    n = BUF_AVAILABLE(_writebuf);
    if (n > 0) {
        uint16_t n1 = _writebuf_size - _writebuf_head;
        perf_begin(_perf_uart);
        if (n1 >= n) {
            // do as a single write
            _write_fd(&_writebuf[_writebuf_head], n);
        } else {
            // split into two writes
            int ret = _write_fd(&_writebuf[_writebuf_head], n1);
            if (ret == n1 && n > n1) {
                _write_fd(&_writebuf[_writebuf_head], n - n1);                
            }
        }
        perf_end(_perf_uart);
    }

    // try to fill the read buffer
    uint16_t _head;
    n = BUF_SPACE(_readbuf);
    if (n > 0) {
        uint16_t n1 = _readbuf_size - _readbuf_tail;
        perf_begin(_perf_uart);
        if (n1 >= n) {
            // one read will do
            assert(_readbuf_tail+n <= _readbuf_size);
            _read_fd(&_readbuf[_readbuf_tail], n);
        } else {
            assert(_readbuf_tail+n1 <= _readbuf_size);
            int ret = _read_fd(&_readbuf[_readbuf_tail], n1);
            if (ret == n1 && n > n1) {
                assert(_readbuf_tail+(n-n1) <= _readbuf_size);
                _read_fd(&_readbuf[_readbuf_tail], n - n1);                
            }
        }
        perf_end(_perf_uart);
    }

    _in_timer = false;
}

#endif // CONFIG_HAL_BOARD

