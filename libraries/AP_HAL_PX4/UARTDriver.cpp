#include <AP_HAL/AP_HAL.h>

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
#include "GPIO.h"

using namespace PX4;

extern const AP_HAL::HAL& hal;

PX4UARTDriver::PX4UARTDriver(const char *devpath, const char *perf_name) :
	_devpath(devpath),
    _fd(-1),
    _baudrate(57600),
    _initialised(false),
    _in_timer(false),
    _unbuffered_writes(false),
    _perf_uart(perf_alloc(PC_ELAPSED, perf_name)),
    _os_start_auto_space(-1),
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
        _is_usb = true;
        min_tx_buffer = 4096;
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
    while (_in_timer) {
        hal.scheduler->delay(1);
    }
    if (rxS != _readbuf.get_size()) {
        _initialised = false;
        _readbuf.set_size(rxS);
    }

    bool clear_buffers = false;
    if (b != 0) {
        // clear buffers on baudrate change, but not on the console (which is usually USB)
        if (_baudrate != b && hal.console != this) {
            clear_buffers = true;
        }
        _baudrate = b;
    }

    if (clear_buffers) {
      _readbuf.clear();
    }

    /*
      allocate the write buffer
     */
    while (_in_timer) {
        hal.scheduler->delay(1);
    }
    if (txS != _writebuf.get_size()) {
        _initialised = false;
        _writebuf.set_size(txS);
    }

    if (clear_buffers) {
      _writebuf.clear();
    }

	if (_fd == -1) {
        _fd = open(_devpath, O_RDWR);
		if (_fd == -1) {
			return;
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
	}

    if (_writebuf.get_size() && _readbuf.get_size() && _fd != -1) {
        if (!_initialised) {
            if (_is_usb) {
                ((PX4GPIO *)hal.gpio)->set_usb_connected();
            }
            ::printf("initialised %s OK %u %u\n", _devpath,
                     (unsigned)_writebuf.get_size(), (unsigned)_readbuf.get_size());
        }
        _initialised = true;
    }
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
    if (fcontrol == FLOW_CONTROL_AUTO) {
        // reset flow control auto state machine
        _total_written = 0;
        _first_write_time = 0;
    }
    _flow_control = fcontrol;
}

void PX4UARTDriver::configure_parity(uint8_t v) {
    if (_fd == -1) {
        return;
    }
    struct termios t;
    tcgetattr(_fd, &t);
    if (v != 0) {
        // enable parity
        t.c_cflag |= PARENB;
        if (v == 1) {
            t.c_cflag |= PARODD;
        } else {
            t.c_cflag &= ~PARODD;
        }
    }
    else {
        // disable parity
        t.c_cflag &= ~PARENB;
    }
    tcsetattr(_fd, TCSANOW, &t);
}

void PX4UARTDriver::set_stop_bits(int n) {
    if (_fd == -1) {
        return;
    }
    struct termios t;
    tcgetattr(_fd, &t);
    if (n > 1) t.c_cflag |= CSTOPB;
    else t.c_cflag &= ~CSTOPB;
    tcsetattr(_fd, TCSANOW, &t);
}

bool PX4UARTDriver::set_unbuffered_writes(bool on) {
    _unbuffered_writes = on;
    return _unbuffered_writes;
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
    if ((AP_HAL::millis() - _last_initialise_attempt_ms) < 2000) {
        return;
    }
    _last_initialise_attempt_ms = AP_HAL::millis();
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED || !hal.util->get_soft_armed()) {
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

    _readbuf.set_size(0);
    _writebuf.set_size(0);
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
uint32_t PX4UARTDriver::available()
{
    if (!_initialised) {
        try_initialise();
        return 0;
    }

    return _readbuf.available();
}

/*
  return number of bytes that can be added to the write buffer
 */
uint32_t PX4UARTDriver::txspace()
{
    if (!_initialised) {
        try_initialise();
        return 0;
    }

    return _writebuf.space();
}

/*
  read one byte from the read buffer
 */
int16_t PX4UARTDriver::read()
{
    if (!_semaphore.take_nonblocking()) {
        return -1;
    }
    if (!_initialised) {
        try_initialise();
        _semaphore.give();
        return -1;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        _semaphore.give();
        return -1;
    }

    _semaphore.give();
    return byte;
}

/*
   write one byte
 */
size_t PX4UARTDriver::write(uint8_t c)
{
    if (!_semaphore.take_nonblocking()) {
        return -1;
    }
    if (!_initialised) {
        try_initialise();
        _semaphore.give();
        return 0;
    }

    if (_unbuffered_writes) {
        // write one byte to the file descriptor
        return _write_fd(&c, 1);
    }

    while (_writebuf.space() == 0) {
        if (_nonblocking_writes) {
            _semaphore.give();
            return 0;
        }
        _semaphore.give();
        hal.scheduler->delay(1);
        if (!_semaphore.take_nonblocking()) {
            return -1;
        }
    }
    size_t ret = _writebuf.write(&c, 1);
    _semaphore.give();
    return ret;
}

/*
 * write size bytes
 */
size_t PX4UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_semaphore.take_nonblocking()) {
        return -1;
    }
    if (!_initialised) {
        try_initialise();
        _semaphore.give();
		return 0;
	}

    size_t ret = 0;
    
    if (!_nonblocking_writes) {
        _semaphore.give();
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    if (_unbuffered_writes) {
        // write buffer straight to the file descriptor
        ret = _write_fd(buffer, size);
    } else {
        ret = _writebuf.write(buffer, size);
    }
    _semaphore.give();
    return ret;
}

/*
  try writing n bytes, handling an unresponsive port
 */
int PX4UARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    int ret = 0;

    // the FIONWRITE check is to cope with broken O_NONBLOCK behaviour
    // in NuttX on ttyACM0

    // FIONWRITE is also used for auto flow control detection
    // Assume output flow control is not working if:
    //     port is configured for auto flow control
    // and this is not the first write since flow control turned on
    // and no data has been removed from the buffer since flow control turned on
    // and more than .5 seconds elapsed after writing a total of > 5 characters
    //

    int nwrite = 0;

    if (ioctl(_fd, FIONWRITE, (unsigned long)&nwrite) == 0) {
        if (_flow_control == FLOW_CONTROL_AUTO) {
            if (_first_write_time == 0) {
                if (_total_written == 0) {
                    // save the remaining buffer bytes for comparison next write
                    _os_start_auto_space = nwrite;
                }
            } else {
                if (_os_start_auto_space - nwrite + 1 >= _total_written &&
                    (AP_HAL::micros64() - _first_write_time) > 500*1000UL) {
                    // it doesn't look like hw flow control is working
                    ::printf("disabling flow control on %s _total_written=%u\n",
                             _devpath, (unsigned)_total_written);
                    set_flow_control(FLOW_CONTROL_DISABLE);
                }
            }
        }
        if (nwrite > n) {
            nwrite = n;
        }
        if (nwrite > 0) {
            ret = ::write(_fd, buf, nwrite);
        }
    }

    if (ret > 0) {
        _last_write_time = AP_HAL::micros64();
        _total_written += ret;
        if (! _first_write_time && _total_written > 5) {
            _first_write_time = _last_write_time;
        }
        return ret;
    }

    if (AP_HAL::micros64() - _last_write_time > 2000 &&
        _flow_control == FLOW_CONTROL_DISABLE) {
        _last_write_time = AP_HAL::micros64();

        // we haven't done a successful write for 2ms, which means the
        // port is running at less than 500 bytes/sec. Start
        // discarding bytes, even if this is a blocking port. This
        // prevents the ttyACM0 port blocking startup if the endpoint
        // is not connected
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
    int ret;
    uint32_t n;

    if (!_initialised) return;

    // don't try IO on a disconnected USB port
    if (_is_usb && !hal.gpio->usb_connected()) {
        return;
    }

    _in_timer = true;

    // write any pending bytes
    n = _writebuf.available();
    if (n > 0) {
        ByteBuffer::IoVec vec[2];
        perf_begin(_perf_uart);
        const auto n_vec = _writebuf.peekiovec(vec, n);
        for (int i = 0; i < n_vec; i++) {
            ret = _write_fd(vec[i].data, (uint16_t)vec[i].len);
            if (ret < 0) {
                break;
            }
            _writebuf.advance(ret);

            /* We wrote less than we asked for, stop */
            if ((unsigned)ret != vec[i].len) {
                break;
            }
        }
        perf_end(_perf_uart);
    }

    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];

    perf_begin(_perf_uart);
    const auto n_vec = _readbuf.reserve(vec, _readbuf.space());
    for (int i = 0; i < n_vec; i++) {
        ret = _read_fd(vec[i].data, vec[i].len);
        if (ret < 0) {
            break;
        }
        _readbuf.commit((unsigned)ret);

        // update receive timestamp
        _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
        _receive_timestamp_idx ^= 1;

        /* stop reading as we read less than we asked for */
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }
    perf_end(_perf_uart);

    _in_timer = false;
}

/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the uart. This should be treated as a
  time constraint, not an exact time. It is guaranteed that the
  packet did not start being received after this time, but it
  could have been in a system buffer before the returned time.
  
  This takes account of the baudrate of the link. For transports
  that have no baudrate (such as USB) the time estimate may be
  less accurate.
  
  A return value of zero means the HAL does not support this API
*/
uint64_t PX4UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0 && !_is_usb) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes+available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

#endif
