#include "UARTDriver.h"

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

#include "ConsoleDevice.h"
#include "TCPServerDevice.h"
#include "UARTDevice.h"
#include "UDPDevice.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/packetise.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

UARTDriver::UARTDriver(bool default_console) :
    device_path(nullptr),
    _packetise(false),
    _device{new ConsoleDevice()}
{
    if (default_console) {
        _console = true;
    }
}

/*
  set the tty device to use for this UART
 */
void UARTDriver::set_device_path(const char *path)
{
    device_path = path;
}

/*
  open the tty
 */
void UARTDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (!_initialised) {
        if (device_path == nullptr && _console) {
            _device = new ConsoleDevice();
        } else {
            if (device_path == nullptr) {
                return;
            }

            _device = _parseDevicePath(device_path);

            if (!_device.get()) {
                ::fprintf(stderr, "Argument is not valid. Fallback to console.\n"
                          "Launch with --help to see an example.\n");
                _device = new ConsoleDevice();
            }
        }
    }

    if (!_connected) {
        _connected = _device->open();
        _device->set_blocking(false);
    }
    _initialised = false;

    while (_in_timer) hal.scheduler->delay(1);

    _device->set_speed(b);

    bool clear_buffers = false;
    if (b != 0) {
        if (_baudrate != b && hal.console != this) {
            clear_buffers = true;
        }
        _baudrate = b;
    }

    _allocate_buffers(rxS, txS);

    if (clear_buffers) {
        _readbuf.clear();
        _writebuf.clear();
    }
}

void UARTDriver::_allocate_buffers(uint16_t rxS, uint16_t txS)
{
    /* we have enough memory to have a larger transmit buffer for
     * all ports. This means we don't get delays while waiting to
     * write GPS config packets
     */

    if (rxS < 8192) {
        rxS = 8192;
    }
    if (txS < 32000) {
        txS = 32000;
    }

    if (_writebuf.set_size(txS) && _readbuf.set_size(rxS)) {
        _initialised = true;
    }
}

void UARTDriver::_deallocate_buffers()
{
    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

/*
    Device path accepts the following syntaxes:
        - /dev/ttyO1
        - tcp:*:1243:wait
        - udp:192.168.2.15:1243
*/
AP_HAL::OwnPtr<SerialDevice> UARTDriver::_parseDevicePath(const char *arg)
{
    struct stat st;

    if (stat(arg, &st) == 0 && S_ISCHR(st.st_mode)) {
        return AP_HAL::OwnPtr<SerialDevice>(new UARTDevice(arg));
    } else if (strncmp(arg, "tcp:", 4) != 0 &&
               strncmp(arg, "udp:", 4) != 0 &&
               strncmp(arg, "udpin:", 6)) {
        return nullptr;
    }

    char *devstr = strdup(arg);

    if (devstr == nullptr) {
        return nullptr;
    }

    char *saveptr = nullptr;
    char *protocol, *ip, *port, *flag;

    protocol = strtok_r(devstr, ":", &saveptr);
    ip = strtok_r(nullptr, ":", &saveptr);
    port = strtok_r(nullptr, ":", &saveptr);
    flag = strtok_r(nullptr, ":", &saveptr);

    if (ip == nullptr || port == nullptr) {
        free(devstr);
        return nullptr;
    }

    if (_ip) {
        free(_ip);
        _ip = nullptr;
    }

    if (_flag) {
        free(_flag);
        _flag = nullptr;
    }

    _base_port = (uint16_t) atoi(port);
    _ip = strdup(ip);

    /* Optional flag for TCP */
    if (flag != nullptr) {
        _flag = strdup(flag);
    }

    AP_HAL::OwnPtr<SerialDevice> device = nullptr;

    if (strcmp(protocol, "udp") == 0 || strcmp(protocol, "udpin") == 0) {
        bool bcast = (_flag && strcmp(_flag, "bcast") == 0);
        _packetise = true;
        if (strcmp(protocol, "udp") == 0) {
            device = new UDPDevice(_ip, _base_port, bcast, false);
        } else {
            if (bcast) {
                AP_HAL::panic("Can't combine udpin with bcast");
            }
            device = new UDPDevice(_ip, _base_port, false, true);

        }
    } else {
        bool wait = (_flag && strcmp(_flag, "wait") == 0);
        device = new TCPServerDevice(_ip, _base_port, wait);
    }

    free(devstr);
    return device;
}

/*
  shutdown a UART
 */
void UARTDriver::end()
{
    _initialised = false;
    _connected = false;

    while (_in_timer) {
        hal.scheduler->delay(1);
    }

    _device->close();
    _deallocate_buffers();
}


void UARTDriver::flush()
{
    // we are not doing any buffering, so flush is a no-op
}


/*
  return true if the UART is initialised
 */
bool UARTDriver::is_initialized()
{
    return _initialised;
}


/*
  enable or disable blocking writes
 */
void UARTDriver::set_blocking_writes(bool blocking)
{
    _nonblocking_writes = !blocking;
}


/*
  do we have any bytes pending transmission?
 */
bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}

/*
  return the number of bytes available to be read
 */
uint32_t UARTDriver::available()
{
    if (!_initialised) {
        return 0;
    }
    return _readbuf.available();
}

/*
  how many bytes are available in the output buffer?
 */
uint32_t UARTDriver::txspace()
{
    if (!_initialised) {
        return 0;
    }
    return _writebuf.space();
}

int16_t UARTDriver::read()
{
    if (!_initialised) {
        return -1;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }

    return byte;
}

/* Linux implementations of Print virtual methods */
size_t UARTDriver::write(uint8_t c)
{
    if (!_initialised) {
        return 0;
    }
    if (!_write_mutex.take_nonblocking()) {
        return 0;
    }

    while (_writebuf.space() == 0) {
        if (_nonblocking_writes) {
            _write_mutex.give();
            return 0;
        }
        hal.scheduler->delay(1);
    }
    size_t ret = _writebuf.write(&c, 1);
    _write_mutex.give();
    return ret;
}

/*
  write size bytes to the write buffer
 */
size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
        return 0;
    }
    if (!_write_mutex.take_nonblocking()) {
        return 0;
    }
    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        _write_mutex.give();
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

/*
  try writing n bytes, handling an unresponsive port
 */
int UARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    /*
      allow for delayed connection. This allows ArduPilot to start
      before a network interface is available.
     */
    if (!_connected) {
        _connected = _device->open();
    }
    if (!_connected) {
        return 0;
    }

    return _device->write(buf, n);
}

/*
  try reading n bytes, handling an unresponsive port
 */
int UARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    return _device->read(buf, n);
}


/*
  try to push out one lump of pending bytes
  return true if progress is made
 */
bool UARTDriver::_write_pending_bytes(void)
{
    // write any pending bytes
    uint32_t available_bytes = _writebuf.available();
    uint16_t n = available_bytes;

    if (_packetise && n > 0) {
        // send on MAVLink packet boundaries if possible
        n = mavlink_packetise(_writebuf, n);
    }

    if (n > 0) {
        int ret;

        if (_packetise) {
            // keep as a single UDP packet
            uint8_t tmpbuf[n];
            _writebuf.peekbytes(tmpbuf, n);
            ret = _write_fd(tmpbuf, n);
            if (ret > 0)
                _writebuf.advance(ret);
        } else {
            ByteBuffer::IoVec vec[2];
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
        }
    }

    return _writebuf.available() != available_bytes;
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UARTDriver::_timer_tick(void)
{
    if (!_initialised) return;

    _in_timer = true;

    uint8_t num_send = 10;
    while (num_send != 0 && _write_pending_bytes()) {
        num_send--;
    }

    // try to fill the read buffer
    int ret;
    ByteBuffer::IoVec vec[2];

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

    _in_timer = false;
}

void UARTDriver::configure_parity(uint8_t v) {
    _device->set_parity(v);
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
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        // assume 10 bits per byte.
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes+available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}
