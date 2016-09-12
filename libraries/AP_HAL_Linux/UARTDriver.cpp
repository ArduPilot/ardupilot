// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: -*- nil -*-
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
#include <AP_HAL/utility/RingBuffer.h>

#include "ConsoleDevice.h"
#include "TCPServerDevice.h"
#include "UARTDevice.h"
#include "UARTQFlight.h"
#include "UDPDevice.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

UARTDriver::UARTDriver(bool default_console) :
    device_path(NULL),
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
        if (device_path == NULL && _console) {
            _device = new ConsoleDevice();
        } else {
            if (device_path == NULL) {
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

    _allocate_buffers(rxS, txS);
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
        _writebuf = (uint8_t *)malloc(_writebuf_size);
        _writebuf_head = 0;
        _writebuf_tail = 0;
    }

    if (_writebuf_size != 0 && _readbuf_size != 0) {
        _initialised = true;
    }
}

void UARTDriver::_deallocate_buffers()
{
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
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT
    } else if (strncmp(arg, "qflight:", 8) == 0) {
        return AP_HAL::OwnPtr<SerialDevice>(new QFLIGHTDevice(device_path));
#endif
    } else if (strncmp(arg, "tcp:", 4) != 0 &&
               strncmp(arg, "udp:", 4) != 0 &&
               strncmp(arg, "udpin:", 6)) {
        return nullptr;
    }

    char *devstr = strdup(arg);

    if (devstr == NULL) {
        return nullptr;
    }

    char *saveptr = NULL;
    char *protocol, *ip, *port, *flag;

    protocol = strtok_r(devstr, ":", &saveptr);
    ip = strtok_r(NULL, ":", &saveptr);
    port = strtok_r(NULL, ":", &saveptr);
    flag = strtok_r(NULL, ":", &saveptr);

    if (ip == NULL || port == NULL) {
        free(devstr);
        return nullptr;
    }

    if (_ip) {
        free(_ip);
        _ip = NULL;
    }

    if (_flag) {
        free(_flag);
        _flag = NULL;
    }

    _base_port = (uint16_t) atoi(port);
    _ip = strdup(ip);

    /* Optional flag for TCP */
    if (flag != NULL) {
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
    return !BUF_EMPTY(_writebuf);
}

/*
  return the number of bytes available to be read
 */
uint32_t UARTDriver::available()
{
    if (!_initialised) {
        return 0;
    }
    uint16_t _tail;
    return BUF_AVAILABLE(_readbuf);
}

/*
  how many bytes are available in the output buffer?
 */
uint32_t UARTDriver::txspace()
{
    if (!_initialised) {
        return 0;
    }
    uint16_t _head;
    return BUF_SPACE(_writebuf);
}

int16_t UARTDriver::read()
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

/* Linux implementations of Print virtual methods */
size_t UARTDriver::write(uint8_t c)
{
    if (!_initialised) {
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
size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
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
int UARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    int ret = 0;

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

    ret = _device->write(buf, n);

    if (ret > 0) {
        BUF_ADVANCEHEAD(_writebuf, ret);
        return ret;
    }

    return ret;
}

/*
  try reading n bytes, handling an unresponsive port
 */
int UARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    int ret;

    ret = _device->read(buf, n);

    if (ret > 0) {
        BUF_ADVANCETAIL(_readbuf, ret);
    }

    return ret;
}


/*
  try to push out one lump of pending bytes
  return true if progress is made
 */
bool UARTDriver::_write_pending_bytes(void)
{
    uint16_t n;

    // write any pending bytes
    uint16_t _tail;
    uint16_t available_bytes = BUF_AVAILABLE(_writebuf);
    n = available_bytes;
    if (_packetise && n > 0 &&
        (_writebuf[_writebuf_head] != MAVLINK_STX_MAVLINK1 &&
         _writebuf[_writebuf_head] != MAVLINK_STX)) {
        /*
          we have a non-mavlink packet at the start of the
          buffer. Look ahead for a MAVLink start byte, up to 256 bytes
          ahead
         */
        uint16_t limit = n>256?256:n;
        uint16_t i;
        for (i=0; i<limit; i++) {
            uint8_t b = _writebuf[(_writebuf_head + i) % _writebuf_size];
            if (b == MAVLINK_STX_MAVLINK1 || b == MAVLINK_STX) {
                n = i;
                break;
            }
        }
        // if we didn't find a MAVLink marker then limit the send size to 256
        if (i == limit) {
            n = limit;
        }
    }
    const uint8_t b = _writebuf[_writebuf_head];
    if (_packetise && n > 0 &&
        (b == MAVLINK_STX_MAVLINK1 || b == MAVLINK_STX)) {
        uint8_t min_length = (b == MAVLINK_STX_MAVLINK1)?8:12;
        // this looks like a MAVLink packet - try to write on
        // packet boundaries when possible
        if (n < min_length) {
            // we need to wait for more data to arrive
            n = 0;
        } else {
            // the length of the packet is the 2nd byte, and mavlink
            // packets have a 6 byte header plus 2 byte checksum,
            // giving len+8 bytes
            uint8_t len = _writebuf[(_writebuf_head + 1) % _writebuf_size];
            if (b == MAVLINK_STX) {
                // check for signed packet with extra 13 bytes
                uint8_t incompat_flags = _writebuf[(_writebuf_head + 2) % _writebuf_size];
                if (incompat_flags & MAVLINK_IFLAG_SIGNED) {
                    min_length += MAVLINK_SIGNATURE_BLOCK_LEN;
                }
            }
            if (n < len+min_length) {
                // we don't have a full packet yet
                n = 0;
            } else if (n > len+min_length) {
                // send just 1 packet at a time (so MAVLink packets
                // are aligned on UDP boundaries)
                n = len+min_length;
            }
        }
    }

    if (n > 0) {
        uint16_t n1 = _writebuf_size - _writebuf_head;
        if (n1 >= n) {
            // do as a single write
            _write_fd(&_writebuf[_writebuf_head], n);
        } else {
            // split into two writes
            if (_packetise) {
                // keep as a single UDP packet
                uint8_t tmpbuf[n];
                memcpy(tmpbuf, &_writebuf[_writebuf_head], n1);
                if (n > n1) {
                    memcpy(&tmpbuf[n1], &_writebuf[0], n-n1);
                }
                _write_fd(tmpbuf, n);
            } else {
                int ret = _write_fd(&_writebuf[_writebuf_head], n1);
                if (ret == n1 && n > n1) {
                    _write_fd(&_writebuf[_writebuf_head], n - n1);
                }
            }
        }
    }

    return BUF_AVAILABLE(_writebuf) != available_bytes;
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UARTDriver::_timer_tick(void)
{
    uint16_t n;

    if (!_initialised) return;

    _in_timer = true;

    uint8_t num_send = 10;
    while (num_send != 0 && _write_pending_bytes()) {
        num_send--;
    }

    // try to fill the read buffer
    uint16_t _head;
    n = BUF_SPACE(_readbuf);
    if (n > 0) {
        uint16_t n1 = _readbuf_size - _readbuf_tail;
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
    }

    _in_timer = false;
}
