/*
  Generic UART relay driver for the ap_host process.
  Handles serial port open/configure/read/write and the receive thread.
  Uses termios2/ioctl for arbitrary baud rate support.
*/

#include "RemoteUARTDriver.h"
#include "protocol.h"
#include "slpi_link.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>

// Note: <termios.h> is intentionally NOT included here as it
// conflicts with <asm/termbits.h>. We use ioctl(TCGETS2/TCSETS2)
// with struct termios2 instead, which supports arbitrary baud rates.

RemoteUARTDriver::~RemoteUARTDriver()
{
    _close();
    if (_thread_running) {
        pthread_join(_recv_thread_id, nullptr);
        _thread_running = false;
    }
}

bool RemoteUARTDriver::_open()
{
    if (_fd != -1) {
        return true;
    }

    _fd = ::open(_device_path, O_RDWR | O_NOCTTY);
    if (_fd < 0) {
        _fd = -1;
        fprintf(stderr, "Remote UART: failed to open %s: %s\n",
                _device_path, strerror(errno));
        return false;
    }

    struct termios2 t {};
    if (ioctl(_fd, TCGETS2, &t) != 0) {
        fprintf(stderr, "Remote UART: TCGETS2 failed for %s: %s\n",
                _device_path, strerror(errno));
        _close();
        return false;
    }

    // raw mode (equivalent to cfmakeraw)
    t.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                    INLCR | IGNCR | ICRNL | IXON | IXOFF);
    t.c_oflag &= ~OPOST;
    t.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // 8N1, no flow control
    t.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    t.c_cflag |= CS8 | CLOCAL | CREAD;

    // non-blocking reads with small timeout
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 1;

    if (ioctl(_fd, TCSETS2, &t) != 0) {
        fprintf(stderr, "Remote UART: TCSETS2 failed for %s: %s\n",
                _device_path, strerror(errno));
        _close();
        return false;
    }

    // flush any stale data
    ioctl(_fd, TCFLSH, TCIOFLUSH);

    return true;
}

void RemoteUARTDriver::_close()
{
    if (_fd != -1) {
        ::close(_fd);
        _fd = -1;
    }
}

bool RemoteUARTDriver::_set_speed(uint32_t baudrate)
{
    struct termios2 t {};
    if (ioctl(_fd, TCGETS2, &t) != 0) {
        fprintf(stderr, "Remote UART: TCGETS2 failed for %s: %s\n",
                _device_path, strerror(errno));
        return false;
    }

    t.c_cflag &= ~CBAUD;
#if defined(BOTHER)
    t.c_cflag |= BOTHER;
#endif
    t.c_ispeed = baudrate;
    t.c_ospeed = baudrate;

    if (ioctl(_fd, TCSETS2, &t) != 0) {
        fprintf(stderr, "Remote UART: set_speed(%u) failed for %s: %s\n",
                baudrate, _device_path, strerror(errno));
        return false;
    }

    // flush after speed change
    ioctl(_fd, TCFLSH, TCIFLUSH);

    return true;
}

bool RemoteUARTDriver::configure(uint32_t baudrate, uint32_t device_id)
{
    if (is_open()) {
        if (device_id != _device_id) {
            fprintf(stderr, "Remote UART: device id change not supported "
                    "(current=%u, requested=%u)\n", _device_id, device_id);
            return false;
        }
        if (!_set_speed(baudrate)) {
            return false;
        }
        // printf("Remote UART: reconfigured baud=%u\n", baudrate);
        return true;
    }

    snprintf(_device_path, sizeof(_device_path), "/dev/ttyHS%u", device_id);
    _device_id = device_id;

    if (!_open()) {
        return false;
    }
    if (!_set_speed(baudrate)) {
        _close();
        return false;
    }

    printf("Remote UART: opened %s at baud=%u\n", _device_path, baudrate);
    _start_recv_thread();
    return true;
}

void RemoteUARTDriver::_start_recv_thread()
{
    if (_thread_running) {
        return;
    }
    pthread_create(&_recv_thread_id, nullptr, _recv_thread_fn, this);
    _thread_running = true;
}

void *RemoteUARTDriver::_recv_thread_fn(void *arg)
{
    static_cast<RemoteUARTDriver *>(arg)->_recv_loop();
    return nullptr;
}

void RemoteUARTDriver::_recv_loop()
{
    // printf("Remote UART: read thread started\n");

    while (is_open()) {
        struct qurt_rpc_msg msg {};
        ssize_t nread = ::read(_fd, msg.data, sizeof(msg.data));
        if (nread <= 0) {
            if (nread < 0 && errno != EINTR) {
                fprintf(stderr, "Remote UART: read failed: %s\n", strerror(errno));
            }
            continue;
        }

        msg.msg_id = QURT_MSG_ID_UART_DATA;
        msg.inst = _port_id;
        msg.data_length = nread;
        msg.seq = _rx_seq++;
        if (slpi_link_send((const uint8_t *)&msg, nread + QURT_RPC_MSG_HEADER_LEN)) {
            fprintf(stderr, "Remote UART: slpi_link_send failed\n");
        }
    }

    // printf("Remote UART: read thread exiting\n");
}

bool RemoteUARTDriver::write(const uint8_t *buf, uint16_t len)
{
    if (!is_open() || len == 0) {
        return false;
    }

    // All remote UART writes share the slpi_link callback thread,
    // so we cannot block here - a clogged device would stall every
    // other tunneled port and all other DSP to apps proc messaging.
    // Issue a single non-blocking write and drop any bytes that weren't written.
    // TODO: Look at other options like doing the writes in per port
    // threads that can do retries.
    const ssize_t written = ::write(_fd, buf, len);
    if (written == (ssize_t)len) {
        return true;
    }
    if (written < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            fprintf(stderr, "Remote UART: dropped %u bytes (buffer full)\n", len);
        } else {
            fprintf(stderr, "Remote UART: write failed: %s\n", strerror(errno));
        }
    } else {
        fprintf(stderr, "Remote UART: short write, dropped %u/%u bytes\n",
                len - (uint16_t)written, len);
    }
    return false;
}
