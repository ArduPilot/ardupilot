#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>

#include "TCPServerDevice.h"

extern const AP_HAL::HAL& hal;

TCPServerDevice::TCPServerDevice(const char *ip, uint16_t port, bool wait):
    _ip(ip),
    _port(port),
    _wait(wait)
{
}

TCPServerDevice::~TCPServerDevice()
{
    if (sock != NULL) {
        delete sock;
        sock = NULL;
    }
}

ssize_t TCPServerDevice::write(const uint8_t *buf, uint16_t n)
{
    if (sock == NULL) {
        return -1;
    }
    return sock->send(buf, n);
}

/*
  when we try to read we accept new connections if one isn't already
  established
 */
ssize_t TCPServerDevice::read(uint8_t *buf, uint16_t n)
{
    if (sock == NULL) {
        sock = listener.accept(0);
        if (sock != NULL) {
            sock->set_blocking(_blocking);
        }
    }
    if (sock == NULL) {
        return -1;
    }
    ssize_t ret = sock->recv(buf, n, 1);
    if (ret == 0) {
        // EOF, go back to waiting for a new connection
        delete sock;
        sock = NULL;
        return -1;
    }
    return ret;
}

bool TCPServerDevice::open()
{
    listener.reuseaddress();

    if (!listener.bind(_ip, _port)) {
        if (AP_HAL::millis() - _last_bind_warning > 5000) {
            ::printf("bind failed on %s port %u - %s\n",
                     _ip,
                     _port,
                     strerror(errno));
            _last_bind_warning = AP_HAL::millis();
        }
        return false;
    }

    if (!listener.listen(1)) {
        if (AP_HAL::millis() - _last_bind_warning > 5000) {
            ::printf("listen failed on %s port %u - %s\n",
                     _ip,
                     _port,
                     strerror(errno));
            _last_bind_warning = AP_HAL::millis();
        }
        return false;
    }

    listener.set_blocking(false);

    if (_wait) {
        ::printf("Waiting for connection on %s:%u ....\n",
                 _ip, (unsigned)_port);
        ::fflush(stdout);
        while (sock == NULL) {
            sock = listener.accept(1000);
        }
        sock->set_blocking(_blocking);
        ::printf("connected\n");
        ::fflush(stdout);
    }

    return true;
}

bool TCPServerDevice::close()
{
    if (sock != NULL) {
        delete sock;
        sock = NULL;
    }
    return true;
}

void TCPServerDevice::set_blocking(bool blocking)
{
    _blocking = blocking;
    listener.set_blocking(_blocking);
}

void TCPServerDevice::set_speed(uint32_t speed)
{
}

#endif
