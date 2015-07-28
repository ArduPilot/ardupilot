#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>

#include "TCPServerDevice.h"

TCPServerDevice::TCPServerDevice(const char *ip, uint16_t port):
    _ip(ip),
    _port(port)
{
}

TCPServerDevice::~TCPServerDevice()
{

}

ssize_t TCPServerDevice::write(const uint8_t *buf, uint16_t n)
{
    return listener.send(buf, n);
}

ssize_t TCPServerDevice::read(uint8_t *buf, uint16_t n)
{
    return listener.recv(buf, n, 1);
}

bool TCPServerDevice::open()
{
    listener.reuseaddress();

    if (!listener.connect(_ip, _port)) {
        ::printf("connect failed on %s port %u - %s\n",
                 _ip,
                 _port,
                 strerror(errno));
        ::exit(1);
    }

    listener.set_blocking(false);

    return true;
}

bool TCPServerDevice::close()
{
    return true;
}

void TCPServerDevice::set_blocking(bool blocking)
{
    listener.set_blocking(blocking);
}

void TCPServerDevice::set_speed(uint32_t speed)
{

}

#endif
