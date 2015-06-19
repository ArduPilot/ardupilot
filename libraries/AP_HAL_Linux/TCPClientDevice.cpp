#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>

#include "TCPClientDevice.h"

TCPClientDevice::TCPClientDevice(const char *ip, uint16_t port):
    _ip(ip),
    _port(port)
{
}

TCPClientDevice::~TCPClientDevice()
{

}

ssize_t TCPClientDevice::write(const uint8_t *buf, uint16_t n)
{
    return listener.send(buf, n);
}

ssize_t TCPClientDevice::read(uint8_t *buf, uint16_t n)
{
    return listener.recv(buf, n, 1);
}

bool TCPClientDevice::open()
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

bool TCPClientDevice::close()
{
    return true;
}

void TCPClientDevice::set_blocking(bool blocking)
{
    listener.set_blocking(blocking);
}

void TCPClientDevice::set_speed(uint32_t speed)
{

}

#endif
