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

int TCPClientDevice::write(const uint8_t *buf, uint16_t n)
{
    return listener.send(buf, n);
}

int TCPClientDevice::read(uint8_t *buf, uint16_t n)
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

void TCPClientDevice::set_nonblocking()
{
    listener.set_blocking(false);
}

void TCPClientDevice::set_speed(uint32_t speed)
{

}

#endif
