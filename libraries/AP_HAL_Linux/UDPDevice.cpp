#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "UDPDevice.h"

UDPDevice::UDPDevice(const char *ip, uint16_t port):
    _ip(ip),
    _port(port)
{
}

UDPDevice::~UDPDevice()
{

}

ssize_t UDPDevice::write(const uint8_t *buf, uint16_t n)
{
    if (!socket.pollout(0)) {
        return -1;
    }
    return socket.send(buf, n);
}

ssize_t UDPDevice::read(uint8_t *buf, uint16_t n)
{
    return socket.recv(buf, n, 0);
}

bool UDPDevice::open()
{
    return socket.connect(_ip, _port);
}

bool UDPDevice::close()
{
    return true;
}

void UDPDevice::set_blocking(bool blocking)
{
    socket.set_blocking(blocking);
}

void UDPDevice::set_speed(uint32_t speed)
{

}

#endif
