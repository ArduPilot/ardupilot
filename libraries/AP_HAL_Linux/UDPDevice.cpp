#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdio.h>

#include "UDPDevice.h"

UDPDevice::UDPDevice(const char *ip, uint16_t port):
    _ip(ip),
    _port(port)
{
}

UDPDevice::~UDPDevice()
{

}

int UDPDevice::write(const uint8_t *buf, uint16_t n)
{
    return socket.sendto(buf, n, _ip, _port, 1);
}

int UDPDevice::read(uint8_t *buf, uint16_t n)
{
    return socket.recvfrom(buf, n, _ip, _port, 1);
}

bool UDPDevice::open()
{
    return true;
}

bool UDPDevice::close()
{
    return true;
}

void UDPDevice::set_nonblocking()
{
    socket.set_blocking(false);
}

void UDPDevice::set_speed(uint32_t speed)
{

}

#endif
