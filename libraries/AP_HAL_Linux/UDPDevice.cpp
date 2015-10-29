#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "UDPDevice.h"

UDPDevice::UDPDevice(const char *ip, uint16_t port, bool bcast):
    _ip(ip),
    _port(port),
    _bcast(bcast)
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
    if (_connected) {
        return socket.send(buf, n);
    }
    return socket.sendto(buf, n, _ip, _port);
}

ssize_t UDPDevice::read(uint8_t *buf, uint16_t n)
{
    ssize_t ret = socket.recv(buf, n, 0);
    if (!_connected && ret > 0) {
        const char *ip;
        uint16_t port;
        socket.last_recv_address(ip, port);
        _connected = socket.connect(ip, port);
    }
    return ret;
}

bool UDPDevice::open()
{
    if (_bcast) {
        // open now, then connect on first received packet
        socket.set_broadcast();
        return true;
    }
    _connected = socket.connect(_ip, _port);
    return _connected;
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
