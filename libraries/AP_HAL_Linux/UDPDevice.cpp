#include "UDPDevice.h"

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>

#include <AP_HAL/AP_HAL.h>

// UDP read() will give up after receiving this many packets
// including ones that are discarded (e.g. our own multicasts)
// Tune in hwdef.dat if necessary
#ifndef HAL_LINUX_MAX_UDP_PACKETS
    #define HAL_LINUX_MAX_UDP_PACKETS 20
#endif

UDPDevice::UDPDevice(const char *ip, uint16_t port, bool bcast, bool input):
    _ip(ip),
    _port(port),
    _bcast(bcast),
    _input(input)
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
    if (_input) {
        // can't send yet
        return -1;
    }
    return socket.sendto(buf, n, _ip, _port);
}

ssize_t UDPDevice::read(uint8_t *buf, uint16_t n)
{
    ssize_t bytes_read = 0;
    ssize_t packets_read = 0;

    // recv() only retrieves a single datagram on UDP sockets
    // peek & read multiple until the buffer full
    // poll each time because FIONREAD can't distinguish between
    // no data and empty datagrams
    while (bytes_read < n && socket.pollin(0) && packets_read < HAL_LINUX_MAX_UDP_PACKETS) {
        int in_size = 0;
        if (ioctl(socket.get_read_fd(), FIONREAD, &in_size) < 0) {
            return -1;
        }

        // read the first packet unconditionally
        // read subsequent packets if they fit into the remaining buffer
        // this ensures that if the incoming packet is somehow larger than
        // the entire RX ring-buffer, we get a truncated packet rather
        // than end up being unable to read any further
        // NOTE: Linux HAL allocates a 8192 byte RX buffer so this should
        // not happen unless the buffer size is redefined
        if (bytes_read > 0 && in_size > n - bytes_read) {
            break;
        }

        ssize_t ret = socket.recv(&buf[bytes_read], n - bytes_read, 0);
        if (!_connected && ret > 0) {
            const char *ip;
            uint16_t port;
            socket.last_recv_address(ip, port);
            _connected = socket.connect(ip, port);
        }

        // socket.read() may return -1 if we're reading ourselves on multicast
        if (ret > 0) {
            bytes_read += ret;
        }

        packets_read++;
    }

    if (bytes_read == 0) {
        return -1;
    }

    return bytes_read;
}

bool UDPDevice::open()
{
    if (_input) {
        socket.bind(_ip, _port);
        return true;
    }
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
