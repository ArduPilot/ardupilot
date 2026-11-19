#include "UDPDevice.h"

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/uio.h>

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
    // a datagram larger than buf is read cut, further ones only if they fit
    const ByteBuffer::IoVec vec { buf, n };
    return read_datagrams(&vec, 1, n);
}

/*
  read whole datagrams into vec[0], then vec[1], see SerialDevice.h
 */
ssize_t UDPDevice::read_datagrams(const ByteBuffer::IoVec *vec, uint8_t n_vec, uint32_t max_space)
{
    ssize_t space = 0;
    for (uint8_t i = 0; i < n_vec; i++) {
        space += vec[i].len;
    }

    ssize_t bytes_read = 0;
    ssize_t packets_read = 0;

    // recv() only retrieves a single datagram on UDP sockets
    // peek & read multiple until the buffer full
    // poll each time because FIONREAD can't distinguish between
    // no data and empty datagrams
    while (bytes_read < space && socket.pollin(0) && packets_read < HAL_LINUX_MAX_UDP_PACKETS) {
        int in_size = 0;
        if (ioctl(socket.get_read_fd(), FIONREAD, &in_size) < 0) {
            break;
        }

        // only read a datagram if it fits into the space that is left,
        // so that it is never cut at the end of the ring buffer; it
        // waits in the socket until the reader has made room. A first
        // datagram larger than max_space can never fit: read it cut
        // rather than end up being unable to read any further
        if (in_size > space - bytes_read && (bytes_read > 0 || (uint32_t)in_size <= max_space)) {
            break;
        }

        // the part of vec that is still free, as one or two buffers
        struct iovec iov[2];
        int iovcnt = 0;
        uint32_t skip = bytes_read;
        for (uint8_t i = 0; i < n_vec && iovcnt < 2; i++) {
            if (skip >= vec[i].len) {
                skip -= vec[i].len;
                continue;
            }
            iov[iovcnt].iov_base = vec[i].data + skip;
            iov[iovcnt].iov_len = vec[i].len - skip;
            skip = 0;
            iovcnt++;
        }

        ssize_t ret = socket.recv_iov(iov, iovcnt);
        if (!_connected && ret > 0) {
            const char *ip;
            uint16_t port;
            socket.last_recv_address(ip, port);
            _connected = socket.connect(ip, port);
        }

        // recv_iov() returns -1 if we're reading ourselves on multicast
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
