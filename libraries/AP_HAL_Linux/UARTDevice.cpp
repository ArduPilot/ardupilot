#include "UARTDevice.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

UARTDevice::UARTDevice(const char *device_path):
    _device_path(device_path)
{
}

UARTDevice::~UARTDevice()
{
}

bool UARTDevice::close()
{
    if (_fd != -1) {
        if (::close(_fd) < 0) {
            return false;
        }
    }

    _fd = -1;

    return true;
}

bool UARTDevice::open()
{
    _fd = ::open(_device_path, O_RDWR | O_CLOEXEC | O_NOCTTY);

    if (_fd < 0) {
        ::fprintf(stderr, "Failed to open UART device %s - %s\n",
                  _device_path, strerror(errno));
        return false;
    }

    _disable_crlf();

    return true;
}

ssize_t UARTDevice::read(uint8_t *buf, uint16_t n)
{
    return ::read(_fd, buf, n);
}

ssize_t UARTDevice::write(const uint8_t *buf, uint16_t n)
{
    struct pollfd fds;
    fds.fd = _fd;
    fds.events = POLLOUT;
    fds.revents = 0;

    int ret = 0;

    if (poll(&fds, 1, 0) == 1) {
        ret = ::write(_fd, buf, n);
    }

    return ret;
}

void UARTDevice::set_blocking(bool blocking)
{
    int flags = fcntl(_fd, F_GETFL, 0);

    if (blocking) {
        flags = flags & ~O_NONBLOCK;
    } else {
        flags = flags | O_NONBLOCK;
    }

    if (fcntl(_fd, F_SETFL, flags) < 0) {
        ::fprintf(stderr, "Failed to make UART nonblocking %s - %s\n",
                  _device_path, strerror(errno));
    }

}

void UARTDevice::_disable_crlf()
{
    struct termios2 t = { 0 };

    if (ioctl(_fd, TCGETS2, &t) != 0) {
        ::fprintf(stderr, "Failed to read serial options for %s - %s\n",
                  _device_path, strerror(errno));
        return;
    }

    // disable LF -> CR/LF
    t.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL | IXON | IXOFF);
    t.c_oflag &= ~(OPOST | ONLCR);
    t.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
    t.c_cc[VMIN] = 0;

    if (ioctl(_fd, TCSETS2, &t) != 0) {
        ::fprintf(stderr, "Failed to disable crlf on %s - %s\n",
                  _device_path, strerror(errno));
        return;
    }
}

void UARTDevice::set_speed(uint32_t baudrate)
{
    struct termios2 tio = { 0 };

    if (ioctl(_fd, TCGETS2, &tio) != 0) {
        ::fprintf(stderr, "Failed to read serial options for %s - %s\n",
                  _device_path, strerror(errno));
        return;
    }

    // use CBAUDEX to gain access to "non-standard" rates that are common for eg. RC receivers
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= CBAUDEX;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    if (ioctl(_fd, TCSETS2, &tio) != 0) {
        ::fprintf(stderr, "Failed to set serial baud to %d for %s - %s\n",
                  baudrate, _device_path, strerror(errno));
        return;
    }
}

void UARTDevice::set_flow_control(AP_HAL::UARTDriver::flow_control flow_control_setting)
{
    if (_flow_control == flow_control_setting) {
        return;
    }

    struct termios2 t = { 0 };

    if (ioctl(_fd, TCGETS2, &t) != 0) {
        ::fprintf(stderr, "Failed to read serial options for %s - %s\n",
                  _device_path, strerror(errno));
        return;
    }

    if (flow_control_setting != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE) {
        t.c_cflag |= CRTSCTS;
    } else {
        t.c_cflag &= ~CRTSCTS;
    }

    if (ioctl(_fd, TCSETS2, &t) != 0) {
        ::fprintf(stderr, "Failed to set flow control for %s - %s\n",
                  _device_path, strerror(errno));
        return;
    }

    _flow_control = flow_control_setting;
}

void UARTDevice::set_parity(int v)
{
    struct termios2 t = { 0 };

    if (ioctl(_fd, TCGETS2, &t) != 0) {
        ::fprintf(stderr, "Failed to read serial options for %s - %s\n",
                  _device_path, strerror(errno));
        return;
    }

    if (v != 0) {
        // enable parity
        t.c_cflag |= PARENB;
        if (v == 1) {
            t.c_cflag |= PARODD;
        } else {
            t.c_cflag &= ~PARODD;
        }
    }
    else {
        // disable parity
        t.c_cflag &= ~PARENB;
    }

    if (ioctl(_fd, TCSETS2, &t) != 0) {
        ::fprintf(stderr, "Failed to set parity for %s - %s\n",
                  _device_path, strerror(errno));
        return;
    }
}
