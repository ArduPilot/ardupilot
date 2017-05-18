#include "UARTDevice.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <termios.h>
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
    _fd = ::open(_device_path, O_RDWR | O_CLOEXEC);

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
    struct termios t;
    memset(&t, 0, sizeof(t));

    tcgetattr(_fd, &t);

    // disable LF -> CR/LF
    t.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL | IXON | IXOFF);
    t.c_oflag &= ~(OPOST | ONLCR);
    t.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
    t.c_cc[VMIN] = 0;

    tcsetattr(_fd, TCSANOW, &t);
}

void UARTDevice::set_speed(uint32_t baudrate)
{
    struct termios t;
    memset(&t, 0, sizeof(t));

    tcgetattr(_fd, &t);
    cfsetspeed(&t, baudrate);
    tcsetattr(_fd, TCSANOW, &t);
}

void UARTDevice::set_flow_control(AP_HAL::UARTDriver::flow_control flow_control_setting)
{
    struct termios t;

    if (_flow_control == flow_control_setting) {
        return;
    }

    tcgetattr(_fd, &t);

    if (flow_control_setting != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE) {
        t.c_cflag |= CRTSCTS;
    } else {
        t.c_cflag &= ~CRTSCTS;
    }

    tcsetattr(_fd, TCSANOW, &t);

    _flow_control = flow_control_setting;
}
