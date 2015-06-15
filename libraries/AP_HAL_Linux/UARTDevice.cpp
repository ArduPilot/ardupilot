#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include "UARTDevice.h"

#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

UARTDevice::UARTDevice(char *device_path): 
    _device_path(device_path)
{
}

UARTDevice::~UARTDevice()
{

}

bool UARTDevice::close()
{
    if (::close(_fd) < 0) {
        return false;
    }

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

int UARTDevice::read(uint8_t *buf, uint16_t n)
{
    return ::read(_fd, buf, n);
}

int UARTDevice::write(const uint8_t *buf, uint16_t n)
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

void UARTDevice::set_nonblocking()
{
    if (fcntl(_fd, F_SETFL, fcntl(_fd, F_GETFL, 0) | O_NONBLOCK) < 0) {
        ::fprintf(stderr, "Failed to set UART nonblocking %s - %s\n",
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

#endif
