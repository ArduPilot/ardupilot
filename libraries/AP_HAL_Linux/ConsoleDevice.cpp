#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "ConsoleDevice.h"

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

ConsoleDevice::ConsoleDevice() 
{
}

ConsoleDevice::~ConsoleDevice()
{

}

bool ConsoleDevice::close()
{
    if (::close(_rd_fd) < 0) {
        return false;
    }

    if (::close(_wr_fd) < 0) {
        return false;
    }

    return true;
}

bool ConsoleDevice::open()
{
    _rd_fd = STDIN_FILENO;
    _wr_fd = STDOUT_FILENO;

    return true;
}

int ConsoleDevice::read(uint8_t *buf, uint16_t n)
{
    return ::read(_rd_fd, buf, n);
}

int ConsoleDevice::write(const uint8_t *buf, uint16_t n)
{
    return ::write(_wr_fd, buf, n);
}

void ConsoleDevice::set_nonblocking()
{
    if (fcntl(_rd_fd, F_SETFL, fcntl(_rd_fd, F_GETFL, 0) | O_NONBLOCK) < 0) {
        ::fprintf(stderr, "Failed to set Console nonblocking %s\n", strerror(errno));
    }

    if (fcntl(_wr_fd, F_SETFL, fcntl(_wr_fd, F_GETFL, 0) | O_NONBLOCK) < 0) {
        ::fprintf(stderr, "Failed to set Console nonblocking %s\n",strerror(errno));
    }

}

void ConsoleDevice::set_speed(uint32_t baudrate)
{
}

#endif
