#include <AP_HAL/AP_HAL.h>

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
    _closed = true;

    return true;
}

bool ConsoleDevice::open()
{
    _rd_fd = STDIN_FILENO;
    _wr_fd = STDOUT_FILENO;

    _closed = false;

    return true;
}

ssize_t ConsoleDevice::read(uint8_t *buf, uint16_t n)
{
    if (_closed) {
        return -EAGAIN;
    }

    return ::read(_rd_fd, buf, n);
}

ssize_t ConsoleDevice::write(const uint8_t *buf, uint16_t n)
{
    if (_closed) {
        return -EAGAIN;
    }

    return ::write(_wr_fd, buf, n);
}

void ConsoleDevice::set_blocking(bool blocking)
{
    int rd_flags;
    int wr_flags;
   
    rd_flags  = fcntl(_rd_fd, F_GETFL, 0);
    wr_flags  = fcntl(_wr_fd, F_GETFL, 0);
    
    if (blocking) {
        rd_flags = rd_flags & ~O_NONBLOCK;
        wr_flags = wr_flags & ~O_NONBLOCK;
    } else {
        rd_flags = rd_flags | O_NONBLOCK;
        wr_flags = wr_flags | O_NONBLOCK;
    }

    if (fcntl(_rd_fd, F_SETFL, rd_flags) < 0) {
        ::fprintf(stderr, "Failed to set Console nonblocking %s\n", strerror(errno));
    }

    if (fcntl(_wr_fd, F_SETFL, wr_flags) < 0) {
        ::fprintf(stderr, "Failed to set Console nonblocking %s\n",strerror(errno));
    }

}

void ConsoleDevice::set_speed(uint32_t baudrate)
{
}

#endif
