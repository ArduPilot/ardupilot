#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include <AP_HAL/AP_HAL.h>

#include "ConsoleDevice.h"


ConsoleDevice::ConsoleDevice()
{
}

ConsoleDevice::~ConsoleDevice()
{
    close();
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

    if (!_set_signal_handlers()) {
        close();
        return false;
    }

    return true;
}

bool ConsoleDevice::_set_signal_handlers(void) const
{
    struct sigaction sa;
    sigemptyset(&sa.sa_mask);
    sa.sa_handler = SIG_IGN;

    return (sigaction(SIGTTIN, &sa, nullptr) == 0);

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
