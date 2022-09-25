#include "FIFODevice.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

FIFODevice::FIFODevice(const char *rd_device, const char *wr_device):
    _rd_device(rd_device), _wr_device(wr_device),
    _rd_fd(-1), _wr_fd(-1)
{
}

FIFODevice::~FIFODevice()
{
}

bool FIFODevice::close()
{
    bool ret = true;

    if (_rd_fd >= 0) {
        if (::close(_rd_fd) < 0) {
            ret = false;
        }
        _rd_fd = -1;
    }

    if (_wr_fd >= 0) {
        if (::close(_wr_fd) < 0) {
            ret = false;
        }
        _wr_fd = -1;
    }

    return ret;
}

bool FIFODevice::open()
{
    _rd_fd = ::open(_rd_device, O_CLOEXEC | O_NONBLOCK | O_RDONLY);

    if (_rd_fd < 0) {
        ::fprintf(stderr, "Failed to open FIFO device %s for reading - %s\n",
                  _rd_device, strerror(errno));
        return false;
    }

    //O_RDWR is used only to allow opening a FIFO that has not been opened yet.
    _wr_fd = ::open(_wr_device, O_CLOEXEC | O_NONBLOCK | O_RDWR);

    if (_wr_fd < 0) {
        ::fprintf(stderr, "Failed to open FIFO device %s for writing - %s\n",
                  _wr_device, strerror(errno));
        ::close(_rd_fd);
        _rd_fd = -1;
        return false;
    }

    return true;
}

ssize_t FIFODevice::read(uint8_t *buf, uint16_t n)
{
    return ::read(_rd_fd, buf, n);
}

ssize_t FIFODevice::write(const uint8_t *buf, uint16_t n)
{
    return ::write(_wr_fd, buf, n);
}

static void fifo_set_fd_blocking_helper(int fd, const char * devname, bool blocking)
{
    int flags  = fcntl(fd, F_GETFL, 0);

    if (blocking) {
        flags = flags & ~O_NONBLOCK;
    } else {
        flags = flags | O_NONBLOCK;
    }

    if (fcntl(fd, F_SETFL, flags) < 0) {
        ::fprintf(stderr, "Failed to set FIFO %s nonblocking: %s\n", devname, strerror(errno));
    }
}

void FIFODevice::set_blocking(bool blocking)
{
    fifo_set_fd_blocking_helper(_rd_fd, _rd_device, blocking);
    fifo_set_fd_blocking_helper(_wr_fd, _wr_device, blocking);
}

void FIFODevice::set_speed(uint32_t baudrate)
{
    //Ignored for FIFO: quite fast.
}

void FIFODevice::set_flow_control(AP_HAL::UARTDriver::flow_control flow_control_setting)
{
    //Ignored for FIFO: effectively FLOW_CONTROL_ENABLE once pipe is opened
}

void FIFODevice::set_parity(int v)
{
    //Ignored for FIFO: in-memory ring buffer
}
