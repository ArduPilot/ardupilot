#include "RCInput_UART.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

#define MAGIC 0x55AA

using namespace Linux;

RCInput_UART::RCInput_UART(const char *path)
{
    _fd = open(path, O_RDONLY|O_NOCTTY|O_NONBLOCK|O_NDELAY|O_CLOEXEC);
    if (_fd < 0) {
        AP_HAL::panic("RCInput_UART: Error opening '%s': %s",
                             path, strerror(errno));
    }
}

RCInput_UART::~RCInput_UART()
{
    close(_fd);
}

void RCInput_UART::init()
{
    struct termios options;

    tcgetattr(_fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    options.c_iflag &= ~(IXON|IXOFF|IXANY);
    options.c_oflag &= ~OPOST;

    if (tcsetattr(_fd, TCSANOW, &options) != 0) {
        AP_HAL::panic("RCInput_UART: error configuring device: %s",
                             strerror(errno));
    }

    tcflush(_fd, TCIOFLUSH);

    _pdata = (uint8_t *)&_data;
    _remain = sizeof(_data);
}

void RCInput_UART::_timer_tick()
{
    ssize_t n;

    if ((n = ::read(_fd, _pdata, _remain)) <= 0)
        return;

    _remain -= n;
    _pdata += n;

    if (_remain != 0)
        return;

    if (_data.magic != MAGIC) {
        /* try to find the magic number and move
         * it to the beginning of our buffer */
        uint16_t magic = MAGIC;

        _pdata = (uint8_t *)memmem(&_data, sizeof(_data), &magic, sizeof(magic));

        if (!_pdata)
            _pdata = (uint8_t *)&_data + sizeof(_data) - 1;

        _remain = _pdata - (uint8_t *)&_data;
        n = sizeof(_data) - _remain;
        memmove(&_data, _pdata, n);
        _pdata = (uint8_t *)&_data + n;
        return;
    }

    _update_periods(_data.values, CHANNELS);
    _pdata = (uint8_t *)&_data;
    _remain = sizeof(_data);
}
