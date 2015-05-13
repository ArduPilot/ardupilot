#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <inttypes.h>
#include <unistd.h>
#include <errno.h>

#include "RCInput_UART.h"

#define SERIAL_FILE "/dev/ttyS2"
#define MAGIC 0x55AA

extern const AP_HAL::HAL& hal;

using namespace Linux;

void LinuxRCInput_UART::init(void*)
{
    char s[256];
    struct termios options;

    if ((_fd = open(SERIAL_FILE, O_RDONLY|O_NOCTTY|O_NONBLOCK|O_NDELAY)) < 0) {
        snprintf(s, sizeof(s), "Error opening '%s': %s",
                 SERIAL_FILE, strerror(errno));
        hal.scheduler->panic(s);
    }

    tcgetattr(_fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    options.c_iflag &= ~(IXON|IXOFF|IXANY);
    options.c_oflag &= ~OPOST;

    if (tcsetattr(_fd, TCSANOW, &options) != 0) {
        snprintf(s, sizeof(s), "Error configuring '%s': %s",
                 SERIAL_FILE, strerror(errno));
        hal.scheduler->panic(s);
    }

    tcflush(_fd, TCIOFLUSH);

    _pdata = (uint8_t *)&_data;
    _remain = sizeof(_data);
}

void LinuxRCInput_UART::_timer_tick()
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
         * it to the beggining of our buffer */
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

#endif
