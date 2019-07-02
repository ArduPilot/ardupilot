/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  this is a driver for multiple RCInput methods on one board
 */

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include "RCInput_RCProtocol.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE

extern const AP_HAL::HAL& hal;

using namespace Linux;

/*
  open a SBUS UART
 */
int RCInput_RCProtocol::open_sbus(const char *path)
{
    int fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd == -1) {
        return -1;
    }
    struct termios2 tio {};

    if (ioctl(fd, TCGETS2, &tio) != 0) {
        close(fd);
        fd = -1;
        return -1;
    }
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR
                     | IGNCR | ICRNL | IXON);
    tio.c_iflag |= (INPCK | IGNPAR);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
    // use BOTHER to specify speed directly in c_[io]speed member
    tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
    tio.c_ispeed = 100000;
    tio.c_ospeed = 100000;
    if (ioctl(fd, TCSETS2, &tio) != 0) {
        close(fd);
        fd = -1;
        return -1;
    }

    return fd;
}

/*
  open a 115200 UART
 */
int RCInput_RCProtocol::open_115200(const char *path)
{
    int fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd == -1) {
        return -1;
    }
    struct termios2 tio {};

    if (ioctl(fd, TCGETS2, &tio) != 0) {
        close(fd);
        fd = -1;
        return -1;
    }

    tio.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
    tio.c_cflag |= CS8 | B115200;

    tio.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    tio.c_iflag &= ~(IXON|IXOFF|IXANY);
    tio.c_oflag &= ~OPOST;

    if (ioctl(fd, TCSETS2, &tio) != 0) {
        close(fd);
        fd = -1;
        return -1;
    }

    return fd;
}

// constructor
RCInput_RCProtocol::RCInput_RCProtocol(const char *_dev_sbus, const char *_dev_115200) :
    dev_sbus(_dev_sbus),
    dev_115200(_dev_115200)
{
}

void RCInput_RCProtocol::init()
{
    if (dev_sbus) {
        fd_sbus = open_sbus(dev_sbus);
    } else {
        fd_sbus = -1;
    }
    if (dev_115200) {
        fd_115200 = open_115200(dev_115200);
    } else {
        fd_115200 = -1;
    }
    rcp.init();
    printf("SBUS FD %d  115200 FD %d\n", fd_sbus, fd_115200);
}

void RCInput_RCProtocol::_timer_tick(void)
{
    uint8_t b[40];

    if (fd_sbus != -1) {
        ssize_t n = ::read(fd_sbus, &b[0], sizeof(b));
        if (n > 0) {
            for (uint8_t i=0; i<n; i++) {
                rcp.process_byte(b[i], 100000);
            }
        }
    }
    if (fd_115200 != -1) {
        ssize_t n = ::read(fd_115200, &b[0], sizeof(b));
        if (n > 0) {
            for (uint8_t i=0; i<n; i++) {
                rcp.process_byte(b[i], 115200);
            }
        }
    }

    if (rcp.new_input()) {
        uint8_t n = rcp.num_channels();
        for (uint8_t i=0; i<n; i++) {
            _pwm_values[i] = rcp.read(i);
        }
        _num_channels = n;
        rc_input_count++;
    }
}

#endif // HAL
