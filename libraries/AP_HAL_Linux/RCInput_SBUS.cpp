/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  this is a driver for DSM input in the QFLIGHT board. It could be
  extended to other boards in future by providing an open/read/write
  abstraction
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#include "RCInput_SBUS.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCInput_SBUS::init()
{
}

void RCInput_SBUS::set_device_path(const char *path)
{
    device_path = path;
    printf("Set SBUS device path %s\n", path);
}

#ifndef B100000
// disco uses 010000 for 100000 baud (BOTHER)
#define B100000 010000
#endif

void RCInput_SBUS::_timer_tick(void)
{
    if (device_path == nullptr) {
        return;
    }
    /*
      we defer the open to the timer tick to ensure all RPC calls are
      made in the same thread
     */
    if (fd == -1) {
        fd = open(device_path, O_RDONLY | O_NONBLOCK);
        if (fd != -1) {
            printf("Opened SBUS input %s fd=%d\n", device_path, (int)fd);
            fflush(stdout);
            struct termios2 tio {};

            int ret = ioctl(fd, TCGETS2, &tio);
            printf("ret1=%d\n", ret);

            tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR
                             | IGNCR | ICRNL | IXON);
            tio.c_iflag |= (INPCK | IGNPAR);
            tio.c_oflag &= ~OPOST;
            tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            tio.c_ispeed = B100000;
            tio.c_ospeed = B100000;
            tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
            tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
            tio.c_cc[VMIN] = 25;
            tio.c_cc[VTIME] = 0;
            ret = ioctl(fd, TCSETS2, &tio);
            printf("SBUS ret=%d %s\n", ret, strerror(errno));
        }
    }
    if (fd != -1) {
        uint8_t bytes[25];
        int32_t nread;
        uint8_t counter = 100;
        do {
            nread = ::read(fd, bytes, sizeof(bytes));
            if (nread > 0 && bytes[0] == 0x0f) {
                add_sbus_input(bytes, nread);
            }
        } while (nread == 25 && counter--);
    }
}
#endif // CONFIG_HAL_BOARD_SUBTYPE

