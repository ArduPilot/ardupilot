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
#include <termios.h>
#include <errno.h>

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
            struct termios t;
            memset(&t, 0, sizeof(t));
            tcgetattr(fd, &t);
            t.c_ispeed = B100000;
            t.c_ospeed = B100000;
            t.c_cc[VTIME] = 0;
            t.c_cflag |= (CSTOPB | PARENB);
            t.c_cflag &= ~(BRKINT | PARODD);
            t.c_iflag &= ~(INLCR | ICRNL | IMAXBEL | IXON | IXOFF | ICANON | IEXTEN | ECHO);
            tcsetattr(fd, TCSANOW, &t);
        }
    }
    if (fd != -1) {
        uint8_t bytes[25];
        int32_t nread;
        do {
            nread = ::read(fd, bytes, sizeof(bytes));
            if (nread > 0) {
                add_sbus_input(bytes, nread);
            }
        } while (nread == 25);
    }
}
#endif // CONFIG_HAL_BOARD_SUBTYPE

