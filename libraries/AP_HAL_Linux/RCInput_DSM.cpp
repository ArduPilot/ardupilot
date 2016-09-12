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

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT
#include "RCInput_DSM.h"
#include <AP_HAL_Linux/qflight/qflight_util.h>
#include <AP_HAL_Linux/qflight/qflight_dsp.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCInput_DSM::init()
{
}

void RCInput_DSM::set_device_path(const char *path)
{
    device_path = path;
    printf("Set DSM device path %s\n", path);
}

void RCInput_DSM::_timer_tick(void)
{
    if (device_path == nullptr) {
        return;
    }
    int ret;
    /*
      we defer the open to the timer tick to ensure all RPC calls are
      made in the same thread
     */
    if (fd == -1) {
        ret = qflight_UART_open(device_path, &fd);
        if (ret == 0) {
            printf("Opened DSM input %s fd=%d\n", device_path, (int)fd);
            fflush(stdout);
            qflight_UART_set_baudrate(fd, 115200);
        }
    }
    if (fd != -1) {
        uint8_t bytes[16];
        int32_t nread;
        ret = qflight_UART_read(fd, bytes, sizeof(bytes), &nread);
        if (ret == 0 && nread > 0) {
            // printf("Read %u DSM bytes at %u\n", (unsigned)nread, AP_HAL::millis());
            fflush(stdout);
            add_dsm_input(bytes, nread);
        }
    }
}
#endif // CONFIG_HAL_BOARD_SUBTYPE

