/*
  ToshibaLED PX4 driver
*/
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

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "ToshibaLED_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_rgbled.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

bool ToshibaLED_PX4::hw_init()
{
    // open the rgb led device
    _rgbled_fd = open(RGBLED_DEVICE_PATH, 0);
    if (_rgbled_fd == -1) {
        hal.console->printf("Unable to open " RGBLED_DEVICE_PATH);
        return false;
    }
    ioctl(_rgbled_fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_ON);
    last.v = 0;
    next.v = 0;
    hal.scheduler->register_io_process(AP_HAL_MEMBERPROC(&ToshibaLED_PX4::update_timer));
    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool ToshibaLED_PX4::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    union rgb_value v;
    v.r = red;
    v.g = green;
    v.b = blue;
    // this does an atomic 32 bit update
    next.v = v.v;
    return true;
}

void ToshibaLED_PX4::update_timer(void)
{
    if (last.v == next.v) {
        return;
    }
    rgbled_rgbset_t v;
    union rgb_value newv;
    newv.v = next.v;
    v.red   = newv.r;
    v.green = newv.g;
    v.blue  = newv.b;

    ioctl(_rgbled_fd, RGBLED_SET_RGB, (unsigned long)&v);

    last.v = next.v;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
