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
    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool ToshibaLED_PX4::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgbled_rgbset_t v;

    v.red   = red;
    v.green = green;
    v.blue  = blue;

    int ret = ioctl(_rgbled_fd, RGBLED_SET_RGB, (unsigned long)&v);
    return (ret == 0);
}


#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
