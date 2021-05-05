/*
   Copyright (C) 2017 Emlid Ltd. All rights reserved.

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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "Led_Sysfs.h"

#include <AP_HAL_Linux/Led_Sysfs.h>

Led_Sysfs::Led_Sysfs(const char *red, const char *green, const char *blue,
                    uint8_t off_brightness, uint8_t low_brightness, uint8_t medium_brightness, uint8_t high_brightness):
    RGBLed(off_brightness, high_brightness, medium_brightness, low_brightness),
    red_led(red),
    green_led(green),
    blue_led(blue)
{
}

bool Led_Sysfs::init()
{
    if (red_led.init() && green_led.init() && blue_led.init()) {
        return true;
    }

    return false;
}

bool Led_Sysfs::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    red_led.set_brightness(red);
    green_led.set_brightness(green);
    blue_led.set_brightness(blue);

    return true;
}
#endif
