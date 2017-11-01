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
#include "Navio2Led.h"

#include <AP_HAL_Linux/Led_Sysfs.h>

#define NAVIO2_LED_LOW    0x00
#define NAVIO2_LED_MEDIUM 0x00
#define NAVIO2_LED_HIGH   0x00
#define NAVIO2_LED_OFF    0xFF

#define NAVIO2_LED_RED_NAME   "rgb_led0"
#define NAVIO2_LED_GREEN_NAME "rgb_led2"
#define NAVIO2_LED_BLUE_NAME  "rgb_led1"

Navio2Led::Navio2Led():
    RGBLed(NAVIO2_LED_OFF, NAVIO2_LED_HIGH, NAVIO2_LED_MEDIUM, NAVIO2_LED_LOW),
    red_led(NAVIO2_LED_RED_NAME),
    green_led(NAVIO2_LED_GREEN_NAME),
    blue_led(NAVIO2_LED_BLUE_NAME)
{
}

bool Navio2Led::hw_init()
{
    if (red_led.init() && green_led.init() && blue_led.init()) {
        return true;
    }

    return false;
}

bool Navio2Led::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    red_led.set_brightness(red);
    green_led.set_brightness(green);
    blue_led.set_brightness(blue);

    return true;
}
#endif
