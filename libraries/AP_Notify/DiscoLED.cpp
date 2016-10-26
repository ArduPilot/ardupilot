/*
   Copyright (C) 2016 Mathieu Othacehe. All rights reserved.

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
#include <AP_HAL_Linux/PWM_Sysfs.h>
#include "DiscoLED.h"

#define RED_PWM_INDEX   9
#define GREEN_PWM_INDEX 8
#define BLUE_PWM_INDEX  15

#define DISCO_LED_LOW    0x33
#define DISCO_LED_MEDIUM 0x7F
#define DISCO_LED_HIGH   0xFF
#define DISCO_LED_OFF    0x00

DiscoLED::DiscoLED():
    RGBLed(DISCO_LED_OFF, DISCO_LED_HIGH, DISCO_LED_MEDIUM, DISCO_LED_LOW)
{
}

bool DiscoLED::hw_init()
{
    red_pwm = new Linux::PWM_Sysfs_Bebop(RED_PWM_INDEX);
    green_pwm = new Linux::PWM_Sysfs_Bebop(GREEN_PWM_INDEX);
    blue_pwm = new Linux::PWM_Sysfs_Bebop(BLUE_PWM_INDEX);

    red_pwm_period = red_pwm->get_period();
    green_pwm_period = green_pwm->get_period();
    blue_pwm_period = blue_pwm->get_period();

    red_pwm->enable(true);
    green_pwm->enable(true);
    blue_pwm->enable(true);

    return true;
}

bool DiscoLED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    red_pwm->set_duty_cycle(red / UINT8_MAX * red_pwm_period);
    green_pwm->set_duty_cycle(green / UINT8_MAX * green_pwm_period);
    blue_pwm->set_duty_cycle(blue / UINT8_MAX * blue_pwm_period);

    return true;
}
#endif
