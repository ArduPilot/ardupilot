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
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/Led_Sysfs.h>
#include <AP_HAL_Linux/PWM_Sysfs.h>

#include "RGBLed.h"

class DiscoLED: public RGBLed
{
public:
    DiscoLED();
    bool init(void) override;

protected:
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    Linux::PWM_Sysfs_Bebop red_pwm;
    Linux::PWM_Sysfs_Bebop green_pwm;
    Linux::PWM_Sysfs_Bebop blue_pwm;

    Linux::Led_Sysfs red_led;
    Linux::Led_Sysfs green_led;
    Linux::Led_Sysfs blue_led;

    uint32_t red_pwm_period;
    uint32_t green_pwm_period;
    uint32_t blue_pwm_period;

    enum led_backend {
        LED_SYSFS,
        PWM_SYSFS
    };

    enum led_backend backend;
};
#endif
