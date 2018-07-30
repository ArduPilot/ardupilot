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
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#include <AP_HAL_Linux/RCOutput_Bebop.h>
#include <AP_HAL_Linux/RCOutput_Disco.h>
#include "ToneAlarm_Disco.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

using namespace Linux;

ToneAlarm_Disco::ToneAlarm_Disco(){}

bool ToneAlarm_Disco::init()
{
    bebop_out = RCOutput_Disco::from(hal.rcout);

    return true;
}

void ToneAlarm_Disco::set_buzzer_tone(float frequency, float volume, uint32_t duration_ms)
{
    if (is_zero(frequency) || is_zero(volume)) {
        bebop_out->play_note(0, 0, 0);
    } else {
        bebop_out->play_note(TONEALARM_PWM_POWER, (uint16_t)roundf(frequency), duration_ms);
    }
}

#endif
