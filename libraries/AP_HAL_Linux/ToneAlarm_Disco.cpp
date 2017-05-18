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

extern const AP_HAL::HAL &hal;

using namespace Linux;

ToneAlarm_Disco::ToneAlarm_Disco()
{
    // initialy no tune to play
    tune_num = -1;
    tune_pos = 0;
}

bool ToneAlarm_Disco::init()
{
    // play startup tune
    tune_num = 0;

    bebop_out = RCOutput_Disco::from(hal.rcout);

    return true;
}

void ToneAlarm_Disco::stop()
{
    bebop_out->play_note(0, 0, 0);
}

bool ToneAlarm_Disco::play()
{
    uint32_t cur_time = AP_HAL::millis();

    if (tune_num != prev_tune_num){
        tune_changed = true;
        return true;
    }

    if (cur_note != 0){
        bebop_out->play_note(TONEALARM_PWM_POWER, cur_note, duration);
        cur_note = 0;
        prev_time = cur_time;
    }

    if ((cur_time - prev_time) > duration){
        stop();
        if (tune[tune_num][tune_pos] == '\0'){
            if (!tune_repeat[tune_num]){
                tune_num = -1;
            }

            tune_pos = 0;
            tune_comp = true;
            return false;
        }
        return true;
    }

    return false;
}

#endif
