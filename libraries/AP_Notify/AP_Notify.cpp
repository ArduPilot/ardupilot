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

#include <AP_Notify.h>

// static flags, to allow for direct class update from device drivers
struct AP_Notify::notify_type AP_Notify::flags;

// initialisation
void AP_Notify::init(bool enable_external_leds)
{
    AP_Notify::flags.external_leds = enable_external_leds;

    boardled.init();
    toshibaled.init();

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    tonealarm.init();
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
    externalled.init();
    buzzer.init();
#endif
}

// main update function, called at 50Hz
void AP_Notify::update(void)
{
    boardled.update();
    toshibaled.update();

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    tonealarm.update();
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
    externalled.update();
    buzzer.update();
#endif
}
