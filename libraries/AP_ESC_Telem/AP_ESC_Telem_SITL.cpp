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

#include "AP_ESC_Telem_SITL.h"
#include "AP_ESC_Telem.h"
#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

AP_ESC_Telem_SITL::AP_ESC_Telem_SITL()
{
}

void AP_ESC_Telem_SITL::update()
{
    SITL::SITL* sitl = AP::sitl();

    if (!sitl) {
        return;
    }

    if (is_zero(sitl->throttle)) {
        return;
    }

#if HAL_WITH_ESC_TELEM
    for (uint8_t i = 0; i < sitl->state.num_motors; i++) {
        update_rpm(i, sitl->state.rpm[sitl->state.vtol_motor_start+i]);
    }
#endif

}

#endif