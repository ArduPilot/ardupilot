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

#include "GCS_MAVLink.h"
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#endif

#if HAL_GCS_ENABLED

MAV_RESULT GCS_MAVLINK_Periph::handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    hal.scheduler->delay(10);
    periph.prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    NVIC_SystemReset();
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    HAL_SITL::actually_reboot();
#endif
}

#endif // #if HAL_GCS_ENABLED
