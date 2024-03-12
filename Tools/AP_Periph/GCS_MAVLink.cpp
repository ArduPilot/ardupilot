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

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU
};
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
#if HAL_WITH_MCU_MONITORING
    MSG_MCU_STATUS,
#endif
    MSG_MEMINFO,
    MSG_GPS_RAW,
    MSG_GPS_RTK,
};

static const ap_message STREAM_POSITION_msgs[] = {
#if AP_AHRS_ENABLED
    MSG_LOCATION,
    MSG_LOCAL_POSITION
#endif
};

static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

uint8_t GCS_MAVLINK_Periph::sysid_my_gcs() const
{
    return periph.g.sysid_this_mav;
}

uint8_t GCS_Periph::sysid_this_mav() const
{
    return periph.g.sysid_this_mav;
}

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
