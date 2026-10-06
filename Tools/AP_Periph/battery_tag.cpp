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
/*
  battery info support.  The battery infomation node is attached to a
  specific battery and recorded the number of cycles, and total armed
  time.

  The node generates BatteryTag messages which are consumed by a
  BatteryTag lua script on the flight controller
 */
#include "AP_Periph.h"

#if AP_PERIPH_BATTERY_TAG_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo BatteryTag::var_info[] {
    // @Param: _NUM_CYCLES
    // @DisplayName: Number of cycles
    // @Description: Number of cycles the battery has been through
    AP_GROUPINFO("_NUM_CYCLES", 1, BatteryTag, num_cycles, 0),

    // @Param: _ARM_HOURS
    // @DisplayName: Number of armed hours
    // @Description: Number of hours the battery has been armed
    AP_GROUPINFO("_ARM_HOURS", 2, BatteryTag, armed_hours, 0),

    // @Param: _CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Battery capacity in mAh
    AP_GROUPINFO("_CAPACITY", 3, BatteryTag, capacity_mAh, 0),

    // @Param: _FIRST_USE
    // @DisplayName: First use time
    // @Description: First use time in minutes since 1/1/1970
    AP_GROUPINFO("_FIRST_USE", 4, BatteryTag, first_use_min, 0),

    // @Param: _LAST_USE
    // @DisplayName: Last use time
    // @Description: Last use time in minutes since 1/1/1970
    AP_GROUPINFO("_LAST_USE", 5, BatteryTag, last_use_min, 0),

    // @Param: _SERIAL
    // @DisplayName: Serial number
    // @Description: Serial number
    AP_GROUPINFO("_SERIAL", 6, BatteryTag, serial_num, 0),

    // @Param: _CYCLE_MIN
    // @DisplayName: Cycle minimum time
    // @Description: Cycle minimum time. Minimum time that vehicle is armed in minutes for counting a battery cycle
    AP_GROUPINFO("_CYCLE_MIN", 7, BatteryTag, cycle_min, 1),
    
    AP_GROUPEND
};

BatteryTag::BatteryTag(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void BatteryTag::update(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 1000U) {
        return;
    }
    last_update_ms = now;

    const bool armed = hal.util->get_soft_armed();
    if (armed && arm_start_ms == 0) {
        arm_start_ms = now;
    }

    if (!armed && was_armed) {
        /*
          update total armed time
         */
        const float armed_minutes = (now - arm_start_ms)*(0.001/60);
        armed_hours.set_and_save(armed_hours + armed_minutes/60);

        /*
          update number of cycles if we were armed for more than
          BTAG_CYCLE_MIN minutes
         */
        if (armed_minutes >= cycle_min) {
            num_cycles.set_and_save(num_cycles + 1);
        }
    }

    uint64_t utc_usec;
    if (AP::rtc().get_utc_usec(utc_usec)) {
        const uint32_t utc_minutes = utc_usec / 60e6;
        if (armed && first_use_min == 0) {
            /*
              first time the battery has been armed
             */
            first_use_min.set_and_save(utc_minutes);
        }
        if (!armed && was_armed) {
            // record when battery was last used
            last_use_min.set_and_save(utc_minutes);
        }
    }

    if (armed && serial_num == 0) {
        // auto-create serial number if not set by vendor
        char sysid[50];
        if (hal.util->get_system_id(sysid)) {
            // 31 bit for AP_Int32
            const auto crc = crc32_small(0, (uint8_t*)sysid, strnlen(sysid, sizeof(sysid))) & 0x7FFFFFFFUL;
            serial_num.set_and_save(crc);
        }
    }

    was_armed = armed;

    if (now - last_bcast_ms >= 10000U) {
        last_bcast_ms = now;

        // announce the battery tag information every 10s
        ardupilot_equipment_power_BatteryTag pkt {};

        pkt.serial_number = serial_num;
        pkt.num_cycles = num_cycles;
        pkt.armed_hours = armed_hours;
        pkt.battery_capacity_mAh = capacity_mAh;
        pkt.first_use_mins = first_use_min;
        pkt.last_arm_time_mins = last_use_min;

        uint8_t buffer[ARDUPILOT_EQUIPMENT_POWER_BATTERYTAG_MAX_SIZE];
        uint16_t total_size = ardupilot_equipment_power_BatteryTag_encode(&pkt, buffer, !periph.canfdout());

        periph.canard_broadcast(ARDUPILOT_EQUIPMENT_POWER_BATTERYTAG_SIGNATURE,
                                ARDUPILOT_EQUIPMENT_POWER_BATTERYTAG_ID,
                                CANARD_TRANSFER_PRIORITY_LOW,
                                &buffer[0],
                                total_size);
    }
}

#endif  // AP_PERIPH_BATTERY_TAG_ENABLED
