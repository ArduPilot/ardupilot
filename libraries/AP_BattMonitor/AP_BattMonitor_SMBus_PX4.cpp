/*
  Battery SMBus PX4 driver
*/
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

#include <AP_HAL.h>
#include <GCS_MAVLink.h>
#include <GCS.h>
#include <AP_Notify.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "AP_BattMonitor_SMBus_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_batt_smbus.h>
#include <uORB/topics/battery_status.h>

extern const AP_HAL::HAL& hal;

// Constructor
AP_BattMonitor_SMBus_PX4::AP_BattMonitor_SMBus_PX4(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state) :
        AP_BattMonitor_SMBus(mon, instance, mon_state),
        _batt_fd(-1),
        _capacity_updated(false)
{
    // orb subscription for battery status
    _batt_sub = orb_subscribe(ORB_ID(battery_status));
}

void AP_BattMonitor_SMBus_PX4::init()
{
    // open the device
    _batt_fd = open(BATT_SMBUS0_DEVICE_PATH, O_RDWR);
    if (_batt_fd == -1) {
        hal.console->printf("Unable to open " BATT_SMBUS0_DEVICE_PATH);
        _state.healthy = false;
    }
}

// read - read latest voltage and current
void AP_BattMonitor_SMBus_PX4::read()
{
    bool updated = false;
    struct battery_status_s batt_status;

    // check if new info has arrived from the orb
    orb_check(_batt_sub, &updated);

    // retrieve latest info
    if (updated) {
        if (OK == orb_copy(ORB_ID(battery_status), _batt_sub, &batt_status)) {
            _state.voltage = batt_status.voltage_v;
            _state.current_amps = batt_status.current_a;
            _state.last_time_micros = hal.scheduler->micros();
            _state.current_total_mah = batt_status.discharged_mah;
            _state.healthy = true;

            // send a mavlink message too all components if the system is about to power off
            if(!_state.is_powering_off && batt_status.is_powering_off) {
                // create command long mavlink message
                mavlink_command_long_t cmd_msg;
                memset(&cmd_msg, 0, sizeof(cmd_msg));
                cmd_msg.command = MAV_CMD_POWER_OFF_INITIATED;
                // create message
                mavlink_message_t msg;
                mavlink_msg_command_long_encode(0, MAV_COMP_ID_ALL, &msg, &cmd_msg);
                // forward to all components
                GCS_MAVLINK::send_on_all_channels(&msg);
                GCS_MAVLINK::send_statustext_all(PSTR("System is shutting down NOW..."));
                AP_Notify::flags.powering_off = 1;
            }
            _state.is_powering_off = batt_status.is_powering_off;

            // read capacity
            if ((_batt_fd >= 0) && !_capacity_updated) {
                uint16_t tmp;
                if (ioctl(_batt_fd, BATT_SMBUS_GET_CAPACITY, (unsigned long)&tmp) == OK) {
                    _capacity_updated = true;
                    set_capacity(tmp);
                }
            }
        }
    } else if (_state.healthy) {
        // timeout after 5 seconds
        if ((hal.scheduler->micros() - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
            _state.healthy = false;
        }
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
