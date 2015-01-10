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
        AP_BattMonitor_SMBus(mon, instance, mon_state)
{
    // orb subscription for battery status
    _batt_sub = orb_subscribe(ORB_ID(battery_status));
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
        }
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
