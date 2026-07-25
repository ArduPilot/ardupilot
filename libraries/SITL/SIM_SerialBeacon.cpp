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
  Base class for serially-attached simulated positioning beacons
*/

#include "SIM_SerialBeacon.h"

#if AP_SIM_NOOPLOOP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>

using namespace SITL;

void SerialBeacon::update()
{
    if (!init_sitl_pointer()) {
        return;
    }

    // send data out at the beacon's reading rate:
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_sent_ms < reading_interval_ms()) {
        return;
    }
    last_sent_ms = now_ms;

    // the beacon system origin is the same as the vehicle/EKF origin:
    const Location &origin = _sitl->state.home;
    if (!origin.initialised()) {
        return;
    }

    // truth location of the simulated vehicle:
    const Location veh_loc {
        int32_t(_sitl->state.latitude * 1.0e7),
        int32_t(_sitl->state.longitude * 1.0e7),
        int32_t(_sitl->state.altitude * 1.0e2),
        Location::AltFrame::ABSOLUTE
    };

    // vehicle position in NED metres relative to the beacon origin:
    const Vector3f pos_ned = origin.get_distance_NED(veh_loc);

    send_data(pos_ned);
}

#endif  // AP_SIM_NOOPLOOP_ENABLED
