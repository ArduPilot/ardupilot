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

//
//  ExternalAHRS GPS driver
//
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include "AP_GPS_ExternalAHRS.h"

#if HAL_EXTERNAL_AHRS_ENABLED

// Reading does nothing in this class; we simply return whether or not
// the latest reading has been consumed.  By calling this function we assume
// the caller is consuming the new data;
bool AP_GPS_ExternalAHRS::read(void)
{
    if (new_data) {
        new_data = false;
        return true;
    }
    return false;
}

// handles an incoming ExternalAHRS message and sets
// corresponding gps data appropriately;
void AP_GPS_ExternalAHRS::handle_external(const AP_ExternalAHRS::gps_data_message_t &pkt)
{
    check_new_itow(pkt.ms_tow, sizeof(pkt));

    state.time_week = pkt.gps_week;
    state.time_week_ms = pkt.ms_tow;
    if (pkt.fix_type == 0) {
        state.status = AP_GPS::NO_FIX;
    } else {
        state.status = (AP_GPS::GPS_Status)pkt.fix_type;
    }
    state.num_sats = pkt.satellites_in_view;

    Location loc = {};
    loc.lat = pkt.latitude;
    loc.lng = pkt.longitude;
    loc.alt = pkt.msl_altitude;

    state.location = loc;
    state.hdop = pkt.hdop;
    state.vdop = pkt.vdop;

    state.have_vertical_velocity = true;
    state.velocity.x = pkt.ned_vel_north;
    state.velocity.y = pkt.ned_vel_east;
    state.velocity.z = pkt.ned_vel_down;

    velocity_to_speed_course(state);

    state.have_speed_accuracy = true;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;
    state.have_vertical_velocity = true;

    state.horizontal_accuracy = pkt.horizontal_pos_accuracy;
    state.vertical_accuracy = pkt.vertical_pos_accuracy;
    state.speed_accuracy = pkt.horizontal_vel_accuracy;

    state.last_gps_time_ms = AP_HAL::millis();

    new_data = true;
}

/*
  return velocity lag in seconds
 */
bool AP_GPS_ExternalAHRS::get_lag(float &lag_sec) const
{
    // fixed assumed lag
    lag_sec = 0.11;
    return true;
}

#endif // HAL_EXTERNAL_AHRS_ENABLED

