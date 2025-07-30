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
//  MSP GPS driver
//
#include <AP_MSP/msp.h>
#include "AP_GPS_MSP.h"

#if HAL_MSP_GPS_ENABLED

// Reading does nothing in this class; we simply return whether or not
// the latest reading has been consumed.  By calling this function we assume
// the caller is consuming the new data;
bool AP_GPS_MSP::read(void)
{
    if (new_data) {
        new_data = false;
        return true;
    }
    return false;
}

// handles an incoming msp message and sets
// corresponding gps data appropriately;
void AP_GPS_MSP::handle_msp(const MSP::msp_gps_data_message_t &pkt)
{
    check_new_itow(pkt.ms_tow, sizeof(pkt));

    state.time_week = pkt.gps_week;
    state.time_week_ms = pkt.ms_tow;
    state.status = (AP_GPS::GPS_Status)pkt.fix_type;
    state.num_sats = pkt.satellites_in_view;
    state.location = {
        pkt.latitude,
        pkt.longitude,
        pkt.msl_altitude,
        Location::AltFrame::ABSOLUTE
    };
    state.hdop = pkt.hdop;
    state.vdop = GPS_UNKNOWN_DOP;

    state.have_vertical_velocity = true;
    Vector3f vel;
    vel.x = pkt.ned_vel_north * 0.01;
    vel.y = pkt.ned_vel_east * 0.01;
    vel.z = pkt.ned_vel_down * 0.01;
    state.velocity = vel;

    velocity_to_speed_course(state);

    state.have_speed_accuracy = true;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;
    state.have_vertical_velocity = true;

    state.horizontal_accuracy = pkt.horizontal_pos_accuracy * 0.01;
    state.vertical_accuracy = pkt.vertical_pos_accuracy * 0.01;
    state.speed_accuracy = pkt.horizontal_vel_accuracy * 0.01;

    state.last_gps_time_ms = AP_HAL::millis();

    if (pkt.true_yaw != 65535) {
        state.gps_yaw = wrap_360(pkt.true_yaw*0.01);
        state.have_gps_yaw = true;
        state.gps_yaw_time_ms = state.last_gps_time_ms;
    }

    new_data = pkt.fix_type>0;
}

/*
  return velocity lag in seconds
 */
bool AP_GPS_MSP::get_lag(float &lag_sec) const
{
    // measured on Matek M8Q
    lag_sec = 0.11;
    return true;
}

#endif // HAL_MSP_GPS_ENABLED
