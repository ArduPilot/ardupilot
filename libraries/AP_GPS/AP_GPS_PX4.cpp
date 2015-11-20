// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

//  GPS proxy driver for APM on PX4 platforms
//
//  This driver subscribes on PX4s vehicle_gps_position topic and presents the data received to APM.
//  The publisher could be an UAVCAN GNSS module like http://docs.zubax.com/Zubax_GNSS, or any other GNSS
//  hardware supported by the PX4 ecosystem.
//
//  Code by Holger Steinhaus

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AP_GPS_PX4.h"

#include <uORB/uORB.h>

#include <math.h>

extern const AP_HAL::HAL& hal;

AP_GPS_PX4::AP_GPS_PX4(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    _gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
}


#define MS_PER_WEEK             7*24*3600*1000LL
#define DELTA_POSIX_GPS_EPOCH   315964800LL*1000LL
#define LEAP_MS_GPS_2014        16*1000LL

// update internal state if new GPS information is available
bool
AP_GPS_PX4::read(void)
{
    bool updated = false;
    orb_check(_gps_sub, &updated);

    if (updated) {
        if (OK == orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &_gps_pos)) {
            state.last_gps_time_ms = AP_HAL::millis();
            state.status  = (AP_GPS::GPS_Status) (_gps_pos.fix_type | AP_GPS::NO_FIX);
            state.num_sats = _gps_pos.satellites_used;
            state.hdop = uint16_t(_gps_pos.eph*100.0f + .5f);

            if (_gps_pos.fix_type >= 2) {
                state.location.lat = _gps_pos.lat;
                state.location.lng = _gps_pos.lon;
                state.location.alt = _gps_pos.alt/10;

                state.ground_speed = _gps_pos.vel_m_s;
                state.ground_course_cd = wrap_360_cd(degrees(_gps_pos.cog_rad)*100);
                state.hdop = _gps_pos.eph*100;

                // convert epoch timestamp back to gps epoch - evil hack until we get the genuine
                // raw week information (or APM switches to Posix epoch ;-) )
                uint64_t epoch_ms = uint64_t(_gps_pos.time_utc_usec/1000.+.5);
                uint64_t gps_ms = epoch_ms - DELTA_POSIX_GPS_EPOCH + LEAP_MS_GPS_2014;
                state.time_week = uint16_t(gps_ms / uint64_t(MS_PER_WEEK));
                state.time_week_ms = uint32_t(gps_ms - uint64_t(state.time_week)*MS_PER_WEEK);

                if (_gps_pos.time_utc_usec == 0) {
                  // This is a work-around for https://github.com/PX4/Firmware/issues/1474
                  // reject position reports with invalid time, as APM adjusts it's clock after the first lock has been aquired
                  state.status = AP_GPS::NO_FIX;
                }
            }
            if (_gps_pos.fix_type >= 3) {
                state.have_vertical_velocity = _gps_pos.vel_ned_valid;
                state.velocity.x = _gps_pos.vel_n_m_s;
                state.velocity.y = _gps_pos.vel_e_m_s;
                state.velocity.z = _gps_pos.vel_d_m_s;
                state.speed_accuracy = _gps_pos.s_variance_m_s;
                state.have_speed_accuracy = true;
            }
            else {
                state.have_vertical_velocity = false;
            }
        }
    }

    return updated;
}
#endif
