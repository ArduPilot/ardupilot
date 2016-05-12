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

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include "AP_GPS_QURT.h"
extern "C" {
#include <csr_gps_api.h>
#include <csr_gps_common.h>
}

extern const AP_HAL::HAL& hal;

AP_GPS_QURT::AP_GPS_QURT(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    HAP_PRINTF("Trying csr_gps_init");
    int ret = csr_gps_init("/dev/tty-3");
    if (ret == -1) {
        HAP_PRINTF("Trying csr_gps_deinit");
        csr_gps_deinit();
        ret = csr_gps_init("/dev/tty-3");
    }
    if (ret == 0) {
        HAP_PRINTF("Initialised csr_gps");
        initialised = true;
    } else {
        HAP_PRINTF("Failed to initialise csr_gps ret=%d", ret);
        initialised = false;
    }
}

AP_GPS_QURT::~AP_GPS_QURT(void)
{
    if (initialised) {
        csr_gps_deinit();
    }
}


// update internal state if new GPS information is available
bool
AP_GPS_QURT::read(void)
{
    if (!initialised) {
        return false;
    }
    struct osp_geo_data data {};
    if (csr_gps_get_geo_data(&data) != 0) {
        return false;
    }
    state.last_gps_time_ms = AP_HAL::millis();
    if (data.tow == last_tow) {
        // same data again
        return false;
    }

	if (data.nav_type & NAV_TYPE_4SV_OR_MORE_KF_SOLUTION ||
	    data.nav_type & NAV_TYPE_3SV_KF_SOLUTION) {
		state.status = AP_GPS::GPS_OK_FIX_3D;
	} else if (data.nav_type & NAV_TYPE_2SV_KF_SOLUTION) {
		state.status = AP_GPS::GPS_OK_FIX_2D;
	} else if (data.nav_type & NAV_TYPE_1SV_KF_SOLUTION) {
		state.status = AP_GPS::NO_FIX;
	} else {
		state.status = AP_GPS::NO_GPS;
	}

    state.num_sats = data.sv_in_fix;
    state.hdop = data.HDOP;
    state.vdop = 0;

    state.location.lat = bswap_32(data.lat);
    state.location.lng = bswap_32(data.lon);
    state.location.alt = data.alt_from_MSL;

    state.ground_speed = data.speed_over_ground;
    state.ground_course = data.course_over_ground*0.01f;

    // convert epoch timestamp back to gps epoch - evil hack until we get the genuine
    // raw week information (or APM switches to Posix epoch ;-) )
    state.time_week = data.ext_week_num;
    state.time_week_ms = data.tow;
    
    if (state.time_week == 0) {
        // reject bad time
        state.status = AP_GPS::NO_FIX;
    }
    
    state.have_vertical_velocity = true;
    float gps_heading = radians(state.ground_course);
    state.velocity.x = state.ground_speed * cosf(gps_heading);
    state.velocity.y = state.ground_speed * sinf(gps_heading);
    state.velocity.z = -data.climb_rate;
    state.speed_accuracy = data.est_hor_vel_error * 0.01f;
    state.horizontal_accuracy = data.est_hor_pos_error * 0.01f;
    state.vertical_accuracy = data.est_vert_pos_error * 0.01f;
    state.have_speed_accuracy = true;
    return true;
}
#endif
