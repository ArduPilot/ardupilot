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
#include "Copter.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Copter class
 */
Copter::Copter(void)
    : DataFlash(fwver.fw_string, g.log_bitmask),
    flight_modes(&g.flight_mode1),
    control_mode(STABILIZE),
    scaleLongDown(1),
    simple_cos_yaw(1.0f),
    simple_sin_yaw(0.0f),
    super_simple_last_bearing(0),
    super_simple_cos_yaw(1.0),
    super_simple_sin_yaw(0.0f),
    initial_armed_bearing(0),
    climb_rate(0),
    target_rangefinder_alt(0.0f),
    baro_alt(0),
    baro_climbrate(0.0f),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
    yaw_look_at_WP_bearing(0.0f),
    yaw_look_at_heading(0),
    yaw_look_at_heading_slew(0),
    yaw_look_ahead_bearing(0.0f),
    inertial_nav(ahrs),
    pmTest1(0),
    fast_loopTimer(0),
    mainLoop_count(0),
    auto_trim_counter(0),
    in_mavlink_delay(false),
    param_loader(var_info),
    flightmode(&mode_stabilize)
{
    memset(&current_loc, 0, sizeof(current_loc));

    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;
}

Copter copter;
