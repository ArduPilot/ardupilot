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
#include "Sub.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Sub class
 */
Sub::Sub()
    : DataFlash(g.log_bitmask),
          control_mode(MANUAL),
          motors(MAIN_LOOP_RATE),
          scaleLongDown(1),
          auto_mode(Auto_WP),
          guided_mode(Guided_WP),
          circle_pilot_yaw_override(false),
          initial_armed_bearing(0),
          desired_climb_rate(0),
          loiter_time_max(0),
          loiter_time(0),
          climb_rate(0),
          target_rangefinder_alt(0.0f),
          auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
          yaw_look_at_WP_bearing(0.0f),
          yaw_look_at_heading(0),
          yaw_look_at_heading_slew(0),
          yaw_look_ahead_bearing(0.0f),
          condition_value(0),
          condition_start(0),
          G_Dt(MAIN_LOOP_SECONDS),
          inertial_nav(ahrs),
          ahrs_view(ahrs, ROTATION_NONE),
          attitude_control(ahrs_view, aparm, motors, MAIN_LOOP_SECONDS),
          pos_control(ahrs_view, inertial_nav, motors, attitude_control),
          wp_nav(inertial_nav, ahrs_view, pos_control, attitude_control),
          loiter_nav(inertial_nav, ahrs_view, pos_control, attitude_control),
          circle_nav(inertial_nav, ahrs_view, pos_control),
          param_loader(var_info),
          last_pilot_yaw_input_ms(0)
{
    memset(&current_loc, 0, sizeof(current_loc));

    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;

    failsafe.last_heartbeat_ms = 0;

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    failsafe.pilot_input = true;
#endif
}

Sub sub;
