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
    : logger(g.log_bitmask),
          control_mode(MANUAL),
          motors(MAIN_LOOP_RATE),
          auto_mode(Auto_WP),
          guided_mode(Guided_WP),
          auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
          inertial_nav(ahrs),
          ahrs_view(ahrs, ROTATION_NONE),
          attitude_control(ahrs_view, aparm, motors, scheduler.get_loop_period_s()),
          pos_control(ahrs_view, inertial_nav, motors, attitude_control, scheduler.get_loop_period_s()),
          wp_nav(inertial_nav, ahrs_view, pos_control, attitude_control),
          loiter_nav(inertial_nav, ahrs_view, pos_control, attitude_control),
          circle_nav(inertial_nav, ahrs_view, pos_control),
          param_loader(var_info)
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    failsafe.pilot_input = true;
#endif
}

Sub sub;
AP_Vehicle& vehicle = sub;
