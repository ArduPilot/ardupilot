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
#include "version.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Sub class
 */
Sub::Sub(void) :
    DataFlash {FIRMWARE_STRING},
          mission(ahrs,
                  FUNCTOR_BIND_MEMBER(&Sub::start_command, bool, const AP_Mission::Mission_Command &),
                  FUNCTOR_BIND_MEMBER(&Sub::verify_command_callback, bool, const AP_Mission::Mission_Command &),
                  FUNCTOR_BIND_MEMBER(&Sub::exit_mission, void)),
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
          baro_alt(0),
          baro_climbrate(0.0f),
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
          pos_control(ahrs_view, inertial_nav, motors, attitude_control,
                      g.p_alt_hold, g.p_vel_z, g.pid_accel_z,
                      g.p_pos_xy, g.pi_vel_xy),
#if AVOIDANCE_ENABLED == ENABLED
          avoid(ahrs, inertial_nav, fence, g2.proximity),
#endif
          wp_nav(inertial_nav, ahrs_view, pos_control, attitude_control),
          circle_nav(inertial_nav, ahrs_view, pos_control),
          pmTest1(0),
          fast_loopTimer(0),
          mainLoop_count(0),
          ServoRelayEvents(relay),
#if CAMERA == ENABLED
          camera(&relay),
#endif
#if MOUNT == ENABLED
          camera_mount(ahrs, current_loc),
#endif
#if AC_FENCE == ENABLED
          fence(ahrs, inertial_nav),
#endif
#if AC_RALLY == ENABLED
          rally(ahrs),
#endif
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
          terrain(ahrs, mission, rally),
#endif
          in_mavlink_delay(false),
          gcs_out_of_time(false),
          param_loader(var_info),
          last_pilot_yaw_input_ms(0)
{
    memset(&current_loc, 0, sizeof(current_loc));

    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;

    failsafe.last_heartbeat_ms = 0;

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    failsafe.manual_control = true;
#endif
}

Sub sub;
