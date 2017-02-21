/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"
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
  constructor for main Sub class
 */

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Sub::Sub(void) :
    flight_modes(&g.flight_mode1),
    mission(ahrs, 
            FUNCTOR_BIND_MEMBER(&Sub::start_command, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Sub::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Sub::exit_mission, void)),
    control_mode(STABILIZE),
    motors(MAIN_LOOP_RATE),
    scaleLongDown(1),
    wp_bearing(0),
    home_bearing(0),
    home_distance(0),
    wp_distance(0),
    auto_mode(Auto_TakeOff),
    guided_mode(Guided_TakeOff),
    rtl_state(RTL_InitialClimb),
    rtl_state_complete(false),
    circle_pilot_yaw_override(false),
    simple_cos_yaw(1.0f),
    simple_sin_yaw(0.0f),
    super_simple_last_bearing(0),
    super_simple_cos_yaw(1.0),
    super_simple_sin_yaw(0.0f),
    initial_armed_bearing(0),
    desired_climb_rate(0),
    loiter_time_max(0),
    loiter_time(0),
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
    condition_value(0),
    condition_start(0),
    G_Dt(MAIN_LOOP_SECONDS),
    inertial_nav(ahrs),
    attitude_control(ahrs, aparm, motors, MAIN_LOOP_SECONDS),
    pos_control(ahrs, inertial_nav, motors, attitude_control,
                g.p_alt_hold, g.p_vel_z, g.pid_accel_z,
                g.p_pos_xy, g.pi_vel_xy),
	avoid(ahrs, inertial_nav, fence, g2.proximity),
    wp_nav(inertial_nav, ahrs, pos_control, attitude_control),
    circle_nav(inertial_nav, ahrs, pos_control),
    pmTest1(0),
    fast_loopTimer(0),
    mainLoop_count(0),
    rtl_loiter_start_time(0),
    auto_trim_counter(0),
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
#if SPRAYER == ENABLED
    sprayer(&inertial_nav),
#endif
#if PARACHUTE == ENABLED
    parachute(relay),
#endif
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    terrain(ahrs, mission, rally),
#endif
#if PRECISION_LANDING == ENABLED
    precland(ahrs, inertial_nav, g.pi_precland, MAIN_LOOP_SECONDS),
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
}

Sub sub;
