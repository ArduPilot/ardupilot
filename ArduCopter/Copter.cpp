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
#include "version.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Copter class
 */
Copter::Copter(void) :
    DataFlash{FIRMWARE_STRING},
    flight_modes(&g.flight_mode1),
    mission(ahrs, 
            FUNCTOR_BIND_MEMBER(&Copter::start_command, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Copter::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Copter::exit_mission, void)),
    control_mode(STABILIZE),
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
    loiter_time_max(0),
    loiter_time(0),
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry(ahrs, battery, rangefinder),
#endif
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
#if AC_AVOID_ENABLED == ENABLED
    avoid(ahrs, inertial_nav, fence, g2.proximity),
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
    precland(ahrs, inertial_nav),
#endif
#if FRAME_CONFIG == HELI_FRAME
    // ToDo: Input Manager is only used by Heli for 3.3, but will be used by all frames for 3.4
    input_manager(MAIN_LOOP_RATE),
#endif
    in_mavlink_delay(false),
    gcs_out_of_time(false),
    param_loader(var_info)
{
    memset(&current_loc, 0, sizeof(current_loc));

    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;
}

Copter copter;
