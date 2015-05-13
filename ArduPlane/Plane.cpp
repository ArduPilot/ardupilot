/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

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
  constructor for main Plane class
 */

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

Plane::Plane(void) :
    ins_sample_rate(AP_InertialSensor::RATE_50HZ),
#if defined(HAL_BOARD_LOG_DIRECTORY)
    DataFlash(HAL_BOARD_LOG_DIRECTORY),
#endif
    flight_modes(&g.flight_mode1),
#if AP_AHRS_NAVEKF_AVAILABLE
    ahrs(ins, barometer, gps, rangefinder),
#else
    ahrs(ins, barometer, gps),
#endif
    L1_controller(ahrs),
    TECS_controller(ahrs, aparm),
    rollController(ahrs, aparm, DataFlash),
    pitchController(ahrs, aparm, DataFlash),
    yawController(ahrs, aparm),
    steerController(ahrs),
    num_gcs(MAVLINK_COMM_NUM_BUFFERS),
    nav_controller(&L1_controller),
    SpdHgt_Controller(&TECS_controller),
    ServoRelayEvents(relay),
#if CAMERA == ENABLED
    camera(&relay),
#endif
    rally(ahrs),
    control_mode(INITIALISING),
    previous_mode(INITIALISING),
    oldSwitchPosition(254),
    ground_start_count(5),
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry(ahrs, battery),
#endif
    airspeed(aparm),
    flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL),
    aerodynamic_load_factor(1.0f),
    mission(ahrs, 
            AP_HAL_MEMBERPROC(&Plane::start_command_callback), 
            AP_HAL_MEMBERPROC(&Plane::verify_command_callback), 
            AP_HAL_MEMBERPROC(&Plane::exit_mission_callback)),
#if AP_TERRAIN_AVAILABLE
    terrain(ahrs, mission, rally),
#endif
#if OBC_FAILSAFE == ENABLED
    obc(mission, barometer, gps, rcmap),
#endif
    home(ahrs.get_home()),
    G_Dt(0.02f),
#if MOUNT == ENABLED
    camera_mount(ahrs, current_loc),
#endif
    arming(ahrs, barometer, compass, home_is_set, AP_HAL_MEMBERPROC(&Plane::gcs_send_text_P)),
    param_loader(var_info)
{
    elevon.trim1 = 1500;
    elevon.trim2 = 1500;
    elevon.ch1_temp = 1500;
    elevon.ch2_temp = 1500;

    steer_state.hold_course_cd = -1;
    steer_state.locked_course = false;
    steer_state.locked_course_err = 0;

    auto_state.takeoff_complete = true;
    auto_state.next_wp_no_crosstrack = true;
    auto_state.no_crosstrack = true;
    auto_state.next_turn_angle = 90.0f;
}

Plane plane;
