/*
   Lead developer: Andrew Tridgell
 
   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger, Tom Pittenger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See https://dev.ardupilot.org for details

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

#include "Plane.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Plane, &plane, func, rate_hz, max_time_micros)


/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in Hz) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Plane::scheduler_tasks[] = {
                           // Units:   Hz      us
    SCHED_TASK(ahrs_update,           400,    400),
    SCHED_TASK(read_radio,             50,    100),
    SCHED_TASK(check_short_failsafe,   50,    100),
    SCHED_TASK(update_speed_height,    50,    200),
    SCHED_TASK(update_control_mode,   400,    100),
    SCHED_TASK(stabilize,             400,    100),
    SCHED_TASK(set_servos,            400,    100),
    SCHED_TASK(update_throttle_hover, 100,     90),
    SCHED_TASK(read_control_switch,     7,    100),
    SCHED_TASK(update_GPS_50Hz,        50,    300),
    SCHED_TASK(update_GPS_10Hz,        10,    400),
    SCHED_TASK(navigate,               10,    150),
    SCHED_TASK(update_compass,         10,    200),
    SCHED_TASK(read_airspeed,          10,    100),
    SCHED_TASK(update_alt,             10,    200),
    SCHED_TASK(adjust_altitude_target, 10,    200),
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,           10,    100),
#endif
    SCHED_TASK(ekf_check,              10,     75),
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_receive,   300,  500),
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_send,      300,  750),
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &plane.ServoRelayEvents, update_events,          50,  150),
    SCHED_TASK_CLASS(AP_BattMonitor, &plane.battery, read, 10, 300),
    SCHED_TASK_CLASS(AP_Baro, &plane.barometer, accumulate, 50, 150),
    SCHED_TASK_CLASS(AP_Notify,      &plane.notify,  update, 50, 300),
    SCHED_TASK(read_rangefinder,       50,    100),
    SCHED_TASK_CLASS(AP_ICEngine, &plane.g2.ice_control, update, 10, 100),
    SCHED_TASK_CLASS(Compass,          &plane.compass,              cal_update, 50, 50),
    SCHED_TASK(accel_cal_update,       10,    50),
#if OPTFLOW == ENABLED
    SCHED_TASK_CLASS(OpticalFlow, &plane.optflow, update,    50,    50),
#endif
    SCHED_TASK(one_second_loop,         1,    400),
    SCHED_TASK(check_long_failsafe,     3,    400),
    SCHED_TASK(rpm_update,             10,    100),
    SCHED_TASK(airspeed_ratio_update,   1,    100),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount, &plane.camera_mount, update, 50, 100),
#endif // HAL_MOUNT_ENABLED
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera, &plane.camera, update,      50, 100),
#endif // CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Scheduler, &plane.scheduler, update_logging,         0.2,    100),
    SCHED_TASK(compass_save,          0.1,    200),
    SCHED_TASK(Log_Write_Fast,         25,    300),
    SCHED_TASK(update_logging1,        25,    300),
    SCHED_TASK(update_logging2,        25,    300),
#if SOARING_ENABLED == ENABLED
    SCHED_TASK(update_soaring,         50,    400),
#endif
    SCHED_TASK(parachute_check,        10,    200),
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK_CLASS(AP_Terrain, &plane.terrain, update, 10, 200),
#endif // AP_TERRAIN_AVAILABLE
    SCHED_TASK(update_is_flying_5Hz,    5,    100),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Logger, &plane.logger, periodic_tasks, 50, 400),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &plane.ins, periodic, 50, 50),
    SCHED_TASK(avoidance_adsb_update,  10,    100),
    SCHED_TASK_CLASS(RC_Channels,       (RC_Channels*)&plane.g2.rc_channels, read_aux_all,           10,    200),
    SCHED_TASK_CLASS(AP_Button, &plane.button, update, 5, 100),
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats, &plane.g2.stats, update, 1, 100),
#endif
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper, &plane.g2.gripper, update, 10, 75),
#endif
#if OSD_ENABLED == ENABLED
    SCHED_TASK(publish_osd_info, 1, 10),
#endif
#if GENERATOR_ENABLED
    SCHED_TASK_CLASS(AP_Generator_RichenPower,          &plane.generator,      update,    10,     50),
#endif
#if LANDING_GEAR_ENABLED == ENABLED
    SCHED_TASK(landing_gear_update, 5, 50),
#endif
#if EFI_ENABLED
    SCHED_TASK(efi_update,             10,    200),
#endif
};

void Plane::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Plane::_failsafe_priorities[7];

// update AHRS system
void Plane::ahrs_update()
{
    arming.update_soft_armed();

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // update hil before AHRS update
        gcs().update_receive();
    }
#endif

    ahrs.update();

    if (should_log(MASK_LOG_IMU)) {
        logger.Write_IMU();
    }

    // calculate a scaled roll limit based on current pitch
    roll_limit_cd = aparm.roll_limit_cd;
    pitch_limit_min_cd = aparm.pitch_limit_min_cd;

    if (!quadplane.tailsitter_active()) {
        roll_limit_cd *= ahrs.cos_pitch();
        pitch_limit_min_cd *= fabsf(ahrs.cos_roll());
    }

    // updated the summed gyro used for ground steering and
    // auto-takeoff. Dot product of DCM.c with gyro vector gives earth
    // frame yaw rate
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);

    // check if we have had a yaw reset from the EKF
    quadplane.check_yaw_reset();

    // update inertial_nav for quadplane
    quadplane.inertial_nav.update();
}

/*
  update 50Hz speed/height controller
 */
void Plane::update_speed_height(void)
{
    if (auto_throttle_mode) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // throttle suppressed, as this needs to be running for
	    // takeoff detection
        SpdHgt_Controller->update_50hz();
    }

    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        quadplane.update_throttle_mix();
    }
}


/*
  read and update compass
 */
void Plane::update_compass(void)
{
    if (AP::compass().enabled() && compass.read()) {
        ahrs.set_compass(&compass);
    }
}

/*
  do 10Hz logging
 */
void Plane::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_IMU))
        logger.Write_IMU();

    if (should_log(MASK_LOG_ATTITUDE_MED))
        logger.Write_AOA_SSA(ahrs);
}

/*
  do 10Hz logging - part2
 */
void Plane::update_logging2(void)
{
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#else
        write_notch_log_messages();
#endif
    }
    
    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
        Log_Write_Guided();
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        logger.Write_Vibration();
}


/*
  check for AFS failsafe check
 */
#if ADVANCED_FAILSAFE == ENABLED
void Plane::afs_fs_check(void)
{
    // perform AFS failsafe checks
    afs.check(failsafe.last_heartbeat_ms, geofence_breached(), failsafe.AFS_last_valid_rc_ms);
}
#endif

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

void Plane::one_second_loop()
{
    // make it possible to change control channel ordering at runtime
    set_control_channels();

#if HAL_WITH_IO_MCU
    iomcu.setup_mixing(&rcmap, g.override_channel.get(), g.mixing_gain, g2.manual_rc_mask);
#endif

    // make it possible to change orientation at runtime
    ahrs.update_orientation();

    adsb.set_stall_speed_cm(aparm.airspeed_min);
    adsb.set_max_speed(aparm.airspeed_max);

    ahrs.writeDefaultAirSpeed((float)((aparm.airspeed_min + aparm.airspeed_max)/2));

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    SRV_Channels::enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::Required::NO;

#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif

    // update home position if NOT armed and gps position has
    // changed. Update every 5s at most
    if (!arming.is_armed() &&
        gps.last_message_time_ms() - last_home_update_ms > 5000 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            last_home_update_ms = gps.last_message_time_ms();
            update_home();
            
            // reset the landing altitude correction
            landing.alt_offset = 0;
    }
}

void Plane::compass_save()
{
    if (AP::compass().enabled() &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !hal.util->get_soft_armed()) {
        /*
          only save offsets when disarmed
         */
        compass.save_offsets();
    }
}

void Plane::efi_update(void)
{
#if EFI_ENABLED
    g2.efi.update();
#endif
}

/*
  once a second update the airspeed calibration ratio
 */
void Plane::airspeed_ratio_update(void)
{
    if (!airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // don't calibrate when not moving
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    if (labs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max_cd ||
        ahrs.pitch_sensor < pitch_limit_min_cd) {
        // don't calibrate when going beyond normal flight envelope
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg, aparm.airspeed_max);
}


/*
  read the GPS and update position
 */
void Plane::update_GPS_50Hz(void)
{
    gps.update();

    // get position from AHRS
    have_position = ahrs.get_position(current_loc);
    ahrs.get_relative_position_D_home(relative_altitude);
    relative_altitude *= -1.0f;
}

/*
  read update GPS position - 10Hz update
 */
void Plane::update_GPS_10Hz(void)
{
    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else {
                if (!set_home_persistently(gps.location())) {
                    // silently ignore failure...
                }

                next_WP_loc = prev_WP_loc = home;

                ground_start_count = 0;
            }
        }

        // see if we've breached the geo-fence
        geofence_check(false);

        // update wind estimate
        ahrs.estimate_wind();
    } else if (gps.status() < AP_GPS::GPS_OK_FIX_3D && ground_start_count != 0) {
        // lost 3D fix, start again
        ground_start_count = 5;
    }

    calc_gndspeed_undershoot();
}

/*
  main control mode dependent update code
 */
void Plane::update_control_mode(void)
{
    Mode *effective_mode = control_mode;
    if (control_mode == &mode_auto && g.auto_fbw_steer == 42) {
        effective_mode = &mode_fbwa;
    }

    if (effective_mode != &mode_auto) {
        // hold_course is only used in takeoff and landing
        steer_state.hold_course_cd = -1;
    }

    // ensure we are fly-forward when we are flying as a pure fixed
    // wing aircraft. This helps the EKF produce better state
    // estimates as it can make stronger assumptions
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        ahrs.set_fly_forward(false);
    } else if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        ahrs.set_fly_forward(landing.is_flying_forward());
    } else {
        ahrs.set_fly_forward(true);
    }

    effective_mode->update();
}

void Plane::update_navigation()
{
    // wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
    // ------------------------------------------------------------------------

    uint16_t radius = 0;
    uint16_t qrtl_radius = abs(g.rtl_radius);
    if (qrtl_radius == 0) {
        qrtl_radius = abs(aparm.loiter_radius);
    }
    
    switch (control_mode->mode_number()) {
    case Mode::Number::AUTO:
        if (ahrs.home_is_set()) {
            mission.update();
        }
        break;
            
    case Mode::Number::RTL:
        if (quadplane.available() && quadplane.rtl_mode == 1 &&
            (nav_controller->reached_loiter_target() ||
             current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc) ||
             auto_state.wp_distance < MAX(qrtl_radius, quadplane.stopping_distance())) &&
            AP_HAL::millis() - last_mode_change_ms > 1000) {
            /*
              for a quadplane in RTL mode we switch to QRTL when we
              are within the maximum of the stopping distance and the
              RTL_RADIUS
             */
            set_mode(mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
            break;
        } else if (g.rtl_autoland == 1 &&
            !auto_state.checked_for_autoland &&
            reached_loiter_target() && 
            labs(altitude_error_cm) < 1000) {
            // we've reached the RTL point, see if we have a landing sequence
            if (mission.jump_to_landing_sequence()) {
                // switch from RTL -> AUTO
                mission.set_force_resume(true);
                set_mode(mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);
            }

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            auto_state.checked_for_autoland = true;
        }
        else if (g.rtl_autoland == 2 &&
            !auto_state.checked_for_autoland) {
            // Go directly to the landing sequence
            if (mission.jump_to_landing_sequence()) {
                // switch from RTL -> AUTO
                mission.set_force_resume(true);
                set_mode(mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);
            }

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            auto_state.checked_for_autoland = true;
        }
        radius = abs(g.rtl_radius);
        if (radius > 0) {
            loiter.direction = (g.rtl_radius < 0) ? -1 : 1;
        }
        // fall through to LOITER
        FALLTHROUGH;

    case Mode::Number::LOITER:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::TAKEOFF:
        update_loiter(radius);
        break;

    case Mode::Number::CRUISE:
        update_cruise();
        break;

    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::TRAINING:
    case Mode::Number::INITIALISING:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CIRCLE:
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::QAUTOTUNE:
    case Mode::Number::QACRO:
        // nothing to do
        break;
    }
}

/*
  set the flight stage
 */
void Plane::set_flight_stage(AP_Vehicle::FixedWing::FlightStage fs)
{
    if (fs == flight_stage) {
        return;
    }

    landing.handle_flight_stage_change(fs == AP_Vehicle::FixedWing::FLIGHT_LAND);

    if (fs == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Landing aborted, climbing to %dm",
                        int(auto_state.takeoff_altitude_rel_cm/100));
    }

    flight_stage = fs;
    Log_Write_Status();
}

void Plane::update_alt()
{
    barometer.update();

    if (quadplane.available()) {
        quadplane.motors->set_air_density_ratio(barometer.get_air_density_ratio());
    }

    // calculate the sink rate.
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f*sink_rate;
#if PARACHUTE == ENABLED
    parachute.set_sink_rate(auto_state.sink_rate);
#endif
    geofence_check(true);

    update_flight_stage();

    if (auto_throttle_mode && !throttle_suppressed) {        

        float distance_beyond_land_wp = 0;
        if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND && current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
            distance_beyond_land_wp = current_loc.get_distance(next_WP_loc);
        }

        float target_alt = relative_target_altitude_cm();

        if (control_mode == &mode_rtl && !rtl.done_climb && g2.rtl_climb_min > 0) {
            // ensure we do the initial climb in RTL. We add an extra
            // 10m in the demanded height to push TECS to climb
            // quickly
            target_alt = MAX(target_alt, prev_WP_loc.alt + (g2.rtl_climb_min+10)*100);
        }

        SpdHgt_Controller->update_pitch_throttle(target_alt,
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 distance_beyond_land_wp,
                                                 get_takeoff_pitch_min_cd(),
                                                 throttle_nudge,
                                                 tecs_hgt_afe(),
                                                 aerodynamic_load_factor);
    }
}

/*
  recalculate the flight_stage
 */
void Plane::update_flight_stage(void)
{
    // Update the speed & height controller states
    if (auto_throttle_mode && !throttle_suppressed) {        
        if (control_mode == &mode_auto) {
            if (quadplane.in_vtol_auto()) {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_VTOL);
            } else if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_TAKEOFF);
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                if (landing.is_commanded_go_around() || flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
                    // abort mode is sticky, it must complete while executing NAV_LAND
                    set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND);
                } else if (landing.get_abort_throttle_enable() && get_throttle_input() >= 90 &&
                           landing.request_go_around()) {
                    gcs().send_text(MAV_SEVERITY_INFO,"Landing aborted via throttle");
                    set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND);
                } else {
                    set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_LAND);
                }
            } else if (quadplane.in_assisted_flight()) {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_VTOL);
            } else {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_NORMAL);
            }
        } else if (control_mode != &mode_takeoff) {
            // If not in AUTO then assume normal operation for normal TECS operation.
            // This prevents TECS from being stuck in the wrong stage if you switch from
            // AUTO to, say, FBWB during a landing, an aborted landing or takeoff.
            set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_NORMAL);
        }
    } else if (quadplane.in_vtol_mode() ||
               quadplane.in_assisted_flight()) {
        set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_VTOL);
    } else {
        set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_NORMAL);
    }
}




/*
    If land_DisarmDelay is enabled (non-zero), check for a landing then auto-disarm after time expires

    only called from AP_Landing, when the landing library is ready to disarm
 */
void Plane::disarm_if_autoland_complete()
{
    if (landing.get_disarm_delay() > 0 &&
        !is_flying() &&
        arming.arming_required() != AP_Arming::Required::NO &&
        arming.is_armed()) {
        /* we have auto disarm enabled. See if enough time has passed */
        if (millis() - auto_state.last_flying_ms >= landing.get_disarm_delay()*1000UL) {
            if (arming.disarm(AP_Arming::Method::AUTOLANDED)) {
                gcs().send_text(MAV_SEVERITY_INFO,"Auto disarmed");
            }
        }
    }
}



/*
  the height above field elevation that we pass to TECS
 */
float Plane::tecs_hgt_afe(void)
{
    /*
      pass the height above field elevation as the height above
      the ground when in landing, which means that TECS gets the
      rangefinder information and thus can know when the flare is
      coming.
    */
    float hgt_afe;
    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        hgt_afe = height_above_target();
        hgt_afe -= rangefinder_correction();
    } else {
        // when in normal flight we pass the hgt_afe as relative
        // altitude to home
        hgt_afe = relative_altitude;
    }
    return hgt_afe;
}

#if OSD_ENABLED == ENABLED
void Plane::publish_osd_info()
{
    AP_OSD::NavInfo nav_info;
    nav_info.wp_distance = auto_state.wp_distance;
    nav_info.wp_bearing = nav_controller->target_bearing_cd();
    nav_info.wp_xtrack_error = nav_controller->crosstrack_error();
    nav_info.wp_number = mission.get_current_nav_index();
    osd.set_nav_info(nav_info);
}
#endif

// set target location (for use by scripting)
bool Plane::set_target_location(const Location& target_loc)
{
    if (plane.control_mode != &plane.mode_guided) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    plane.guided_WP_loc = target_loc;
    // add home alt if needed
    if (plane.guided_WP_loc.relative_alt) {
        plane.guided_WP_loc.alt += plane.home.alt;
        plane.guided_WP_loc.relative_alt = 0;
    }
    plane.set_guided_WP();
    return true;
}

// set target location (for use by scripting)
bool Plane::get_target_location(Location& target_loc)
{
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::AUTO:
    case Mode::Number::LOITER:
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
        target_loc = next_WP_loc;
        return true;
        break;
    default:
        break;
    }
    return false;
}

AP_HAL_MAIN_CALLBACKS(&plane);
