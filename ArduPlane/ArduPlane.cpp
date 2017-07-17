/*
   Lead developer: Andrew Tridgell
 
   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See http://dev.ardupilot.org for details

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
    SCHED_TASK(update_flight_mode,    400,    100),
    SCHED_TASK(stabilize,             400,    100),
    SCHED_TASK(set_servos,            400,    100),
    SCHED_TASK(read_control_switch,     7,    100),
    SCHED_TASK(gcs_retry_deferred,     50,    500),
    SCHED_TASK(update_GPS_50Hz,        50,    300),
    SCHED_TASK(update_GPS_10Hz,        10,    400),
    SCHED_TASK(navigate,               10,    150),
    SCHED_TASK(update_compass,         10,    200),
    SCHED_TASK(read_airspeed,          10,    100),
    SCHED_TASK(update_alt,             10,    200),
    SCHED_TASK(adjust_altitude_target, 10,    200),
    SCHED_TASK(afs_fs_check,           10,    100),
    SCHED_TASK(gcs_update,             50,    500),
    SCHED_TASK(gcs_data_stream_send,   50,    500),
    SCHED_TASK(update_events,          50,    150),
    SCHED_TASK(read_battery,           10,    300),
    SCHED_TASK(compass_accumulate,     50,    200),
    SCHED_TASK(barometer_accumulate,   50,    150),
    SCHED_TASK(update_notify,          50,    300),
    SCHED_TASK(read_rangefinder,       50,    100),
    SCHED_TASK(ice_update,             10,    100),
    SCHED_TASK(compass_cal_update,     50,    50),
    SCHED_TASK(accel_cal_update,       10,    50),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,    50,    50),
#endif
    SCHED_TASK(one_second_loop,         1,    400),
    SCHED_TASK(check_long_failsafe,     3,    400),
    SCHED_TASK(read_receiver_rssi,     10,    100),
    SCHED_TASK(rpm_update,             10,    100),
    SCHED_TASK(airspeed_ratio_update,   1,    100),
    SCHED_TASK(update_mount,           50,    100),
    SCHED_TASK(update_trigger,         50,    100),
    SCHED_TASK(log_perf_info,         0.2,    100),
    SCHED_TASK(compass_save,          0.1,    200),
    SCHED_TASK(Log_Write_Fast,         25,    300),
    SCHED_TASK(update_logging1,        25,    300),
    SCHED_TASK(update_logging2,        25,    300),
    SCHED_TASK(update_soaring,         50,    400),
    SCHED_TASK(parachute_check,        10,    200),
    SCHED_TASK(terrain_update,         10,    200),
    SCHED_TASK(update_is_flying_5Hz,    5,    100),
    SCHED_TASK(dataflash_periodic,     50,    400),
    SCHED_TASK(avoidance_adsb_update,  10,    100),
    SCHED_TASK(button_update,           5,    100),
    SCHED_TASK(stats_update,            1,    100),
};

/*
  update AP_Stats
 */
void Plane::stats_update(void)
{
    g2.stats.update();
}


void Plane::setup() 
{
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    AP_Notify::flags.failsafe_battery = false;

    rssi.init();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

void Plane::loop()
{
    uint32_t loop_us = 1000000UL / scheduler.get_loop_rate_hz();

    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    perf.delta_us_fast_loop  = timer - perf.fast_loopTimer_us;
    G_Dt = perf.delta_us_fast_loop * 1.0e-6f;

    if (perf.delta_us_fast_loop > loop_us + 500) {
        perf.num_long++;
    }

    if (perf.delta_us_fast_loop > perf.G_Dt_max && perf.fast_loopTimer_us != 0) {
        perf.G_Dt_max = perf.delta_us_fast_loop;
    }

    if (perf.delta_us_fast_loop < perf.G_Dt_min || perf.G_Dt_min == 0) {
        perf.G_Dt_min = perf.delta_us_fast_loop;
    }
    perf.fast_loopTimer_us = timer;

    perf.mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    scheduler.run(loop_us);
}

void Plane::update_soft_armed()
{
    hal.util->set_soft_armed(arming.is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    DataFlash.set_vehicle_armed(hal.util->get_soft_armed());
}

// update AHRS system
void Plane::ahrs_update()
{
    update_soft_armed();

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // update hil before AHRS update
        gcs_update();
    }
#endif

    ahrs.update();

    if (should_log(MASK_LOG_IMU)) {
        Log_Write_IMU();
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

    // update inertial_nav for quadplane
    quadplane.inertial_nav.update(G_Dt);
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
}


/*
  update camera mount
 */
void Plane::update_mount(void)
{
#if MOUNT == ENABLED
    camera_mount.update();
#endif
}

/*
  update camera trigger
 */
void Plane::update_trigger(void)
{
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
    if (camera.check_trigger_pin()) {
        gcs().send_message(MSG_CAMERA_FEEDBACK);
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
        }
    }    
#endif
}

/*
  read and update compass
 */
void Plane::update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        compass.learn_offsets();
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass(compass);
        }
    } else {
        ahrs.set_compass(nullptr);
    }
}

/*
  if the compass is enabled then try to accumulate a reading
 */
void Plane::compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }    
}

/*
  try to accumulate a baro reading
 */
void Plane::barometer_accumulate(void)
{
    barometer.accumulate();
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
        Log_Write_IMU();

    if (should_log(MASK_LOG_ATTITUDE_MED))
        Log_Write_AOA_SSA();
}

/*
  do 10Hz logging - part2
 */
void Plane::update_logging2(void)
{
    if (should_log(MASK_LOG_CTUN))
        Log_Write_Control_Tuning();
    
    if (should_log(MASK_LOG_NTUN))
        Log_Write_Nav_Tuning();

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        DataFlash.Log_Write_Vibration(ins);
}


/*
  check for AFS failsafe check
 */
void Plane::afs_fs_check(void)
{
    // perform AFS failsafe checks
    afs.check(failsafe.last_heartbeat_ms, geofence_breached(), failsafe.AFS_last_valid_rc_ms);
}


/*
  update aux servo mappings
 */
void Plane::update_aux(void)
{
    SRV_Channels::enable_aux_servos();
}

void Plane::one_second_loop()
{
    // send a heartbeat
    gcs().send_message(MSG_HEARTBEAT);

    // make it possible to change control channel ordering at runtime
    set_control_channels();

#if HAVE_PX4_MIXER
    if (!hal.util->get_soft_armed() && (last_mixer_crc == -1)) {
        // if disarmed try to configure the mixer
        setup_failsafe_mixing();
    }
#endif // CONFIG_HAL_BOARD

    // make it possible to change orientation at runtime
    ahrs.set_orientation();

    adsb.set_stall_speed_cm(aparm.airspeed_min);

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    update_aux();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::NO;

#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data(DataFlash);
    }
#endif

    // update home position if soft armed and gps position has
    // changed. Update every 5s at most
    if (!hal.util->get_soft_armed() &&
        gps.last_message_time_ms() - last_home_update_ms > 5000 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            last_home_update_ms = gps.last_message_time_ms();
            update_home();
            
            // reset the landing altitude correction
            landing.alt_offset = 0;
    }
    
    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    update_sensor_status_flags();
}

void Plane::log_perf_info()
{
    if (scheduler.debug() != 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "PERF: %u/%u Dt=%u/%u Log=%u",
                          (unsigned)perf.num_long,
                          (unsigned)perf.mainLoop_count,
                          (unsigned)perf.G_Dt_max,
                          (unsigned)perf.G_Dt_min,
                          (unsigned)(DataFlash.num_dropped() - perf.last_log_dropped));
    }

    if (should_log(MASK_LOG_PM)) {
        Log_Write_Performance();
    }

    resetPerfData();
}

void Plane::compass_save()
{
    if (g.compass_enabled &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !hal.util->get_soft_armed()) {
        /*
          only save offsets when disarmed
         */
        compass.save_offsets();
    }
}

void Plane::terrain_update(void)
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();
#endif
}


void Plane::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
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
    gcs_send_airspeed_calibration(vg);
}


/*
  read the GPS and update position
 */
void Plane::update_GPS_50Hz(void)
{
    // get position from AHRS
    have_position = ahrs.get_position(current_loc);
    ahrs.get_relative_position_D_home(relative_altitude);
    relative_altitude *= -1.0f;

    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];
    gps.update();

    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            if (should_log(MASK_LOG_GPS)) {
                Log_Write_GPS(i);
            }
        }
    }
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
                init_home();

                // set system clock for log timestamps
                uint64_t gps_timestamp = gps.time_epoch_usec();
                
                hal.util->set_system_clock(gps_timestamp);

                // update signing timestamp
                GCS_MAVLINK::update_signing_timestamp(gps_timestamp);

                if (g.compass_enabled) {
                    // Set compass declination automatically
                    const Location &loc = gps.location();
                    compass.set_initial_location(loc.lat, loc.lng);
                }
                ground_start_count = 0;
            }
        }

        // see if we've breached the geo-fence
        geofence_check(false);

#if CAMERA == ENABLED
        if (camera.update_location(current_loc, plane.ahrs ) == true) {
            do_take_picture();
        }
#endif        

        // update wind estimate
        ahrs.estimate_wind();
    } else if (gps.status() < AP_GPS::GPS_OK_FIX_3D && ground_start_count != 0) {
        // lost 3D fix, start again
        ground_start_count = 5;
    }

    calc_gndspeed_undershoot();
}

/*
  main handling for AUTO mode
 */
void Plane::handle_auto_mode(void)
{
    uint16_t nav_cmd_id;

    if (mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        set_mode(RTL, MODE_REASON_MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    nav_cmd_id = mission.get_current_nav_cmd().id;

    if (quadplane.in_vtol_auto()) {
        quadplane.control_auto(next_WP_loc);
    } else if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)) {
        takeoff_calc_roll();
        takeoff_calc_pitch();
        calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        calc_nav_roll();
        calc_nav_pitch();
        
        // allow landing to restrict the roll limits
        nav_roll_cd = landing.constrain_roll(nav_roll_cd, g.level_roll_limit*100UL);

        if (landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        } else {
            calc_throttle();
        }
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            steer_state.hold_course_cd = -1;
        }
        calc_nav_roll();
        calc_nav_pitch();
        calc_throttle();
    }
}

/*
  main flight mode dependent update code 
 */
void Plane::update_flight_mode(void)
{
    enum FlightMode effective_mode = control_mode;
    if (control_mode == AUTO && g.auto_fbw_steer == 42) {
        effective_mode = FLY_BY_WIRE_A;
    }

    if (effective_mode != AUTO) {
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

    switch (effective_mode) 
    {
    case AUTO:
        handle_auto_mode();
        break;

    case AVOID_ADSB:
    case GUIDED:
        if (auto_state.vtol_loiter && quadplane.available()) {
            quadplane.guided_update();
            break;
        }
        // no break

    case RTL:
    case LOITER:
        calc_nav_roll();
        calc_nav_pitch();
        calc_throttle();
        break;
        
    case TRAINING: {
        training_manual_roll = false;
        training_manual_pitch = false;
        update_load_factor();
        
        // if the roll is past the set roll limit, then
        // we set target roll to the limit
        if (ahrs.roll_sensor >= roll_limit_cd) {
            nav_roll_cd = roll_limit_cd;
        } else if (ahrs.roll_sensor <= -roll_limit_cd) {
            nav_roll_cd = -roll_limit_cd;                
        } else {
            training_manual_roll = true;
            nav_roll_cd = 0;
        }
        
        // if the pitch is past the set pitch limits, then
        // we set target pitch to the limit
        if (ahrs.pitch_sensor >= aparm.pitch_limit_max_cd) {
            nav_pitch_cd = aparm.pitch_limit_max_cd;
        } else if (ahrs.pitch_sensor <= pitch_limit_min_cd) {
            nav_pitch_cd = pitch_limit_min_cd;
        } else {
            training_manual_pitch = true;
            nav_pitch_cd = 0;
        }
        if (fly_inverted()) {
            nav_pitch_cd = -nav_pitch_cd;
        }
        break;
    }

    case ACRO: {
        // handle locked/unlocked control
        if (acro_state.locked_roll) {
            nav_roll_cd = acro_state.locked_roll_err;
        } else {
            nav_roll_cd = ahrs.roll_sensor;
        }
        if (acro_state.locked_pitch) {
            nav_pitch_cd = acro_state.locked_pitch_cd;
        } else {
            nav_pitch_cd = ahrs.pitch_sensor;
        }
        break;
    }

    case AUTOTUNE:
    case FLY_BY_WIRE_A: {
        // set nav_roll and nav_pitch using sticks
        nav_roll_cd  = channel_roll->norm_input() * roll_limit_cd;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        update_load_factor();
        float pitch_input = channel_pitch->norm_input();
        if (pitch_input > 0) {
            nav_pitch_cd = pitch_input * aparm.pitch_limit_max_cd;
        } else {
            nav_pitch_cd = -(pitch_input * pitch_limit_min_cd);
        }
        adjust_nav_pitch_throttle();
        nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
        if (fly_inverted()) {
            nav_pitch_cd = -nav_pitch_cd;
        }
        if (failsafe.ch3_failsafe && g.short_fs_action == 2) {
            // FBWA failsafe glide
            nav_roll_cd = 0;
            nav_pitch_cd = 0;
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
        }
        if (g.fbwa_tdrag_chan > 0) {
            // check for the user enabling FBWA taildrag takeoff mode
            bool tdrag_mode = (hal.rcin->read(g.fbwa_tdrag_chan-1) > 1700);
            if (tdrag_mode && !auto_state.fbwa_tdrag_takeoff_mode) {
                if (auto_state.highest_airspeed < g.takeoff_tdrag_speed1) {
                    auto_state.fbwa_tdrag_takeoff_mode = true;
                    gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
                }
            }
        }
        break;
    }

    case FLY_BY_WIRE_B:
        // Thanks to Yury MonZon for the altitude limit code!
        nav_roll_cd = channel_roll->norm_input() * roll_limit_cd;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        update_load_factor();
        update_fbwb_speed_height();
        break;
        
    case CRUISE:
        /*
          in CRUISE mode we use the navigation code to control
          roll when heading is locked. Heading becomes unlocked on
          any aileron or rudder input
        */
        if ((channel_roll->get_control_in() != 0 ||
             rudder_input != 0)) {                
            cruise_state.locked_heading = false;
            cruise_state.lock_timer_ms = 0;
        }                 
        
        if (!cruise_state.locked_heading) {
            nav_roll_cd = channel_roll->norm_input() * roll_limit_cd;
            nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
            update_load_factor();
        } else {
            calc_nav_roll();
        }
        update_fbwb_speed_height();
        break;
        
    case STABILIZE:
        nav_roll_cd        = 0;
        nav_pitch_cd       = 0;
        // throttle is passthrough
        break;
        
    case CIRCLE:
        // we have no GPS installed and have lost radio contact
        // or we just want to fly around in a gentle circle w/o GPS,
        // holding altitude at the altitude we set when we
        // switched into the mode
        nav_roll_cd  = roll_limit_cd / 3;
        update_load_factor();
        calc_nav_pitch();
        calc_throttle();
        break;

    case MANUAL:
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, channel_roll->get_control_in_zero_dz());
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, channel_pitch->get_control_in_zero_dz());
        steering_control.steering = steering_control.rudder = channel_rudder->get_control_in_zero_dz();
        break;

    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL: {
        // set nav_roll and nav_pitch using sticks
        int16_t roll_limit = MIN(roll_limit_cd, quadplane.aparm.angle_max);
        nav_roll_cd  = (channel_roll->get_control_in() / 4500.0) * roll_limit;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
        float pitch_input = channel_pitch->norm_input();
        // Scale from normalized input [-1,1] to centidegrees
        if (quadplane.tailsitter_active()) {
            // For tailsitters, the pitch range is symmetrical: [-Q_ANGLE_MAX,Q_ANGLE_MAX]
            nav_pitch_cd = pitch_input * quadplane.aparm.angle_max;
        } else {
            // pitch is further constrained by LIM_PITCH_MIN/MAX which may impose
            // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
            if (pitch_input > 0) {
                nav_pitch_cd = pitch_input * MIN(aparm.pitch_limit_max_cd, quadplane.aparm.angle_max);
            } else {
                nav_pitch_cd = pitch_input * MIN(-pitch_limit_min_cd, quadplane.aparm.angle_max);
            }
            nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
        }
        break;
    }
        
    case INITIALISING:
        // handled elsewhere
        break;
    }
}

void Plane::update_navigation()
{
    // wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
    // ------------------------------------------------------------------------

    uint16_t radius = 0;
    
    switch(control_mode) {
    case AUTO:
        if (home_is_set != HOME_UNSET) {
            mission.update();
        }
        break;
            
    case RTL:
        if (quadplane.available() && quadplane.rtl_mode == 1 &&
            nav_controller->reached_loiter_target() &&
            AP_HAL::millis() - last_mode_change_ms > 1000) {
            set_mode(QRTL, MODE_REASON_UNKNOWN);
            break;
        } else if (g.rtl_autoland == 1 &&
            !auto_state.checked_for_autoland &&
            reached_loiter_target() && 
            labs(altitude_error_cm) < 1000) {
            // we've reached the RTL point, see if we have a landing sequence
            if (mission.jump_to_landing_sequence()) {
                // switch from RTL -> AUTO
                set_mode(AUTO, MODE_REASON_UNKNOWN);
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
                set_mode(AUTO, MODE_REASON_UNKNOWN);
            }

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            auto_state.checked_for_autoland = true;
        }
        radius = abs(g.rtl_radius);
        if (radius > 0) {
            loiter.direction = (g.rtl_radius < 0) ? -1 : 1;
        }
        // no break, fall through to LOITER

    case LOITER:
    case AVOID_ADSB:
    case GUIDED:
        update_loiter(radius);
        break;

    case CRUISE:
        update_cruise();
        break;

    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case INITIALISING:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CIRCLE:
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL:
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
                          auto_state.takeoff_altitude_rel_cm/100);
    }

    flight_stage = fs;

    if (should_log(MASK_LOG_MODE)) {
        Log_Write_Status();
    }
}

void Plane::update_alt()
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
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
    
    geofence_check(true);

    update_flight_stage();

    if (auto_throttle_mode && !throttle_suppressed) {        

        float distance_beyond_land_wp = 0;
        if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND && location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) {
            distance_beyond_land_wp = get_distance(current_loc, next_WP_loc);
        }

        SpdHgt_Controller->update_pitch_throttle(relative_target_altitude_cm(),
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
        if (control_mode==AUTO) {
            if (quadplane.in_vtol_auto()) {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_VTOL);
            } else if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_TAKEOFF);
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                if (landing.is_commanded_go_around() || flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
                    // abort mode is sticky, it must complete while executing NAV_LAND
                    set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND);
                } else if (landing.get_abort_throttle_enable() && channel_throttle->get_control_in() >= 90 &&
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
        } else {
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

    // tell AHRS the airspeed to true airspeed ratio
    airspeed.set_EAS2TAS(barometer.get_EAS2TAS());
}




#if OPTFLOW == ENABLED
// called at 50hz
void Plane::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        const Vector3f &posOffset = optflow.get_pos_offset();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update, posOffset);
        Log_Write_Optflow();
    }
}
#endif


/*
    If land_DisarmDelay is enabled (non-zero), check for a landing then auto-disarm after time expires

    only called from AP_Landing, when the landing library is ready to disarm
 */
void Plane::disarm_if_autoland_complete()
{
    if (landing.get_disarm_delay() > 0 &&
        !is_flying() &&
        arming.arming_required() != AP_Arming::NO &&
        arming.is_armed()) {
        /* we have auto disarm enabled. See if enough time has passed */
        if (millis() - auto_state.last_flying_ms >= landing.get_disarm_delay()*1000UL) {
            if (disarm_motors()) {
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

AP_HAL_MAIN_CALLBACKS(&plane);
