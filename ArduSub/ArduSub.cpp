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

// ArduSub scheduling, originally copied from ArduCopter

#include "Sub.h"

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
AP_Task Sub::scheduler_tasks[] = {
    make_task("fifty_hz_loop",              &sub, &Sub::fifty_hz_loop,         50,     75),
    make_task("update_GPS",                 &sub, &Sub::update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    make_task("update_optical_flow",        &sub, &Sub::update_optical_flow,  200,    160),
#endif
    make_task("update_batt_compass",        &sub, &Sub::update_batt_compass,   10,    120),
    make_task("read_rangefinder",           &sub, &Sub::read_rangefinder,      20,    100),
    make_task("update_altitude",            &sub, &Sub::update_altitude,       10,    100),
    make_task("three_hz_loop",              &sub, &Sub::three_hz_loop,          3,     75),
    make_task("update_turn_counter",        &sub, &Sub::update_turn_counter,   10,     50),
    make_task("compass_accumulate",         &sub, &Sub::compass_accumulate,   100,    100),
    make_task("barometer_accumulate",       &sub, &Sub::barometer_accumulate,  50,     90),
    make_task("update_notify",              &sub, &Sub::update_notify,         50,     90),
    make_task("one_hz_loop",                &sub, &Sub::one_hz_loop,            1,    100),
    make_task("gcs_check_input",            &sub, &Sub::gcs_check_input,      400,    180),
    make_task("gcs_send_heartbeat",         &sub, &Sub::gcs_send_heartbeat,     1,    110),
    make_task("gcs_send_deferred",          &sub, &Sub::gcs_send_deferred,     50,    550),
    make_task("gcs_data_stream_send",       &sub, &Sub::gcs_data_stream_send,  50,    550),
    make_task("update_mount",               &sub, &Sub::update_mount,          50,     75),
#if CAMERA == ENABLED
    make_task("update_trigger",             &sub, &Sub::update_trigger,        50,     75),
#endif
    make_task("ten_hz_logging_loop",        &sub, &Sub::ten_hz_logging_loop,   10,    350),
    make_task("twentyfive_hz_logging",      &sub, &Sub::twentyfive_hz_logging, 25,    110),
    make_task("dataflash_periodic",         &sub, &Sub::dataflash_periodic,    400,   300),
    make_task("ins_periodic",               &sub, &Sub::ins_periodic,          400,    50),
    make_task("perf_update",                &sub, &Sub::perf_update,           0.1,    75),
#if RPM_ENABLED == ENABLED
    make_task("rpm_update",                 &sub, &Sub::rpm_update,            10,    200),
#endif
    make_task("compass_cal_update",         &sub, &Sub::compass_cal_update,   100,    100),
    make_task("accel_cal_update",           &sub, &Sub::accel_cal_update,      10,    100),
    make_task("terrain_update",             &sub, &Sub::terrain_update,        10,    100),
#if GRIPPER_ENABLED == ENABLED
    make_task("gripper_update",             &sub, &Sub::gripper_update,        10,     75),
#endif
#ifdef USERHOOK_FASTLOOP
    make_task("userhook_FastLoop",          &sub, &Sub::userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    make_task("userhook_50Hz",              &sub, &Sub::userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    make_task("userhook_MediumLoop",        &sub, &Sub::userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    make_task("userhook_SlowLoop",          &sub, &Sub::userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    make_task("userhook_SuperSlowLoop",     &sub, &Sub::userhook_SuperSlowLoop, 1,     75),
#endif
};


void Sub::setup()
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    perf_info.reset(scheduler.get_loop_rate_hz());
    fast_loopTimer = AP_HAL::micros();
}

void Sub::perf_update(void)
{
    if (should_log(MASK_LOG_PM)) {
        Log_Write_Performance();
    }
    if (scheduler.debug()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PERF: %u/%u max=%lu min=%lu avg=%lu sd=%lu",
                          (unsigned)perf_info.get_num_long_running(),
                          (unsigned)perf_info.get_num_loops(),
                          (unsigned long)perf_info.get_max_time(),
                          (unsigned long)perf_info.get_min_time(),
                          (unsigned long)perf_info.get_avg_time(),
                          (unsigned long)perf_info.get_stddev_time());
    }
    perf_info.reset(scheduler.get_loop_rate_hz());
    pmTest1 = 0;
}

void Sub::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info.check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();
    
    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    const uint32_t loop_us = scheduler.get_loop_period_us();
    const uint32_t time_available = (timer + loop_us) - micros();
    scheduler.run(time_available > loop_us ? 0u : time_available);
}


// Main loop - 400hz
void Sub::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();

    if (control_mode != MANUAL) { //don't run rate controller in manual mode
        // run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }

    // send outputs to the motors library
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading
    check_ekf_yaw_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've reached the surface or bottom
    update_surface_and_bottom_detector();

#if MOUNT == ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// 50 Hz tasks
void Sub::fifty_hz_loop()
{
    // check pilot input failsafe
    failsafe_pilot_input_check();

    failsafe_crash_check();

    failsafe_ekf_check();

    failsafe_sensors_check();

    // Update rc input/output
    RC_Channels::set_pwm_all();
    SRV_Channels::output_ch_all();
}

// updates the status of notify
// should be called at 50hz
void Sub::update_notify()
{
    notify.update();
}

// update_mount - update camera mount position
// should be run at 50hz
void Sub::update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif
}

#if CAMERA == ENABLED
// update camera trigger
void Sub::update_trigger(void)
{
    camera.update_trigger();
}
#endif

// update_batt_compass - read battery and compass
// should be called at 10hz
void Sub::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if (g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Sub::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && mode_requires_GPS(control_mode)) {
        Log_Write_Nav_Tuning();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration(ins);
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control.control_monitor_log();
    }
}

// twentyfive_hz_logging_loop
// should be run at 25hz
void Sub::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU(ins);
    }
}

void Sub::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

void Sub::ins_periodic()
{
    ins.periodic();
}

// three_hz_loop - 3.3hz loop
void Sub::three_hz_loop()
{
    leak_detector.update();

    failsafe_leak_check();

    failsafe_internal_pressure_check();

    failsafe_internal_temperature_check();

    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

    ServoRelayEvents.update_events();
}

// one_hz_loop - runs at 1Hz
void Sub::one_hz_loop()
{
    bool arm_check = arming.pre_arm_checks(false);
    ap.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_gps_check = position_ok();

    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        // set all throttle channel settings
        motors.set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // update position controller alt limits
    update_poscon_alt_max();

    // log terrain data
    terrain_logging();
}

// called at 50hz
void Sub::update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // log GPS message
            if (should_log(MASK_LOG_GPS) && !ahrs.have_ekf_logging()) {
                DataFlash.Log_Write_GPS(gps, i);
            }

            gps_updated = true;
        }
    }

    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Sub::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
    // <true> tells AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
    ahrs_view.update(true);
}

// read baro and rangefinder altitude at 10hz
void Sub::update_altitude()
{
    // read in baro altitude
    read_barometer();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

AP_HAL_MAIN_CALLBACKS(&sub);
