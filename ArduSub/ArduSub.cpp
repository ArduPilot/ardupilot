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

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    SCHED_TASK(fifty_hz_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,  200,    160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK(auto_disarm_check,     10,     50),
    SCHED_TASK(auto_trim,             10,     75),
    SCHED_TASK(read_rangefinder,      20,    100),
    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(run_nav_updates,       50,    100),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK(update_turn_counter,   10,     50),
    SCHED_TASK(compass_accumulate,   100,    100),
    SCHED_TASK(barometer_accumulate,  50,     90),
    SCHED_TASK(update_notify,         50,     90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(ekf_check,             10,     75),
    SCHED_TASK(lost_vehicle_check,    10,     50),
    SCHED_TASK(gcs_check_input,      400,    180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK(gcs_send_deferred,     50,    550),
    SCHED_TASK(gcs_data_stream_send,  50,    550),
    SCHED_TASK(update_mount,          50,     75),
    SCHED_TASK(update_trigger,        50,     75),
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK(dataflash_periodic,    400,    300),
    SCHED_TASK(perf_update,           0.1,    75),
#if RPM_ENABLED == ENABLED
    SCHED_TASK(rpm_update,            10,    200),
#endif
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
    SCHED_TASK(terrain_update,        10,    100),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK(gripper_update,            10,     75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
};


void Sub::setup()
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = AP_HAL::micros();
}

/*
  if the compass is enabled then try to accumulate a reading
 */
void Sub::compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
  try to accumulate a baro reading
 */
void Sub::barometer_accumulate(void)
{
    barometer.accumulate();
}

void Sub::perf_update(void)
{
    if (should_log(MASK_LOG_PM)) {
        Log_Write_Performance();
    }
    if (scheduler.debug()) {
        gcs_send_text_fmt(MAV_SEVERITY_WARNING, "PERF: %u/%u %lu %lu\n",
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void Sub::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available);
}


// Main loop - 400hz
void Sub::fast_loop()
{
    if (control_mode != MANUAL) { //don't run rate controller in manual mode
        // run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // send outputs to the motors library
    motors_output();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading
    check_ekf_yaw_reset();

    crash_check(MAIN_LOOP_SECONDS);

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
    // check auto_armed status
    update_auto_armed();

    // check pilot input failsafe
    failsafe_manual_control_check();

    // Update servo output
    RC_Channels::set_pwm_all();
    SRV_Channels::limit_slew_rate(SRV_Channel::k_mount_tilt, g.cam_slew_limit, 0.02f);
    SRV_Channels::output_ch_all();
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


// update camera trigger
void Sub::update_trigger(void)
{
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
    if (camera.check_trigger_pin()) {
        gcs_send_message(MSG_CAMERA_FEEDBACK);
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
        }
    }
#endif
}

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
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info());
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
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif
}

void Sub::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

// three_hz_loop - 3.3hz loop
void Sub::three_hz_loop()
{
    set_leak_status(leak_detector.update());

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

    update_events();

#if CH6_TUNE_ENABLED == ENABLED
    // update ch6 in flight tuning
    tuning();
#endif
}

// one_hz_loop - runs at 1Hz
void Sub::one_hz_loop()
{
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
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

    check_usb_mux();

    // update position controller alt limits
    update_poscon_alt_max();

    // enable/disable raw gyro/accel logging
    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));

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

        // checks to initialise home and take location based pictures
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

#if CAMERA == ENABLED
            if (camera.update_location(current_loc, sub.ahrs) == true) {
                do_take_picture();
            }
#endif
        }
    }
}

void Sub::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Sub::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0) {
        return;
    }

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    } else {
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Sub::update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if (force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

void Sub::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
    ahrs_view.update();
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
