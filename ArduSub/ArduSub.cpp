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
    SCHED_TASK_CLASS(OpticalFlow,          &sub.optflow,             update,         200, 160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK(read_rangefinder,      20,    100),
    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK(update_turn_counter,   10,     50),
    SCHED_TASK_CLASS(AP_Baro,             &sub.barometer,    accumulate,          50,  90),
    SCHED_TASK_CLASS(AP_Notify,           &sub.notify,       update,              50,  90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_send,        400, 550),
#if MOUNT == ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &sub.camera_mount, update,              50,  75),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &sub.camera,       update_trigger,      50,  75),
#endif
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK_CLASS(DataFlash_Class,     &sub.DataFlash,    periodic_tasks,     400, 300),
    SCHED_TASK_CLASS(AP_InertialSensor,   &sub.ins,          periodic,           400,  50),
    SCHED_TASK_CLASS(AP_Scheduler,        &sub.scheduler,    update_logging,     0.1,  75),
#if RPM_ENABLED == ENABLED
    SCHED_TASK(rpm_update,            10,    200),
#endif
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
    SCHED_TASK(terrain_update,        10,    100),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &sub.g2.gripper,       update,              10,  75),
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

constexpr int8_t Sub::_failsafe_priorities[5];

void Sub::setup()
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
}

void Sub::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_loop_period_s();
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
    rc().read_input();
    SRV_Channels::output_ch_all();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Sub::update_batt_compass()
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if (g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass();
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
        DataFlash.Log_Write_Rate(&ahrs_view, motors, attitude_control, pos_control);
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
        pos_control.write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration();
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
        DataFlash.Log_Write_Rate(&ahrs_view, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU();
    }
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

    // init compass location for declination
    init_compass_location();

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    ahrs.set_likely_flying(hal.util->get_soft_armed());
}

// called at 50hz
void Sub::update_GPS()
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            gps_updated = true;
            break;
        }
    }

    if (gps_updated) {
#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Sub::read_AHRS()
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

bool Sub::control_check_barometer()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if (!ap.depth_sensor_present) { // can't hold depth without a depth sensor
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor is not connected.");
        return false;
    } else if (failsafe.sensor_health) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor error.");
        return false;
    }
#endif
    return true;
}

AP_HAL_MAIN_CALLBACKS(&sub);
