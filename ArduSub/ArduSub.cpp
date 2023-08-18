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

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Sub, &sub, func)

/*
  scheduler table - all tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

 */

const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &sub.ins, update),
    // run low level rate controllers that only require IMU data
    FAST_TASK(run_rate_controller),
    // send outputs to the motors library immediately
    FAST_TASK(motors_output),
     // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),
    // Inertial Nav
    FAST_TASK(read_inertia),
    // check if ekf has reset target heading
    FAST_TASK(check_ekf_yaw_reset),
    // run the attitude controllers
    FAST_TASK(update_flight_mode),
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),
    // check if we've reached the surface or bottom
    FAST_TASK(update_surface_and_bottom_detector),
#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    FAST_TASK_CLASS(AP_Mount, &sub.camera_mount, update_fast),
#endif

    SCHED_TASK(fifty_hz_loop,         50,     75,   3),
    SCHED_TASK_CLASS(AP_GPS, &sub.gps, update, 50, 200,   6),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow,          &sub.optflow,             update,         200, 160,   9),
#endif
    SCHED_TASK(update_batt_compass,   10,    120,  12),
    SCHED_TASK(read_rangefinder,      20,    100,  15),
    SCHED_TASK(update_altitude,       10,    100,  18),
    SCHED_TASK(three_hz_loop,          3,     75,  21),
    SCHED_TASK(update_turn_counter,   10,     50,  24),
    SCHED_TASK_CLASS(AP_Baro,             &sub.barometer,    accumulate,          50,  90,  27),
    SCHED_TASK(one_hz_loop,            1,    100,  33),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180,  36),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_send,        400, 550,  39),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &sub.camera_mount, update,              50,  75,  45),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &sub.camera,       update,              50,  75,  48),
#endif
    SCHED_TASK(ten_hz_logging_loop,   10,    350,  51),
    SCHED_TASK(twentyfive_hz_logging, 25,    110,  54),
    SCHED_TASK_CLASS(AP_Logger,           &sub.logger,       periodic_tasks,     400, 300,  57),
    SCHED_TASK_CLASS(AP_InertialSensor,   &sub.ins,          periodic,           400,  50,  60),
    SCHED_TASK_CLASS(AP_Scheduler,        &sub.scheduler,    update_logging,     0.1,  75,  63),
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,              &sub.rpm_sensor,   update,              10, 200,  66),
#endif
    SCHED_TASK(terrain_update,        10,    100,  72),
#if AP_GRIPPER_ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &sub.g2.gripper,   update,              10,  75,  75),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK(stats_update,           1,    200,  76),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75,  78),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75,  81),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75,  84),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75,  87),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75,  90),
#endif
};

void Sub::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Sub::_failsafe_priorities[5];

void Sub::run_rate_controller()
{
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    motors.set_dt(last_loop_time_s);
    attitude_control.set_dt(last_loop_time_s);
    pos_control.set_dt(last_loop_time_s);

    //don't run rate controller in manual or motordetection modes
    if (control_mode != Mode::Number::MANUAL && control_mode != Mode::Number::MOTOR_DETECT) {
        // run low level rate controllers that only require IMU data and set loop time
        attitude_control.rate_controller_run();
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

    if (AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Sub::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        ahrs_view.Write_Rate(motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        motors.Log_Write();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (sub.flightmode->requires_GPS() || !sub.flightmode->has_manual_throttle())) {
        pos_control.write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control.control_monitor_log();
    }
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

// twentyfive_hz_logging_loop
// should be run at 25hz
void Sub::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        ahrs_view.Write_Rate(motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_IMU();
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

#if AP_FENCE_ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AP_FENCE_ENABLED

#if AP_SERVORELAYEVENTS_ENABLED
    ServoRelayEvents.update_events();
#endif
}

// one_hz_loop - runs at 1Hz
void Sub::one_hz_loop()
{
    bool arm_check = arming.pre_arm_checks(false);
    ap.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_gps_check = position_ok();
    AP_Notify::flags.flying = motors.armed();

    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }

    if (!motors.armed()) {
        motors.update_throttle_range();
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // log terrain data
    terrain_logging();

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    set_likely_flying(hal.util->get_soft_armed());
}

void Sub::read_AHRS()
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
    // <true> tells AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
    ahrs_view.update();
}

// read baro and rangefinder altitude at 10hz
void Sub::update_altitude()
{
    // read in baro altitude
    read_barometer();

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        AP::ins().write_notch_log_messages();
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
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

// vehicle specific waypoint info helpers
bool Sub::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Sub::send_nav_controller_output()
    distance = sub.wp_nav.get_wp_distance_to_destination() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Sub::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Sub::send_nav_controller_output()
    bearing = sub.wp_nav.get_wp_bearing_to_destination() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Sub::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // no crosstrack error reported, see GCS_MAVLINK_Sub::send_nav_controller_output()
    xtrack_error = 0;
    return true;
}

#if STATS_ENABLED == ENABLED
/*
  update AP_Stats
*/
void Sub::stats_update(void)
{
    g2.stats.set_flying(motors.armed());
    g2.stats.update();
}
#endif

AP_HAL_MAIN_CALLBACKS(&sub);
