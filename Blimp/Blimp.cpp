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

#include "Blimp.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Blimp, &blimp, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Blimp, &blimp, func)

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
const AP_Scheduler::Task Blimp::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &blimp.ins, update),
    // send outputs to the motors library immediately
    FAST_TASK(motors_output),
     // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),
    // Inertial Nav
    FAST_TASK(read_inertia),
    // check if ekf has reset target heading or position
    FAST_TASK(check_ekf_reset),
    // run the attitude controllers
    FAST_TASK(update_flight_mode),
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),

    SCHED_TASK(rc_loop,              100,    130,   3),
    SCHED_TASK(throttle_loop,         50,     75,   6),
    SCHED_TASK_CLASS(AP_GPS, &blimp.gps, update, 50, 200,   9),
    SCHED_TASK(update_batt_compass,   10,    120,  12),
    SCHED_TASK_CLASS(RC_Channels,          (RC_Channels*)&blimp.g2.rc_channels,      read_aux_all,    10,     50,  15),
    SCHED_TASK(arm_motors_check,      10,     50,  18),
    SCHED_TASK(update_altitude,       10,    100,  21),
    SCHED_TASK(three_hz_loop,          3,     75,  24),
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &blimp.ServoRelayEvents,      update_events, 50,     75,  27),
    SCHED_TASK_CLASS(AP_Baro,              &blimp.barometer,             accumulate,    50,     90,  30),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(full_rate_logging,     50,    50,  33),
#endif
    SCHED_TASK_CLASS(AP_Notify,            &blimp.notify,              update,          50,  90,  36),
    SCHED_TASK(one_hz_loop,            1,    100,  39),
    SCHED_TASK(ekf_check,             10,     75,  42),
    SCHED_TASK(check_vibration,       10,     50,  45),
    SCHED_TASK(gpsglitch_check,       10,     50,  48),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&blimp._gcs,          update_receive, 400, 180,  51),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&blimp._gcs,          update_send,    400, 550,  54),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350,  57),
    SCHED_TASK(twentyfive_hz_logging, 25,    110,  60),
    SCHED_TASK_CLASS(AP_Logger,      &blimp.logger,           periodic_tasks, 400, 300,  63),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &blimp.ins,                 periodic,       400,  50,  66),
    SCHED_TASK_CLASS(AP_Scheduler,         &blimp.scheduler,           update_logging, 0.1,  75,  69),
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats,             &blimp.g2.stats,            update,           1, 100,  75),
#endif
};

void Blimp::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Blimp::_failsafe_priorities[4];

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Blimp::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Blimp::throttle_loop()
{
    // check auto_armed status
    update_auto_armed();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Blimp::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if (AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

// Full rate logging of attitude, rate and pid loops
void Blimp::full_rate_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
    if (should_log(MASK_LOG_PID)) {
        Log_Write_PIDs();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Blimp::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
    // log EKF attitude data
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
        if (rssi.enabled()) {
            logger.Write_RSSI();
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
}


// twentyfive_hz_logging - should be run at 25hz
void Blimp::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }
}

// three_hz_loop - 3.3hz loop
void Blimp::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();
}

// one_hz_loop - runs at 1Hz
void Blimp::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    AP_Notify::flags.flying = !ap.land_complete;
}

void Blimp::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);

    IGNORE_RETURN(ahrs.get_velocity_NED(vel_ned));
    IGNORE_RETURN(ahrs.get_relative_position_NED_home(pos_ned));

    vel_yaw = ahrs.get_yaw_rate_earth();
    Vector2f vel_xy_filtd = vel_xy_filter.apply({vel_ned.x, vel_ned.y});
    vel_ned_filtd = {vel_xy_filtd.x, vel_xy_filtd.y, vel_z_filter.apply(vel_ned.z)};
    vel_yaw_filtd = vel_yaw_filter.apply(vel_yaw);
}

// read baro and log control tuning
void Blimp::update_altitude()
{
    // read in baro altitude
    read_barometer();

    if (should_log(MASK_LOG_CTUN)) {
        AP::ins().write_notch_log_messages();
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }
}

//Conversions are in 2D so that up remains up in world frame when the blimp is not exactly level.
void Blimp::rotate_BF_to_NE(Vector2f &vec)
{
    float ne_x = vec.x*ahrs.cos_yaw() - vec.y*ahrs.sin_yaw();
    float ne_y = vec.x*ahrs.sin_yaw() + vec.y*ahrs.cos_yaw();
    vec.x = ne_x;
    vec.y = ne_y;
}

void Blimp::rotate_NE_to_BF(Vector2f &vec)
{
    float bf_x = vec.x*ahrs.cos_yaw() + vec.y*ahrs.sin_yaw();
    float bf_y = -vec.x*ahrs.sin_yaw() + vec.y*ahrs.cos_yaw();
    vec.x = bf_x;
    vec.y = bf_y;

}

/*
  constructor for main Blimp class
 */
Blimp::Blimp(void)
    : logger(g.log_bitmask),
      flight_modes(&g.flight_mode1),
      control_mode(Mode::Number::MANUAL),
      rc_throttle_control_in_filter(1.0f),
      inertial_nav(ahrs),
      param_loader(var_info),
      flightmode(&mode_manual)
{
}

Blimp blimp;
AP_Vehicle& vehicle = blimp;

AP_HAL_MAIN_CALLBACKS(&blimp);
