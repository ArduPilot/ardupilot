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

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Blimp, &blimp, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Blimp::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,              100,    130),
    SCHED_TASK(throttle_loop,         50,     75),
    SCHED_TASK_CLASS(AP_GPS, &blimp.gps, update, 50, 200),
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK_CLASS(RC_Channels,          (RC_Channels*)&blimp.g2.rc_channels,      read_aux_all,    10,     50),
    SCHED_TASK(arm_motors_check,      10,     50),
    // SCHED_TASK(auto_disarm_check,     10,     50),
    // SCHED_TASK(auto_trim,             10,     75),
    SCHED_TASK(update_altitude,       10,    100),
    // SCHED_TASK(run_nav_updates,       50,    100),
    // SCHED_TASK(update_throttle_hover,100,     90),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &blimp.ServoRelayEvents,      update_events, 50,     75),
    SCHED_TASK_CLASS(AP_Baro,              &blimp.barometer,           accumulate,      50,  90),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(fourhundred_hz_logging,400,    50),
#endif
    SCHED_TASK_CLASS(AP_Notify,            &blimp.notify,              update,          50,  90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(ekf_check,             10,     75),
    SCHED_TASK(check_vibration,       10,     50),
    // SCHED_TASK(gpsglitch_check,       10,     50),
    // SCHED_TASK(landinggear_update,    10,     75),
    // SCHED_TASK(standby_update,        100,    75),
    // SCHED_TASK(lost_vehicle_check,    10,     50),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&blimp._gcs,          update_receive, 400, 180),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&blimp._gcs,          update_send,    400, 550),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK_CLASS(AP_Logger,      &blimp.logger,           periodic_tasks, 400, 300),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &blimp.ins,                 periodic,       400,  50),

    SCHED_TASK_CLASS(AP_Scheduler,         &blimp.scheduler,           update_logging, 0.1,  75),
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
    // SCHED_TASK_CLASS(AP_TempCalibration,   &blimp.g2.temp_calibration, update,          10, 100),

#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats,             &blimp.g2.stats,            update,           1, 100),
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

// Main loop - 50hz
void Blimp::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();

    // send outputs to the motors library immediately
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading or position
    check_ekf_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }

    AP_Vehicle::fast_loop(); //just does gyro fft
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
//copied in from Copter's Attitude.cpp
float Blimp::get_non_takeoff_throttle()
{
    return 0.0f; //MIR no idle throttle.
}

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

    if (AP::compass().enabled()) {
        // update compass with throttle value - used for compassmot
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

// Full rate logging of attitude, rate and pid loops
// should be run at 400hz
void Blimp::fourhundred_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !blimp.flightmode->logs_attitude()) {
        Log_Write_Attitude();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Blimp::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST) && !blimp.flightmode->logs_attitude()) {
        Log_Write_Attitude();
    }
    // log EKF attitude data
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
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

    arming.update();

    if (!motors->armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.update_orientation();

        // update_using_interlock();
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    AP_Notify::flags.flying = !ap.land_complete;
}

void Blimp::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
}

// read baro and log control tuning
void Blimp::update_altitude()
{
    // read in baro altitude
    read_barometer();

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#else
        write_notch_log_messages();
#endif
    }
}

// vehicle specific waypoint info helpers
bool Blimp::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Blimp::send_nav_controller_output()
    distance = flightmode->wp_distance() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Blimp::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Blimp::send_nav_controller_output()
    bearing = flightmode->wp_bearing() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Blimp::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Blimp::send_nav_controller_output()
    xtrack_error = flightmode->crosstrack_error() * 0.01;
    return true;
}

/*
  constructor for main Blimp class
 */
Blimp::Blimp(void)
    : logger(g.log_bitmask),
      flight_modes(&g.flight_mode1),
      control_mode(Mode::Number::MANUAL),
      land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
      rc_throttle_control_in_filter(1.0f),
      inertial_nav(ahrs),
      param_loader(var_info),
      flightmode(&mode_manual)
{
    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;
}

Blimp blimp;
AP_Vehicle& vehicle = blimp;

AP_HAL_MAIN_CALLBACKS(&blimp);
