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
   This is the ArduRover firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Randy Mackay, Grant Morphett

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin, Grant Morphett

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat...

   Please contribute your ideas! See https://ardupilot.org/dev for details
*/

#include "Rover.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#include "AP_Gripper/AP_Gripper.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _priority) SCHED_TASK_CLASS(Rover, &rover, func, _interval_ticks, _max_time_micros, _priority)

/*
  scheduler table - all regular tasks should be listed here.

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

  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in Hz) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    //         Function name,          Hz,     us,
    SCHED_TASK(read_radio,             50,    200,   3),
    SCHED_TASK(ahrs_update,           400,    400,   6),
    SCHED_TASK(read_rangefinders,      50,    200,   9),
    SCHED_TASK(update_current_mode,   400,    200,  12),
    SCHED_TASK(set_servos,            400,    200,  15),
    SCHED_TASK_CLASS(AP_GPS,              &rover.gps,              update,         50,  300,  18),
    SCHED_TASK_CLASS(AP_Baro,             &rover.barometer,        update,         10,  200,  21),
    SCHED_TASK_CLASS(AP_Beacon,           &rover.g2.beacon,        update,         50,  200,  24),
#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity,        &rover.g2.proximity,     update,         50,  200,  27),
#endif
    SCHED_TASK_CLASS(AP_WindVane,         &rover.g2.windvane,      update,         20,  100,  30),
    SCHED_TASK(update_wheel_encoder,   50,    200,  36),
    SCHED_TASK(update_compass,         10,    200,  39),
    SCHED_TASK(update_logging1,        10,    200,  45),
    SCHED_TASK(update_logging2,        10,    200,  48),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_receive,                    400,    500,  51),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_send,                       400,   1000,  54),
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_mode_switch,        7,    200,  57),
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_aux_all,           10,    200,  60),
    SCHED_TASK_CLASS(AP_BattMonitor,      &rover.battery,          read,           10,  300,  63),
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &rover.ServoRelayEvents, update_events,  50,  200,  66),
#if AP_GRIPPER_ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &rover.g2.gripper,       update,         10,   75,  69),
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,      400,     50,  70),
#endif
#endif
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,              &rover.rpm_sensor,       update,         10,  100,  72),
#endif
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &rover.camera_mount,     update,         50,  200,  75),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &rover.camera,           update,         50,  200,  78),
#endif
    SCHED_TASK(gcs_failsafe_check,     10,    200,  81),
    SCHED_TASK(fence_check,            10,    200,  84),
    SCHED_TASK(ekf_check,              10,    100,  87),
    SCHED_TASK_CLASS(ModeSmartRTL,        &rover.mode_smartrtl,    save_position,   3,  200,  90),
    SCHED_TASK_CLASS(AP_Notify,           &rover.notify,           update,         50,  300,  93),
    SCHED_TASK(one_second_loop,         1,   1500,  96),
#if HAL_SPRAYER_ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,          &rover.g2.sprayer,       update,          3,  90,  99),
#endif
    SCHED_TASK(compass_save,            0.1,  200, 105),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Logger,           &rover.logger,           periodic_tasks, 50,  300, 108),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,   &rover.ins,              periodic,      400,  200, 111),
    SCHED_TASK_CLASS(AP_Scheduler,        &rover.scheduler,        update_logging, 0.1, 200, 114),
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button,           &rover.button,           update,          5,  200, 117),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK(stats_update,            1,    200, 120),
#endif
    SCHED_TASK(crash_check,            10,    200, 123),
    SCHED_TASK(cruise_learn_update,    50,    200, 126),
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,           10,    200, 129),
#endif
};


void Rover::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Rover::_failsafe_priorities[7];

Rover::Rover(void) :
    AP_Vehicle(),
    param_loader(var_info),
    logger{g.log_bitmask},
    modes(&g.mode1),
    control_mode(&mode_initializing)
{
}

#if AP_SCRIPTING_ENABLED
// set target location (for use by scripting)
bool Rover::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_desired_location(target_loc);
}

// set target velocity (for use by scripting)
bool Rover::set_target_velocity_NED(const Vector3f& vel_ned)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    // convert vector length into speed
    const float target_speed_m = safe_sqrt(sq(vel_ned.x) + sq(vel_ned.y));

    // convert vector direction to target yaw
    const float target_yaw_cd = degrees(atan2f(vel_ned.y, vel_ned.x)) * 100.0f;

    // send target heading and speed
    mode_guided.set_desired_heading_and_speed(target_yaw_cd, target_speed_m);

    return true;
}

// set steering and throttle (-1 to +1) (for use by scripting)
bool Rover::set_steering_and_throttle(float steering, float throttle)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    // set steering and throttle
    mode_guided.set_steering_and_throttle(steering, throttle);
    return true;
}

// set desired turn rate (degrees/sec) and speed (m/s). Used for scripting
bool Rover::set_desired_turn_rate_and_speed(float turn_rate, float speed)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    // set turn rate and speed. Turn rate is expected in centidegrees/s and speed in meters/s
    mode_guided.set_desired_turn_rate_and_speed(turn_rate * 100.0f, speed);
    return true;
}

// set desired nav speed (m/s). Used for scripting.
bool Rover::set_desired_speed(float speed)
{
    return control_mode->set_desired_speed(speed);
}

// get control output (for use in scripting)
// returns true on success and control_value is set to a value in the range -1 to +1
bool Rover::get_control_output(AP_Vehicle::ControlOutput control_output, float &control_value)
{
    switch (control_output) {
    case AP_Vehicle::ControlOutput::Roll:
        control_value = constrain_float(g2.motors.get_roll(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Pitch:
        control_value = constrain_float(g2.motors.get_pitch(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Walking_Height:
        control_value = constrain_float(g2.motors.get_walking_height(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Throttle:
        control_value = constrain_float(g2.motors.get_throttle() / 100.0f, -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Yaw:
        control_value = constrain_float(g2.motors.get_steering() / 4500.0f, -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Lateral:
        control_value = constrain_float(g2.motors.get_lateral() / 100.0f, -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::MainSail:
        control_value = constrain_float(g2.motors.get_mainsail() / 100.0f, -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::WingSail:
        control_value = constrain_float(g2.motors.get_wingsail() / 100.0f, -1.0f, 1.0f);
        return true;
    default:
        return false;
    }
    return false;
}

// returns true if mode supports NAV_SCRIPT_TIME mission commands
bool Rover::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)mode_auto.mode_number();
}

// lua scripts use this to retrieve the contents of the active command
bool Rover::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (control_mode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2, arg3, arg4);
}

// lua scripts use this to indicate when they have complete the command
void Rover::nav_script_time_done(uint16_t id)
{
    if (control_mode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}
#endif // AP_SCRIPTING_ENABLED

#if STATS_ENABLED == ENABLED
/*
  update AP_Stats
*/
void Rover::stats_update(void)
{
    g2.stats.set_flying(g2.motors.active());
    g2.stats.update();
}
#endif


// update AHRS system
void Rover::ahrs_update()
{
    arming.update_soft_armed();

    // AHRS may use movement to calculate heading
    update_ahrs_flyforward();

    ahrs.update();

    // update position
    have_position = ahrs.get_location(current_loc);

    // set home from EKF if necessary and possible
    if (!ahrs.home_is_set()) {
        if (!set_home_to_current_location(false)) {
            // ignore this failure
        }
    }

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = velocity.xy().length();
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        ground_speed = ahrs.groundspeed();
    }

    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Sail();
    }

    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }

    if (should_log(MASK_LOG_VIDEO_STABILISATION)) {
        ahrs.write_video_stabilisation();
    }
}

/*
  check for GCS failsafe - 10Hz
 */
void Rover::gcs_failsafe_check(void)
{
    if (!g.fs_gcs_enabled) {
        // gcs failsafe disabled
        return;
    }

    // check for updates from GCS within 2 seconds
    const uint32_t gcs_last_seen_ms = gcs().sysid_myggcs_last_seen_time_ms();
    bool do_failsafe = true;
    if (gcs_last_seen_ms == 0) {
        // we've never seen the GCS, so we never failsafe for not seeing it
        do_failsafe = false;
    } else if (millis() - gcs_last_seen_ms <= 2000) {
        // we've never seen the GCS in the last couple of seconds, so all good
        do_failsafe = false;
    }

    failsafe_trigger(FAILSAFE_EVENT_GCS, "GCS", do_failsafe);
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Sail();
    }

    if (should_log(MASK_LOG_THR)) {
        Log_Write_Throttle();
        g2.beacon.log();
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
        if (g2.pos_control.is_active()) {
            g2.pos_control.write_log();
            logger.Write_PID(LOG_PIDN_MSG, g2.pos_control.get_vel_pid().get_pid_info_x());
            logger.Write_PID(LOG_PIDE_MSG, g2.pos_control.get_vel_pid().get_pid_info_y());
        }
    }

#if HAL_PROXIMITY_ENABLED
    if (should_log(MASK_LOG_RANGEFINDER)) {
        g2.proximity.log();
    }
#endif
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging2(void)
{
    if (should_log(MASK_LOG_STEERING)) {
        Log_Write_Steering();
    }

    if (should_log(MASK_LOG_RC)) {
        Log_Write_RC();
        g2.wheel_encoder.Log_Write();
    }

    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_Vibration();
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }
}


/*
  once a second events
 */
void Rover::one_second_loop(void)
{
    set_control_channels();

    // cope with changes to aux functions
    SRV_Channels::enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed();
    AP_Notify::flags.flying = hal.util->get_soft_armed();

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // attempt to update home position and baro calibration if not armed:
    if (!hal.util->get_soft_armed()) {
        update_home();
    }

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    set_likely_flying(hal.util->get_soft_armed());

    // send latest param values to wp_nav
    g2.wp_nav.set_turn_params(g2.turn_radius, g2.motors.have_skid_steering());
    g2.pos_control.set_turn_params(g2.turn_radius, g2.motors.have_skid_steering());
}

void Rover::update_current_mode(void)
{
    // check for emergency stop
    if (SRV_Channels::get_emergency_stop()) {
        // relax controllers, motor stopping done at output level
        g2.attitude_control.relax_I();
    }

    control_mode->update();
}

// vehicle specific waypoint info helpers
bool Rover::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Rover::send_nav_controller_output()
    if (!rover.control_mode->is_autopilot_mode()) {
        return false;
    }
    distance = control_mode->get_distance_to_destination();
    return true;
}

// vehicle specific waypoint info helpers
bool Rover::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Rover::send_nav_controller_output()
    if (!rover.control_mode->is_autopilot_mode()) {
        return false;
    }
    bearing = control_mode->wp_bearing();
    return true;
}

// vehicle specific waypoint info helpers
bool Rover::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Rover::send_nav_controller_output()
    if (!rover.control_mode->is_autopilot_mode()) {
        return false;
    }
    xtrack_error = control_mode->crosstrack_error();
    return true;
}


Rover rover;
AP_Vehicle& vehicle = rover;

AP_HAL_MAIN_CALLBACKS(&rover);
