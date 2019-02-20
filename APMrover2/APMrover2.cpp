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
   This is the APMrover2 firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Grant Morphett

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin, Grant Morphett

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier 

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat...

   Please contribute your ideas! See http://dev.ardupilot.org for details
*/

#include "Rover.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Rover rover;

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(Rover, &rover, func, _interval_ticks, _max_time_micros)

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in Hz) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    //         Function name,          Hz,     us,
    SCHED_TASK(read_radio,             50,    200),
    SCHED_TASK(ahrs_update,           400,    400),
    SCHED_TASK(read_rangefinders,      50,    200),
    SCHED_TASK(update_current_mode,   400,    200),
    SCHED_TASK(set_servos,            400,    200),
    SCHED_TASK(update_GPS,             50,    300),
    SCHED_TASK_CLASS(AP_Baro,             &rover.barometer,        update,         10,  200),
    SCHED_TASK_CLASS(AP_Beacon,           &rover.g2.beacon,        update,         50,  200),
    SCHED_TASK_CLASS(AP_Proximity,        &rover.g2.proximity,     update,         50,  200),
    SCHED_TASK_CLASS(AP_WindVane,         &rover.g2.windvane,      update,         20,  100),
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_VisualOdom,       &rover.g2.visual_odom,   update,         50,  200),
#endif
    SCHED_TASK(update_wheel_encoder,   50,    200),
    SCHED_TASK(update_compass,         10,    200),
    SCHED_TASK(update_mission,         50,    200),
    SCHED_TASK(update_logging1,        10,    200),
    SCHED_TASK(update_logging2,        10,    200),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_receive,                    400,    500),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_send,                       400,   1000),
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_mode_switch,        7,    200),
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_aux_all,           10,    200),
    SCHED_TASK_CLASS(AP_BattMonitor,      &rover.battery,          read,           10,  300),
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &rover.ServoRelayEvents, update_events,  50,  200),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &rover.g2.gripper,      update,         10,   75),
#endif
    SCHED_TASK(rpm_update,             10,    100),
#if MOUNT == ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &rover.camera_mount,     update,         50,  200),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &rover.camera,           update_trigger, 50,  200),
#endif
    SCHED_TASK(gcs_failsafe_check,     10,    200),
    SCHED_TASK(fence_check,            10,    200),
    SCHED_TASK(ekf_check,              10,    100),
    SCHED_TASK_CLASS(ModeSmartRTL,        &rover.mode_smartrtl,    save_position,   3,  200),
    SCHED_TASK_CLASS(AP_Notify,           &rover.notify,           update,         50,  300),
    SCHED_TASK(one_second_loop,         1,   1500),
    SCHED_TASK_CLASS(AC_Sprayer,          &rover.g2.sprayer,           update,      3,  90),
    SCHED_TASK(compass_cal_update,     50,    200),
    SCHED_TASK(compass_save,           0.1,   200),
    SCHED_TASK(accel_cal_update,       10,    200),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Logger,     &rover.logger,        periodic_tasks, 50,  300),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,   &rover.ins,              periodic,      400,  200),
    SCHED_TASK_CLASS(AP_Scheduler,        &rover.scheduler,        update_logging, 0.1, 200),
    SCHED_TASK_CLASS(AP_Button,           &rover.button,           update,          5,  200),
#if STATS_ENABLED == ENABLED
    SCHED_TASK(stats_update,            1,    200),
#endif
    SCHED_TASK(crash_check,            10,    200),
    SCHED_TASK(cruise_learn_update,    50,    200),
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,           10,    200),
#endif
    SCHED_TASK(read_airspeed,          10,    100),
    SCHED_TASK(set_nav_controller,     10,    100),
};

constexpr int8_t Rover::_failsafe_priorities[7];

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

/*
  setup is called when the sketch starts
 */
void Rover::setup()
{
    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
}

/*
  loop() is called rapidly while the sketch is running
 */
void Rover::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_last_loop_time_s();
}

void Rover::update_soft_armed()
{
    hal.util->set_soft_armed(arming.is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    logger.set_vehicle_armed(hal.util->get_soft_armed());
}

// update AHRS system
void Rover::ahrs_update()
{
    update_soft_armed();

#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs().update();
#endif

    // AHRS may use movement to calculate heading
    update_ahrs_flyforward();

    ahrs.update();

    // update position
    have_position = ahrs.get_position(current_loc);

    // set home from EKF if necessary and possible
    if (!ahrs.home_is_set()) {
        if (!set_home_to_current_location(false)) {
            // ignore this failure
        }
    }

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = norm(velocity.x, velocity.y);
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        ground_speed = ahrs.groundspeed();
    }

    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Sail();
    }

    if (should_log(MASK_LOG_IMU)) {
        logger.Write_IMU();
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
    failsafe_trigger(FAILSAFE_EVENT_GCS, failsafe.last_heartbeat_ms != 0 && (millis() - failsafe.last_heartbeat_ms) > 2000);
}

/*
  check for new compass data - 10Hz
 */
void Rover::update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        // update offsets
        if (should_log(MASK_LOG_COMPASS)) {
            logger.Write_Compass();
        }
    }
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
        logger.Write_Beacon(g2.beacon);
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
    }

    if (should_log(MASK_LOG_RANGEFINDER)) {
        logger.Write_Proximity(g2.proximity);
    }
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
        logger.Write_Vibration();
    }
}


/*
  once a second events
 */
void Rover::one_second_loop(void)
{
    // send a heartbeat
    gcs().send_message(MSG_HEARTBEAT);

    // allow orientation change at runtime to aid config
    ahrs.set_orientation();

    set_control_channels();

    // cope with changes to aux functions
    SRV_Channels::enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::NO;

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // attempt to update home position and baro calibration if not armed:
    if (!hal.util->get_soft_armed()) {
        update_home();
    }

    // init compass location for declination
    init_compass_location();

    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    gcs().update_sensor_status_flags();

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    ahrs.set_likely_flying(hal.util->get_soft_armed());
}

void Rover::update_GPS(void)
{
    gps.update();
    if (gps.last_message_time_ms() != last_gps_msg_ms) {
        last_gps_msg_ms = gps.last_message_time_ms();

#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Rover::update_current_mode(void)
{
    control_mode->update();
}

void Rover::set_nav_controller(void)
{
    switch ((AP_Navigation::ControllerType)g.nav_controller.get()) {

    default:
    case AP_Navigation::CONTROLLER_DEFAULT:
        // no break, fall through to L1 as default controller

    case AP_Navigation::CONTROLLER_L1:
        Rover::nav_controller = &L1_controller;
        break;

    case AP_Navigation::CONTROLLER_LQR:
        Rover::nav_controller = &g2.LQR_controller;
        break;
    }
}

AP_HAL_MAIN_CALLBACKS(&rover);
