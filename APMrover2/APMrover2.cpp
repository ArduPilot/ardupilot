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
  scheduler table - all regular tasks should be listed here, along
  with how often they should be called (in 20ms units) and the maximum
  time they are expected to take (in microseconds)
*/
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    //         Function name,          Hz,     us,
    SCHED_TASK(read_radio,             50,   1000),
    SCHED_TASK(ahrs_update,            50,   6400),
    SCHED_TASK(read_sonars,            50,   2000),
    SCHED_TASK(update_current_mode,    50,   1500),
    SCHED_TASK(set_servos,             50,   1500),
    SCHED_TASK(update_GPS_50Hz,        50,   2500),
    SCHED_TASK(update_GPS_10Hz,        10,   2500),
    SCHED_TASK(update_alt,             10,   3400),
    SCHED_TASK(update_beacon,          50,     50),
    SCHED_TASK(update_visual_odom,     50,     50),
    SCHED_TASK(navigate,               10,   1600),
    SCHED_TASK(update_compass,         10,   2000),
    SCHED_TASK(update_commands,        10,   1000),
    SCHED_TASK(update_logging1,        10,   1000),
    SCHED_TASK(update_logging2,        10,   1000),
    SCHED_TASK(gcs_retry_deferred,     50,   1000),
    SCHED_TASK(gcs_update,             50,   1700),
    SCHED_TASK(gcs_data_stream_send,   50,   3000),
    SCHED_TASK(read_control_switch,     7,   1000),
    SCHED_TASK(read_trim_switch,       10,   1000),
    SCHED_TASK(read_battery,           10,   1000),
    SCHED_TASK(read_receiver_rssi,     10,   1000),
    SCHED_TASK(update_events,          50,   1000),
    SCHED_TASK(check_usb_mux,           3,   1000),
    SCHED_TASK(mount_update,           50,    600),
    SCHED_TASK(update_trigger,         50,    600),
    SCHED_TASK(gcs_failsafe_check,     10,    600),
    SCHED_TASK(compass_accumulate,     50,    900),
    SCHED_TASK(update_notify,          50,    300),
    SCHED_TASK(one_second_loop,         1,   3000),
    SCHED_TASK(compass_cal_update,     50,    100),
    SCHED_TASK(accel_cal_update,       10,    100),
    SCHED_TASK(dataflash_periodic,     50,    300),
    SCHED_TASK(button_update,           5,    100),
    SCHED_TASK(stats_update,            1,    100),
    SCHED_TASK(crash_check,            10,   1000),
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,           10,    100),
#endif
};

/*
  update AP_Stats
*/
void Rover::stats_update(void)
{
    g2.stats.set_flying(motor_active());
    g2.stats.update();
}

/*
  setup is called when the sketch starts
 */
void Rover::setup()
{
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    in_auto_reverse = false;

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

/*
  loop() is called rapidly while the sketch is running
 */
void Rover::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    const uint32_t timer = AP_HAL::micros();

    delta_us_fast_loop  = timer - fast_loopTimer_us;
    G_Dt                = delta_us_fast_loop * 1.0e-6f;
    fast_loopTimer_us   = timer;

    if (delta_us_fast_loop > G_Dt_max) {
        G_Dt_max = delta_us_fast_loop;
    }

    mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t remaining = (timer + 20000) - micros();
    if (remaining > 19500) {
        remaining = 19500;
    }
    scheduler.run(remaining);
}

void Rover::update_soft_armed()
{
    hal.util->set_soft_armed(arming.is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    DataFlash.set_vehicle_armed(hal.util->get_soft_armed());
}

// update AHRS system
void Rover::ahrs_update()
{
    update_soft_armed();

#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs_update();
#endif

    // when in reverse we need to tell AHRS not to assume we are a
    // 'fly forward' vehicle, otherwise it will see a large
    // discrepancy between the mag and the GPS heading and will try to
    // correct for it, leading to a large yaw error
    ahrs.set_fly_forward(!in_reverse);

    ahrs.update();

    // update home from EKF if necessary
    update_home_from_EKF();

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = norm(velocity.x, velocity.y);
    }

    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_IMU)) {
        DataFlash.Log_Write_IMU(ins);
    }
}

/*
  update camera mount - 50Hz
 */
void Rover::mount_update(void)
{
#if MOUNT == ENABLED
    camera_mount.update();
#endif
}

/*
  update camera trigger - 50Hz
 */
void Rover::update_trigger(void)
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

void Rover::update_alt()
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
}

/*
  check for GCS failsafe - 10Hz
 */
void Rover::gcs_failsafe_check(void)
{
    if (g.fs_gcs_enabled) {
        failsafe_trigger(FAILSAFE_EVENT_GCS, last_heartbeat_ms != 0 && (millis() - last_heartbeat_ms) > 2000);
    }
}

/*
  check for new compass data - 10Hz
 */
void Rover::update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        // update offsets
        compass.learn_offsets();
        if (should_log(MASK_LOG_COMPASS)) {
            DataFlash.Log_Write_Compass(compass);
        }
    } else {
        ahrs.set_compass(nullptr);
    }
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        Log_Write_Beacon();
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
    }
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging2(void)
{
    if (should_log(MASK_LOG_STEERING)) {
        if (control_mode == STEERING || control_mode == AUTO || control_mode == RTL || control_mode == GUIDED) {
            Log_Write_Steering();
        }
    }

    if (should_log(MASK_LOG_RC)) {
        Log_Write_RC();
    }

    if (should_log(MASK_LOG_IMU)) {
        DataFlash.Log_Write_Vibration(ins);
    }
}


/*
  update aux servo mappings
 */
void Rover::update_aux(void)
{
    SRV_Channels::enable_aux_servos();
}

/*
  once a second events
 */
void Rover::one_second_loop(void)
{
    if (should_log(MASK_LOG_CURRENT)) {
        Log_Write_Current();
    }
    // send a heartbeat
    gcs_send_message(MSG_HEARTBEAT);

    // allow orientation change at runtime to aid config
    ahrs.set_orientation();

    set_control_channels();

    // cope with changes to aux functions
    update_aux();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::NO;

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    static uint8_t counter;

    counter++;

    // write perf data every 20s
    if (counter % 10 == 0) {
        if (scheduler.debug() != 0) {
            hal.console->printf("G_Dt_max=%u\n", G_Dt_max);
        }
        if (should_log(MASK_LOG_PM)) {
            Log_Write_Performance();
        }
        G_Dt_max = 0;
        resetPerfData();
    }

    // save compass offsets once a minute
    if (counter >= 60) {
        if (g.compass_enabled) {
            compass.save_offsets();
        }
        counter = 0;
    }

    // update home position if not soft armed and gps position has
    // changed. Update every 1s at most
    if (!hal.util->get_soft_armed() &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        update_home();
    }

    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    update_sensor_status_flags();
}

void Rover::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

void Rover::update_GPS_50Hz(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];
    gps.update();

    for (uint8_t i=0; i < gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            if (should_log(MASK_LOG_GPS)) {
                DataFlash.Log_Write_GPS(gps, i);
            }
        }
    }
}


void Rover::update_GPS_10Hz(void)
{
    have_position = ahrs.get_position(current_loc);

    if (gps.last_message_time_ms() != last_gps_msg_ms) {
        last_gps_msg_ms = gps.last_message_time_ms();

        // set system time if necessary
        set_system_time_from_GPS();

        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

            // get ground speed estimate from AHRS
            ground_speed = ahrs.groundspeed();

#if CAMERA == ENABLED
            if (camera.update_location(current_loc, rover.ahrs) == true) {
                do_take_picture();
            }
#endif
        }
    }
}

void Rover::update_current_mode(void)
{
    switch (control_mode) {
    case AUTO:
    case RTL:
        if (!in_auto_reverse) {
            set_reverse(false);
        }
        if (!do_auto_rotation) {
            calc_lateral_acceleration();
            calc_nav_steer();
            calc_throttle(g.speed_cruise);
        } else {
            do_yaw_rotation();
        }
        break;

    case GUIDED: {
        if (!in_auto_reverse) {
            set_reverse(false);
        }
        switch (guided_mode) {
        case Guided_Angle:
            nav_set_yaw_speed();
            break;

        case Guided_WP:
            if (rtl_complete || verify_RTL()) {
                // we have reached destination so stop where we are
                if (fabsf(g2.motors.get_throttle()) > g.throttle_min.get()) {
                    gcs_send_mission_item_reached_message(0);
                }
                g2.motors.set_throttle(g.throttle_min.get());
                g2.motors.set_steering(0.0f);
                lateral_acceleration = 0.0f;
            } else {
                calc_lateral_acceleration();
                calc_nav_steer();
                calc_throttle(rover.guided_control.target_speed);
                Log_Write_GuidedTarget(guided_mode, Vector3f(next_WP.lat, next_WP.lng, next_WP.alt),
                                       Vector3f(rover.guided_control.target_speed, g2.motors.get_throttle(), 0.0f));
            }
            break;

        case Guided_Velocity:
            nav_set_speed();
            break;

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "Unknown GUIDED mode");
            break;
        }
        break;
    }

    case STEERING: {
        /*
          in steering mode we control lateral acceleration
          directly. We first calculate the maximum lateral
          acceleration at full steering lock for this speed. That is
          V^2/R where R is the radius of turn. We get the radius of
          turn from half the STEER2SRV_P.
         */
        float max_g_force = ground_speed * ground_speed / steerController.get_turn_radius();

        // constrain to user set TURN_MAX_G
        max_g_force = constrain_float(max_g_force, 0.1f, g.turn_max_g * GRAVITY_MSS);

        lateral_acceleration = max_g_force * (channel_steer->get_control_in()/4500.0f);
        calc_nav_steer();

        // and throttle gives speed in proportion to cruise speed, up
        // to 50% throttle, then uses nudging above that.
        float target_speed = channel_throttle->get_control_in() * 0.01f * 2 * g.speed_cruise;
        set_reverse(target_speed < 0);
        if (in_reverse) {
            target_speed = constrain_float(target_speed, -g.speed_cruise, 0);
        } else {
            target_speed = constrain_float(target_speed, 0, g.speed_cruise);
        }
        calc_throttle(target_speed);
        break;
    }

    case LEARNING:
    case MANUAL:
        // mark us as in_reverse when using a negative throttle to
        // stop AHRS getting off
        set_reverse(is_negative(g2.motors.get_throttle()));
        break;

    case HOLD:
        // hold position - stop motors and center steering
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        if (!in_auto_reverse) {
            set_reverse(false);
        }
        break;

    case INITIALISING:
        break;
    }
}

void Rover::update_navigation()
{
    switch (control_mode) {
    case MANUAL:
    case HOLD:
    case LEARNING:
    case STEERING:
    case INITIALISING:
        break;

    case AUTO:
        mission.update();
        if (do_auto_rotation) {
            do_yaw_rotation();
        }
        break;

    case RTL:
        // no loitering around the wp with the rover, goes direct to the wp position
        if (verify_RTL()) {
            g2.motors.set_throttle(g.throttle_min.get());
            set_mode(HOLD);
        } else {
            calc_lateral_acceleration();
            calc_nav_steer();
        }
        break;

    case GUIDED:
        switch (guided_mode) {
        case Guided_Angle:
            nav_set_yaw_speed();
            break;

        case Guided_WP:
            // no loitering around the wp with the rover, goes direct to the wp position
            if (rtl_complete || verify_RTL()) {
                // we have reached destination so stop where we are
                g2.motors.set_throttle(g.throttle_min.get());
                g2.motors.set_steering(0.0f);
                lateral_acceleration = 0.0f;
            } else {
                calc_lateral_acceleration();
                calc_nav_steer();
            }
            break;

        case Guided_Velocity:
            nav_set_speed();
            break;

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "Unknown GUIDED mode");
            break;
        }
        break;
    }
}

AP_HAL_MAIN_CALLBACKS(&rover);
