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
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen,
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  Sebastian Quilter   :SmartRTL
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.org/
 *
 */

#include "Copter.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,              100,    130),
    SCHED_TASK(throttle_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK_CLASS(OpticalFlow,          &copter.optflow,             update,         200, 160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK_CLASS(RC_Channels,          (RC_Channels*)&copter.g2.rc_channels,      read_aux_all,    10,     50),
    SCHED_TASK(arm_motors_check,      10,     50),
#if TOY_MODE_ENABLED == ENABLED
    SCHED_TASK_CLASS(ToyMode,              &copter.g2.toy_mode,         update,          10,  50),
#endif
    SCHED_TASK(auto_disarm_check,     10,     50),
    SCHED_TASK(auto_trim,             10,     75),
#if RANGEFINDER_ENABLED == ENABLED
    SCHED_TASK(read_rangefinder,      20,    100),
#endif
#if PROXIMITY_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Proximity,         &copter.g2.proximity,        update,         100,  50),
#endif
#if BEACON_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Beacon,            &copter.g2.beacon,           update,         400,  50),
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    SCHED_TASK(update_visual_odom,   400,     50),
#endif
    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(run_nav_updates,       50,    100),
    SCHED_TASK(update_throttle_hover,100,     90),
#if MODE_SMARTRTL_ENABLED == ENABLED
    SCHED_TASK_CLASS(Copter::ModeSmartRTL, &copter.mode_smartrtl,       save_position,    3, 100),
#endif
#if SPRAYER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,           &copter.sprayer,             update,           3,  90),
#endif
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &copter.ServoRelayEvents,      update_events, 50,     75),
    SCHED_TASK_CLASS(AP_Baro,              &copter.barometer,           accumulate,      50,  90),
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,      400,     50),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(fourhundred_hz_logging,400,    50),
#endif
    SCHED_TASK_CLASS(AP_Notify,            &copter.notify,              update,          50,  90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(ekf_check,             10,     75),
    SCHED_TASK(gpsglitch_check,       10,     50),
    SCHED_TASK(landinggear_update,    10,     75),
    SCHED_TASK(lost_vehicle_check,    10,     50),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550),
#if MOUNT == ENABLED
    SCHED_TASK_CLASS(AP_Mount,             &copter.camera_mount,        update,          50,  75),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,            &copter.camera,              update_trigger,  50,  75),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK_CLASS(DataFlash_Class,      &copter.DataFlash,           periodic_tasks, 400, 300),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &copter.ins,                 periodic,       400,  50),
    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75),
#if RPM_ENABLED == ENABLED
    SCHED_TASK(rpm_update,            10,    200),
#endif
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
    SCHED_TASK_CLASS(AP_TempCalibration,   &copter.g2.temp_calibration, update,          10, 100),
#if ADSB_ENABLED == ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,          10,    100),
#endif
#if AC_TERRAIN == ENABLED
    SCHED_TASK(terrain_update,        10,    100),
#endif
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,           &copter.g2.gripper,          update,          10,  75),
#endif
#if WINCH_ENABLED == ENABLED
    SCHED_TASK(winch_update,          10,     50),
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
    SCHED_TASK_CLASS(AP_Button,            &copter.g2.button,           update,           5, 100),
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats,             &copter.g2.stats,            update,           1, 100),
#endif
#if OSD_ENABLED == ENABLED
    SCHED_TASK(publish_osd_info, 1, 10),
#endif
};

constexpr int8_t Copter::_failsafe_priorities[7];

void Copter::setup()
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
}

void Copter::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_last_loop_time_s();
}


// Main loop - 400hz
void Copter::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();

    // run low level rate controllers that only require IMU data
    attitude_control->rate_controller_run();

    // send outputs to the motors library immediately
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

#if FRAME_CONFIG == HELI_FRAME
    update_heli_control_dynamics();
#endif //HELI_FRAME

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading or position
    check_ekf_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

#if MOUNT == ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_thr_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if(g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass();
        }
    }
}

// Full rate logging of attitude, rate and pid loops
// should be run at 400hz
void Copter::fourhundred_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
        if (rssi.enabled()) {
            DataFlash.Log_Write_RSSI(rssi);
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS())) {
        pos_control->write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control->control_monitor_log();
#if PROXIMITY_ENABLED == ENABLED
        DataFlash.Log_Write_Proximity(g2.proximity);  // Write proximity sensor distances
#endif
#if BEACON_ENABLED == ENABLED
        DataFlash.Log_Write_Beacon(g2.beacon);
#endif
    }
#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs().send_message(MSG_SERVO_OUTPUT_RAW);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU();
    }
#endif

#if PRECISION_LANDING == ENABLED
    // log output
    Log_Write_Precland();
#endif
}

// three_hz_loop - 3.3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED


    // update ch6 in flight tuning
    tuning();
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    arming.update();

    if (!motors->armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // set all throttle channel settings
        motors->set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
#endif
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // log terrain data
    terrain_logging();

#if ADSB_ENABLED == ENABLED
    adsb.set_is_flying(!ap.land_complete);
#endif

    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    update_sensor_status_flags();

    // init compass location for declination
    init_compass_location();
}

// called at 50hz
void Copter::update_GPS(void)
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

void Copter::init_simple_bearing()
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
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
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
void Copter::update_super_simple_bearing(bool force_update)
{
    if (!force_update) {
        if (ap.simple_mode != 2) {
            return;
        }
        if (home_distance() < SUPER_SIMPLE_RADIUS) {
            return;
        }
    }

    const int32_t bearing = home_bearing();

    // check the bearing to home has changed by at least 5 degrees
    if (labs(super_simple_last_bearing - bearing) < 500) {
        return;
    }

    super_simple_last_bearing = bearing;
    const float angle_rad = radians((super_simple_last_bearing+18000)/100);
    super_simple_cos_yaw = cosf(angle_rad);
    super_simple_sin_yaw = sinf(angle_rad);
}

void Copter::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs().update();
#endif

    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
}

// read baro and log control tuning
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

#if OSD_ENABLED == ENABLED
void Copter::publish_osd_info()
{
    AP_OSD::NavInfo nav_info;
    nav_info.wp_distance = flightmode->wp_distance() * 1.0e-2f;
    nav_info.wp_bearing = flightmode->wp_bearing();
    nav_info.wp_xtrack_error = flightmode->crosstrack_error() * 1.0e-2f;
    nav_info.wp_number = mode_auto.mission.get_current_nav_index();
    osd.set_nav_info(nav_info);
}
#endif

AP_HAL_MAIN_CALLBACKS(&copter);
