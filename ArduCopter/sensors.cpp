#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;

    motors->set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.init();
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    if (rangefinder.num_sensors() > 0 &&
        should_log(MASK_LOG_CTUN)) {
        DataFlash.Log_Write_RFND(rangefinder);
    }

    rangefinder_state.alt_healthy = ((rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::RangeFinder_Good) && (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_alt = rangefinder.distance_cm_orient(ROTATION_PITCH_270);

 #if RANGEFINDER_TILT_CORRECTION == ENABLED
    // correct alt for angle of the rangefinder
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
 #endif

    rangefinder_state.alt_cm = temp_alt;

    // filter rangefinder for use by AC_WPNav
    uint32_t now = AP_HAL::millis();

    if (rangefinder_state.alt_healthy) {
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
        } else {
            rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
    }

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav->set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());

#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok()
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

/*
  update RPM sensors
 */
void Copter::rpm_update(void)
{
#if RPM_ENABLED == ENABLED
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
#endif
}

// initialise compass
void Copter::init_compass()
{
    if (!g.compass_enabled) {
        return;
    }

    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        hal.console->printf("COMPASS INIT ERROR\n");
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

/*
  initialise compass's location used for declination
 */
void Copter::init_compass_location()
{
    // update initial location used for declination
    if (!ap.compass_init_location) {
        Location loc;
        if (ahrs.get_position(loc)) {
            compass.set_initial_location(loc.lat, loc.lng);
            ap.compass_init_location = true;
        }
    }
}

// initialise optical flow sensor
void Copter::init_optflow()
{
#if OPTFLOW == ENABLED
    // initialise optical flow sensor
    optflow.init();
#endif      // OPTFLOW == ENABLED
}

// called at 200hz
#if OPTFLOW == ENABLED
void Copter::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        const Vector3f &posOffset = optflow.get_pos_offset();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update, posOffset);
        if (g.log_bitmask & MASK_LOG_OPTFLOW) {
            Log_Write_Optflow();
        }
    }
}
#endif  // OPTFLOW == ENABLED

void Copter::compass_cal_update()
{
    static uint32_t compass_cal_stick_gesture_begin = 0;

    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }

    if (compass.is_calibrating()) {
        if (channel_yaw->get_control_in() < -4000 && channel_throttle->get_control_in() > 900) {
            compass.cancel_calibration_all();
        }
    } else {
        bool stick_gesture_detected = compass_cal_stick_gesture_begin != 0 && !motors->armed() && channel_yaw->get_control_in() > 4000 && channel_throttle->get_control_in() > 900;
        uint32_t tnow = millis();

        if (!stick_gesture_detected) {
            compass_cal_stick_gesture_begin = tnow;
        } else if (tnow-compass_cal_stick_gesture_begin > 1000*COMPASS_CAL_STICK_GESTURE_TIME) {
#ifdef CAL_ALWAYS_REBOOT
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,true);
#else
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,false);
#endif
        }
    }
}

void Copter::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }

#ifdef CAL_ALWAYS_REBOOT
    if (ins.accel_cal_requires_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}

// initialise proximity sensor
void Copter::init_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.init();
    g2.proximity.set_rangefinder(&rangefinder);
#endif
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink. If a bit is set
// then it indicates that the sensor or subsystem is present but
// not functioning correctly.
void Copter::update_sensor_status_flags(void)
{
    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (precland.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (copter.DataFlash.logging_present()) { // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_present()) {
        control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
    }
#endif
#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder.has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif

    // all sensors are present except these, which may be set as enabled below:
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS &
                                                         ~MAV_SYS_STATUS_LOGGING &
                                                         ~MAV_SYS_STATUS_SENSOR_BATTERY &
                                                         ~MAV_SYS_STATUS_GEOFENCE &
                                                         ~MAV_SYS_STATUS_SENSOR_LASER_POSITION &
                                                         ~MAV_SYS_STATUS_SENSOR_PROXIMITY);

    switch (control_mode) {
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
    case RTL:
    case CIRCLE:
    case LAND:
    case POSHOLD:
    case BRAKE:
    case THROW:
    case SMART_RTL:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case ALT_HOLD:
    case GUIDED_NOGPS:
    case SPORT:
    case AUTOTUNE:
    case FLOWHOLD:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    default:
        // stabilize, acro, drift, and flip have no automatic x,y or z control (i.e. all manual)
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    if (copter.DataFlash.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }

    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
    }
#endif
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif


    // default to all healthy
    control_sensors_health = control_sensors_present;

    if (!barometer.all_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (!g.compass_enabled || !compass.healthy() || !ahrs.use_compass()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (!gps.is_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (!ap.rc_receiver_present || failsafe.radio) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
#if OPTFLOW == ENABLED
    if (!optflow.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (precland.enabled() && !precland.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled() && !g2.visual_odom.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (copter.DataFlash.logging_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_LOGGING;
    }

#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in copter
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder_state.enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (!rangefinder_state.alt_healthy) {
            control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

    if (!ap.initialised || ins.calibrating()) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    if (!copter.battery.healthy() || copter.battery.has_failsafed()) {
         control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_GEOFENCE;
    }
#endif

#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}

// init visual odometry sensor
void Copter::init_visual_odom()
{
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    g2.visual_odom.init();
#endif
}

// update visual odometry sensor
void Copter::update_visual_odom()
{
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    // check for updates
    if (g2.visual_odom.enabled() && (g2.visual_odom.get_last_update_ms() != visual_odom_last_update_ms)) {
        visual_odom_last_update_ms = g2.visual_odom.get_last_update_ms();
        float time_delta_sec = g2.visual_odom.get_time_delta_usec() / 1000000.0f;
        ahrs.writeBodyFrameOdom(g2.visual_odom.get_confidence(),
                                g2.visual_odom.get_position_delta(),
                                g2.visual_odom.get_angle_delta(),
                                time_delta_sec,
                                visual_odom_last_update_ms,
                                g2.visual_odom.get_pos_offset());
        // log sensor data
        DataFlash.Log_Write_VisualOdom(time_delta_sec,
                                       g2.visual_odom.get_angle_delta(),
                                       g2.visual_odom.get_position_delta(),
                                       g2.visual_odom.get_confidence());
    }
#endif
}

// winch and wheel encoder initialisation
void Copter::winch_init()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.init();
    g2.winch.init(&g2.wheel_encoder);
#endif
}

// winch and wheel encoder update
void Copter::winch_update()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.update();
    g2.winch.update();
#endif
}

// Calculates the time and MAH required to perform an RTL sequence from current location to landing
void Copter::energyToRTL()
{
    // Variables for parameters, constraints, and conversions
    //uint32_t masRate_hover = motors->get_current_hover() / 3600;  // Milliamps per second in hover
    uint32_t masRate_hover = motors->get_current_hover() / 3600;    // Milliamps per second in hover
    uint32_t masRate_climb = masRate_hover * g2.rtl_pwr_climb;      // Milliamps per second in a climb as a +/- % of hover
    uint32_t masRate_cruise = masRate_hover * g2.rtl_pwr_cruise;    // Milliamps per second in cruise as a +/- % of hover
    uint32_t masRate_descent = masRate_hover * g2.rtl_pwr_cruise;   // Milliamps per second in a descent as a +/- % of hover
    
    // Variables for totals
    uint32_t rtl_total_sec;                 // Total seconds to complete RTL
    uint32_t rtl_total_mah;                 // Total MAH to complete and RTL
    static uint32_t rtl_total_mah_last = 0; // Last sent rtl_total_mah

    // These parameters and values are required to proceed. Abort if any are not set or unavailable.
    // Set totals to zero if we can't proceed.
    if (masRate_climb == 0 || masRate_cruise == 0 || masRate_descent == 0 || masRate_hover == 0
      || !battery.has_current() || !ahrs.home_is_set() || !motors->armed()) {
        rtl_total_sec = 0; 
        rtl_total_mah = 0;
    } else {

        // Braking takes a lot if moving fast
        uint32_t sec_braking;
        uint32_t mah_braking;
        if (inertial_nav.get_velocity_xy() > 510) {
            sec_braking = 20;
            mah_braking = sec_braking * masRate_climb;
        } else {
            sec_braking = 0;
            mah_braking = 0;
        }

        // Climb
        uint32_t target_alt_cm = MAX(current_loc.alt + g.rtl_climb_min, g.rtl_altitude);            // Altitude copter will RTL at
        uint32_t sec_climb = ((target_alt_cm - current_loc.alt) / (copter.wp_nav->get_speed_up())); // Time in seconds to climb
        if (sec_climb > 0 ) { sec_climb += 3; }                                                     // 3 seconds accounts for accel/decel
        uint32_t mah_climb = sec_climb * masRate_climb;                                             // MAH total required to climb

        // Cruise
        uint32_t rtl_cruiseSpeed = ((g.rtl_speed_cms > 0) ? g.rtl_speed_cms : copter.wp_nav->get_speed_xy());    // Cruise speed
        uint32_t sec_cruise = (home_distance() / rtl_cruiseSpeed) + 8;                                           // Cruise time in seconds (distance to cruise + 8 seconds for accel/decel)
        if (sec_cruise > 0 ) { sec_cruise += 8; }                                                                // 8 seconds accounts for accel/decel
        uint32_t mah_cruise = sec_cruise * masRate_cruise;                                                       // Cruise mAh consumed

        // Hover
        uint32_t sec_hover = (g.rtl_loiter_time * .001);        // Hover time + 10 seconds to cover transition between phases
        uint32_t mah_hover = sec_hover * masRate_hover;         // Hover mAh consumed

        // Descent
        uint32_t rtl_desSpeed = (g.land_speed_high > 0) ? g.land_speed_high : copter.wp_nav->get_speed_down();  // Descent rate
        uint32_t desc_cm = constrain_int32(target_alt_cm - g2.land_alt_low, 0, target_alt_cm);  // Descent from altitude to final landing start
        uint32_t sec_desc = desc_cm / rtl_desSpeed;                                             // Descent time in seconds
        uint32_t mah_descent = (sec_desc * masRate_descent);                                    // Descent mAh consumed
        
        // Final Landing Descent
        uint32_t land_desc_cm = g2.land_alt_low;            // Final descent distance
        uint32_t sec_land = land_desc_cm / g.land_speed;    // Final descent time in seconds
        uint32_t mah_land = sec_land * masRate_hover;       // Final descent mAh consumed


        // BATT_FS_LOW_ACT parameter options: 0:None,1:Land,2:RTL,3:SmartRTL,4:SmartRTL or Land,5:Terminate
        // Calculate time to land if the failsafe is set for land
        // Calculate time to RTL if the failsafe is set for RTL
        // All other failsafe options are 0 since we can't calculate them
        switch (battery.get_failsafe_low_action()) {
            case Failsafe_Action_Land:  // BATT_FS_LOW_ACT = 1 (Land only)
                rtl_total_sec = sec_desc + sec_land;
                rtl_total_mah = mah_descent + mah_land;
                break;
            case Failsafe_Action_RTL:   // BATT_FS_LOW_ACT = 2 (RTL)
                rtl_total_sec = sec_braking + sec_climb + sec_cruise + sec_hover + sec_desc + sec_land;
                rtl_total_mah = mah_braking + mah_climb + mah_cruise + mah_hover + mah_descent + mah_land;
                break;
            default: // BATT_FS_LOW_ACT = Anythinig else
                rtl_total_sec = 0; 
                rtl_total_mah = 0;
                break;
        }
        
        // Messaging for testing only
        static uint32_t lastmsg = AP_HAL::millis();
        if (AP_HAL::millis() - lastmsg > 30000) {
            // gcs().send_text(MAV_SEVERITY_WARNING, "CL:%d; CR:%d; HV:%d; DS:%d; LN:%d; mAh:%d Sec:%d", mah_climb,mah_cruise,mah_hover,mah_descent,mah_land,rtl_total_mah,rtl_total_sec);
            gcs().send_text(MAV_SEVERITY_WARNING, "mAh:%d Sec:%d", rtl_total_mah,rtl_total_sec);
            lastmsg = AP_HAL::millis();
        }
    }

    // Update battery monitor
    if (rtl_total_mah != rtl_total_mah_last) {
        battery.set_rtl_mah(0,rtl_total_mah, rtl_total_sec);
        rtl_total_mah_last = rtl_total_mah;
    }
}
