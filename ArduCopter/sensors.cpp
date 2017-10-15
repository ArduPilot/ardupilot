#include "Copter.h"

void Copter::init_barometer(bool full_calibration)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Calibrating barometer");
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");
}

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

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
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
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
  if the compass is enabled then try to accumulate a reading
  also update initial location used for declination
 */
void Copter::compass_accumulate(void)
{
    if (!g.compass_enabled) {
        return;
    }

    compass.accumulate();

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

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
void Copter::read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.has_current()) {
        compass.set_current(battery.current_amps());
    }

    // update motors with voltage and current
    if (battery.get_type() != AP_BattMonitor::BattMonitor_TYPE_NONE) {
        motors->set_voltage(battery.voltage());
        AP_Notify::flags.battery_voltage = battery.voltage();
    }
    if (battery.has_current()) {
        motors->set_current(battery.current_amps());
        motors->set_resistance(battery.get_resistance());
        motors->set_voltage_resting_estimate(battery.voltage_resting_estimate());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, rtl_mah_calc())) {
        failsafe_battery_event();
    }

    // log battery info to the dataflash
    if (should_log(MASK_LOG_CURRENT)) {
        Log_Write_Current();
    }
}

// Energy based battery failsafe calculations
//  - Returns the legacy FS_BATT_MAH value if the FS_BATT_RESERVE parameter is 0 or RTL is not possible
//  - Calculates watts to RTL based on estimated resting voltage and hover wattage values, adds reserve watts, returns calculated MAH
float Copter::rtl_mah_calc(void)
{
    // If FS_BATT_RESERVE is disabled, or HOVER_WATT is 0, or no home position available, or EKF is bad, return the default MAH
    if (!(g2.fs_batt_reserve) || !(g2.hover_watt) || ap.home_state == HOME_UNSET || !position_ok()) { 
        return g.fs_batt_mah;
    };

    // Watts per second for the various phases of RTL and landing
    float ws_hover = (g2.hover_watt) / 3600;
    float ws_climb = (ws_hover * g2.fs_batt_mult_clm);
    float ws_desc = (ws_hover * g2.fs_batt_mult_dsc);
    float ws_cruise = (ws_hover * g2.fs_batt_mult_crs);

    // Altitude calculations based vehicle altitude and RTL parameters
    int32_t target_alt_cm = MAX(current_loc.alt + g.rtl_climb_min, g.rtl_altitude);         // Altitude copter will RTL at
    int32_t climb_cm = target_alt_cm - current_loc.alt;                                     // vertical distance to climb
    int32_t desc_cm = constrain_int32(target_alt_cm - LAND_START_ALT, 0, target_alt_cm);    // vertical distance to descend
    int32_t final_desc_cm = LAND_START_ALT;                                                 // vertical distance for final landing phase
    
    // Time initialization
    uint32_t sec_climb = 0;
    uint32_t sec_cruise = 0;
    uint32_t sec_hover = 0;
    uint32_t sec_desc = 0;
    uint32_t sec_final = 0;
    uint32_t sec_total = 0;

    // Watts and MAH initialization
    float watts_climb = 0;
    float watts_cruise = 0;
    float watts_hover = 0;
    float watts_desc = 0;
    float watts_final = 0;
    float watts_rtl_total = 0;
    uint32_t mah_rtl_total = 0;
    uint32_t mah_reserve = 0;
    uint32_t mah_rtl_plus_reserve = 0;

    // Calculate and add watts for climb and cruise, but only if the failsafe is RTL
    if (g.failsafe_battery_enabled == FS_BATT_RTL) {

        // Calculate climb time and watts
        sec_climb = climb_cm / (wp_nav->get_speed_up());
        watts_climb = sec_climb * ws_climb;

        // Calculate cruise time and watts
        sec_cruise = (home_distance) / ((g.rtl_speed_cms > 0) ? g.rtl_speed_cms : wp_nav->get_speed_xy());
        watts_cruise = sec_cruise * ws_cruise;
        
        // Calculate hover time and watts
        sec_hover = g.rtl_loiter_time / 1000;
        watts_hover = sec_hover * ws_hover;
    }

    // Calculate and add descent energy to final approach altitude
    sec_desc = desc_cm / (g.land_speed_high > 0) ? g.land_speed_high : wp_nav->get_speed_down();
    watts_desc = sec_desc * ws_desc;

    // Calculate and add final descent energy
    sec_final = final_desc_cm / g.land_speed;
    watts_final = sec_final * ws_hover;
    
    // RTL total watts, mah, and seconds
    watts_rtl_total = watts_climb + watts_cruise + watts_hover + watts_desc + watts_final;
    mah_rtl_total = 1000*(watts_rtl_total / battery.voltage_resting_estimate());
    sec_total = sec_climb + sec_cruise + sec_hover + sec_desc + sec_final;
    
    // Calculate reserve and add to total mah
    mah_reserve = (g2.fs_batt_reserve/100) * battery.pack_capacity_mah();
    mah_rtl_plus_reserve = mah_rtl_total + mah_reserve;

    Log_Write_FS_Energy(watts_climb, watts_cruise, watts_hover, watts_desc, watts_final, watts_rtl_total, mah_rtl_total, mah_rtl_plus_reserve, sec_total);
    
    return mah_rtl_plus_reserve;
    }

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Copter::read_receiver_rssi(void)
{
    receiver_rssi = rssi.read_receiver_rssi_uint8();
}

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

#if GRIPPER_ENABLED == ENABLED
// gripper update
void Copter::gripper_update()
{
    g2.gripper.update();
}
#endif

/*
  update AP_Button
 */
void Copter::button_update(void)
{
    g2.button.update();
}

// initialise proximity sensor
void Copter::init_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.init();
    g2.proximity.set_rangefinder(&rangefinder);
#endif
}

// update proximity sensor
void Copter::update_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.update();
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
    if (copter.g2.proximity.get_status() > AP_Proximity::Proximity_NotConnected) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif
    if (copter.battery.healthy()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }


    // all present sensors enabled by default except altitude and position control and motors which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS &
                                                         ~MAV_SYS_STATUS_LOGGING &
                                                         ~MAV_SYS_STATUS_SENSOR_BATTERY);

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

    if (g.fs_batt_voltage > 0 || g.fs_batt_mah > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }



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
    if (copter.g2.proximity.get_status() < AP_Proximity::Proximity_Good) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_LASER_POSITION;
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
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder.has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

    if (!ap.initialised || ins.calibrating()) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    if (copter.failsafe.battery) {
         control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_BATTERY;                                                                    }
    
#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}

// init beacons used for non-gps position estimates
void Copter::init_beacon()
{
    g2.beacon.init();
}

// update beacons
void Copter::update_beacon()
{
    g2.beacon.update();
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
