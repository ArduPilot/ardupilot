// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// true if we are out of time in our event timeslice
static bool	gcs_out_of_time;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = is_flying() ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    uint32_t custom_mode = control_mode;
    
    if (failsafe.state != FAILSAFE_NONE) {
        system_status = MAV_STATE_CRITICAL;
    }

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (control_mode) {
    case MANUAL:
    case TRAINING:
    case ACRO:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case CRUISE:
        base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                    MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    if (!training_manual_pitch || !training_manual_roll) {
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;        
    }

    if (control_mode != MANUAL && control_mode != INITIALISING) {
        // stabiliser of some form is enabled
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    if (g.stick_mixing != STICK_MIXING_DISABLED && control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (control_mode != INITIALISING && arming.is_armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch - radians(g.pitch_trim_cd*0.01),
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}

#if GEOFENCE_ENABLED == ENABLED
static NOINLINE void send_fence_status(mavlink_channel_t chan)
{
    geofence_send_status(chan);
}
#endif


static NOINLINE void send_extended_status1(mavlink_channel_t chan)
{
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }

    if (airspeed.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }
    if (g_gps != NULL && g_gps->status() > GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }

    // all present sensors enabled by default except rate control, attitude stabilization, yaw, altitude, position control and motor output which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL & ~MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION & ~MAV_SYS_STATUS_SENSOR_YAW_POSITION & ~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL & ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);

    if (airspeed.enabled() && airspeed.use()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }

    switch (control_mode) {
    case MANUAL:
        break;

    case ACRO:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        break;

    case STABILIZE:
    case FLY_BY_WIRE_A:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case FLY_BY_WIRE_B:
    case CRUISE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS; // motor control
        break;

    case TRAINING:
        if (!training_manual_roll || !training_manual_pitch) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation        
        }
        break;

    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL; // altitude control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS; // motor control
        break;

    case INITIALISING:
        break;
    }

    // default to all healthy
    control_sensors_health = control_sensors_present & ~(MAV_SYS_STATUS_SENSOR_3D_MAG | 
                                                         MAV_SYS_STATUS_SENSOR_GPS |
                                                         MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
    if (g.compass_enabled && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (g_gps != NULL && g_gps->status() > GPS::NO_GPS) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (!ins.healthy()) {
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }
    if (airspeed.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }

    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average(20000) * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        fix_time = g_gps->last_fix_time;
    } else {
        fix_time = millis();
    }
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude_cm * 10,        // millimeters above sea level
        relative_altitude() * 1.0e3,    // millimeters above ground
        g_gps->velocity_north() * 100,  // X speed cm/s (+ve North)
        g_gps->velocity_east()  * 100,  // Y speed cm/s (+ve East)
        g_gps->velocity_down()  * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll_cd * 0.01,
        nav_pitch_cd * 0.01,
        nav_controller->nav_bearing_cd() * 0.01f,
        nav_controller->target_bearing_cd() * 0.01f,
        wp_distance,
        altitude_error_cm * 0.01,
        airspeed_error_cm,
        nav_controller->crosstrack_error());
}

static void NOINLINE send_gps_raw(mavlink_channel_t chan)
{
    static uint32_t last_send_time;
    if (last_send_time != 0 && last_send_time == g_gps->last_fix_time && g_gps->status() >= GPS::GPS_OK_FIX_3D) {
        return;
    }
    last_send_time = g_gps->last_fix_time;
    mavlink_msg_gps_raw_int_send(
        chan,
        g_gps->last_fix_time*(uint64_t)1000,
        g_gps->status(),
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude_cm * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed_cm,  // cm/s
        g_gps->ground_course_cd, // 1/100 degrees,
        g_gps->num_sats);
}

static void NOINLINE send_system_time(mavlink_channel_t chan)
{
    mavlink_msg_system_time_send(
        chan,
        g_gps->time_epoch_usec(),
        hal.scheduler->millis());
}

#if HIL_MODE != HIL_MODE_DISABLED
void NOINLINE send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * channel_roll->norm_output(),
        10000 * channel_pitch->norm_output(),
        10000 * channel_throttle->norm_output(),
        10000 * channel_rudder->norm_output(),
        0,
        0,
        0,
        0,
        receiver_rssi);
}
#endif

static void NOINLINE send_radio_in(mavlink_channel_t chan)
{
    mavlink_msg_rc_channels_raw_send(
        chan,
        millis(),
        0, // port
        hal.rcin->read(CH_1),
        hal.rcin->read(CH_2),
        hal.rcin->read(CH_3),
        hal.rcin->read(CH_4),
        hal.rcin->read(CH_5),
        hal.rcin->read(CH_6),
        hal.rcin->read(CH_7),
        hal.rcin->read(CH_8),
        receiver_rssi);
}

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
#if HIL_MODE != HIL_MODE_DISABLED
    if (!g.hil_servos) {
        mavlink_msg_servo_output_raw_send(
            chan,
            micros(),
            0,     // port
            RC_Channel::rc_channel(0)->radio_out,
            RC_Channel::rc_channel(1)->radio_out,
            RC_Channel::rc_channel(2)->radio_out,
            RC_Channel::rc_channel(3)->radio_out,
            RC_Channel::rc_channel(4)->radio_out,
            RC_Channel::rc_channel(5)->radio_out,
            RC_Channel::rc_channel(6)->radio_out,
            RC_Channel::rc_channel(7)->radio_out);
        return;
    }
#endif
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0,     // port
        hal.rcout->read(0),
        hal.rcout->read(1),
        hal.rcout->read(2),
        hal.rcout->read(3),
        hal.rcout->read(4),
        hal.rcout->read(5),
        hal.rcout->read(6),
        hal.rcout->read(7));
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    float aspeed;
    if (airspeed.enabled()) {
        aspeed = airspeed.get_airspeed();
    } else if (!ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 0;
    }
    float throttle_norm = channel_throttle->norm_output() * 100;
    throttle_norm = constrain_int16(throttle_norm, -100, 100);
    uint16_t throttle = ((uint16_t)(throttle_norm + 100)) / 2;
    mavlink_msg_vfr_hud_send(
        chan,
        aspeed,
        (float)g_gps->ground_speed_cm * 0.01f,
        (ahrs.yaw_sensor / 100) % 360,
        throttle,
        current_loc.alt / 100.0,
        barometer.get_climb_rate());
}

static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    const Vector3f &accel = ins.get_accel();
    const Vector3f &gyro = ins.get_gyro();
    const Vector3f &mag = compass.get_field();

    mavlink_msg_raw_imu_send(
        chan,
        micros(),
        accel.x * 1000.0 / GRAVITY_MSS,
        accel.y * 1000.0 / GRAVITY_MSS,
        accel.z * 1000.0 / GRAVITY_MSS,
        gyro.x * 1000.0,
        gyro.y * 1000.0,
        gyro.z * 1000.0,
        mag.x,
        mag.y,
        mag.z);

    if (ins.get_gyro_count() <= 1 &&
        ins.get_accel_count() <= 1 &&
        compass.get_count() <= 1) {
        return;
    }
    const Vector3f &accel2 = ins.get_accel(1);
    const Vector3f &gyro2 = ins.get_gyro(1);
    const Vector3f &mag2 = compass.get_field(1);
    mavlink_msg_scaled_imu2_send(
        chan,
        millis(),
        accel2.x * 1000.0f / GRAVITY_MSS,
        accel2.y * 1000.0f / GRAVITY_MSS,
        accel2.z * 1000.0f / GRAVITY_MSS,
        gyro2.x * 1000.0f,
        gyro2.y * 1000.0f,
        gyro2.z * 1000.0f,
        mag2.x,
        mag2.y,
        mag2.z);        
}

static void NOINLINE send_raw_imu2(mavlink_channel_t chan)
{
    float pressure = barometer.get_pressure();
    mavlink_msg_scaled_pressure_send(
        chan,
        millis(),
        pressure*0.01f, // hectopascal
        (pressure - barometer.get_ground_pressure())*0.01f, // hectopascal
        barometer.get_temperature()*100); // 0.01 degrees C
}

static void NOINLINE send_raw_imu3(mavlink_channel_t chan)
{
    // run this message at a much lower rate - otherwise it
    // pointlessly wastes quite a lot of bandwidth
    static uint8_t counter;
    if (counter++ < 10) {
        return;
    }
    counter = 0;

    Vector3f mag_offsets = compass.get_offsets();
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.get_pressure(),
                                    barometer.get_temperature()*100,
                                    gyro_offsets.x,
                                    gyro_offsets.y,
                                    gyro_offsets.z,
                                    accel_offsets.x,
                                    accel_offsets.y,
                                    accel_offsets.z);
}

static void NOINLINE send_ahrs(mavlink_channel_t chan)
{
    const Vector3f &omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        0,
        0,
        ahrs.get_error_rp(),
        ahrs.get_error_yaw());
}


#if HIL_MODE != HIL_MODE_DISABLED
/*
  keep last HIL_STATE message to allow sending SIM_STATE
 */
static mavlink_hil_state_t last_hil_state;
#endif

// report simulator state
static void NOINLINE send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.simstate_send(chan);
#elif HIL_MODE != HIL_MODE_DISABLED
    mavlink_msg_simstate_send(chan,
                              last_hil_state.roll,
                              last_hil_state.pitch,
                              last_hil_state.yaw,
                              last_hil_state.xacc*0.001*GRAVITY_MSS,
                              last_hil_state.yacc*0.001*GRAVITY_MSS,
                              last_hil_state.zacc*0.001*GRAVITY_MSS,
                              last_hil_state.rollspeed,
                              last_hil_state.pitchspeed,
                              last_hil_state.yawspeed,
                              last_hil_state.lat,
                              last_hil_state.lon);
#endif
}

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        board_voltage(),
        hal.i2c->lockup_count());
}

static void NOINLINE send_wind(mavlink_channel_t chan)
{
    Vector3f wind = ahrs.wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)), // use negative, to give
                                          // direction wind is coming from
        wind.length(),
        wind.z);
}

static void NOINLINE send_rangefinder(mavlink_channel_t chan)
{
    if (!sonar.enabled()) {
        // no sonar to report
        return;
    }
    mavlink_msg_rangefinder_send(
        chan,
        sonar.distance_cm() * 0.01f,
        sonar.voltage());
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(
        chan,
        g.command_index);
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
    mavlink_msg_statustext_send(
        chan,
        s->severity,
        s->text);
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    uint32_t tnow = millis() >> 10;
    if (tnow > (uint32_t)g.telem_delay) {
        return false;
    }
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // this is USB telemetry, so won't be an Xbee
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry by the TELEM_DELAY time
    return true;
}


// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    if (telemetry_delayed(chan)) {
        return false;
    }

    // if we don't have at least 1ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!in_mavlink_delay && scheduler.time_available_usec() < 1200) {
        gcs_out_of_time = true;
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        gcs[chan-MAVLINK_COMM_0].last_heartbeat_time = hal.scheduler->millis();
        send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_extended_status1(chan);
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        gcs[chan-MAVLINK_COMM_0].send_meminfo();
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        if (control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            send_nav_controller_output(chan);
        }
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        send_gps_raw(chan);
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time(chan);
        break;

    case MSG_SERVO_OUT:
#if HIL_MODE != HIL_MODE_DISABLED
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
#endif
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(chan);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu1(chan);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_raw_imu2(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_raw_imu3(chan);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        gcs[chan-MAVLINK_COMM_0].queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        gcs[chan-MAVLINK_COMM_0].queued_waypoint_send();
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

#if GEOFENCE_ENABLED == ENABLED
    case MSG_FENCE_STATUS:
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        send_fence_status(chan);
        break;
#endif

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(chan);
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder(chan);
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind(chan);
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning

    case MSG_LIMITS_STATUS:
        // unused
        break;
    }
    return true;
}


#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[MAVLINK_COMM_NUM_BUFFERS];

// send a message using mavlink
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message])) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str)
{
    if (telemetry_delayed(chan)) {
        return;
    }

    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
        s->severity = (uint8_t)severity;
        strncpy((char *)s->text, str, sizeof(s->text));
        mavlink_send_message(chan, MSG_STATUSTEXT);
    } else {
        // send immediately
        mavlink_msg_statustext_send(chan, severity, str);
    }
}

/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  1),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  1),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  1),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  1),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  1),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  1),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  1),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  1),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  10),
    AP_GROUPEND
};

void
GCS_MAVLINK::update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;

    // process received bytes
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = comm_receive_ch(chan);

#if CLI_ENABLED == ENABLED
        /* allow CLI to be started by hitting enter 3 times, if no
         *  heartbeat packets have been received */
        if (mavlink_active == 0 && (millis() - _cli_timeout) < 20000 && 
            comm_is_idle(chan)) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                run_cli(_port);
            }
        }
#endif

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            // we exclude radio packets to make it possible to use the
            // CLI over the radio
            if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
                mavlink_active = true;
            }
            handleMessage(&msg);
        }
    }

    if (!waypoint_receiving) {
        return;
    }

    uint32_t tnow = millis();

    if (waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last &&
        tnow > waypoint_timelast_request + 500 + (stream_slowdown*20)) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (millis() - waypoint_timelast_receive) > waypoint_receive_timeout) {
        waypoint_receiving = false;
    }
}

// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRates[stream_num].get();

    // send at a much lower rate while handling waypoints and
    // parameter sends
    if ((stream_num != STREAM_PARAMS) && 
        (waypoint_receiving || _queued_parameter != NULL)) {
        rate *= 0.25;
    }

    if (rate <= 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate) + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_MAVLINK::data_stream_send(void)
{
    gcs_out_of_time = false;

    if (!in_mavlink_delay) {
        handle_log_send(DataFlash);
    }

    if (_queued_parameter != NULL) {
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }

    if (gcs_out_of_time) return;

    if (in_mavlink_delay) {
#if HIL_MODE != HIL_MODE_DISABLED
        // in HIL we need to keep sending servo values to ensure
        // the simulator doesn't pause, otherwise our sensor
        // calibration could stall
        if (stream_trigger(STREAM_RAW_CONTROLLER)) {
            send_message(MSG_SERVO_OUT);
        }
        if (stream_trigger(STREAM_RC_CHANNELS)) {
            send_message(MSG_RADIO_OUT);
        }
#endif
        // don't send any other stream types while in the delay callback
        return;
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_FENCE_STATUS);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        // sent with GPS read
        send_message(MSG_LOCATION);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_WIND);
        send_message(MSG_RANGEFINDER);
        send_message(MSG_SYSTEM_TIME);
    }
}



void
GCS_MAVLINK::send_message(enum ap_message id)
{
    mavlink_send_message(chan, id);
}

void
GCS_MAVLINK::send_text_P(gcs_severity severity, const prog_char_t *str)
{
    mavlink_statustext_t m;
    uint8_t i;
    for (i=0; i<sizeof(m.text); i++) {
        m.text[i] = pgm_read_byte((const prog_char *)(str++));
        if (m.text[i] == '\0') {
            break;
        }
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
    mavlink_send_text(chan, severity, (const char *)m.text);
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    struct Location tell_command = {};                // command for telemetry

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        // decode
        mavlink_request_data_stream_t packet;
        mavlink_msg_request_data_stream_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        int16_t freq = 0;     // packet frequency

        if (packet.start_stop == 0)
            freq = 0;                     // stop sending
        else if (packet.start_stop == 1)
            freq = packet.req_message_rate;                     // start sending
        else
            break;

        switch (packet.req_stream_id) {
        case MAV_DATA_STREAM_ALL:
            // note that we don't set STREAM_PARAMS - that is internal only
            for (uint8_t i=0; i<STREAM_PARAMS; i++) {
                streamRates[i].set_and_save_ifchanged(freq);
            }
            break;
        case MAV_DATA_STREAM_RAW_SENSORS:
            streamRates[STREAM_RAW_SENSORS].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTENDED_STATUS:
            streamRates[STREAM_EXTENDED_STATUS].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_RC_CHANNELS:
            streamRates[STREAM_RC_CHANNELS].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_RAW_CONTROLLER:
            streamRates[STREAM_RAW_CONTROLLER].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_POSITION:
            streamRates[STREAM_POSITION].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTRA1:
            streamRates[STREAM_EXTRA1].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTRA2:
            streamRates[STREAM_EXTRA2].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTRA3:
            streamRates[STREAM_EXTRA3].set_and_save_ifchanged(freq);
            break;
        }
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        // do command
        send_text_P(SEVERITY_LOW,PSTR("command received: "));

        switch(packet.command) {

        case MAV_CMD_NAV_LOITER_UNLIM:
            set_mode(LOITER);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            set_mode(RTL);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_MISSION_START:
            set_mode(AUTO);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if (packet.param1 == 1 ||
                packet.param2 == 1) {
                startup_INS_ground(true);
            } else if (packet.param3 == 1) {
                init_barometer();
                if (airspeed.enabled()) {
                    zero_airspeed();
                }
            }
            if (packet.param4 == 1) {
                trim_radio();
            }
#if !defined( __AVR_ATmega1280__ )
            if (packet.param5 == 1) {
                float trim_roll, trim_pitch;
                AP_InertialSensor_UserInteract_MAVLink interact(chan);
                if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                }
            }
#endif
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (packet.target_component == MAV_COMP_ID_SYSTEM_CONTROL) {
                if (packet.param1 == 1.0f) {
                    // run pre_arm_checks and arm_checks and display failures
                    if (arming.arm(AP_Arming::MAVLINK)) {
                        //only log if arming was successful
                        channel_throttle->enable_out();                        
                        Log_Arm_Disarm();
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                } else if (packet.param1 == 0.0f)  {
                    if (arming.disarm()) {
                        if (arming.arming_required() == AP_Arming::YES_ZERO_PWM) {
                            channel_throttle->disable_out();  
                        }
                        // reset the mission on disarm
                        change_command(0);
                        //only log if disarming was successful
                        Log_Arm_Disarm();
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                } else {
                    result = MAV_RESULT_UNSUPPORTED;
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_MODE:
            switch ((uint16_t)packet.param1) {
            case MAV_MODE_MANUAL_ARMED:
            case MAV_MODE_MANUAL_DISARMED:
                set_mode(MANUAL);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_AUTO_ARMED:
            case MAV_MODE_AUTO_DISARMED:
                set_mode(AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                set_mode(FLY_BY_WIRE_A);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1 || packet.param1 == 3) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(packet.param1 == 3);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        default:
            break;
        }

        mavlink_msg_command_ack_send(
            chan,
            packet.command,
            result);

        break;
    }


    case MAVLINK_MSG_ID_SET_MODE:
    {
        // decode
        mavlink_set_mode_t packet;
        mavlink_msg_set_mode_decode(msg, &packet);

        if (!(packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
            // we ignore base_mode as there is no sane way to map
            // from that bitmap to a APM flight mode. We rely on
            // custom_mode instead.
            break;
        }
        switch (packet.custom_mode) {
        case MANUAL:
        case CIRCLE:
        case STABILIZE:
        case TRAINING:
        case ACRO:
        case FLY_BY_WIRE_A:
        case FLY_BY_WIRE_B:
        case CRUISE:
        case AUTO:
        case RTL:
        case LOITER:
            set_mode((enum FlightMode)packet.custom_mode);
            break;
        }

        break;
    }

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        // decode
        mavlink_mission_request_list_t packet;
        mavlink_msg_mission_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // Start sending waypoints
        mavlink_msg_mission_count_send(
            chan,msg->sysid,
            msg->compid,
            g.command_total + 1);     // + home

        waypoint_receiving       = false;
        waypoint_dest_sysid      = msg->sysid;
        waypoint_dest_compid     = msg->compid;
        break;
    }


    // XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        // decode
        mavlink_mission_request_t packet;
        mavlink_msg_mission_request_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // send waypoint
        tell_command = get_cmd_with_index_raw(packet.seq);

        // set frame of waypoint
        uint8_t frame;

        if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;     // reference frame
        } else {
            frame = MAV_FRAME_GLOBAL;     // reference frame
        }

        float param1 = 0, param2 = 0, param3 = 0, param4 = 0;

        // time that the mav should loiter in milliseconds
        uint8_t current = 0;     // 1 (true), 0 (false)

        if (packet.seq == (uint16_t)g.command_index)
            current = 1;

        uint8_t autocontinue = 1;     // 1 (true), 0 (false)

        float x = 0, y = 0, z = 0;

        if (tell_command.id < MAV_CMD_NAV_LAST || tell_command.id == MAV_CMD_CONDITION_CHANGE_ALT) {
            // command needs scaling
            x = tell_command.lat/1.0e7;     // local (x), global (latitude)
            y = tell_command.lng/1.0e7;     // local (y), global (longitude)
            z = tell_command.alt/1.0e2;
        }

        switch (tell_command.id) {                                              // Switch to map APM command fields inot MAVLink command fields

        case MAV_CMD_NAV_LOITER_TIME:
        case MAV_CMD_NAV_LOITER_TURNS:
            if (tell_command.options & MASK_OPTIONS_LOITER_DIRECTION) {
                param3 = -abs(g.loiter_radius);
            } else {
                param3 = abs(g.loiter_radius);
            }
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_DO_SET_HOME:
            param1 = tell_command.p1;
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            if (tell_command.options & MASK_OPTIONS_LOITER_DIRECTION) {
                param3 = -abs(g.loiter_radius);
            } else {
                param3 = abs(g.loiter_radius);
            }
            break;
        case MAV_CMD_CONDITION_CHANGE_ALT:
            x=0;                                // Clear fields loaded above that we don't want sent for this command
            y=0;
        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
            param1 = tell_command.lat;
            break;

        case MAV_CMD_DO_JUMP:
            param2 = tell_command.lat;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            param4 = tell_command.lng*0.001f; // time
            param3 = tell_command.lat;        // repeat
            param2 = tell_command.alt;        // pwm
            param1 = tell_command.p1;         // channel
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            param3 = tell_command.lat*0.001f; // time
            param2 = tell_command.alt;        // count
            param1 = tell_command.p1;         // relay number
            break;

        case MAV_CMD_DO_CHANGE_SPEED:
            param3 = tell_command.lat;
            param2 = tell_command.alt;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_SET_PARAMETER:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_SET_SERVO:
            param2 = tell_command.alt;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            param1 = tell_command.alt;
            break;
        }

        mavlink_msg_mission_item_send(chan,msg->sysid,
                                      msg->compid,
                                      packet.seq,
                                      frame,
                                      tell_command.id,
                                      current,
                                      autocontinue,
                                      param1,
                                      param2,
                                      param3,
                                      param4,
                                      x,
                                      y,
                                      z);
        break;
    }


    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        // decode
        mavlink_mission_ack_t packet;
        mavlink_msg_mission_ack_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        // decode
        mavlink_param_request_list_t packet;
        mavlink_msg_param_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // mark the firmware version in the tlog
        send_text_P(SEVERITY_LOW, PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text_P(SEVERITY_LOW, PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

        // send system ID if we can
        char sysid[40];
        if (hal.util->get_system_id(sysid)) {
            mavlink_send_text(chan, SEVERITY_LOW, sysid);
        }

        // Start sending parameters - next call to ::update will kick the first one out
        _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
        _queued_parameter_index = 0;
        _queued_parameter_count = _count_parameters();
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        // decode
        mavlink_param_request_read_t packet;
        mavlink_msg_param_request_read_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        enum ap_var_type p_type;
        AP_Param *vp;
        char param_name[AP_MAX_NAME_SIZE+1];
        if (packet.param_index != -1) {
            AP_Param::ParamToken token;
            vp = AP_Param::find_by_index(packet.param_index, &p_type, &token);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter index %d"), packet.param_index);
                break;
            }
            vp->copy_name_token(token, param_name, AP_MAX_NAME_SIZE, true);
            param_name[AP_MAX_NAME_SIZE] = 0;
        } else {
            strncpy(param_name, packet.param_id, AP_MAX_NAME_SIZE);
            param_name[AP_MAX_NAME_SIZE] = 0;
            vp = AP_Param::find(param_name, &p_type);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter %.16s"), packet.param_id);
                break;
            }
        }

        float value = vp->cast_to_float(p_type);
        mavlink_msg_param_value_send(
            chan,
            param_name,
            value,
            mav_var_type(p_type),
            _count_parameters(),
            packet.param_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        // decode
        mavlink_mission_clear_all_t packet;
        mavlink_msg_mission_clear_all_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        // clear all commands
        g.command_total.set_and_save(0);

        // note that we don't send multiple acks, as otherwise a
        // GCS that is doing a clear followed by a set may see
        // the additional ACKs as ACKs of the set operations
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        // decode
        mavlink_mission_set_current_t packet;
        mavlink_msg_mission_set_current_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // set current command
        change_command(packet.seq);

        mavlink_msg_mission_current_send(chan, g.command_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        // decode
        mavlink_mission_count_t packet;
        mavlink_msg_mission_count_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.count > MAX_WAYPOINTS) {
            packet.count = MAX_WAYPOINTS;
        }
        g.command_total.set_and_save(packet.count - 1);

        waypoint_timelast_receive = millis();
        waypoint_timelast_request = 0;
        waypoint_receiving   = true;
        waypoint_request_i   = 0;
        waypoint_request_last= g.command_total;
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        // decode
        mavlink_mission_write_partial_list_t packet;
        mavlink_msg_mission_write_partial_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.start_index > g.command_total ||
            packet.end_index > g.command_total ||
            packet.end_index < packet.start_index) {
            send_text_P(SEVERITY_LOW,PSTR("flight plan update rejected"));
            break;
        }

        waypoint_timelast_receive = millis();
        waypoint_timelast_request = 0;
        waypoint_receiving   = true;
        waypoint_request_i   = packet.start_index;
        waypoint_request_last= packet.end_index;
        break;
    }

#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
    {
        mavlink_set_mag_offsets_t packet;
        mavlink_msg_set_mag_offsets_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        compass.set_offsets(Vector3f(packet.mag_ofs_x, packet.mag_ofs_y, packet.mag_ofs_z));
        break;
    }
#endif

    // XXX receive a WP from GCS and store in EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        // decode
        mavlink_mission_item_t packet;
        uint8_t result = MAV_MISSION_ACCEPTED;

        mavlink_msg_mission_item_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // defaults
        tell_command.id = packet.command;

        switch (packet.frame)
        {
        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
        {
            tell_command.lat = 1.0e7f*packet.x;                                     // in as DD converted to * t7
            tell_command.lng = 1.0e7f*packet.y;                                     // in as DD converted to * t7
            tell_command.alt = packet.z*1.0e2f;                                     // in as m converted to cm
            tell_command.options = 0;                                     // absolute altitude
            break;
        }

#ifdef MAV_FRAME_LOCAL_NED
        case MAV_FRAME_LOCAL_NED:                         // local (relative to home position)
        {
            tell_command.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + home.lat;
            tell_command.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + home.lng;
            tell_command.alt = -packet.z*1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
            break;
        }
#endif

#ifdef MAV_FRAME_LOCAL
        case MAV_FRAME_LOCAL:                         // local (relative to home position)
        {
            tell_command.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + home.lat;
            tell_command.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + home.lng;
            tell_command.alt = packet.z*1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
            break;
        }
#endif

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:                         // absolute lat/lng, relative altitude
        {
            tell_command.lat = 1.0e7f * packet.x;                                     // in as DD converted to * t7
            tell_command.lng = 1.0e7f * packet.y;                                     // in as DD converted to * t7
            tell_command.alt = packet.z * 1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;                                     // store altitude relative!! Always!!
            break;
        }

        default:
            result = MAV_MISSION_UNSUPPORTED_FRAME;
            break;
        }


        if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

        // Switch to map APM command fields into MAVLink command fields
        switch (tell_command.id) {
        case MAV_CMD_NAV_LOITER_UNLIM:
		    if (packet.param3 < 0) {
		        tell_command.options |= MASK_OPTIONS_LOITER_DIRECTION;
		    }
        case MAV_CMD_NAV_WAYPOINT:
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        case MAV_CMD_NAV_LAND:
            break;

        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_NAV_LOITER_TIME:
            if (packet.param3 < 0) {
                tell_command.options |= MASK_OPTIONS_LOITER_DIRECTION;
            } 
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_DO_SET_HOME:
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_CONDITION_CHANGE_ALT:
            tell_command.lat = packet.param1;
            break;

        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
            tell_command.lat = packet.param1;
            break;

        case MAV_CMD_DO_JUMP:
            tell_command.lat = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            tell_command.lng = packet.param4*1000; // time
            tell_command.lat = packet.param3;      // count
            tell_command.alt = packet.param2;      // PWM
            tell_command.p1  = packet.param1;      // channel
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            tell_command.lat = packet.param3*1000; // time
            tell_command.alt = packet.param2;      // count
            tell_command.p1  = packet.param1;      // relay number
            break;

        case MAV_CMD_DO_CHANGE_SPEED:
            tell_command.lat = packet.param3*1000; // convert to milliseconds
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_SET_PARAMETER:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_SET_SERVO:
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:
            break;

        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            // use alt so we can support 32 bit values
            tell_command.alt = packet.param1;
            break;

        default:
            result = MAV_MISSION_UNSUPPORTED;
            break;
        }

        if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

        if(packet.current == 2) {                                               //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
            guided_WP = tell_command;

            // add home alt if needed
            if (guided_WP.options & MASK_OPTIONS_RELATIVE_ALT) {
                guided_WP.alt += home.alt;
            }

            set_mode(GUIDED);

            // make any new wp uploaded instant (in case we are already in Guided mode)
            set_guided_WP();

            // verify we recevied the command
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                0);

        } else if(packet.current == 3) {                                               //current = 3 is a flag to tell us this is a alt change only

            // add home alt if needed
            if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                tell_command.alt += home.alt;
            }

            next_WP.alt = tell_command.alt;

            // verify we recevied the command
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                0);

        } else {
            // Check if receiving waypoints (mission upload expected)
            if (!waypoint_receiving) {
                result = MAV_MISSION_ERROR;
                goto mission_failed;
            }

            // check if this is the requested waypoint
            if (packet.seq != waypoint_request_i) {
                result = MAV_MISSION_INVALID_SEQUENCE;
                goto mission_failed;
            }

            set_cmd_with_index(tell_command, packet.seq);

            // update waypoint receiving state machine
            waypoint_timelast_receive = millis();
            waypoint_timelast_request = 0;
            waypoint_request_i++;

            if (waypoint_request_i > waypoint_request_last) {
                mavlink_msg_mission_ack_send(
                    chan,
                    msg->sysid,
                    msg->compid,
                    result);

                send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
                waypoint_receiving = false;
                // XXX ignores waypoint radius for individual waypoints, can
                // only set WP_RADIUS parameter
            }
        }
        break;

mission_failed:
        // we are rejecting the mission/waypoint
        mavlink_msg_mission_ack_send(
            chan,
            msg->sysid,
            msg->compid,
            result);
        break;
    }

#if GEOFENCE_ENABLED == ENABLED
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (g.fence_action != FENCE_ACTION_NONE) {
            send_text_P(SEVERITY_LOW,PSTR("fencing must be disabled"));
        } else if (packet.count != g.fence_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7;
            point.y = packet.lng*1.0e7;
            set_fence_point_with_index(point, packet.idx);
        }
        break;
    }

    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (packet.idx >= g.fence_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point = get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send(chan, msg->sysid, msg->compid, packet.idx, g.fence_total,
                                         point.x*1.0e-7, point.y*1.0e-7);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    // receive a rally point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_RALLY_POINT: {
        mavlink_rally_point_t packet;
        mavlink_msg_rally_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        
        if (packet.idx >= g.rally_total || 
            packet.idx >= MAX_RALLYPOINTS) {
            send_text_P(SEVERITY_LOW,PSTR("bad rally point message ID"));
            break;
        }

        if (packet.count != g.rally_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad rally point message count"));
            break;
        }

        RallyLocation rally_point;
        rally_point.lat = packet.lat;
        rally_point.lng = packet.lng;
        rally_point.alt = packet.alt;
        rally_point.break_alt = packet.break_alt;
        rally_point.land_dir = packet.land_dir;
        rally_point.flags = packet.flags;
        set_rally_point_with_index(packet.idx, rally_point);
        break;
    }

    //send a rally point to the GCS
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT: {
        mavlink_rally_fetch_point_t packet;
        mavlink_msg_rally_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (packet.idx > g.rally_total) {
            send_text_P(SEVERITY_LOW, PSTR("bad rally point index"));   
            break;
        }
        RallyLocation rally_point;
        if (!get_rally_point_with_index(packet.idx, rally_point)) {
            send_text_P(SEVERITY_LOW, PSTR("failed to set rally point"));   
            break;
        }

        mavlink_msg_rally_point_send(chan, msg->sysid, msg->compid, packet.idx, 
                                     g.rally_total, rally_point.lat, rally_point.lng, 
                                     rally_point.alt, rally_point.break_alt, rally_point.land_dir, 
                                     rally_point.flags);
        break;
    }    

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        AP_Param                  *vp;
        enum ap_var_type var_type;

        // decode
        mavlink_param_set_t packet;
        mavlink_msg_param_set_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // set parameter

        char key[AP_MAX_NAME_SIZE+1];
        strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
        key[AP_MAX_NAME_SIZE] = 0;

        // find the requested parameter
        vp = AP_Param::find(key, &var_type);
        if ((NULL != vp) &&                                 // exists
            !isnan(packet.param_value) &&                       // not nan
            !isinf(packet.param_value)) {                       // not inf

            // add a small amount before casting parameter values
            // from float to integer to avoid truncating to the
            // next lower integer value.
            float rounding_addition = 0.01;

            // handle variables with standard type IDs
            if (var_type == AP_PARAM_FLOAT) {
                ((AP_Float *)vp)->set_and_save(packet.param_value);
            } else if (var_type == AP_PARAM_INT32) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -2147483648.0, 2147483647.0);
                ((AP_Int32 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT16) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -32768, 32767);
                ((AP_Int16 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT8) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -128, 127);
                ((AP_Int8 *)vp)->set_and_save(v);
            } else {
                // we don't support mavlink set on this parameter
                break;
            }

            // Report back the new value if we accepted the change
            // we send the value we actually set, which could be
            // different from the value sent, in case someone sent
            // a fractional value to an integer type
            mavlink_msg_param_value_send(
                chan,
                key,
                vp->cast_to_float(var_type),
                mav_var_type(var_type),
                _count_parameters(),
                -1);     // XXX we don't actually know what its index is...
#if LOGGING_ENABLED == ENABLED
            DataFlash.Log_Write_Parameter(key, vp->cast_to_float(var_type));
#endif
        }

        break;
    }     // end case

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system,packet.target_component))
            break;

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        hal.rcin->set_overrides(v, 8);

        // a RC override message is consiered to be a 'heartbeat' from
        // the ground station for failsafe purposes
        failsafe.last_heartbeat_ms = millis();
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from
        // our GCS for failsafe purposes
        if (msg->sysid != g.sysid_my_gcs) break;
        failsafe.last_heartbeat_ms = millis();
        break;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:
    {
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        last_hil_state = packet;

        float vel = pythagorous2(packet.vx, packet.vy);
        float cog = wrap_360_cd(ToDeg(atan2f(packet.vy, packet.vx)) * 100);

        if (g_gps != NULL) {
            // set gps hil sensor
            g_gps->setHIL(packet.time_usec/1000,
                          packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                          vel*1.0e-2, cog*1.0e-2, 0, 10);
        }

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * (GRAVITY_MSS/1000.0);
        accels.y = packet.yacc * (GRAVITY_MSS/1000.0);
        accels.z = packet.zacc * (GRAVITY_MSS/1000.0);

        ins.set_gyro(gyros);
        ins.set_accel(accels);

        barometer.setHIL(packet.alt*0.001f);
        compass.setHIL(packet.roll, packet.pitch, packet.yaw);
        airspeed.disable();

        // cope with DCM getting badly off due to HIL lag
        if (g.hil_err_limit > 0 &&
            (fabsf(packet.roll - ahrs.roll) > ToRad(g.hil_err_limit) ||
             fabsf(packet.pitch - ahrs.pitch) > ToRad(g.hil_err_limit) ||
             wrap_PI(fabsf(packet.yaw - ahrs.yaw)) > ToRad(g.hil_err_limit))) {
            ahrs.reset_attitude(packet.roll, packet.pitch, packet.yaw);
        }
        break;
    }
#endif // HIL_MODE

#if CAMERA == ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
    {
        camera.configure_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        camera.control_msg(msg);
        break;
    }
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
    {
        camera_mount.configure_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
    {
        camera_mount.control_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_MOUNT_STATUS:
    {
        camera_mount.status_msg(msg);
        break;
    }
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        mavlink_radio_t packet;
        mavlink_msg_radio_decode(msg, &packet);

        // record if the GCS has been receiving radio messages from
        // the aircraft
        if (packet.remrssi != 0) {
            failsafe.last_radio_status_remrssi_ms = hal.scheduler->millis();
        }

        // use the state of the transmit buffer in the radio to
        // control the stream rate, giving us adaptive software
        // flow control
        if (packet.txbuf < 20 && stream_slowdown < 100) {
            // we are very low on space - slow down a lot
            stream_slowdown += 3;
        } else if (packet.txbuf < 50 && stream_slowdown < 100) {
            // we are a bit low on space, slow down slightly
            stream_slowdown += 1;
        } else if (packet.txbuf > 95 && stream_slowdown > 10) {
            // the buffer has plenty of space, speed up a lot
            stream_slowdown -= 2;
        } else if (packet.txbuf > 90 && stream_slowdown != 0) {
            // the buffer has enough space, speed up a bit
            stream_slowdown--;
        }
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        in_log_download = true;
        // fallthru
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!in_mavlink_delay) {
            handle_log_message(msg, DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        in_log_download = false;
        if (!in_mavlink_delay) {
            handle_log_message(msg, DataFlash);
        }
        break;

    default:
        // forward unknown messages to the other link if there is one
        for (uint8_t i=0; i<num_gcs; i++) {
            if (gcs[i].initialised && i != (uint8_t)chan) {
                mavlink_channel_t out_chan = (mavlink_channel_t)i;
                // only forward if it would fit in the transmit buffer
                if (comm_get_txspace(out_chan) > ((uint16_t)msg->len) + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
                    _mavlink_resend_uart(out_chan, msg);
                }
            }
        }
        break;

    } // end switch
} // end handle mavlink

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
static void mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs[0].initialised || in_mavlink_delay) return;

    in_mavlink_delay = true;

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_message(MSG_HEARTBEAT);
        gcs_send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_update();
        gcs_data_stream_send();
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs_send_text_P(SEVERITY_LOW, PSTR("Initialising APM..."));
    }
    check_usb_mux();

    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_message(id);
        }
    }
}

/*
 *  send data streams in the given rate range on both links
 */
static void gcs_data_stream_send(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].data_stream_send();
        }
    }
}

/*
 *  look for incoming commands on the GCS links
 */
static void gcs_update(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].update();
        }
    }
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_text_P(severity, str);
        }
    }
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message_P(str);
#endif
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    gcs[0].pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
    va_end(arg_list);
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message(gcs[0].pending_status.text);
#endif
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            mavlink_send_message((mavlink_channel_t)i, MSG_STATUSTEXT);
        }
    }
}

/*
  send airspeed calibration data
 */
static void gcs_send_airspeed_calibration(const Vector3f &vg)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (comm_get_txspace((mavlink_channel_t)i) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= 
            MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN) {
            airspeed.log_mavlink_send((mavlink_channel_t)i, vg);
        }
    }
}

/**
   retry any deferred messages
 */
static void gcs_retry_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

