// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS)

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true if we are out of time in our event timeslice
static bool	gcs_out_of_time;

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
    case AUTOTUNE:
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
    if (control_mode != INITIALISING && ahrs.get_armed()) {
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
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (geofence_present()) {
        control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
    }

    // all present sensors enabled by default except rate control, attitude stabilization, yaw, altitude, position control, geofence and motor output which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL & ~MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION & ~MAV_SYS_STATUS_SENSOR_YAW_POSITION & ~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL & ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS & ~MAV_SYS_STATUS_GEOFENCE);

    if (airspeed.enabled() && airspeed.use()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }

    if (geofence_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
    }

    switch (control_mode) {
    case MANUAL:
        break;

    case ACRO:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        break;

    case STABILIZE:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case FLY_BY_WIRE_B:
    case CRUISE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
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
        break;

    case INITIALISING:
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // default: all present sensors healthy except baro, 3D_MAG, GPS, DIFFERNTIAL_PRESSURE.   GEOFENCE always defaults to healthy.
    control_sensors_health = control_sensors_present & ~(MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                         MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                         MAV_SYS_STATUS_SENSOR_GPS |
                                                         MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
    control_sensors_health |= MAV_SYS_STATUS_GEOFENCE;

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }
    if (ahrs.have_inertial_nav() && !ins.calibrated()) {
        // trying to use EKF without properly calibrated accelerometers
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (barometer.all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (g.compass_enabled && compass.healthy(0) && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (!ins.get_gyro_health_all() || (!g.skip_gyro_cal && !ins.gyro_calibrated_ok_all())) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }
    if (airspeed.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }
    if (geofence_breached()) {
        control_sensors_health &= ~MAV_SYS_STATUS_GEOFENCE;
    }

    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.has_current()) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

#if AP_TERRAIN_AVAILABLE
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

    if (rangefinder.num_sensors() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (g.rangefinder_landing) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
        if (rangefinder.healthy()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;            
        }
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
    uint32_t fix_time_ms;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time_ms = gps.last_fix_time_ms();
    } else {
        fix_time_ms = hal.scheduler->millis();
    }
    const Vector3f &vel = gps.velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time_ms,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        gps.location().alt * 10UL,      // millimeters above sea level
        relative_altitude() * 1.0e3f,    // millimeters above ground
        vel.x * 100,  // X speed cm/s (+ve North)
        vel.y * 100,  // Y speed cm/s (+ve East)
        vel.z * -100, // Z speed cm/s (+ve up)
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
        auto_state.wp_distance,
        altitude_error_cm * 0.01,
        airspeed_error_cm,
        nav_controller->crosstrack_error());
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
        10000 * channel_roll->norm_output() * (channel_roll->get_reverse()?-1:1),
        10000 * channel_pitch->norm_output() * (channel_pitch->get_reverse()?-1:1),
        10000 * channel_throttle->norm_output() * (channel_throttle->get_reverse()?-1:1),
        10000 * channel_rudder->norm_output() * (channel_rudder->get_reverse()?-1:1),
        0,
        0,
        0,
        0,
        receiver_rssi);
}
#endif

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
    mavlink_msg_vfr_hud_send(
        chan,
        aspeed,
        gps.ground_speed(),
        (ahrs.yaw_sensor / 100) % 360,
        throttle_percentage(),
        current_loc.alt / 100.0,
        barometer.get_climb_rate());
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
        hal.analogin->board_voltage()*1000,
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
    if (!rangefinder.healthy()) {
        // no sonar to report
        return;
    }
    mavlink_msg_rangefinder_send(
        chan,
        rangefinder.distance_cm() * 0.01f,
        rangefinder.voltage_mv()*0.001f);
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(chan, mission.get_current_nav_index());
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

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (txspace < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_ ## id ## _LEN) return false


// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK::try_send_message(enum ap_message id)
{
    uint16_t txspace = comm_get_txspace(chan);

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
        CHECK_PAYLOAD_SIZE(POWER_STATUS);
        gcs[chan-MAVLINK_COMM_0].send_power_status();
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
        gcs[chan-MAVLINK_COMM_0].send_gps_raw(gps);
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        gcs[chan-MAVLINK_COMM_0].send_system_time(gps);
        break;

    case MSG_SERVO_OUT:
#if HIL_MODE != HIL_MODE_DISABLED
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
#endif
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        gcs[chan-MAVLINK_COMM_0].send_radio_in(receiver_rssi);
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
        gcs[chan-MAVLINK_COMM_0].send_raw_imu(ins, compass);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        gcs[chan-MAVLINK_COMM_0].send_scaled_pressure(barometer);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        gcs[chan-MAVLINK_COMM_0].send_sensor_offsets(ins, compass, barometer);
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
        gcs[chan-MAVLINK_COMM_0].send_ahrs(ahrs);
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
        CHECK_PAYLOAD_SIZE(AHRS2);
        gcs[chan-MAVLINK_COMM_0].send_ahrs2(ahrs);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder(chan);
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        terrain.send_request(chan);
#endif
        break;

    case MSG_CAMERA_FEEDBACK:
#if CAMERA == ENABLED
        CHECK_PAYLOAD_SIZE(CAMERA_FEEDBACK);
        camera.send_feedback(chan, gps, ahrs, current_loc);
#endif
        break;

    case MSG_BATTERY2:
        CHECK_PAYLOAD_SIZE(BATTERY2);
        gcs[chan-MAVLINK_COMM_0].send_battery2(battery);
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind(chan);
        break;

    case MSG_MOUNT_STATUS:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);
        camera_mount.status_msg(chan);
#endif // MOUNT == ENABLED
        break;

    case MSG_OPTICAL_FLOW:
#if OPTFLOW == ENABLED
        CHECK_PAYLOAD_SIZE(OPTICAL_FLOW);
        gcs[chan-MAVLINK_COMM_0].send_opticalflow(ahrs, optflow);
#endif
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning

    case MSG_LIMITS_STATUS:
        // unused
        break;
    }
    return true;
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
        rate *= 0.25f;
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
#if AP_TERRAIN_AVAILABLE
        send_message(MSG_TERRAIN);
#endif
        send_message(MSG_BATTERY2);
        send_message(MSG_MOUNT_STATUS);
        send_message(MSG_OPTICAL_FLOW);
    }
}


/*
  handle a request to switch to guided mode. This happens via a
  callback from handle_mission_item()
 */
void GCS_MAVLINK::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    guided_WP_loc = cmd.content.location;
    
    // add home alt if needed
    if (guided_WP_loc.flags.relative_alt) {
        guided_WP_loc.alt += home.alt;
        guided_WP_loc.flags.relative_alt = 0;
    }

    set_mode(GUIDED);

    // make any new wp uploaded instant (in case we are already in Guided mode)
    set_guided_WP();
}

/*
  handle a request to change current WP altitude. This happens via a
  callback from handle_mission_item()
 */
void GCS_MAVLINK::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    next_WP_loc.alt = cmd.content.location.alt;
    if (cmd.content.location.flags.relative_alt) {
        next_WP_loc.alt += home.alt;
    }
    next_WP_loc.flags.relative_alt = false;
    next_WP_loc.flags.terrain_alt = cmd.content.location.flags.terrain_alt;
    reset_offset_altitude();
}


void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        handle_request_data_stream(msg, true);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

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

#if MOUNT == ENABLED
        // Sets the region of interest (ROI) for the camera
        case MAV_CMD_DO_SET_ROI:
            Location roi_loc;
            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
            if (roi_loc.lat == 0 && roi_loc.lng == 0 && roi_loc.alt == 0) {
                // switch off the camera tracking if enabled
                if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                    camera_mount.set_mode_to_default();
                }
            } else {
                // send the command to the camera mount
                camera_mount.set_roi_cmd(&roi_loc);
            }
            result = MAV_RESULT_ACCEPTED;
            break;
#endif

        case MAV_CMD_MISSION_START:
            set_mode(AUTO);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if (packet.param3 == 1) {
                in_calibration = true;
                init_barometer();
                if (airspeed.enabled()) {
                    zero_airspeed(false);
                }
                in_calibration = false;
            } else if (packet.param1 == 1 ||
                       packet.param2 == 1) {
                startup_INS_ground(true);
            } else if (packet.param4 == 1) {
                trim_radio();
            } 
#if !defined( __AVR_ATmega1280__ )
            else if (packet.param5 == 1) {
                float trim_roll, trim_pitch;
                AP_InertialSensor_UserInteract_MAVLink interact(chan);
                if (g.skip_gyro_cal) {
                    // start with gyro calibration, otherwise if the user
                    // has SKIP_GYRO_CAL=1 they don't get to do it
                    ins.init_gyro();
                }
                if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                }
            }
#endif
            else {
                    send_text_P(SEVERITY_LOW, PSTR("Unsupported preflight calibration"));
            }
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            if (packet.param1 == 2) {
                // save first compass's offsets
                compass.set_and_save_offsets(0, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            if (packet.param1 == 5) {
                // save secondary compass's offsets
                compass.set_and_save_offsets(1, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
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
                    mission.stop();
                    //only log if disarming was successful
                    Log_Arm_Disarm();
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
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

        case MAV_CMD_DO_LAND_START:
            result = MAV_RESULT_FAILED;
            
            // attempt to switch to next DO_LAND_START command in the mission
            if (jump_to_landing_sequence()) {
                result = MAV_RESULT_ACCEPTED;
            } 
            break;

        case MAV_CMD_DO_GO_AROUND:
            result = MAV_RESULT_FAILED;

            //Not allowing go around at FLIGHT_LAND_FINAL stage on purpose --
            //if plane is close to the ground a go around coudld be dangerous.
            if (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
                //Just tell the autopilot we're done landing so it will 
                //proceed to the next mission item.  If there is no next mission
                //item the plane will head to home point and loiter.
                auto_state.commanded_go_around = true;
               
                result = MAV_RESULT_ACCEPTED;
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Go around command accepted."));           
            } else {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Rejected go around command."));
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
            result = MAV_RESULT_ACCEPTED;
            
            if (!geofence_present()) {
                result = MAV_RESULT_FAILED;
            } switch((uint16_t)packet.param1) {
                case 0:
                    if (! geofence_set_enabled(false, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED;
                    }
                break;
                case 1:
                    if (! geofence_set_enabled(true, GCS_TOGGLED)) {
                        result = MAV_RESULT_FAILED; 
                    }
                break;
                default:
                    result = MAV_RESULT_FAILED;
                break;
            }
            break;

        default:
            break;
        }

        mavlink_msg_command_ack_send_buf(
            msg,
            chan,
            packet.command,
            result);

        break;
    }


    case MAVLINK_MSG_ID_SET_MODE:
    {
        handle_set_mode(msg, mavlink_set_mode);
        break;
    }

    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        handle_mission_request_list(mission, msg);
        break;
    }


    // XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        handle_mission_request(mission, msg);
        break;
    }


    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        // nothing to do
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        // mark the firmware version in the tlog
        send_text_P(SEVERITY_LOW, PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text_P(SEVERITY_LOW, PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif
        handle_param_request_list(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        handle_param_request_read(msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        handle_mission_clear_all(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        // disable cross-track when user asks for WP change, to
        // prevent unexpected flight paths
        auto_state.next_wp_no_crosstrack = true;
        handle_mission_set_current(mission, msg);
        if (control_mode == AUTO && mission.state() == AP_Mission::MISSION_STOPPED) {
            mission.resume();
        }
        break;
    }

    // GCS provides the full number of commands it wishes to upload
    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        handle_mission_count(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        handle_mission_write_partial_list(mission, msg);
        break;
    }

    // GCS has sent us a command from GCS, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        handle_mission_item(msg, mission);
        break;
    }

#if GEOFENCE_ENABLED == ENABLED
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (g.fence_action != FENCE_ACTION_NONE) {
            send_text_P(SEVERITY_LOW,PSTR("fencing must be disabled"));
        } else if (packet.count != g.fence_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7f;
            point.y = packet.lng*1.0e7f;
            set_fence_point_with_index(point, packet.idx);
        }
        break;
    }

    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (packet.idx >= g.fence_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point = get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send_buf(msg, chan, msg->sysid, msg->compid, packet.idx, g.fence_total,
                                             point.x*1.0e-7, point.y*1.0e-7);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    // receive a rally point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_RALLY_POINT: {
        mavlink_rally_point_t packet;
        mavlink_msg_rally_point_decode(msg, &packet);
        
        if (packet.idx >= rally.get_rally_total() || 
            packet.idx >= rally.get_rally_max()) {
            send_text_P(SEVERITY_LOW,PSTR("bad rally point message ID"));
            break;
        }

        if (packet.count != rally.get_rally_total()) {
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
        rally.set_rally_point_with_index(packet.idx, rally_point);
        break;
    }

    //send a rally point to the GCS
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT: {
        mavlink_rally_fetch_point_t packet;
        mavlink_msg_rally_fetch_point_decode(msg, &packet);
        if (packet.idx > rally.get_rally_total()) {
            send_text_P(SEVERITY_LOW, PSTR("bad rally point index"));   
            break;
        }
        RallyLocation rally_point;
        if (!rally.get_rally_point_with_index(packet.idx, rally_point)) {
            send_text_P(SEVERITY_LOW, PSTR("failed to set rally point"));   
            break;
        }

        mavlink_msg_rally_point_send_buf(msg,
                                         chan, msg->sysid, msg->compid, packet.idx, 
                                         rally.get_rally_total(), rally_point.lat, rally_point.lng, 
                                         rally_point.alt, rally_point.break_alt, rally_point.land_dir, 
                                         rally_point.flags);
        break;
    }    

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        handle_param_set(msg, &DataFlash);
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        if (hal.rcin->set_overrides(v, 8)) {
            failsafe.last_valid_rc_ms = hal.scheduler->millis();
        }

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

        // set gps hil sensor
        Location loc;
        memset(&loc, 0, sizeof(loc));
        loc.lat = packet.lat;
        loc.lng = packet.lon;
        loc.alt = packet.alt/10;
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        // setup airspeed pressure based on 3D speed, no wind
        airspeed.setHIL(sq(vel.length()) / 2.0f + 2013);

        gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                   packet.time_usec/1000,
                   loc, vel, 10, 0, true);

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

        ins.set_gyro(0, gyros);
        ins.set_accel(0, accels);

        barometer.setHIL(packet.alt*0.001f);
        compass.setHIL(packet.roll, packet.pitch, packet.yaw);

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
        break;
    }

    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        do_take_picture();
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
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        handle_radio_status(msg, DataFlash, should_log(MASK_LOG_PM));
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

#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, gps);
        break;
#endif

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        terrain.handle_data(chan, msg);
#endif
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
#if CLI_ENABLED == ENABLED
            gcs[i].update(run_cli);
#else
            gcs[i].update(NULL);
#endif
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
    gcs[0].send_message(MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            gcs[i].send_message(MSG_STATUSTEXT);
        }
    }
}

/*
  send airspeed calibration data
 */
static void gcs_send_airspeed_calibration(const Vector3f &vg)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            if (comm_get_txspace((mavlink_channel_t)i) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= 
                MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN) {
                airspeed.log_mavlink_send((mavlink_channel_t)i, vg);
            }
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

