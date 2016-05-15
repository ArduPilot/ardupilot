// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS)

// forward declarations to make compiler happy
static bool do_guided(const AP_Mission::Mission_Command& cmd);

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true if we are out of time in our event timeslice
static bool	gcs_out_of_time;


// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (txspace < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_ ## id ## _LEN) return false

// prototype this for use inside the GCS class
static void gcs_send_text_fmt(const prog_char_t *fmt, ...);

static void gcs_send_heartbeat(void)
{
    gcs_send_message(MSG_HEARTBEAT);
}

static void gcs_send_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

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
    uint8_t system_status = ap.land_complete ? MAV_STATE_STANDBY : MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    // set system as critical if any failsafe have triggered
    if (failsafe.radio || failsafe.battery || failsafe.gps || failsafe.gcs || failsafe.ekf)  {
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
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
    case POSHOLD:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (motors.armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
#if (FRAME_CONFIG == QUAD_FRAME)
        MAV_TYPE_QUADROTOR,
#elif (FRAME_CONFIG == TRI_FRAME)
        MAV_TYPE_TRICOPTER,
#elif (FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME)
        MAV_TYPE_HEXAROTOR,
#elif (FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME)
        MAV_TYPE_OCTOROTOR,
#elif (FRAME_CONFIG == HELI_FRAME)
        MAV_TYPE_HELICOPTER,
#elif (FRAME_CONFIG == SINGLE_FRAME)  //because mavlink did not define a singlecopter, we use a rocket
        MAV_TYPE_ROCKET,
#elif (FRAME_CONFIG == COAX_FRAME)  //because mavlink did not define a singlecopter, we use a rocket
        MAV_TYPE_ROCKET,
#else
  #error Unrecognised frame type
#endif
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    const Vector3f &gyro = ins.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        gyro.x,
        gyro.y,
        gyro.z);
}

#if AC_FENCE == ENABLED
static NOINLINE void send_limits_status(mavlink_channel_t chan)
{
    fence_send_mavlink_status(chan);
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
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (g.optflow_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

    // all present sensors enabled by default except altitude and position control which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);

    switch (control_mode) {
    case ALT_HOLD:
    case AUTO:
    case GUIDED:
    case LOITER:
    case RTL:
    case CIRCLE:
    case LAND:
    case OF_LOITER:
    case POSHOLD:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case SPORT:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    }

    // default to all healthy except baro, compass, gps and receiver which we set individually
    control_sensors_health = control_sensors_present & ~(MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                         MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                         MAV_SYS_STATUS_SENSOR_GPS |
                                                         MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
    if (barometer.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (g.compass_enabled && compass.healthy(0) && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() > AP_GPS::NO_GPS && (!gps_glitch.glitching()||ap.usb_connected)) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (ap.rc_receiver_present && !failsafe.radio) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
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
        (uint16_t)(scheduler.load_average(MAIN_LOOP_MICROS) * 1000),
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
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time = gps.last_fix_time_ms();
    } else {
        fix_time = millis();
    }
    const Vector3f &vel = gps.velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        gps.location().alt * 10UL,      // millimeters above sea level
        current_loc.alt * 10,           // millimeters above ground
        vel.x * 100,  // X speed cm/s (+ve North)
        vel.y * 100,  // Y speed cm/s (+ve East)
        vel.x * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);               // compass heading in 1/100 degree
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    if(is_copter_nav_active()) {
        const Vector3f &targets = attitude_control.angle_ef_targets();
        mavlink_msg_nav_controller_output_send(
            chan,
            targets.x / 1.0e2f,
            targets.y / 1.0e2f,
            targets.z / 1.0e2f,
            wp_bearing / 1.0e2f,
            wp_distance / 1.0e2f,
            pos_control.get_alt_error() / 1.0e2f,
            0,
            0);
    } else {
        mavlink_msg_nav_controller_output_send(
            chan,
            nav_roll_cd / 1.0e2f,
            nav_pitch_cd / 1.0e2f,
            nav_controller->nav_bearing_cd() * 0.01f,
            nav_controller->target_bearing_cd() * 0.01f,
            wp_distance / 1.0e2f,
            auto_throttle_mode*(SpdHgt_Controller.get_current_alt_setpoint() - inertial_nav.get_altitude()) / 1.0e2f,
            0,
            nav_controller->crosstrack_error());
    }
}

// report simulator state
static void NOINLINE send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.simstate_send(chan);
#endif
}

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        hal.analogin->board_voltage()*1000,
        hal.i2c->lockup_count());
}

//BEV adding this one
static void NOINLINE send_bev_status(mavlink_channel_t chan)
{
    uint8_t transition_status = 0, gear_status = 0;
    if(motors.transition_get_direction() == BEV_TransitionState::DIRECTION_TO_COPTER || motors.transition_is_nav_suppressed()) {
        transition_status = BEV_STATUS_TRANSITION_TO_COPTER;
    } else if (motors.transition_get_direction() == BEV_TransitionState::DIRECTION_TO_PLANE) {
        transition_status = BEV_STATUS_TRANSITION_TO_PLANE;
    } else if (motors.transition_is_full_plane()) {
        transition_status = BEV_STATUS_TRANSITION_FULL_PLANE;
    } else {
        transition_status = BEV_STATUS_TRANSITION_FULL_COPTER;
    }

    if(motors.gear_is_raised()) {
        gear_status = BEV_STATUS_GEAR_UP;
    } else {
        gear_status = BEV_STATUS_GEAR_DOWN;
    }

    mavlink_msg_bev_status_send(
            chan,
            transition_status,
            gear_status);
}

#if HIL_MODE != HIL_MODE_DISABLED
static void NOINLINE send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with HIL maintainers

#if FRAME_CONFIG == HELI_FRAME

    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        0,
        0,
        0,
        0,
        receiver_rssi);
#else
 #if X_PLANE == ENABLED
    /* update by JLN for X-Plane HIL */
    if(motors.armed() && ap.auto_armed) {
        mavlink_msg_rc_channels_scaled_send(
            chan,
            millis(),
            0,         // port 0
            g.rc_1.servo_out,
            g.rc_2.servo_out,
            10000 * g.rc_3.norm_output(),
            g.rc_4.servo_out,
            10000 * g.rc_1.norm_output(),
            10000 * g.rc_2.norm_output(),
            10000 * g.rc_3.norm_output(),
            10000 * g.rc_4.norm_output(),
            receiver_rssi);
    }else{
        mavlink_msg_rc_channels_scaled_send(
            chan,
            millis(),
            0,         // port 0
            0,
            0,
            -10000,
            0,
            10000 * g.rc_1.norm_output(),
            10000 * g.rc_2.norm_output(),
            10000 * g.rc_3.norm_output(),
            10000 * g.rc_4.norm_output(),
            receiver_rssi);
    }

 #else
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0,         // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        10000 * g.rc_1.norm_output(),
        10000 * g.rc_2.norm_output(),
        10000 * g.rc_3.norm_output(),
        10000 * g.rc_4.norm_output(),
        receiver_rssi);
 #endif
#endif
}
#endif // HIL_MODE

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0,     // port
        hal.rcout->read(0), //left elevon
        hal.rcout->read(1), //right elevon
        hal.rcout->read(2), //motors 1-6 below
        hal.rcout->read(3),
        hal.rcout->read(4),
        hal.rcout->read(5),
        hal.rcout->read(6),
        hal.rcout->read(7));
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    if(is_copter_nav_active()) {
        mavlink_msg_vfr_hud_send(
            chan,
            gps.ground_speed(),
            gps.ground_speed(),
            (ahrs.yaw_sensor / 100) % 360,
            g.rc_3.servo_out/10,
            current_loc.alt / 100.0f,
            climb_rate / 100.0f);
    } else {
        mavlink_msg_vfr_hud_send(
            chan,
            gps.ground_speed(),
            gps.ground_speed(),
            (ahrs.yaw_sensor / 100) % 360,
            channel_throttle_out/10,
            current_loc.alt / 100.0f,
            climb_rate / 100.0f);
    }
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(chan, mission.get_current_nav_index());
}

#if CONFIG_SONAR == ENABLED
static void NOINLINE send_rangefinder(mavlink_channel_t chan)
{
    // exit immediately if sonar is disabled
    if (!sonar.healthy()) {
        return;
    }
    mavlink_msg_rangefinder_send(
            chan,
            sonar_alt * 0.01f,
            sonar.voltage_mv() * 0.001f);
}
#endif

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
bool GCS_MAVLINK::try_send_message(enum ap_message id)
{
    uint16_t txspace = comm_get_txspace(chan);

    if (telemetry_delayed(chan)) {
        return false;
    }

#if HIL_MODE != HIL_MODE_SENSORS
    // if we don't have at least 250 micros remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (scheduler.time_available_usec() < 250 && motors.armed()) {
        gcs_out_of_time = true;
        return false;
    }
#endif

    switch(id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        gcs[chan-MAVLINK_COMM_0].last_heartbeat_time = hal.scheduler->millis();
        send_heartbeat(chan);
        break;

    case MSG_EXTENDED_STATUS1:
        // send extended status only once vehicle has been initialised
        // to avoid unnecessary errors being reported to user
        if (ap.initialised) {
            CHECK_PAYLOAD_SIZE(SYS_STATUS);
            send_extended_status1(chan);
            CHECK_PAYLOAD_SIZE(POWER_STATUS);
            gcs[chan-MAVLINK_COMM_0].send_power_status();
        }
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
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        send_nav_controller_output(chan);
        break;

    case MSG_GPS_RAW:
        return gcs[chan-MAVLINK_COMM_0].send_gps_raw(gps);

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

#if CONFIG_SONAR == ENABLED
    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder(chan);
        break;
#endif

    case MSG_TERRAIN:
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

#if AC_FENCE == ENABLED
    case MSG_LIMITS_STATUS:
        CHECK_PAYLOAD_SIZE(LIMITS_STATUS);
        send_limits_status(chan);
        break;
#endif

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        gcs[chan-MAVLINK_COMM_0].send_ahrs(ahrs);
        break;

    case MSG_SIMSTATE:
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
#endif
#if AP_AHRS_NAVEKF_AVAILABLE
        CHECK_PAYLOAD_SIZE(AHRS2);
        gcs[chan-MAVLINK_COMM_0].send_ahrs2(ahrs);
#endif
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

        //BEV adding this one
    case MSG_BEV_STATUS:
        CHECK_PAYLOAD_SIZE(BEV_STATUS);
        send_bev_status(chan);
        break;

    case MSG_FENCE_STATUS:
    case MSG_WIND:
        // unused
        break;

    case MSG_RETRY_DEFERRED:
    case MSG_LIMITS_STATUS:
    case MSG_RANGEFINDER:
    case MSG_BATTERY2:
        break; // just here to prevent a warning
    }

    return true;
}


const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  0),
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
    if (waypoint_receiving) {
        // don't interfere with mission transfer
        return;
    }

    if (!in_mavlink_delay && !motors.armed()) {
        handle_log_send(DataFlash);
    }

    gcs_out_of_time = false;

    if (_queued_parameter != NULL) {
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
        // don't send anything else at the same time as parameters
        return;
    }

    if (gcs_out_of_time) return;

    if (in_mavlink_delay) {
        // don't send any other stream types while in the delay callback
        return;
    }

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
        send_message(MSG_LIMITS_STATUS);
        send_message(MSG_BEV_STATUS);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
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
        send_message(MSG_SYSTEM_TIME);
        send_message(MSG_RANGEFINDER);
#if AP_TERRAIN_AVAILABLE
        //send_message(MSG_TERRAIN);
#endif
    }
}


void GCS_MAVLINK::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    do_guided(cmd);
    //BEV point camera to desired target if flying as plane
    if(is_plane_nav_active() && get_key_level() >= BEV_Key::KEY_PRO) {
        //BEV decide if this is desirable behavior, and update w/ new camera gimbal
        //camera_gimbal.point_here(cmd.content.location.lat, cmd.content.location.lng, 0); //default to zero alt
    }
}

void GCS_MAVLINK::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    //BEV only adjust the target altitude if not in auto
    if( (control_mode != AUTO) )
    {
        alt_hold_gs_des_alt = cmd.content.location.alt;
        //allow setting new target in gudied mode as well
        next_WP_loc.alt = cmd.content.location.alt;
    }
    //allow change alt commands to have effect in copter guided mode
    if( (control_mode == GUIDED) && is_copter_nav_active()) {
        Vector3f destination = wp_nav.get_destination();
        destination.z = cmd.content.location.alt;
        guided_set_destination(destination);
    }
}


void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    uint8_t result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:      // MAV ID: 0
    {
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if(msg->sysid != g.sysid_my_gcs) break;
        failsafe.last_heartbeat_ms = millis();
        pmTest1++;
        break;
    }

    case MAVLINK_MSG_ID_SET_MODE:       // MAV ID: 11
    {
        // decode
        mavlink_set_mode_t packet;
        mavlink_msg_set_mode_decode(msg, &packet);

        // exit immediately if this command is not meant for this vehicle
        if (mavlink_check_target(packet.target_system, 0)) {
            break;
        }

        // only accept custom modes because there is no easy mapping from Mavlink flight modes to AC flight modes
        if (packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
            if (set_mode(packet.custom_mode)) {
                result = MAV_RESULT_ACCEPTED;
            }
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, MAVLINK_MSG_ID_SET_MODE, result);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:         // MAV ID: 20
    {
        handle_param_request_read(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:         // MAV ID: 21
    {
        // mark the firmware version in the tlog
        send_text_P(SEVERITY_LOW, PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text_P(SEVERITY_LOW, PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif
        send_text_P(SEVERITY_LOW, PSTR("Frame: " FRAME_CONFIG_STRING));
        handle_param_request_list(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:     // 23
    {
        handle_param_set(msg, &DataFlash);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: // MAV ID: 38
    {
        handle_mission_write_partial_list(mission, msg);
        break;
    }

    // GCS has sent us a command from GCS, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:           // MAV ID: 39
    {
        handle_mission_item(msg, mission);
        break;
    }

    // read an individual command from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:     // MAV ID: 40
    {
        handle_mission_request(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:    // MAV ID: 41
    {
        handle_mission_set_current(mission, msg);
        break;
    }

    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:       // MAV ID: 43
    {
        handle_mission_request_list(mission, msg);
        break;
    }

    // GCS provides the full number of commands it wishes to upload
    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
    case MAVLINK_MSG_ID_MISSION_COUNT:          // MAV ID: 44
    {
        handle_mission_count(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:      // MAV ID: 45
    {
        handle_mission_clear_all(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:    // MAV ID: 66
    {
        handle_request_data_stream(msg, false);
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:       // MAV ID: 70
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        // exit immediately if this command is not meant for this vehicle
        if (mavlink_check_target(packet.target_system,packet.target_component)) {
            break;
        }

        //BEV require pro or mapping key to use joystick
        if(get_key_level() < BEV_Key::KEY_PRO) {
            return;
        }

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        //BEV if all channels are set to zero the override is terminated and set_overrides returns false
        if(!hal.rcin->set_overrides(v, 8)) {
            failsafe.last_rc_override_ms = 0; //prevents the failsafe from triggering
            //BEV add logic to always trigger rc override failsafe if no RC receiver when ending
            //joystick control
            if((failsafe.radio) || (g.failsafe_throttle == FS_THR_DISABLED)) {
                failsafe_rc_override_on_event();
            }
        } else {
            // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
            failsafe.rc_override_active = true;
            // a RC override message is consitered to be a 'heartbeat' from the ground station for failsafe purposes
            failsafe.last_rc_override_ms = millis();
        }
        break;
    }

    // Pre-Flight calibration requests
    case MAVLINK_MSG_ID_COMMAND_LONG:       // MAV ID: 76
    {
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        // exit immediately if this command is not meant for this vehicle
        if (mavlink_check_target(packet.target_system, packet.target_component)) {
            break;
        }

        switch(packet.command) {

        case MAV_CMD_NAV_TAKEOFF:
            //BEV not supported
            break;
            // param4 : yaw angle   (not supported)
            // param5 : latitude    (not supported)
            // param6 : longitude   (not supported)
            // param7 : altitude [metres]
            //if (motors.armed() &&  control_mode == GUIDED) {
            //    set_auto_armed(true);
            //    float takeoff_alt = packet.param7 * 100;      // Convert m to cm
            //    takeoff_alt = max(takeoff_alt,current_loc.alt);
            //    takeoff_alt = max(takeoff_alt,100.0f);
            //    guided_takeoff_start(takeoff_alt);
            //    result = MAV_RESULT_ACCEPTED;
            //} else {
            result = MAV_RESULT_FAILED;
            //}
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            if (set_mode(LOITER)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            if (set_mode(RTL)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LAND:
            if (set_mode(LAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_CONDITION_YAW:
            //BEV not supported
            break;
            // param1 : target angle [0-360]
            // param2 : speed during change [deg per second]
            // param3 : direction (-1:ccw, +1:cw)
            // param4 : relative offset (1) or absolute angle (0)
            if ((packet.param1 >= 0.0f)   &&
            	(packet.param1 <= 360.0f) &&
            	((fabs(packet.param4) < 0.1f) || (fabs(packet.param4-1.0f) < 0.1f))) {
            	set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, (uint8_t)packet.param4);
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_CHANGE_SPEED:
            //BEV not supported
            break;
            // param1 : unused
            // param2 : new speed in m/s
            // param3 : unused
            // param4 : unused
            if (packet.param2 > 0.0f) {
                wp_nav.set_speed_xy(packet.param2 * 100.0f);
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_SET_ROI:
            //BEV not supported
            break;
            // param1 : regional of interest mode (not supported)
            // param2 : mission index/ target id (not supported)
            // param3 : ROI index (not supported)
            // param5 : x / lat
            // param6 : y / lon
            // param7 : z / alt
            Location roi_loc;
            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
            set_auto_yaw_roi(roi_loc);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_MISSION_START:
            if (set_mode(AUTO)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if (packet.param1 == 1) {
                // gyro offset calibration
                ins.init_gyro();
                // reset ahrs gyro bias
                ahrs.reset_gyro_drift();
                result = MAV_RESULT_ACCEPTED;
            }
            if (packet.param3 == 1) {
                init_barometer(false);                      // fast barometer calibration
                result = MAV_RESULT_ACCEPTED;
            }
            //BEV nope. This is too dangerous
            /*
            if (packet.param1 == 3) {
                trim_radio();
                result = MAV_RESULT_ACCEPTED;
            }
            */
            if (packet.param5 == 1) {
                float trim_roll, trim_pitch;
                // this blocks
                AP_InertialSensor_UserInteract_MAVLink interact(chan);
                if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    //BEV don't reset trim roll and pitch when doing cal
                    //ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                }
                result = MAV_RESULT_ACCEPTED;
            }
            if (packet.param6 == 1) {
                // compassmot calibration
                result = mavlink_compassmot(chan);
            }
            break;

        case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            if (packet.param1 == 1.0f) {
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
                if(motors.armed()) {
                    //no need to arm if already armed;
                    result = MAV_RESULT_FAILED;
                    return;
                }
                // run pre_arm_checks and arm_checks and display failures
                //BEV verify the flight mode supports arming (i.e. not in Auto w/ no GPS)
                if( mode_requires_GPS(control_mode) && (ahrs.get_gps().status() < AP_GPS::GPS_OK_FIX_3D)) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("GPS Fix Needed"));
                    result = MAV_RESULT_UNSUPPORTED;
                    break;
                }
                pre_arm_checks(true);
                if(ap.pre_arm_check && arm_checks(true)) {
                    if (init_arm_motors()) {
                    result = MAV_RESULT_ACCEPTED;
                    } else {
                        AP_Notify::flags.arming_failed = true;  // init_arm_motors function will reset flag back to false
                        result = MAV_RESULT_UNSUPPORTED;
                    }
                }else{
                    AP_Notify::flags.arming_failed = true;  // init_arm_motors function will reset flag back to false
                    result = MAV_RESULT_UNSUPPORTED;
                }
            } else if (packet.param1 == 0.0f)  {
                if(!motors.armed()) {
                    //no need to disarm if already disarmed;
                    result = MAV_RESULT_FAILED;
                    return;
                }
                //log that there was a GCS request to disarm
                if(ap.logging_started) {
                    DataFlash.Log_Write_Message_P(PSTR("GCS Disarm"));
                }
                init_disarm_motors();
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;


        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            //BEV don't do if armed
            if(motors.armed()) {
                break;
            }
            if (packet.param1 == 1 || packet.param1 == 3) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(packet.param1 == 3.0f);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
#if AC_FENCE == ENABLED
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
                case 0:
                    fence.enable(false);
                    break;
                case 1:
                    fence.enable(true);
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
#else
            // if fence code is not included return failure
            result = MAV_RESULT_FAILED;
#endif
            break;

        case MAV_CMD_DO_MOTOR_TEST:
            // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
            // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
            // param3 : throttle (range depends upon param2)
            // param4 : timeout (in seconds)
            result = mavlink_motor_test_start(chan, (uint8_t)packet.param1, (uint8_t)packet.param2, (uint16_t)packet.param3, packet.param4);
            break;

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);

        break;
    }

    case MAVLINK_MSG_ID_COMMAND_ACK:        // MAV ID: 77
    {
        command_ack_counter++;
        break;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:          // MAV ID: 90
    {
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        // set gps hil sensor
        Location loc;
        loc.lat = packet.lat;
        loc.lng = packet.lon;
        loc.alt = packet.alt/10;
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                   packet.time_usec/1000,
                   loc, vel, 10, 0, true);

        if (!ap.home_is_set) {
            init_home();
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

        ins.set_gyro(0, gyros);

        ins.set_accel(0, accels);

        barometer.setHIL(packet.alt*0.001f);
        compass.setHIL(packet.roll, packet.pitch, packet.yaw);

        break;
    }
#endif //  HIL_MODE != HIL_MODE_DISABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:       // MAV ID: 109
    {
        handle_radio_status(msg, DataFlash, should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        in_log_download = true;
    // fallthru
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!in_mavlink_delay && !motors.armed()) {
            handle_log_message(msg, DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        in_log_download = false;
        if (!in_mavlink_delay && !motors.armed()) {
            handle_log_message(msg, DataFlash);
        }
        break;

#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, gps);
        break;
#endif

#if CAMERA == ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
        //BEV this is only allowed if mapping key
        if(get_key_level() != BEV_Key::KEY_MAPPING) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Mapping key needed"));
        }
        do_take_picture();
        break;
#endif // CAMERA == ENABLED

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
    {
        //BEV this is only allowed if pro or mapping key
        if(get_key_level() < BEV_Key::KEY_PRO) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Pro key needed"));
            return;
        }
        mavlink_mount_control_t packet;
        mavlink_msg_mount_control_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        //BEV update w/ new gimbal library
        payload_manager.gimbal.point_here(packet.input_a, packet.input_b, packet.input_c);

        //BEV not intuitive, but have the center command also toggle servos. This weird
        //bit comes from the limited number of joystuck buttons we have
        if(!packet.input_a && !packet.input_b) {
            //it's a center command. Also toggle the servos
            servos.toggle();
        }
        break;
    }

    //BEV receive our own messages
    case MAVLINK_MSG_ID_BEV_REQUEST :
    {
        mavlink_bev_request_t packet;
        mavlink_msg_bev_request_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        //send ack if request successful
        if(handle_bev_request(packet.request)) {
            mavlink_msg_command_ack_send_buf(msg, chan, MAVLINK_MSG_ID_BEV_REQUEST, MAV_RESULT_ACCEPTED);
        } else {
            mavlink_msg_command_ack_send_buf(msg, chan, MAVLINK_MSG_ID_BEV_REQUEST, MAV_RESULT_TEMPORARILY_REJECTED);
        }
        break;
    }

    //BEV receive gimbal speed messages
    case MAVLINK_MSG_ID_BEV_GIMBAL_SPEED :
    {
        //BEV this is only allowed if pro or mapping key
        if(get_key_level() < BEV_Key::KEY_PRO) {
            return;
        }

        mavlink_bev_gimbal_speed_t packet;
        mavlink_msg_bev_gimbal_speed_decode(msg, &packet);
        if(mavlink_check_target(packet.target_system, packet.target_component))
            break;

        //BEV update w/ new gimbal driver
        payload_manager.gimbal.set_pitch_yaw_speed(packet.pitch_rate, packet.yaw_rate);
        break;
    }
    }     // end switch
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
        gcs_send_heartbeat();
        gcs_send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_check_input();
        gcs_data_stream_send();
        gcs_send_deferred();
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
static void gcs_check_input(void)
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
    gcs[0].send_message(MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            gcs[i].send_message(MSG_STATUSTEXT);
        }
    }
}
