#include "Sub.h"

#include "GCS_Mavlink.h"

<<<<<<< HEAD
// default sensors are present and healthy: gyro, accelerometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS)

void Sub::gcs_send_heartbeat(void)
{
    gcs_send_message(MSG_HEARTBEAT);
}

void Sub::gcs_send_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
    gcs().service_statustext();
}

=======
>>>>>>> 14ad9a58bde667b94cfc1aae2e896cebef07ffdf
/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

MAV_TYPE GCS_Sub::frame_type() const
{
    return MAV_TYPE_SUBMARINE;
}

MAV_MODE GCS_MAVLINK_Sub::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (sub.control_mode) {
    case AUTO:
    case GUIDED:
    case CIRCLE:
    case POSHOLD:
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    default:
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    if (sub.motors.armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return (MAV_MODE)_base_mode;
}

uint32_t GCS_Sub::custom_mode() const
{
    return sub.control_mode;
}

MAV_STATE GCS_MAVLINK_Sub::vehicle_system_status() const
{
<<<<<<< HEAD
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (ap.depth_sensor_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif

    // all present sensors enabled by default except altitude and position control and motors which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                              ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                              ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);

    switch (control_mode) {
    case ALT_HOLD:
    case AUTO:
    case GUIDED:
    case CIRCLE:
    case SURFACE:
    case POSHOLD:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    default:
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // default to all healthy except baro, compass, gps and receiver which we set individually
    control_sensors_health = control_sensors_present & ~(MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR_RC_RECEIVER);

    if (sensor_health.depth) { // check the internal barometer only
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (g.compass_enabled && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
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

    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.has_current() && battery.healthy()) {
        // percent remaining is not necessarily accurate at the moment
        //battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in Sub
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
=======
    // set system as critical if any failsafe have triggered
    if (sub.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
>>>>>>> 14ad9a58bde667b94cfc1aae2e896cebef07ffdf
    }

    if (sub.motors.armed()) {
        return MAV_STATE_ACTIVE;
    }

<<<<<<< HEAD
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

void NOINLINE Sub::send_location(mavlink_channel_t chan)
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
    const Vector3f &vel = inertial_nav.get_velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        ap.depth_sensor_present ? (ahrs.get_home().alt + current_loc.alt) * 10UL : 0, // millimeters above sea level
        ap.depth_sensor_present ? current_loc.alt * 10 : 0, // millimeters above ground
        vel.x,                          // X speed cm/s (+ve North)
        vel.y,                          // Y speed cm/s (+ve East)
        vel.z,                          // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);               // compass heading in 1/100 degree
=======
    return MAV_STATE_STANDBY;
>>>>>>> 14ad9a58bde667b94cfc1aae2e896cebef07ffdf
}

void GCS_MAVLINK_Sub::send_nav_controller_output() const
{
    const Vector3f &targets = sub.attitude_control.get_att_target_euler_cd();
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        sub.wp_nav.get_wp_bearing_to_destination() * 1.0e-2f,
        MIN(sub.wp_nav.get_wp_distance_to_destination() * 1.0e-2f, UINT16_MAX),
        sub.pos_control.get_alt_error() * 1.0e-2f,
        0,
        0);
}

int16_t GCS_MAVLINK_Sub::vfr_hud_throttle() const
{
    return (int16_t)(sub.motors.get_throttle() * 100);
}

// Work around to get temperature sensor data out
void GCS_MAVLINK_Sub::send_scaled_pressure3()
{
    if (!sub.celsius.healthy()) {
        return;
    }
    mavlink_msg_scaled_pressure3_send(
        chan,
        AP_HAL::millis(),
        0,
        0,
        sub.celsius.temperature() * 100);
}

bool GCS_MAVLINK_Sub::send_info()
{
    // Just do this all at once, hopefully the hard-wire telemetry requirement means this is ok
    // Name is char[10]
    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("CamTilt",
                     1 - (SRV_Channels::get_output_norm(SRV_Channel::k_mount_tilt) / 2.0f + 0.5f));

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("CamPan",
                     1 - (SRV_Channels::get_output_norm(SRV_Channel::k_mount_pan) / 2.0f + 0.5f));

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("TetherTrn",
                     sub.quarter_turn_count/4);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("Lights1",
                     SRV_Channels::get_output_norm(SRV_Channel::k_rcin9) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("Lights2",
                     SRV_Channels::get_output_norm(SRV_Channel::k_rcin10) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("PilotGain", sub.gain);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("InputHold", sub.input_hold_engaged);

    return true;
}

bool NOINLINE Sub::send_info(mavlink_channel_t chan)
{
    // Just do this all at once, hopefully the hard-wire telemetry requirement means this is ok
    // Name is char[10]
    CHECK_PAYLOAD_SIZE2(NAMED_VALUE_FLOAT);
    mavlink_msg_named_value_float_send(
            chan,
            AP_HAL::millis(),
            "CamTilt",
            1 - (SRV_Channels::get_output_norm(SRV_Channel::k_mount_tilt) / 2.0f + 0.5f));

    CHECK_PAYLOAD_SIZE2(NAMED_VALUE_FLOAT);
    mavlink_msg_named_value_float_send(
            chan,
            AP_HAL::millis(),
            "CamPan",
            1 - (SRV_Channels::get_output_norm(SRV_Channel::k_mount_pan) / 2.0f + 0.5f));

    CHECK_PAYLOAD_SIZE2(NAMED_VALUE_FLOAT);
    mavlink_msg_named_value_float_send(
            chan,
            AP_HAL::millis(),
            "TetherTrn",
            quarter_turn_count/4);

    CHECK_PAYLOAD_SIZE2(NAMED_VALUE_FLOAT);
    mavlink_msg_named_value_float_send(
            chan,
            AP_HAL::millis(),
            "Lights1",
            SRV_Channels::get_output_norm(SRV_Channel::k_rcin9) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE2(NAMED_VALUE_FLOAT);
    mavlink_msg_named_value_float_send(
            chan,
            AP_HAL::millis(),
            "Lights2",
            SRV_Channels::get_output_norm(SRV_Channel::k_rcin10) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE2(NAMED_VALUE_FLOAT);
    mavlink_msg_named_value_float_send(
            chan,
            AP_HAL::millis(),
            "PilotGain",
            gain);

    CHECK_PAYLOAD_SIZE2(NAMED_VALUE_FLOAT);
    mavlink_msg_named_value_float_send(
            chan,
            AP_HAL::millis(),
            "InputHold",
            input_hold_engaged);

    CHECK_PAYLOAD_SIZE2(NAMED_VALUE_FLOAT);
    mavlink_msg_named_value_float_send(
            chan,
            AP_HAL::millis(),
            "StickMode",
            roll_pitch_flag);

    return true;
}

/*
  send PID tuning message
 */
void GCS_MAVLINK_Sub::send_pid_tuning()
{
    const Parameters &g = sub.g;
    AP_AHRS &ahrs = AP::ahrs();
    AC_AttitudeControl_Sub &attitude_control = sub.attitude_control;

    const Vector3f &gyro = ahrs.get_gyro();
    if (g.gcs_pid_mask & 1) {
        const AP_Logger::PID_Info &pid_info = attitude_control.get_rate_roll_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ROLL,
                                    pid_info.target*0.01f,
                                    degrees(gyro.x),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 2) {
        const AP_Logger::PID_Info &pid_info = attitude_control.get_rate_pitch_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH,
                                    pid_info.target*0.01f,
                                    degrees(gyro.y),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 4) {
        const AP_Logger::PID_Info &pid_info = attitude_control.get_rate_yaw_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
                                    pid_info.target*0.01f,
                                    degrees(gyro.z),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 8) {
        const AP_Logger::PID_Info &pid_info = sub.pos_control.get_accel_z_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ACCZ,
                                    pid_info.target*0.01f,
                                    -(ahrs.get_accel_ef_blended().z + GRAVITY_MSS),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

uint8_t GCS_MAVLINK_Sub::sysid_my_gcs() const
{
    return sub.g.sysid_my_gcs;
}

bool GCS_Sub::vehicle_initialised() const {
    return sub.ap.initialised;
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Sub::try_send_message(enum ap_message id)
{
    switch (id) {
<<<<<<< HEAD

    case MSG_NAMED_FLOAT:
        sub.send_info(chan);
        break;

    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        sub.send_heartbeat(chan);
        sub.send_info(chan);
        break;

    case MSG_EXTENDED_STATUS1:
        // send extended status only once vehicle has been initialised
        // to avoid unnecessary errors being reported to user
        if (sub.ap.initialised) {
            CHECK_PAYLOAD_SIZE(SYS_STATUS);
            sub.send_extended_status1(chan);
            CHECK_PAYLOAD_SIZE(POWER_STATUS);
            send_power_status();
        }
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo();
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        sub.send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        sub.send_location(chan);
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position(sub.ahrs);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        sub.send_nav_controller_output(chan);
        break;

    case MSG_GPS_RAW:
        send_gps_raw(sub.gps);

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time(sub.gps);
        break;

    case MSG_SERVO_OUT:
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(0);
        break;

    case MSG_SERVO_OUTPUT_RAW:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        sub.send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        sub.send_vfr_hud(chan);
        break;
=======
>>>>>>> 14ad9a58bde667b94cfc1aae2e896cebef07ffdf

    case MSG_NAMED_FLOAT:
        send_info();
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        sub.terrain.send_request(chan);
#endif
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }

    return true;
}


const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[GCS_MAVLINK::STREAM_RAW_SENSORS],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK_Parameters, streamRates[GCS_MAVLINK::STREAM_EXTENDED_STATUS],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK_Parameters, streamRates[GCS_MAVLINK::STREAM_RC_CHANNELS],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK_Parameters, streamRates[GCS_MAVLINK::STREAM_POSITION],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[GCS_MAVLINK::STREAM_EXTRA1],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK_Parameters, streamRates[GCS_MAVLINK::STREAM_EXTRA2],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[GCS_MAVLINK::STREAM_EXTRA3],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK_Parameters, streamRates[GCS_MAVLINK::STREAM_PARAMS],  0),
    AP_GROUPEND
};

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU,
    MSG_SCALED_IMU2,
    MSG_SCALED_IMU3,
    MSG_SCALED_PRESSURE,
    MSG_SCALED_PRESSURE2,
    MSG_SCALED_PRESSURE3,
    MSG_SENSOR_OFFSETS
};
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
    MSG_MEMINFO,
    MSG_CURRENT_WAYPOINT,
    MSG_GPS_RAW,
    MSG_GPS_RTK,
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_FENCE_STATUS,
    MSG_NAMED_FLOAT
};
static const ap_message STREAM_POSITION_msgs[] = {
    MSG_LOCATION,
    MSG_LOCAL_POSITION
};
static const ap_message STREAM_RC_CHANNELS_msgs[] = {
    MSG_SERVO_OUTPUT_RAW,
    MSG_RC_CHANNELS,
    MSG_RC_CHANNELS_RAW, // only sent on a mavlink1 connection
};
static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
    MSG_SIMSTATE,
    MSG_AHRS2,
    MSG_AHRS3,
    MSG_PID_TUNING
};
static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD
};
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_HWSTATUS,
    MSG_SYSTEM_TIME,
    MSG_RANGEFINDER,
    MSG_DISTANCE_SENSOR,
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    MSG_TERRAIN,
#endif
    MSG_BATTERY2,
    MSG_BATTERY_STATUS,
    MSG_MOUNT_STATUS,
    MSG_OPTICAL_FLOW,
    MSG_GIMBAL_REPORT,
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
#if RPM_ENABLED == ENABLED
    MSG_RPM,
#endif
    MSG_ESC_TELEMETRY,
};
static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

bool GCS_MAVLINK_Sub::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    return sub.do_guided(cmd);
}

void GCS_MAVLINK_Sub::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    // add home alt if needed
    if (cmd.content.location.relative_alt) {
        cmd.content.location.alt += sub.ahrs.get_home().alt;
    }

    // To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
}

MAV_RESULT GCS_MAVLINK_Sub::_handle_command_preflight_calibration_baro()
{
    if (sub.motors.armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm before calibration.");
        return MAV_RESULT_FAILED;
    }

    if (!sub.control_check_barometer()) {
        return MAV_RESULT_FAILED;
    }

    AP::baro().calibrate(true);
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::_handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    if (is_equal(packet.param6,1.0f)) {
        // compassmot calibration
        //result = sub.mavlink_compassmot(chan);
        gcs().send_text(MAV_SEVERITY_INFO, "#CompassMot calibration not supported");
        return MAV_RESULT_UNSUPPORTED;
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet);
}

<<<<<<< HEAD
    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_LIMITS_STATUS);
        send_message(MSG_NAMED_FLOAT);
=======
MAV_RESULT GCS_MAVLINK_Sub::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
>>>>>>> 14ad9a58bde667b94cfc1aae2e896cebef07ffdf
    }
    sub.set_auto_yaw_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

bool GCS_MAVLINK_Sub::set_home_to_current_location(bool _lock) {
    return sub.set_home_to_current_location(_lock);
}
bool GCS_MAVLINK_Sub::set_home(const Location& loc, bool _lock) {
    return sub.set_home(loc, _lock);
}


MAV_RESULT GCS_MAVLINK_Sub::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    switch (packet.command) {
    case MAV_CMD_NAV_LOITER_UNLIM:
        if (!sub.set_mode(POSHOLD, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_LAND:
        if (!sub.set_mode(SURFACE, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_CONDITION_YAW:
        // param1 : target angle [0-360]
        // param2 : speed during change [deg per second]
        // param3 : direction (-1:ccw, +1:cw)
        // param4 : relative offset (1) or absolute angle (0)
        if ((packet.param1 >= 0.0f)   &&
            (packet.param1 <= 360.0f) &&
            (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
            sub.set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, (uint8_t)packet.param4);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_CHANGE_SPEED:
        // param1 : unused
        // param2 : new speed in m/s
        // param3 : unused
        // param4 : unused
        if (packet.param2 > 0.0f) {
            sub.wp_nav.set_speed_xy(packet.param2 * 100.0f);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_MISSION_START:
        if (sub.motors.armed() && sub.set_mode(AUTO, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_MOTOR_TEST:
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        if (!sub.handle_do_motor_test(packet)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    default:
        return GCS_MAVLINK::handle_command_long_packet(packet);
    }
}



void GCS_MAVLINK_Sub::handleMessage(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {    // MAV ID: 0
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if (msg.sysid != sub.g.sysid_my_gcs) {
            break;
        }
        sub.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL: {     // MAV ID: 69
        if (msg.sysid != sub.g.sysid_my_gcs) {
            break;    // Only accept control from our gcs
        }
        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(&msg, &packet);

        if (packet.target != sub.g.sysid_this_mav) {
            break; // only accept control aimed at us
        }

        sub.transform_manual_control_to_rc_override(packet.x,packet.y,packet.z,packet.r,packet.buttons);

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
        // a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        sub.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

<<<<<<< HEAD

    case MAVLINK_MSG_ID_COMMAND_INT: {
        // decode packet
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(msg, &packet);
        switch (packet.command) {
        case MAV_CMD_DO_SET_ROI: {
            // param1 : /* Region of interest mode (not used)*/
            // param2 : /* MISSION index/ target ID (not used)*/
            // param3 : /* ROI index (not used)*/
            // param4 : /* empty */
            // x : lat
            // y : lon
            // z : alt
            // sanity check location
            if (!check_latlng(packet.x, packet.y)) {
                break;
            }
            Location roi_loc;
            roi_loc.lat = packet.x;
            roi_loc.lng = packet.y;
            roi_loc.alt = (int32_t)(packet.z * 100.0f);
            sub.set_auto_yaw_roi(roi_loc);
            result = MAV_RESULT_ACCEPTED;
            break;
        }
        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);
        break;
    }

    // Pre-Flight calibration requests
    case MAVLINK_MSG_ID_COMMAND_LONG: {     // MAV ID: 76
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        switch (packet.command) {
        case MAV_CMD_PREFLIGHT_STORAGE:
            if (is_equal(packet.param1, 2.0f)) {
                AP_Param::erase_all();
                sub.gcs_send_text(MAV_SEVERITY_WARNING, "All parameters reset, reboot board");
                result= MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            if (sub.set_mode(POSHOLD, MODE_REASON_GCS_COMMAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LAND:
            if (sub.set_mode(SURFACE, MODE_REASON_GCS_COMMAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_CONDITION_YAW:
            // param1 : target angle [0-360]
            // param2 : speed during change [deg per second]
            // param3 : direction (-1:ccw, +1:cw)
            // param4 : relative offset (1) or absolute angle (0)
            if ((packet.param1 >= 0.0f)   &&
                    (packet.param1 <= 360.0f) &&
                    (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
                sub.set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, (uint8_t)packet.param4);
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_CHANGE_SPEED:
            // param1 : unused
            // param2 : new speed in m/s
            // param3 : unused
            // param4 : unused
            if (packet.param2 > 0.0f) {
                sub.wp_nav.set_speed_xy(packet.param2 * 100.0f);
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_SET_HOME:
            // param1 : use current (1=use current location, 0=use specified location)
            // param5 : latitude
            // param6 : longitude
            // param7 : altitude (absolute)
            result = MAV_RESULT_FAILED; // assume failure
            if (is_equal(packet.param1,1.0f) || (is_zero(packet.param5) && is_zero(packet.param6) && is_zero(packet.param7))) {
                if (sub.set_home_to_current_location_and_lock()) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else {
                // sanity check location
                if (!check_latlng(packet.param5, packet.param6)) {
                    break;
                }
                Location new_home_loc;
                new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
                new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
                new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
                if (!sub.far_from_EKF_origin(new_home_loc)) {
                    if (sub.set_home_and_lock(new_home_loc)) {
                        result = MAV_RESULT_ACCEPTED;
                    }
                }
            }
            break;

        case MAV_CMD_DO_FLIGHTTERMINATION:
            if (packet.param1 > 0.5f) {
                sub.init_disarm_motors();
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_ROI:
            // param1 : regional of interest mode (not supported)
            // param2 : mission index/ target id (not supported)
            // param3 : ROI index (not supported)
            // param5 : x / lat
            // param6 : y / lon
            // param7 : z / alt
            // sanity check location
            if (!check_latlng(packet.param5, packet.param6)) {
                break;
            }
            Location roi_loc;
            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
            sub.set_auto_yaw_roi(roi_loc);
            result = MAV_RESULT_ACCEPTED;
            break;

#if CAMERA == ENABLED
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
            sub.camera.configure(packet.param1,
                                 packet.param2,
                                 packet.param3,
                                 packet.param4,
                                 packet.param5,
                                 packet.param6,
                                 packet.param7);

            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:
            if (sub.camera.control(packet.param1,
                                   packet.param2,
                                   packet.param3,
                                   packet.param4,
                                   packet.param5,
                                   packet.param6)) {
                sub.log_picture();
            }
            result = MAV_RESULT_ACCEPTED;
            break;
#endif // CAMERA == ENABLED
        case MAV_CMD_DO_MOUNT_CONTROL:
#if MOUNT == ENABLED
            sub.camera_mount.control(packet.param1, packet.param2, packet.param3, (MAV_MOUNT_MODE) packet.param7);
            result = MAV_RESULT_ACCEPTED;
#endif
            break;

        case MAV_CMD_MISSION_START:
            if (sub.motors.armed() && sub.set_mode(AUTO, MODE_REASON_GCS_COMMAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            // exit immediately if armed
            if (sub.motors.armed()) {
                result = MAV_RESULT_FAILED;
                break;
            }
            if (is_equal(packet.param1,1.0f)) {
                if (sub.calibrate_gyros()) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param3,1.0f)) {
                if (sub.motors.armed()) {
                    sub.gcs_send_text(MAV_SEVERITY_INFO, "Disarm before calibration.");
                    result = MAV_RESULT_FAILED;
                } else if (!sub.control_check_barometer()) {
                    result = MAV_RESULT_FAILED;
                } else {
                    sub.init_barometer(true);
                    result = MAV_RESULT_ACCEPTED;
                }
            } else if (is_equal(packet.param4,1.0f)) {
                result = MAV_RESULT_UNSUPPORTED;
            } else if (is_equal(packet.param5,1.0f)) {
                // 3d accel calibration
                result = MAV_RESULT_ACCEPTED;
                if (!sub.calibrate_gyros()) {
                    result = MAV_RESULT_FAILED;
                    break;
                }
                sub.ins.acal_init();
                sub.ins.get_acal()->start(this);

            } else if (is_equal(packet.param5,2.0f)) {
                // calibrate gyros
                if (!sub.calibrate_gyros()) {
                    result = MAV_RESULT_FAILED;
                    break;
                }
                // accel trim
                float trim_roll, trim_pitch;
                if (sub.ins.calibrate_trim(trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    sub.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (is_equal(packet.param6,1.0f)) {
                // compassmot calibration
                //result = sub.mavlink_compassmot(chan);
                sub.gcs_send_text(MAV_SEVERITY_INFO, "#CompassMot calibration not supported");
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS: {
            uint8_t compassNumber = -1;
            if (is_equal(packet.param1, 2.0f)) {
                compassNumber = 0;
            } else if (is_equal(packet.param1, 5.0f)) {
                compassNumber = 1;
            } else if (is_equal(packet.param1, 6.0f)) {
                compassNumber = 2;
            }
            if (compassNumber != (uint8_t) -1) {
                sub.compass.set_and_save_offsets(compassNumber, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            break;
        }

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (is_equal(packet.param1,1.0f)) {
                // attempt to arm and return success or failure
                if (sub.init_arm_motors(true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else if (is_zero(packet.param1))  {
                // force disarming by setting param2 = 21196 is deprecated
                // see COMMAND_LONG DO_FLIGHTTERMINATION
                sub.init_disarm_motors();
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_GET_HOME_POSITION:
            if (sub.ap.home_state != HOME_UNSET) {
                send_home(sub.ahrs.get_home());
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (sub.ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (sub.ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (sub.ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (sub.ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (is_equal(packet.param1,1.0f) || is_equal(packet.param1,3.0f)) {
                // Send an invalid signal to the motors to prevent spinning due to neutral (1500) pwm pulse being cut short
                // For that matter, send an invalid signal to all channels to prevent undesired/unexpected behavior
                hal.rcout->cork();
                for (int i=0; i<NUM_RC_CHANNELS; i++) {
                    // Set to 1 because 0 is interpreted as flag to ignore update
                    hal.rcout->write(i, 1);
                }
                hal.rcout->push();

                result = MAV_RESULT_ACCEPTED;
                // send ack before we reboot
                mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);

                AP_Notify::flags.firmware_update = 1;
                sub.update_notify();
                hal.scheduler->delay(200);
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(is_equal(packet.param1,3.0f));
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
#if AC_FENCE == ENABLED
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
            case 0:
                sub.fence.enable(false);
                break;
            case 1:
                sub.fence.enable(true);
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
            if (sub.handle_do_motor_test(packet)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

#if GRIPPER_ENABLED == ENABLED
        case MAV_CMD_DO_GRIPPER:
            // param1 : gripper number (ignored)
            // param2 : action (0=release, 1=grab). See GRIPPER_ACTIONS enum.
            if (!sub.g2.gripper.enabled()) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
                switch ((uint8_t)packet.param2) {
                case GRIPPER_ACTION_RELEASE:
                    sub.g2.gripper.release();
                    break;
                case GRIPPER_ACTION_GRAB:
                    sub.g2.gripper.grab();
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
                }
            }
            break;
#endif

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
            if (is_equal(packet.param1,1.0f)) {
                send_autopilot_version(FIRMWARE_VERSION);
                result = MAV_RESULT_ACCEPTED;
            }
            break;
        }

        case MAV_CMD_DO_START_MAG_CAL:
        case MAV_CMD_DO_ACCEPT_MAG_CAL:
        case MAV_CMD_DO_CANCEL_MAG_CAL:
            result = sub.compass.handle_mag_cal_command(packet);

            break;

        case MAV_CMD_DO_SEND_BANNER: {
            result = MAV_RESULT_ACCEPTED;

            send_text(MAV_SEVERITY_INFO, FIRMWARE_STRING);

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
            send_text(MAV_SEVERITY_INFO, "PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION);
#endif

            // send system ID if we can
            char sysid[40];
            if (hal.util->get_system_id(sysid)) {
                send_text(MAV_SEVERITY_INFO, sysid);
            }

            break;
        }

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);

        break;
    }

=======
    
>>>>>>> 14ad9a58bde667b94cfc1aae2e896cebef07ffdf
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: { // MAV ID: 82
        // decode packet
        mavlink_set_attitude_target_t packet;
        mavlink_msg_set_attitude_target_decode(&msg, &packet);

        // ensure type_mask specifies to use attitude
        // the thrust can be used from the altitude hold
        if (packet.type_mask & (1<<6)) {
            sub.set_attitude_target_no_gps = {AP_HAL::millis(), packet};
        }

        // ensure type_mask specifies to use attitude and thrust
        if ((packet.type_mask & ((1<<7)|(1<<6))) != 0) {
            break;
        }

        // convert thrust to climb rate
        packet.thrust = constrain_float(packet.thrust, 0.0f, 1.0f);
        float climb_rate_cms = 0.0f;
        if (is_equal(packet.thrust, 0.5f)) {
            climb_rate_cms = 0.0f;
        } else if (packet.thrust > 0.5f) {
            // climb at up to WPNAV_SPEED_UP
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * sub.wp_nav.get_default_speed_up();
        } else {
            // descend at up to WPNAV_SPEED_DN
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * fabsf(sub.wp_nav.get_default_speed_down());
        }
        sub.guided_set_angle(Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]), climb_rate_cms);
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: {   // MAV ID: 84
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((sub.control_mode != GUIDED) && !(sub.control_mode == AUTO && sub.auto_mode == Auto_NavGuided)) {
            break;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
                packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
         */

        // prepare position
        Vector3f pos_vector;
        if (!pos_ignore) {
            // convert to cm
            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                pos_vector += sub.inertial_nav.get_position();
            } else {
                // convert from alt-above-home to alt-above-ekf-origin
                pos_vector.z = sub.pv_alt_above_origin(pos_vector.z);
            }
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // send request
        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.guided_set_destination_posvel(pos_vector, vel_vector);
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.guided_set_velocity(vel_vector);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            sub.guided_set_destination(pos_vector);
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {  // MAV ID: 86
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

        // exit if vehicle is not in Guided, Auto-Guided, or Depth Hold modes
        if ((sub.control_mode != GUIDED)
            && !(sub.control_mode == AUTO && sub.auto_mode == Auto_NavGuided)
            && !(sub.control_mode == ALT_HOLD)) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
         */

        if (!pos_ignore && sub.control_mode == ALT_HOLD) { // Control only target depth when in ALT_HOLD
            sub.pos_control.set_alt_target(packet.alt*100);
            break;
        }

        Vector3f pos_neu_cm;  // position (North, East, Up coordinates) in centimeters

        if (!pos_ignore) {
            // sanity check location
            if (!check_latlng(packet.lat_int, packet.lon_int)) {
                break;
            }
            Location::AltFrame frame;
            if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.coordinate_frame, frame)) {
                // unknown coordinate frame
                break;
            }
            const Location loc{
                packet.lat_int,
                packet.lon_int,
                int32_t(packet.alt*100),
                frame,
            };
            if (!loc.get_vector_from_origin_NEU(pos_neu_cm)) {
                break;
            }
        }

        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.guided_set_destination_posvel(pos_neu_cm, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            sub.guided_set_destination(pos_neu_cm);
        }

        break;
    }

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        sub.terrain.handle_data(chan, msg);
#endif
        break;

<<<<<<< HEAD
#if AC_RALLY == ENABLED
        // receive a rally point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_RALLY_POINT: {
        mavlink_rally_point_t packet;
        mavlink_msg_rally_point_decode(msg, &packet);

        if (packet.idx >= sub.rally.get_rally_total() ||
                packet.idx >= sub.rally.get_rally_max()) {
            send_text(MAV_SEVERITY_NOTICE,"Bad rally point message ID");
            break;
        }

        if (packet.count != sub.rally.get_rally_total()) {
            send_text(MAV_SEVERITY_NOTICE,"Bad rally point message count");
            break;
        }

        // sanity check location
        if (!check_latlng(packet.lat, packet.lng)) {
            break;
        }

        RallyLocation rally_point;
        rally_point.lat = packet.lat;
        rally_point.lng = packet.lng;
        rally_point.alt = packet.alt;
        rally_point.break_alt = packet.break_alt;
        rally_point.land_dir = packet.land_dir;
        rally_point.flags = packet.flags;

        if (!sub.rally.set_rally_point_with_index(packet.idx, rally_point)) {
            send_text(MAV_SEVERITY_CRITICAL, "Error setting rally point");
        }

        break;
    }

    //send a rally point to the GCS
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT: {
        mavlink_rally_fetch_point_t packet;
        mavlink_msg_rally_fetch_point_decode(msg, &packet);

        if (packet.idx > sub.rally.get_rally_total()) {
            send_text(MAV_SEVERITY_NOTICE, "Bad rally point index");
            break;
        }

        RallyLocation rally_point;
        if (!sub.rally.get_rally_point_with_index(packet.idx, rally_point)) {
            send_text(MAV_SEVERITY_NOTICE, "Failed to set rally point");
            break;
        }

        mavlink_msg_rally_point_send_buf(msg,
                                         chan, msg->sysid, msg->compid, packet.idx,
                                         sub.rally.get_rally_total(), rally_point.lat, rally_point.lng,
                                         rally_point.alt, rally_point.break_alt, rally_point.land_dir,
                                         rally_point.flags);
        break;
    }
#endif // AC_RALLY == ENABLED

    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        sub.DataFlash.remote_log_block_status_msg(chan, msg);
        break;

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        send_autopilot_version(FIRMWARE_VERSION);
        break;

=======
>>>>>>> 14ad9a58bde667b94cfc1aae2e896cebef07ffdf
    case MAVLINK_MSG_ID_SET_HOME_POSITION: {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(&msg, &packet);
        if ((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            if (!sub.set_home_to_current_location(true)) {
                // ignore this failure
            }
        } else {
            Location new_home_loc;
            new_home_loc.lat = packet.latitude;
            new_home_loc.lng = packet.longitude;
            new_home_loc.alt = packet.altitude / 10;
            if (sub.far_from_EKF_origin(new_home_loc)) {
                break;
            }
            if (!sub.set_home(new_home_loc, true)) {
                // silently ignored
            }
        }
        break;
    }

    // This adds support for leak detectors in a separate enclosure
    // connected to a mavlink enabled subsystem
    case MAVLINK_MSG_ID_SYS_STATUS: {
        uint32_t MAV_SENSOR_WATER = 0x20000000;
        mavlink_sys_status_t packet;
        mavlink_msg_sys_status_decode(&msg, &packet);
        if ((packet.onboard_control_sensors_enabled & MAV_SENSOR_WATER) && !(packet.onboard_control_sensors_health & MAV_SENSOR_WATER)) {
            sub.leak_detector.set_detect();
        }
    }
        break;

    default:
        handle_common_message(msg);
        break;
    }     // end switch
} // end handle mavlink

uint64_t GCS_MAVLINK_Sub::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
            (sub.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
            GCS_MAVLINK::capabilities()
        );
}

// a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
void GCS_MAVLINK_Sub::handle_rc_channels_override(const mavlink_message_t &msg)
{
    sub.failsafe.last_heartbeat_ms = AP_HAL::millis();
    GCS_MAVLINK::handle_rc_channels_override(msg);
}


/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_flight_termination(const mavlink_command_long_t &packet) {
    if (packet.param1 > 0.5f) {
        sub.arming.disarm();
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

int32_t GCS_MAVLINK_Sub::global_position_int_alt() const {
    if (!sub.ap.depth_sensor_present) {
        return 0;
    }
    return GCS_MAVLINK::global_position_int_alt();
}
int32_t GCS_MAVLINK_Sub::global_position_int_relative_alt() const {
    if (!sub.ap.depth_sensor_present) {
        return 0;
    }
    return GCS_MAVLINK::global_position_int_relative_alt();
}
