#include "Sub.h"

#include "GCS_Mavlink.h"

void Sub::gcs_send_heartbeat()
{
    gcs().send_message(MSG_HEARTBEAT);
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

MAV_STATE GCS_MAVLINK_Sub::system_status() const
{
    // set system as critical if any failsafe have triggered
    if (sub.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    if (sub.motors.armed()) {
        return MAV_STATE_ACTIVE;
    }

    return MAV_STATE_STANDBY;
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
                                    pid_info.desired*0.01f,
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
                                    pid_info.desired*0.01f,
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
                                    pid_info.desired*0.01f,
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
                                    pid_info.desired*0.01f,
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

bool GCS_MAVLINK_Sub::vehicle_initialised() const {
    return sub.ap.initialised;
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Sub::try_send_message(enum ap_message id)
{
    // if we don't have at least 250 micros remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (sub.scheduler.time_available_usec() < 250 && sub.motors.armed()) {
        gcs().set_out_of_time(true);
        return false;
    }

    switch (id) {

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


const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[STREAM_RAW_SENSORS],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[STREAM_EXTENDED_STATUS],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[STREAM_RC_CHANNELS],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[STREAM_POSITION],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[STREAM_EXTRA1],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[STREAM_EXTRA2],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[STREAM_EXTRA3],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[STREAM_PARAMS],  0),
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
    MSG_RADIO_IN
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

MAV_RESULT GCS_MAVLINK_Sub::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!check_latlng(roi_loc)) {
        return MAV_RESULT_FAILED;
    }
    sub.set_auto_yaw_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

bool GCS_MAVLINK_Sub::set_home_to_current_location(bool lock) {
    return sub.set_home_to_current_location(lock);
}
bool GCS_MAVLINK_Sub::set_home(const Location& loc, bool lock) {
    return sub.set_home(loc, lock);
}


MAV_RESULT GCS_MAVLINK_Sub::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    switch (packet.command) {
    case MAV_CMD_NAV_LOITER_UNLIM:
        if (!sub.set_mode(POSHOLD, MODE_REASON_GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_LAND:
        if (!sub.set_mode(SURFACE, MODE_REASON_GCS_COMMAND)) {
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

    case MAV_CMD_DO_SET_HOME:
        // param1 : use current (1=use current location, 0=use specified location)
        // param5 : latitude
        // param6 : longitude
        // param7 : altitude (absolute)
        if (is_equal(packet.param1,1.0f) || (is_zero(packet.param5) && is_zero(packet.param6) && is_zero(packet.param7))) {
            if (sub.set_home_to_current_location(true)) {
                return MAV_RESULT_ACCEPTED;
            }
        } else {
            // ensure param1 is zero
            if (!is_zero(packet.param1)) {
                return MAV_RESULT_FAILED;
            }
            Location new_home_loc;
            new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
            if (!sub.far_from_EKF_origin(new_home_loc)) {
                if (sub.set_home(new_home_loc, true)) {
                    return MAV_RESULT_ACCEPTED;
                }
            }
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_MISSION_START:
        if (sub.motors.armed() && sub.set_mode(AUTO, MODE_REASON_GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (is_equal(packet.param1,1.0f)) {
            // attempt to arm and return success or failure
            if (sub.init_arm_motors(AP_Arming::Method::MAVLINK)) {
                return MAV_RESULT_ACCEPTED;
            }
        } else if (is_zero(packet.param1))  {
            // force disarming by setting param2 = 21196 is deprecated
            // see COMMAND_LONG DO_FLIGHTTERMINATION
            sub.init_disarm_motors();
            return MAV_RESULT_ACCEPTED;
        } else {
            return MAV_RESULT_UNSUPPORTED;
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



void GCS_MAVLINK_Sub::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {    // MAV ID: 0
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if (msg->sysid != sub.g.sysid_my_gcs) {
            break;
        }
        sub.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL: {     // MAV ID: 69
        if (msg->sysid != sub.g.sysid_my_gcs) {
            break;    // Only accept control from our gcs
        }
        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(msg, &packet);

        if (packet.target != sub.g.sysid_this_mav) {
            break; // only accept control aimed at us
        }

        sub.transform_manual_control_to_rc_override(packet.x,packet.y,packet.z,packet.r,packet.buttons);

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
        // a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        sub.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: { // MAV ID: 82
        // decode packet
        mavlink_set_attitude_target_t packet;
        mavlink_msg_set_attitude_target_decode(msg, &packet);

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
        mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

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
        mavlink_msg_set_position_target_global_int_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((sub.control_mode != GUIDED) && !(sub.control_mode == AUTO && sub.auto_mode == Auto_NavGuided)) {
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

        Vector3f pos_neu_cm;  // position (North, East, Up coordinates) in centimeters

        if (!pos_ignore) {
            // sanity check location
            if (!check_latlng(packet.lat_int, packet.lon_int)) {
                break;
            }
            Location::AltFrame frame;
            if (!mavlink_coordinate_frame_to_location_alt_frame(packet.coordinate_frame, frame)) {
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

    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
        sub.rangefinder.handle_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        sub.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION: {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(msg, &packet);
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
        mavlink_msg_sys_status_decode(msg, &packet);
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
            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
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
void GCS_MAVLINK_Sub::handle_rc_channels_override(const mavlink_message_t *msg)
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
void Sub::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs().chan(0).initialised) {
        return;
    }

    logger.EnableWrites(false);

    uint32_t tnow = AP_HAL::millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_heartbeat();
        gcs().send_message(MSG_SYS_STATUS);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs().update_receive();
        gcs().update_send();
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs().send_text(MAV_SEVERITY_INFO, "Initialising APM");
    }

    logger.EnableWrites(true);
}

MAV_RESULT GCS_MAVLINK_Sub::handle_flight_termination(const mavlink_command_long_t &packet) {
    if (packet.param1 > 0.5f) {
        sub.init_disarm_motors();
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

bool GCS_MAVLINK_Sub::set_mode(uint8_t mode)
{
    return sub.set_mode((control_mode_t)mode, MODE_REASON_GCS_COMMAND);
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

// dummy method to avoid linking AFS
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) { return false; }
