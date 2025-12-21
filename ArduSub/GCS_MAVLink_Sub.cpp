#include "Sub.h"

#include "GCS_MAVLink_Sub.h"
#include <AP_RPM/AP_RPM_config.h>

MAV_TYPE GCS_Sub::frame_type() const
{
    return MAV_TYPE_SUBMARINE;
}

uint8_t GCS_MAVLINK_Sub::base_mode() const
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
    case Mode::Number::AUTO:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::POSHOLD:
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

    return _base_mode;
}

uint32_t GCS_Sub::custom_mode() const
{
    return (uint32_t)sub.control_mode;
}

MAV_STATE GCS_MAVLINK_Sub::vehicle_system_status() const
{
    // set system as critical if any failsafe have triggered
    if (sub.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    if (sub.motors.armed()) {
        return MAV_STATE_ACTIVE;
    }
    if (!sub.ap.initialised) {
    	return MAV_STATE_BOOT;
    }

    return MAV_STATE_STANDBY;
}

void GCS_MAVLINK_Sub::send_banner()
{
    GCS_MAVLINK::send_banner();
    send_text(MAV_SEVERITY_INFO, "Frame: %s", sub.motors.get_frame_string());
}

void GCS_MAVLINK_Sub::send_nav_controller_output() const
{
    const Vector3f &targets = sub.attitude_control.get_att_target_euler_cd();
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        sub.wp_nav.get_wp_bearing_to_destination_cd() * 1.0e-2f,
        MIN(sub.wp_nav.get_wp_distance_to_destination_cm() * 1.0e-2f, UINT16_MAX),
        sub.pos_control.get_pos_error_U_cm() * 1.0e-2f,
        0,
        0);
}

int16_t GCS_MAVLINK_Sub::vfr_hud_throttle() const
{
    return (int16_t)(sub.motors.get_throttle() * 100);
}

float GCS_MAVLINK_Sub::vfr_hud_alt() const
{
    return sub.get_alt_msl();
}

// Work around to get temperature sensor data out
void GCS_MAVLINK_Sub::send_scaled_pressure3()
{
#if AP_TEMPERATURE_SENSOR_ENABLED
    float temperature;
    if (!sub.temperature_sensor.get_temperature(temperature)) {
        // Fall back to original behaviour
        GCS_MAVLINK::send_scaled_pressure3();
        return;
    }
    mavlink_msg_scaled_pressure3_send(
        chan,
        AP_HAL::millis(),
        0,
        0,
        temperature * 100,
        0); // TODO: use differential pressure temperature
#else
    // Fall back to standard behaviour
    GCS_MAVLINK::send_scaled_pressure3();
#endif
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
                     (float)sub.quarter_turn_count / 4);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("Lights1",
                     SRV_Channels::get_output_norm(SRV_Channel::k_lights1) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("Lights2",
                     SRV_Channels::get_output_norm(SRV_Channel::k_rcin10) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("PilotGain", sub.gain);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("InputHold", sub.input_hold_engaged);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("RollPitch", sub.roll_pitch_flag);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("RFTarget", sub.mode_surftrak.get_rangefinder_target_cm() * 0.01f);

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
        const AP_PIDInfo &pid_info = attitude_control.get_rate_roll_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ROLL,
                                    pid_info.target*0.01f,
                                    degrees(gyro.x),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 2) {
        const AP_PIDInfo &pid_info = attitude_control.get_rate_pitch_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH,
                                    pid_info.target*0.01f,
                                    degrees(gyro.y),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 4) {
        const AP_PIDInfo &pid_info = attitude_control.get_rate_yaw_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
                                    pid_info.target*0.01f,
                                    degrees(gyro.z),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 8) {
        const AP_PIDInfo &pid_info = sub.pos_control.D_get_accel_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ACCZ,
                                    pid_info.target*0.01f,
                                    -(ahrs.get_accel_ef().z + GRAVITY_MSS),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

bool GCS_Sub::vehicle_initialised() const {
    return sub.ap.initialised;
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Sub::try_send_message(enum ap_message id)
{
    switch (id) {

    case MSG_NAMED_FLOAT:
        send_info();
        break;

#if AP_TERRAIN_AVAILABLE
    case MSG_TERRAIN_REQUEST:
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        sub.terrain.send_request(chan);
        break;
    case MSG_TERRAIN_REPORT:
        CHECK_PAYLOAD_SIZE(TERRAIN_REPORT);
        sub.terrain.send_report(chan);
        break;
#endif

    case MSG_WIND: // other vehicles do something custom with wind:
        return true;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }

    return true;
}

bool GCS_MAVLINK_Sub::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    return sub.do_guided(cmd);
}

MAV_RESULT GCS_MAVLINK_Sub::_handle_command_preflight_calibration_baro(const mavlink_message_t &msg)
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

MAV_RESULT GCS_MAVLINK_Sub::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    if (packet.y == 1) {
        // compassmot calibration
        //result = sub.mavlink_compassmot(chan);
        gcs().send_text(MAV_SEVERITY_INFO, "#CompassMot calibration not supported");
        return MAV_RESULT_UNSUPPORTED;
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}

MAV_RESULT GCS_MAVLINK_Sub::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    sub.mode_auto.set_auto_yaw_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!sub.flightmode->in_guided_mode() && !change_modes) {
        return MAV_RESULT_DENIED;
    }

    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location request_location;
    if (!location_from_command_t(packet, request_location)) {
        return MAV_RESULT_DENIED;
    }

    if (request_location.sanitize(sub.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    // we need to do this first, as we don't want to change the flight mode unless we can also set the target
    if (!sub.mode_guided.guided_set_destination(request_location)) {
        return MAV_RESULT_FAILED;
    }

    if (!sub.flightmode->in_guided_mode()) {
        if (!sub.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        // the position won't have been loaded if we had to change the flight mode, so load it again
        if (!sub.mode_guided.guided_set_destination(request_location)) {
            return MAV_RESULT_FAILED;
        }
    }

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch(packet.command) {

    case MAV_CMD_CONDITION_YAW:
        return handle_MAV_CMD_CONDITION_YAW(packet);

    case MAV_CMD_DO_CHANGE_SPEED:
        return handle_MAV_CMD_DO_CHANGE_SPEED(packet);

    case MAV_CMD_DO_MOTOR_TEST:
        return handle_MAV_CMD_DO_MOTOR_TEST(packet);

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    case MAV_CMD_MISSION_START:
        if (!is_zero(packet.param1) || !is_zero(packet.param2)) {
            // first-item/last item not supported
            return MAV_RESULT_DENIED;
        }
        return handle_MAV_CMD_MISSION_START(packet);

    case MAV_CMD_NAV_LOITER_UNLIM:
        return handle_MAV_CMD_NAV_LOITER_UNLIM(packet);

    case MAV_CMD_NAV_LAND:
        return handle_MAV_CMD_NAV_LAND(packet);

    }

    return GCS_MAVLINK::handle_command_int_packet(packet, msg);
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_NAV_LOITER_UNLIM(const mavlink_command_int_t &packet)
{
        if (!sub.set_mode(Mode::Number::POSHOLD, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_NAV_LAND(const mavlink_command_int_t &packet)
{
        if (!sub.set_mode(Mode::Number::SURFACE, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet)
{
        // param1 : target angle [0-360]
        // param2 : speed during change [deg per second]
        // param3 : direction (-1:ccw, +1:cw)
        // param4 : relative offset (1) or absolute angle (0)
        if ((packet.param1 >= 0.0f)   &&
            (packet.param1 <= 360.0f) &&
            (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
            sub.mode_auto.set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, (uint8_t)packet.param4);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_DENIED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet)
{
    if (!is_positive(packet.param2)) {
        // Target speed must be larger than zero
        return MAV_RESULT_DENIED;
    }

    switch (SPEED_TYPE(packet.param1)) {
        case SPEED_TYPE_CLIMB_SPEED:
        case SPEED_TYPE_DESCENT_SPEED:
        case SPEED_TYPE_ENUM_END:
            break;

        case SPEED_TYPE_AIRSPEED: // Airspeed is treated as ground speed for GCS compatibility
        case SPEED_TYPE_GROUNDSPEED:
            sub.wp_nav.set_speed_NE_cms(packet.param2 * 100.0);
            return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_DENIED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet)
{
        if (sub.motors.armed() && sub.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
}

MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet)
{
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        if (!sub.handle_do_motor_test(packet)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

void GCS_MAVLINK_Sub::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_MANUAL_CONTROL: {     // MAV ID: 69
        if (!gcs().sysid_is_gcs(msg.sysid)) {
            break;    // Only accept control from our gcs
        }
        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(&msg, &packet);

        if (packet.target != gcs().sysid_this_mav()) {
            break; // only accept control aimed at us
        }

        sub.transform_manual_control_to_rc_override(
            packet.x,
            packet.y,
            packet.z,
            packet.r,
            packet.buttons,
            packet.buttons2,
            packet.enabled_extensions,
            packet.s,
            packet.t,
            packet.aux1,
            packet.aux2,
            packet.aux3,
            packet.aux4,
            packet.aux5,
            packet.aux6
        );

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
        // a RC override message is considered to be a 'heartbeat'
        // from the ground station for failsafe purposes
        sysid_mygcs_seen(AP_HAL::millis());
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {     // MAV ID: 70
        if (!gcs().sysid_is_gcs(msg.sysid)) {
            break;    // Only accept control from our gcs
        }

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
        // a RC override message is considered to be a 'heartbeat'
        // from the ground station for failsafe purposes
        
        handle_rc_channels_override(msg);
        break;
    }

    
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
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * sub.wp_nav.get_default_speed_up_cms();
        } else {
            // descend at up to WPNAV_SPEED_DN
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * sub.wp_nav.get_default_speed_down_cms();
        }
        sub.mode_guided.guided_set_angle(Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]), climb_rate_cms);
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: {   // MAV ID: 84
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((sub.control_mode != Mode::Number::GUIDED) && !(sub.control_mode == Mode::Number::AUTO && sub.auto_mode == Auto_NavGuided)) {
            break;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
                packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_FRD) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        // prepare position
        Vector3f pos_vector;
        if (!pos_ignore) {
            // convert to cm
            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            // rotate from body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_FRD ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_FRD ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                pos_vector += sub.inertial_nav.get_position_neu_cm();
            }
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate from body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_FRD || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // prepare yaw
        float yaw_cd =  0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = degrees(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = degrees(packet.yaw_rate) * 100.0f;
        }

        // send request
        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination_posvel(pos_vector, vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_velocity(vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {  // MAV ID: 86
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

        // exit if vehicle is not in Guided, Auto-Guided, or Depth Hold modes
        if ((sub.control_mode != Mode::Number::GUIDED)
            && !(sub.control_mode == Mode::Number::AUTO && sub.auto_mode == Auto_NavGuided)
            && !(sub.control_mode == Mode::Number::ALT_HOLD)) {
            break;
        }

        bool z_ignore        = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_Z_IGNORE;
        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
         */

        if (!z_ignore && sub.control_mode == Mode::Number::ALT_HOLD) { // Control only target depth when in ALT_HOLD
            sub.pos_control.set_pos_desired_U_cm(packet.alt*100);
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
            if (!loc.get_vector_from_origin_NEU_cm(pos_neu_cm)) {
                break;
            }
        }

        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination_posvel(pos_neu_cm, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination(pos_neu_cm);
        }

        break;
    }

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        sub.terrain.handle_data(chan, msg);
#endif
        break;

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
        GCS_MAVLINK::handle_message(msg);
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
#if AP_TERRAIN_AVAILABLE
            (sub.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
            GCS_MAVLINK::capabilities()
        );
}

MAV_RESULT GCS_MAVLINK_Sub::handle_flight_termination(const mavlink_command_int_t &packet)
{
    if (packet.param1 > 0.5f) {
        sub.arming.disarm(AP_Arming::Method::TERMINATION);
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

int32_t GCS_MAVLINK_Sub::global_position_int_alt() const
{
    return static_cast<int32_t>(sub.get_alt_msl() * 1000.0f);
}

int32_t GCS_MAVLINK_Sub::global_position_int_relative_alt() const
{
    return static_cast<int32_t>(sub.get_alt_rel() * 1000.0f);
}

#if HAL_HIGH_LATENCY2_ENABLED
int16_t GCS_MAVLINK_Sub::high_latency_target_altitude() const
{
    AP_AHRS &ahrs = AP::ahrs();
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));

    //return units are m
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        return 0.01 * (global_position_current.alt + sub.pos_control.get_pos_error_U_cm());
    }
    return 0;
    
}

uint8_t GCS_MAVLINK_Sub::high_latency_tgt_heading() const
{
    // return units are deg/2
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        // need to convert -18000->18000 to 0->360/2
        return wrap_360_cd(sub.wp_nav.get_wp_bearing_to_destination_cd()) / 200;
    }
    return 0;      
}
    
uint16_t GCS_MAVLINK_Sub::high_latency_tgt_dist() const
{
    // return units are dm
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        return MIN(sub.wp_nav.get_wp_distance_to_destination_cm() * 0.001, UINT16_MAX);
    }
    return 0;
}

uint8_t GCS_MAVLINK_Sub::high_latency_tgt_airspeed() const
{
    // return units are m/s*5
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        return MIN((sub.pos_control.get_vel_desired_NEU_cms().length()/100) * 5, UINT8_MAX);
    }
    return 0;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

// Send the mode with the given index (not mode number!) return the total number of modes
// Index starts at 1
uint8_t GCS_MAVLINK_Sub::send_available_mode(uint8_t index) const
{
    const Mode* modes[] {
        &sub.mode_manual,
        &sub.mode_stabilize,
        &sub.mode_acro,
        &sub.mode_althold,
        &sub.mode_surftrak,
        &sub.mode_poshold,
        &sub.mode_auto,
        &sub.mode_guided,
        &sub.mode_circle,
        &sub.mode_surface,
        &sub.mode_motordetect,
    };

    const uint8_t mode_count = ARRAY_SIZE(modes);

    // Convert to zero indexed
    const uint8_t index_zero = index - 1;
    if (index_zero >= mode_count) {
        // Mode does not exist!?
        return mode_count;
    }

    // Ask the mode for its name and number
    const char* name = modes[index_zero]->name();
    const uint8_t mode_number = (uint8_t)modes[index_zero]->number();

    mavlink_msg_available_modes_send(
        chan,
        mode_count,
        index,
        MAV_STANDARD_MODE::MAV_STANDARD_MODE_NON_STANDARD,
        mode_number,
        0, // MAV_MODE_PROPERTY bitmask
        name
    );

    return mode_count;
}
