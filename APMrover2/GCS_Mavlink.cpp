#include "Rover.h"

#include "GCS_Mavlink.h"

#include <AP_RangeFinder/RangeFinder_Backend.h>

MAV_TYPE GCS_MAVLINK_Rover::frame_type() const
{
    if (rover.is_boat()) {
        return MAV_TYPE_SURFACE_BOAT;
    }
    return MAV_TYPE_GROUND_ROVER;
}

MAV_MODE GCS_MAVLINK_Rover::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    if (rover.control_mode->has_manual_input()) {
        _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

    if (rover.control_mode->is_autopilot_mode()) {
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }

#if defined(ENABLE_STICK_MIXING) && (ENABLE_STICK_MIXING == ENABLED) // TODO ???? Remove !
    if (control_mode->stick_mixing_enabled()) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    _base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (rover.control_mode != &rover.mode_initializing && rover.arming.is_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

return (MAV_MODE)_base_mode;
}

uint32_t GCS_MAVLINK_Rover::custom_mode() const
{
    return rover.control_mode->mode_number();
}

MAV_STATE GCS_MAVLINK_Rover::system_status() const
{
    if ((rover.failsafe.triggered != 0) || rover.failsafe.ekf) {
        return MAV_STATE_CRITICAL;
    }
    if (rover.control_mode == &rover.mode_initializing) {
        return MAV_STATE_CALIBRATING;
    }
    if (rover.control_mode == &rover.mode_hold) {
        return MAV_STATE_STANDBY;
    }

    return MAV_STATE_ACTIVE;
}

void GCS_Rover::get_sensor_status_flags(uint32_t &present,
                                                uint32_t &enabled,
                                                uint32_t &health)
{
    rover.update_sensor_status_flags();

    present = rover.control_sensors_present;
    enabled = rover.control_sensors_enabled;
    health = rover.control_sensors_health;
}

void GCS_MAVLINK_Rover::send_nav_controller_output() const
{
    if (!rover.control_mode->is_autopilot_mode()) {
        return;
    }

    const Mode *control_mode = rover.control_mode;

    mavlink_msg_nav_controller_output_send(
        chan,
        0,  // roll
        degrees(rover.g2.attitude_control.get_desired_pitch()),
        control_mode->nav_bearing(),
        control_mode->wp_bearing(),
        MIN(control_mode->get_distance_to_destination(), UINT16_MAX),
        0,
        control_mode->speed_error(),
        control_mode->crosstrack_error());
}

void Rover::send_servo_out(mavlink_channel_t chan)
{
    float motor1, motor3;
    if (g2.motors.have_skid_steering()) {
        motor1 = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft) / 1000.0f);
        motor3 = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight) / 1000.0f);
    } else {
        motor1 = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_steering) / 4500.0f);
        motor3 = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 100.0f);
    }
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0,  // port 0
        motor1,
        0,
        motor3,
        0,
        0,
        0,
        0,
        0,
        rssi.read_receiver_rssi_uint8());
}

int16_t GCS_MAVLINK_Rover::vfr_hud_throttle() const
{
    return rover.g2.motors.get_throttle();
}

void GCS_MAVLINK_Rover::send_rangefinder() const
{
    float distance_cm;
    float voltage;
    bool got_one = false;

    // report smaller distance of all rangefinders
    for (uint8_t i=0; i<rover.rangefinder.num_sensors(); i++) {
        AP_RangeFinder_Backend *s = rover.rangefinder.get_backend(i);
        if (s == nullptr) {
            continue;
        }
        if (!got_one ||
            s->distance_cm() < distance_cm) {
            distance_cm = s->distance_cm();
            voltage = s->voltage_mv();
            got_one = true;
        }
    }
    if (!got_one) {
        // no relevant data found
        return;
    }

    mavlink_msg_rangefinder_send(
        chan,
        distance_cm * 0.01f,
        voltage);
}

/*
  send RPM packet
 */
void Rover::send_rpm(mavlink_channel_t chan)
{
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        mavlink_msg_rpm_send(
            chan,
            rpm_sensor.get_rpm(0),
            rpm_sensor.get_rpm(1));
    }
}

/*
  send PID tuning message
 */
void Rover::send_pid_tuning(mavlink_channel_t chan)
{
    const AP_Logger::PID_Info *pid_info;
    // steering PID
    if (g.gcs_pid_mask & 1) {
        pid_info = &g2.attitude_control.get_steering_rate_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_STEER,
                                    degrees(pid_info->desired),
                                    degrees(ahrs.get_yaw_rate_earth()),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // speed to throttle PID
    if (g.gcs_pid_mask & 2) {
        pid_info = &g2.attitude_control.get_throttle_speed_pid().get_pid_info();
        float speed = 0.0f;
        g2.attitude_control.get_forward_speed(speed);
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ACCZ,
                                    pid_info->desired,
                                    speed,
                                    0,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // pitch to throttle pid
    if (g.gcs_pid_mask & 4) {
        pid_info = &g2.attitude_control.get_pitch_to_throttle_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH,
                                    degrees(pid_info->desired),
                                    degrees(ahrs.pitch),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // left wheel rate control pid
    if (g.gcs_pid_mask & 8) {
        pid_info = &g2.wheel_rate_control.get_pid(0).get_pid_info();
        mavlink_msg_pid_tuning_send(chan, 7,
                                    pid_info->desired,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // right wheel rate control pid
    if (g.gcs_pid_mask & 16) {
        pid_info = &g2.wheel_rate_control.get_pid(1).get_pid_info();
        mavlink_msg_pid_tuning_send(chan, 8,
                                    pid_info->desired,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // sailboat heel to mainsail pid
    if (g.gcs_pid_mask & 32) {
        pid_info = &g2.attitude_control.get_sailboat_heel_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, 9,
                                    pid_info->desired,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

void Rover::send_wheel_encoder_distance(mavlink_channel_t chan)
{
    // send wheel encoder data using wheel_distance message
    if (g2.wheel_encoder.num_sensors() > 0) {
        double distances[MAVLINK_MSG_WHEEL_DISTANCE_FIELD_DISTANCE_LEN] {};
        for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
            distances[i] = wheel_encoder_last_distance_m[i];
        }
        mavlink_msg_wheel_distance_send(chan, 1000UL * wheel_encoder_last_ekf_update_ms, g2.wheel_encoder.num_sensors(), distances);
    }
}

uint8_t GCS_MAVLINK_Rover::sysid_my_gcs() const
{
    return rover.g.sysid_my_gcs;
}
bool GCS_MAVLINK_Rover::sysid_enforce() const
{
    return rover.g2.sysid_enforce;
}

uint32_t GCS_MAVLINK_Rover::telem_delay() const
{
    return static_cast<uint32_t>(rover.g.telem_delay);
}

bool GCS_MAVLINK_Rover::vehicle_initialised() const
{
    return rover.control_mode != &rover.mode_initializing;
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Rover::try_send_message(enum ap_message id)
{
    // if we don't have at least 0.2ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!hal.scheduler->in_delay_callback() &&
        !AP_BoardConfig::in_sensor_config_error() &&
        rover.scheduler.time_available_usec() < 200) {
        gcs().set_out_of_time(true);
        return false;
    }

    switch (id) {

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        rover.send_servo_out(chan);
        break;

    case MSG_RPM:
        CHECK_PAYLOAD_SIZE(RPM);
        rover.send_rpm(chan);
        break;

    case MSG_WHEEL_DISTANCE:
        CHECK_PAYLOAD_SIZE(WHEEL_DISTANCE);
        rover.send_wheel_encoder_distance(chan);
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        rover.g2.windvane.send_wind(chan);
        break;

    case MSG_PID_TUNING:
        CHECK_PAYLOAD_SIZE(PID_TUNING);
        rover.send_pid_tuning(chan);
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

void GCS_MAVLINK_Rover::packetReceived(const mavlink_status_t &status, mavlink_message_t &msg)
{
    // pass message to follow library
    rover.g2.follow.handle_msg(msg);
    GCS_MAVLINK::packetReceived(status, msg);
}

/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
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
};
static const ap_message STREAM_POSITION_msgs[] = {
    MSG_LOCATION,
    MSG_LOCAL_POSITION
};
static const ap_message STREAM_RAW_CONTROLLER_msgs[] = {
    MSG_SERVO_OUT,
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
    MSG_PID_TUNING,
};
static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD
};
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_HWSTATUS,
    MSG_WIND,
    MSG_RANGEFINDER,
    MSG_DISTANCE_SENSOR,
    MSG_SYSTEM_TIME,
    MSG_BATTERY2,
    MSG_BATTERY_STATUS,
    MSG_MOUNT_STATUS,
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
    MSG_RPM,
    MSG_WHEEL_DISTANCE,
    MSG_ESC_TELEMETRY,
};
static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RAW_CONTROLLER),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

bool GCS_MAVLINK_Rover::in_hil_mode() const
{
#if HIL_MODE != HIL_MODE_DISABLED
    return rover.g.hil_mode == 1;
#endif
    return false;
}

bool GCS_MAVLINK_Rover::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    if (rover.control_mode != &rover.mode_guided) {
        // only accept position updates when in GUIDED mode
        return false;
    }

    // make any new wp uploaded instant (in case we are already in Guided mode)
    rover.mode_guided.set_desired_location(cmd.content.location);
    return true;
}

void GCS_MAVLINK_Rover::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    // nothing to do
}

MAV_RESULT GCS_MAVLINK_Rover::_handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    if (is_equal(packet.param4, 1.0f)) {
        if (rover.trim_radio()) {
            return MAV_RESULT_ACCEPTED;
        } else {
            return MAV_RESULT_FAILED;
        }
    } else if (is_equal(packet.param6, 1.0f)) {
        if (rover.g2.windvane.start_calibration()) {
            return MAV_RESULT_ACCEPTED;
        } else {
            return MAV_RESULT_FAILED;
        }
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet);
}

bool GCS_MAVLINK_Rover::set_home_to_current_location(bool lock) {
    return rover.set_home_to_current_location(lock);
}
bool GCS_MAVLINK_Rover::set_home(const Location& loc, bool lock) {
    return rover.set_home(loc, lock);
}

MAV_RESULT GCS_MAVLINK_Rover::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    switch (packet.command) {

    case MAV_CMD_DO_CHANGE_SPEED:
        // param1 : unused
        // param2 : new speed in m/s
        if (!rover.control_mode->set_desired_speed(packet.param2)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_DO_SET_REVERSE:
        // param1 : Direction (0=Forward, 1=Reverse)
        rover.control_mode->set_reversed(is_equal(packet.param1,1.0f));
        return MAV_RESULT_ACCEPTED;

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet);
    }
}

MAV_RESULT GCS_MAVLINK_Rover::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    switch (packet.command) {

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        if (rover.set_mode(rover.mode_rtl, MODE_REASON_GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_MISSION_START:
        if (rover.set_mode(rover.mode_auto, MODE_REASON_GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (is_equal(packet.param1, 1.0f)) {
            // run pre_arm_checks and arm_checks and display failures
            if (rover.arm_motors(AP_Arming::MAVLINK)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else if (is_zero(packet.param1))  {
            if (rover.disarm_motors()) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        }
        return MAV_RESULT_UNSUPPORTED;

    case MAV_CMD_DO_CHANGE_SPEED:
        // param1 : unused
        // param2 : new speed in m/s
        if (!rover.control_mode->set_desired_speed(packet.param2)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_DO_SET_HOME:
    {
        // param1 : use current (1=use current location, 0=use specified location)
        // param5 : latitude
        // param6 : longitude
        // param7 : altitude
        if (is_equal(packet.param1, 1.0f)) {
            if (rover.set_home_to_current_location(true)) {
                return MAV_RESULT_ACCEPTED;
            }
        } else {
            // ensure param1 is zero
            if (!is_zero(packet.param1)) {
                return MAV_RESULT_FAILED;
            }
            Location new_home_loc {};
            new_home_loc.lat = static_cast<int32_t>(packet.param5 * 1.0e7f);
            new_home_loc.lng = static_cast<int32_t>(packet.param6 * 1.0e7f);
            new_home_loc.alt = static_cast<int32_t>(packet.param7 * 100.0f);
            if (rover.set_home(new_home_loc, true)) {
                return MAV_RESULT_ACCEPTED;
            }
        }
        return MAV_RESULT_FAILED;
    }

    case MAV_CMD_DO_SET_REVERSE:
        // param1 : Direction (0=Forward, 1=Reverse)
        rover.control_mode->set_reversed(is_equal(packet.param1,1.0f));
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_SET_YAW_SPEED:
    {
        // param1 : yaw angle to adjust direction by in centidegress
        // param2 : Speed - normalized to 0 .. 1

        // exit if vehicle is not in Guided mode
        if (rover.control_mode != &rover.mode_guided) {
            return MAV_RESULT_FAILED;
        }

        // send yaw change and target speed to guided mode controller
        const float speed_max = rover.control_mode->get_speed_default();
        const float target_speed = constrain_float(packet.param2 * speed_max, -speed_max, speed_max);
        rover.mode_guided.set_desired_heading_delta_and_speed(packet.param1, target_speed);
        return MAV_RESULT_ACCEPTED;
    }

    case MAV_CMD_DO_MOTOR_TEST:
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        return rover.mavlink_motor_test_start(chan,
                                              static_cast<uint8_t>(packet.param1),
                                              static_cast<uint8_t>(packet.param2),
                                              static_cast<int16_t>(packet.param3),
                                              packet.param4);

    default:
        return GCS_MAVLINK::handle_command_long_packet(packet);
    }
}


// a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
void GCS_MAVLINK_Rover::handle_rc_channels_override(const mavlink_message_t *msg)
{
    rover.failsafe.last_heartbeat_ms = AP_HAL::millis();
    GCS_MAVLINK::handle_rc_channels_override(msg);
}


void GCS_MAVLINK_Rover::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        if (msg->sysid != rover.g.sysid_my_gcs) {  // Only accept control from our gcs
            break;
        }

        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(msg, &packet);

        if (packet.target != rover.g.sysid_this_mav) {
            break; // only accept control aimed at us
        }

        uint32_t tnow = AP_HAL::millis();
        
        const int16_t roll = (packet.y == INT16_MAX) ? 0 : rover.channel_steer->get_radio_min() + (rover.channel_steer->get_radio_max() - rover.channel_steer->get_radio_min()) * (packet.y + 1000) / 2000.0f;
        const int16_t throttle = (packet.z == INT16_MAX) ? 0 : rover.channel_throttle->get_radio_min() + (rover.channel_throttle->get_radio_max() - rover.channel_throttle->get_radio_min()) * (packet.z + 1000) / 2000.0f;
        RC_Channels::set_override(uint8_t(rover.rcmap.roll() - 1), roll, tnow);
        RC_Channels::set_override(uint8_t(rover.rcmap.throttle() - 1), throttle, tnow);

        // a manual control message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        rover.failsafe.last_heartbeat_ms = tnow;
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // we keep track of the last time we received a heartbeat from our GCS for failsafe purposes
            if (msg->sysid != rover.g.sysid_my_gcs) {
                break;
            }
            rover.failsafe.last_heartbeat_ms = AP_HAL::millis();
            break;
        }

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:   // MAV ID: 82
        {
            // decode packet
            mavlink_set_attitude_target_t packet;
            mavlink_msg_set_attitude_target_decode(msg, &packet);

            // exit if vehicle is not in Guided mode
            if (rover.control_mode != &rover.mode_guided) {
                break;
            }

            // ensure type_mask specifies to use thrust
            if ((packet.type_mask & MAVLINK_SET_ATT_TYPE_MASK_THROTTLE_IGNORE) != 0) {
                break;
            }

            // convert thrust to ground speed
            packet.thrust = constrain_float(packet.thrust, -1.0f, 1.0f);
            const float target_speed = rover.control_mode->get_speed_default() * packet.thrust;

            // if the body_yaw_rate field is ignored, convert quaternion to heading
            if ((packet.type_mask & MAVLINK_SET_ATT_TYPE_MASK_YAW_RATE_IGNORE) != 0) {
                // convert quaternion to heading
                float target_heading_cd = degrees(Quaternion(packet.q[0], packet.q[1], packet.q[2], packet.q[3]).get_euler_yaw()) * 100.0f;
                rover.mode_guided.set_desired_heading_and_speed(target_heading_cd, target_speed);
            } else {
                // use body_yaw_rate field
                rover.mode_guided.set_desired_turn_rate_and_speed((RAD_TO_DEG * packet.body_yaw_rate) * 100.0f, target_speed);
            }
            break;
        }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
        {
            // decode packet
            mavlink_set_position_target_local_ned_t packet;
            mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

            // exit if vehicle is not in Guided mode
            if (rover.control_mode != &rover.mode_guided) {
                break;
            }

            // check for supported coordinate frames
            if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
                packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
                break;
            }

            bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
            bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
            bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
            bool yaw_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
            bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

            // prepare target position
            Location target_loc = rover.current_loc;
            if (!pos_ignore) {
                switch (packet.coordinate_frame) {
                case MAV_FRAME_BODY_NED:
                case MAV_FRAME_BODY_OFFSET_NED: {
                    // rotate from body-frame to NE frame
                    const float ne_x = packet.x * rover.ahrs.cos_yaw() - packet.y * rover.ahrs.sin_yaw();
                    const float ne_y = packet.x * rover.ahrs.sin_yaw() + packet.y * rover.ahrs.cos_yaw();
                    // add offset to current location
                    location_offset(target_loc, ne_x, ne_y);
                    }
                    break;

                case MAV_FRAME_LOCAL_OFFSET_NED:
                    // add offset to current location
                    location_offset(target_loc, packet.x, packet.y);
                    break;

                default:
                    // MAV_FRAME_LOCAL_NED interpret as an offset from home
                    target_loc = rover.ahrs.get_home();
                    location_offset(target_loc, packet.x, packet.y);
                    break;
                }
            }

            float target_speed = 0.0f;
            float target_yaw_cd = 0.0f;

            // consume velocity and convert to target speed and heading
            if (!vel_ignore) {
                const float speed_max = rover.control_mode->get_speed_default();
                // convert vector length into a speed
                target_speed = constrain_float(safe_sqrt(sq(packet.vx) + sq(packet.vy)), -speed_max, speed_max);
                // convert vector direction to target yaw
                target_yaw_cd = degrees(atan2f(packet.vy, packet.vx)) * 100.0f;

                // rotate target yaw if provided in body-frame
                if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                    target_yaw_cd = wrap_180_cd(target_yaw_cd + rover.ahrs.yaw_sensor);
                }
            }

            // consume yaw heading
            if (!yaw_ignore) {
                target_yaw_cd = ToDeg(packet.yaw) * 100.0f;
                // rotate target yaw if provided in body-frame
                if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                    target_yaw_cd = wrap_180_cd(target_yaw_cd + rover.ahrs.yaw_sensor);
                }
            }
            // consume yaw rate
            float target_turn_rate_cds = 0.0f;
            if (!yaw_rate_ignore) {
                target_turn_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
            }

            // handling case when both velocity and either yaw or yaw-rate are provided
            // by default, we consider that the rover will drive forward
            float speed_dir = 1.0f;
            if (!vel_ignore && (!yaw_ignore || !yaw_rate_ignore)) {
                // Note: we are using the x-axis velocity to determine direction even though
                // the frame may have been provided in MAV_FRAME_LOCAL_OFFSET_NED or MAV_FRAME_LOCAL_NED
                if (is_negative(packet.vx)) {
                    speed_dir = -1.0f;
                }
            }

            // set guided mode targets
            if (!pos_ignore) {
                // consume position target
                rover.mode_guided.set_desired_location(target_loc);
            } else if (pos_ignore && !vel_ignore && acc_ignore && yaw_ignore && yaw_rate_ignore) {
                // consume velocity
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, speed_dir * target_speed);
            } else if (pos_ignore && !vel_ignore && acc_ignore && yaw_ignore && !yaw_rate_ignore) {
                // consume velocity and turn rate
                rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, speed_dir * target_speed);
            } else if (pos_ignore && !vel_ignore && acc_ignore && !yaw_ignore && yaw_rate_ignore) {
                // consume velocity and heading
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, speed_dir * target_speed);
            } else if (pos_ignore && vel_ignore && acc_ignore && !yaw_ignore && yaw_rate_ignore) {
                // consume just target heading (probably only skid steering vehicles can do this)
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, 0.0f);
            } else if (pos_ignore && vel_ignore && acc_ignore && yaw_ignore && !yaw_rate_ignore) {
                // consume just turn rate (probably only skid steering vehicles can do this)
                rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, 0.0f);
            }
            break;
        }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
        {
            // decode packet
            mavlink_set_position_target_global_int_t packet;
            mavlink_msg_set_position_target_global_int_decode(msg, &packet);

            // exit if vehicle is not in Guided mode
            if (rover.control_mode != &rover.mode_guided) {
                break;
            }
            // check for supported coordinate frames
            if (packet.coordinate_frame != MAV_FRAME_GLOBAL &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_INT &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT &&
                packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
                break;
            }
            bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
            bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
            bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
            bool yaw_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
            bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

            // prepare target position
            Location target_loc = rover.current_loc;
            if (!pos_ignore) {
                // sanity check location
                if (!check_latlng(packet.lat_int, packet.lon_int)) {
                    // result = MAV_RESULT_FAILED;
                    break;
                }
                target_loc.lat = packet.lat_int;
                target_loc.lng = packet.lon_int;
            }

            float target_speed = 0.0f;
            float target_yaw_cd = 0.0f;

            // consume velocity and convert to target speed and heading
            if (!vel_ignore) {
                const float speed_max = rover.control_mode->get_speed_default();
                // convert vector length into a speed
                target_speed = constrain_float(safe_sqrt(sq(packet.vx) + sq(packet.vy)), -speed_max, speed_max);
                // convert vector direction to target yaw
                target_yaw_cd = degrees(atan2f(packet.vy, packet.vx)) * 100.0f;

                // rotate target yaw if provided in body-frame
                if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                    target_yaw_cd = wrap_180_cd(target_yaw_cd + rover.ahrs.yaw_sensor);
                }
            }

            // consume yaw heading
            if (!yaw_ignore) {
                target_yaw_cd = ToDeg(packet.yaw) * 100.0f;
                // rotate target yaw if provided in body-frame
                if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                    target_yaw_cd = wrap_180_cd(target_yaw_cd + rover.ahrs.yaw_sensor);
                }
            }
            // consume yaw rate
            float target_turn_rate_cds = 0.0f;
            if (!yaw_rate_ignore) {
                target_turn_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
            }

            // handling case when both velocity and either yaw or yaw-rate are provided
            // by default, we consider that the rover will drive forward
            float speed_dir = 1.0f;
            if (!vel_ignore && (!yaw_ignore || !yaw_rate_ignore)) {
                // Note: we are using the x-axis velocity to determine direction even though
                // the frame is provided in MAV_FRAME_GLOBAL_xxx
                if (is_negative(packet.vx)) {
                    speed_dir = -1.0f;
                }
            }

            // set guided mode targets
            if (!pos_ignore) {
                // consume position target
                rover.mode_guided.set_desired_location(target_loc);
            } else if (pos_ignore && !vel_ignore && acc_ignore && yaw_ignore && yaw_rate_ignore) {
                // consume velocity
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, speed_dir * target_speed);
            } else if (pos_ignore && !vel_ignore && acc_ignore && yaw_ignore && !yaw_rate_ignore) {
                // consume velocity and turn rate
                rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, speed_dir * target_speed);
            } else if (pos_ignore && !vel_ignore && acc_ignore && !yaw_ignore && yaw_rate_ignore) {
                // consume velocity
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, speed_dir * target_speed);
            } else if (pos_ignore && vel_ignore && acc_ignore && !yaw_ignore && yaw_rate_ignore) {
                // consume just target heading (probably only skid steering vehicles can do this)
                rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, 0.0f);
            } else if (pos_ignore && vel_ignore && acc_ignore && yaw_ignore && !yaw_rate_ignore) {
                // consume just turn rate(probably only skid steering vehicles can do this)
                rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, 0.0f);
            }
            break;
        }

#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:
        {
            mavlink_hil_state_t packet;
            mavlink_msg_hil_state_decode(msg, &packet);

            // sanity check location
            if (!check_latlng(packet.lat, packet.lon)) {
                break;
            }

            // set gps hil sensor
            Location loc;
            loc.lat = packet.lat;
            loc.lng = packet.lon;
            loc.alt = packet.alt/10;
            Vector3f vel(packet.vx, packet.vy, packet.vz);
            vel *= 0.01f;

            gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                       packet.time_usec/1000,
                       loc, vel, 10, 0);

            // rad/sec
            Vector3f gyros;
            gyros.x = packet.rollspeed;
            gyros.y = packet.pitchspeed;
            gyros.z = packet.yawspeed;

            // m/s/s
            Vector3f accels;
            accels.x = packet.xacc * (GRAVITY_MSS/1000.0f);
            accels.y = packet.yacc * (GRAVITY_MSS/1000.0f);
            accels.z = packet.zacc * (GRAVITY_MSS/1000.0f);

            ins.set_gyro(0, gyros);

            ins.set_accel(0, accels);
            compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
            compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);
            break;
        }
#endif  // HIL_MODE

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
        {
            handle_radio_status(msg, rover.logger, rover.should_log(MASK_LOG_PM));
            break;
        }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        rover.rangefinder.handle_msg(msg);
        rover.g2.proximity.handle_msg(msg);
        break;
    case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
        rover.g2.proximity.handle_msg(msg);
        break;

    default:
        handle_common_message(msg);
        break;
    }  // end switch
}  // end handle mavlink

uint64_t GCS_MAVLINK_Rover::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
            MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
            GCS_MAVLINK::capabilities());
}

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Rover::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs().chan(0).initialised) {
        return;
    }

    // don't allow potentially expensive logging calls:
    logger.EnableWrites(false);

    const uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs().send_message(MSG_HEARTBEAT);
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

AP_AdvancedFailsafe *GCS_MAVLINK_Rover::get_advanced_failsafe() const
{
#if ADVANCED_FAILSAFE == ENABLED
    return &rover.g2.afs;
#else
    return nullptr;
#endif
}

bool GCS_MAVLINK_Rover::set_mode(const uint8_t mode)
{
    Mode *new_mode = rover.mode_from_mode_num((enum Mode::Number)mode);
    if (new_mode == nullptr) {
        return false;
    }
    return rover.set_mode(*new_mode, MODE_REASON_GCS_COMMAND);
}
