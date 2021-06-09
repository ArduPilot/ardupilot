#include "Blimp.h"

#include "GCS_Mavlink.h"

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

MAV_TYPE GCS_Blimp::frame_type() const
{
    return blimp.get_frame_mav_type();
}

MAV_MODE GCS_MAVLINK_Blimp::base_mode() const
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

    // switch (blimp.control_mode) {
    // case Mode::Number::AUTO:
    // case Mode::Number::RTL:
    // case Mode::Number::LOITER:
    // case Mode::Number::AVOID_ADSB:
    // case Mode::Number::FOLLOW:
    // case Mode::Number::GUIDED:
    // case Mode::Number::CIRCLE:
    // case Mode::Number::POSHOLD:
    // case Mode::Number::BRAKE:
    // case Mode::Number::SMART_RTL:
    //     _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    //     // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
    //     // APM does in any mode, as that is defined as "system finds its own goal
    //     // positions", which APM does not currently do
    //     break;
    // default:
    //     break;
    // }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    // we are armed if we are not initialising
    if (blimp.motors != nullptr && blimp.motors->armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return (MAV_MODE)_base_mode;
}

uint32_t GCS_Blimp::custom_mode() const
{
    return (uint32_t)blimp.control_mode;
}

MAV_STATE GCS_MAVLINK_Blimp::vehicle_system_status() const
{
    // set system as critical if any failsafe have triggered
    if (blimp.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    if (blimp.ap.land_complete) {
        return MAV_STATE_STANDBY;
    }

    return MAV_STATE_ACTIVE;
}


void GCS_MAVLINK_Blimp::send_position_target_global_int()
{
    Location target;
    if (!blimp.flightmode->get_wp(target)) {
        return;
    }
    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            POSITION_TARGET_TYPEMASK_FORCE_SET | POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;

    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL, // targets are always global altitude
        TYPE_MASK, // ignore everything except the x/y/z components
        target.lat, // latitude as 1e7
        target.lng, // longitude as 1e7
        target.alt * 0.01f, // altitude is sent as a float
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
}

// void GCS_MAVLINK_Blimp::send_position_target_local_ned()
// {
// #if MODE_GUIDED_ENABLED == ENABLED
//     if (!blimp.flightmode->in_guided_mode()) {
//         return;
//     }

//     const GuidedMode guided_mode = blimp.mode_guided.mode();
//     Vector3f target_pos;
//     Vector3f target_vel;
//     uint16_t type_mask;

//     if (guided_mode == Guided_WP) {
//         type_mask = 0x0FF8; // ignore everything except position
//         target_pos = blimp.wp_nav->get_wp_destination() * 0.01f; // convert to metres
//     } else if (guided_mode == Guided_Velocity) {
//         type_mask = 0x0FC7; // ignore everything except velocity
//         target_vel = blimp.flightmode->get_vel_desired_cms() * 0.01f; // convert to m/s
//     } else {
//         type_mask = 0x0FC0; // ignore everything except position & velocity
//         target_pos = blimp.wp_nav->get_wp_destination() * 0.01f;
//         target_vel = blimp.flightmode->get_vel_desired_cms() * 0.01f;
//     }

//     mavlink_msg_position_target_local_ned_send(
//         chan,
//         AP_HAL::millis(), // time boot ms
//         MAV_FRAME_LOCAL_NED,
//         type_mask,
//         target_pos.x, // x in metres
//         target_pos.y, // y in metres
//         -target_pos.z, // z in metres NED frame
//         target_vel.x, // vx in m/s
//         target_vel.y, // vy in m/s
//         -target_vel.z, // vz in m/s NED frame
//         0.0f, // afx
//         0.0f, // afy
//         0.0f, // afz
//         0.0f, // yaw
//         0.0f); // yaw_rate
// #endif
// }

void GCS_MAVLINK_Blimp::send_nav_controller_output() const
{

}

float GCS_MAVLINK_Blimp::vfr_hud_airspeed() const
{
    Vector3f airspeed_vec_bf;
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // we are running the EKF3 wind estimation code which can give
        // us an airspeed estimate
        return airspeed_vec_bf.length();
    }
    return AP::gps().ground_speed();
}

int16_t GCS_MAVLINK_Blimp::vfr_hud_throttle() const
{
    if (blimp.motors == nullptr) {
        return 0;
    }
    return (int16_t)(blimp.motors->get_throttle() * 100);
}

/*
  send PID tuning message
 */
void GCS_MAVLINK_Blimp::send_pid_tuning()
{
    static const PID_TUNING_AXIS axes[] = {
        PID_TUNING_ROLL,
        PID_TUNING_PITCH,
        PID_TUNING_YAW,
        PID_TUNING_ACCZ
    };
    for (uint8_t i=0; i<ARRAY_SIZE(axes); i++) {
        if (!(blimp.g.gcs_pid_mask & (1<<(axes[i]-1)))) {
            continue;
        }
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
        const AP_Logger::PID_Info *pid_info = nullptr;
        switch (axes[i]) { //TODO This should probably become an acceleration controller?
        // case PID_TUNING_ROLL:
        //     pid_info = &blimp.attitude_control->get_rate_roll_pid().get_pid_info();
        //     break;
        // case PID_TUNING_PITCH:
        //     pid_info = &blimp.attitude_control->get_rate_pitch_pid().get_pid_info();
        //     break;
        // case PID_TUNING_YAW:
        //     pid_info = &blimp.attitude_control->get_rate_yaw_pid().get_pid_info();
        //     break;
        // case PID_TUNING_ACCZ:
        //     pid_info = &blimp.pos_control->get_accel_z_pid().get_pid_info();
        //     break;
        default:
            continue;
        }
        if (pid_info != nullptr) {
            mavlink_msg_pid_tuning_send(chan,
                                        axes[i],
                                        pid_info->target,
                                        pid_info->actual,
                                        pid_info->FF,
                                        pid_info->P,
                                        pid_info->I,
                                        pid_info->D);
        }
    }
}

uint8_t GCS_MAVLINK_Blimp::sysid_my_gcs() const
{
    return blimp.g.sysid_my_gcs;
}
bool GCS_MAVLINK_Blimp::sysid_enforce() const
{
    return blimp.g2.sysid_enforce;
}

uint32_t GCS_MAVLINK_Blimp::telem_delay() const
{
    return (uint32_t)(blimp.g.telem_delay);
}

bool GCS_Blimp::vehicle_initialised() const
{
    return blimp.ap.initialised;
}

// try to send a message, return false if it wasn't sent
bool GCS_MAVLINK_Blimp::try_send_message(enum ap_message id)
{
    switch (id) {

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_SERVO_OUT:
    case MSG_AOA_SSA:
    case MSG_LANDING:
    case MSG_ADSB_VEHICLE:
        // unused
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}


const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, POWER_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, and FENCE_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK_Parameters, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK_Parameters, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Unused
    // @Description: Unused
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK_Parameters, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK_Parameters, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE, SIMSTATE (SITL only), AHRS2 and PID_TUNING to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK_Parameters, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, SYSTEM_TIME, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, BATTERY2, MOUNT_STATUS, OPTICAL_FLOW, GIMBAL_REPORT, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION and RPM to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK_Parameters, streamRates[8],  0),
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
    MSG_CURRENT_WAYPOINT, // MISSION_CURRENT
    MSG_GPS_RAW,
    MSG_GPS_RTK,
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_FENCE_STATUS,
    MSG_POSITION_TARGET_GLOBAL_INT,
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
    MSG_PID_TUNING // Up to four PID_TUNING messages are sent, depending on GCS_PID_MASK parameter
};
static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD
};
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_HWSTATUS,
    MSG_SYSTEM_TIME,
    MSG_WIND,
    MSG_RANGEFINDER,
    MSG_DISTANCE_SENSOR,
    MSG_BATTERY2,
    MSG_BATTERY_STATUS,
    MSG_MOUNT_STATUS,
    MSG_OPTICAL_FLOW,
    MSG_GIMBAL_REPORT,
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
    MSG_RPM,
    MSG_ESC_TELEMETRY,
    MSG_GENERATOR_STATUS,
};
static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};
static const ap_message STREAM_ADSB_msgs[] = {
    MSG_ADSB_VEHICLE
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_ADSB),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

bool GCS_MAVLINK_Blimp::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    // #if MODE_AUTO_ENABLED == ENABLED
    //     // return blimp.mode_auto.do_guided(cmd);
    // #else
    return false;
    // #endif
}

void GCS_MAVLINK_Blimp::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    // add home alt if needed
    if (cmd.content.location.relative_alt) {
        cmd.content.location.alt += blimp.ahrs.get_home().alt;
    }

    // To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
}

void GCS_MAVLINK_Blimp::packetReceived(const mavlink_status_t &status,
                                       const mavlink_message_t &msg)
{
    GCS_MAVLINK::packetReceived(status, msg);
}

bool GCS_MAVLINK_Blimp::params_ready() const
{
    if (AP_BoardConfig::in_config_error()) {
        // we may never have parameters "initialised" in this case
        return true;
    }
    // if we have not yet initialised (including allocating the motors
    // object) we drop this request. That prevents the GCS from getting
    // a confusing parameter count during bootup
    return blimp.ap.initialised_params;
}

void GCS_MAVLINK_Blimp::send_banner()
{
    GCS_MAVLINK::send_banner();
    send_text(MAV_SEVERITY_INFO, "Frame: %s", blimp.get_frame_string());
}

MAV_RESULT GCS_MAVLINK_Blimp::_handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    return GCS_MAVLINK::_handle_command_preflight_calibration(packet);
}


MAV_RESULT GCS_MAVLINK_Blimp::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    // blimp.flightmode->auto_yaw.set_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Blimp::handle_preflight_reboot(const mavlink_command_long_t &packet)
{
    // call parent
    return GCS_MAVLINK::handle_preflight_reboot(packet);
}

bool GCS_MAVLINK_Blimp::set_home_to_current_location(bool _lock)
{
    return blimp.set_home_to_current_location(_lock);
}
bool GCS_MAVLINK_Blimp::set_home(const Location& loc, bool _lock)
{
    return blimp.set_home(loc, _lock);
}

MAV_RESULT GCS_MAVLINK_Blimp::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!blimp.flightmode->in_guided_mode() && !change_modes) {
        return MAV_RESULT_DENIED;
    }

    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location request_location {};
    request_location.lat = packet.x;
    request_location.lng = packet.y;

    if (fabsf(packet.z) > LOCATION_ALT_MAX_M) {
        return MAV_RESULT_DENIED;
    }

    Location::AltFrame frame;
    if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.frame, frame)) {
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }
    request_location.set_alt_cm((int32_t)(packet.z * 100.0f), frame);

    if (request_location.sanitize(blimp.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Blimp::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    switch (packet.command) {
    case MAV_CMD_DO_FOLLOW:
        return MAV_RESULT_UNSUPPORTED;

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet);
    }
}

MAV_RESULT GCS_MAVLINK_Blimp::handle_command_mount(const mavlink_command_long_t &packet)
{
    // if the mount doesn't do pan control then yaw the entire vehicle instead:
    switch (packet.command) {
    default:
        break;
    }
    return GCS_MAVLINK::handle_command_mount(packet);
}

MAV_RESULT GCS_MAVLINK_Blimp::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    switch (packet.command) {

    case MAV_CMD_NAV_TAKEOFF: {
        // param3 : horizontal navigation by pilot acceptable
        // param4 : yaw angle   (not supported)
        // param5 : latitude    (not supported)
        // param6 : longitude   (not supported)
        // param7 : altitude [metres]

        // float takeoff_alt = packet.param7 * 100;      // Convert m to cm

        // if (!blimp.flightmode->do_user_takeoff(takeoff_alt, is_zero(packet.param3))) {
        //     return MAV_RESULT_FAILED;
        //MIR Do I need this?
        // }
        return MAV_RESULT_ACCEPTED;
    }

    // #if MODE_AUTO_ENABLED == ENABLED
    //     case MAV_CMD_DO_LAND_START:
    //         if (blimp.mode_auto.mission.jump_to_landing_sequence() && blimp.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) {
    //             return MAV_RESULT_ACCEPTED;
    //         }
    //         return MAV_RESULT_FAILED;
    // #endif

    // case MAV_CMD_NAV_LOITER_UNLIM:
    //     if (!blimp.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
    //         return MAV_RESULT_FAILED;
    //     }
    //     return MAV_RESULT_ACCEPTED;

    // case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    //     if (!blimp.set_mode(Mode::Number::RTL, ModeReason::GCS_COMMAND)) {
    //         return MAV_RESULT_FAILED;
    //     }
    //     return MAV_RESULT_ACCEPTED;


    case MAV_CMD_CONDITION_YAW:
        // param1 : target angle [0-360]
        // param2 : speed during change [deg per second]
        // param3 : direction (-1:ccw, +1:cw)
        // param4 : relative offset (1) or absolute angle (0)
        if ((packet.param1 >= 0.0f)   &&
            (packet.param1 <= 360.0f) &&
            (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
            // blimp.flightmode->auto_yaw.set_fixed_yaw(
            // packet.param1,
            // packet.param2,
            // (int8_t)packet.param3,
            // is_positive(packet.param4));
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    default:
        return GCS_MAVLINK::handle_command_long_packet(packet);
    }
}

void GCS_MAVLINK_Blimp::handleMessage(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    // #if MODE_GUIDED_ENABLED == ENABLED
    //     case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:   // MAV ID: 82
    //     {
    //         // decode packet
    //         mavlink_set_attitude_target_t packet;
    //         mavlink_msg_set_attitude_target_decode(&msg, &packet);

    //         // exit if vehicle is not in Guided mode or Auto-Guided mode
    //         if (!blimp.flightmode->in_guided_mode()) {
    //             break;
    //         }

    //         // ensure type_mask specifies to use attitude and thrust
    //         if ((packet.type_mask & ((1<<7)|(1<<6))) != 0) {
    //             break;
    //         }

    //         // check if the message's thrust field should be interpreted as a climb rate or as thrust
    //         const bool use_thrust = blimp.g2.dev_options.get() & DevOptionSetAttitudeTarget_ThrustAsThrust;

    //         float climb_rate_or_thrust;
    //         if (use_thrust) {
    //             // interpret thrust as thrust
    //             climb_rate_or_thrust = constrain_float(packet.thrust, -1.0f, 1.0f);
    //         } else {
    //             // convert thrust to climb rate
    //             packet.thrust = constrain_float(packet.thrust, 0.0f, 1.0f);
    //             if (is_equal(packet.thrust, 0.5f)) {
    //                 climb_rate_or_thrust = 0.0f;
    //             } else if (packet.thrust > 0.5f) {
    //                 // climb at up to WPNAV_SPEED_UP
    //                 climb_rate_or_thrust = (packet.thrust - 0.5f) * 2.0f * blimp.wp_nav->get_default_speed_up();
    //             } else {
    //                 // descend at up to WPNAV_SPEED_DN
    //                 climb_rate_or_thrust = (0.5f - packet.thrust) * 2.0f * -fabsf(blimp.wp_nav->get_default_speed_down());
    //             }
    //         }

    //         // if the body_yaw_rate field is ignored, use the commanded yaw position
    //         // otherwise use the commanded yaw rate
    //         bool use_yaw_rate = false;
    //         if ((packet.type_mask & (1<<2)) == 0) {
    //             use_yaw_rate = true;
    //         }

    //         blimp.mode_guided.set_angle(Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]),
    //                 climb_rate_or_thrust, use_yaw_rate, packet.body_yaw_rate, use_thrust);

    //         break;
    //     }

    //     case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
    //     {
    //         // decode packet
    //         mavlink_set_position_target_local_ned_t packet;
    //         mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

    //         // exit if vehicle is not in Guided mode or Auto-Guided mode
    //         if (!blimp.flightmode->in_guided_mode()) {
    //             break;
    //         }

    //         // check for supported coordinate frames
    //         if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
    //             packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
    //             packet.coordinate_frame != MAV_FRAME_BODY_NED &&
    //             packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
    //             break;
    //         }

    //         bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    //         bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
    //         bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
    //         bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
    //         bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

    //         // exit immediately if acceleration provided
    //         if (!acc_ignore) {
    //             break;
    //         }

    //         // prepare position
    //         Vector3f pos_vector;
    //         if (!pos_ignore) {
    //             // convert to cm
    //             pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
    //             // rotate to body-frame if necessary
    //             if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
    //                 packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
    //                 blimp.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
    //             }
    //             // add body offset if necessary
    //             if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
    //                 packet.coordinate_frame == MAV_FRAME_BODY_NED ||
    //                 packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
    //                 pos_vector += blimp.inertial_nav.get_position();
    //             }
    //         }

    //         // prepare velocity
    //         Vector3f vel_vector;
    //         if (!vel_ignore) {
    //             // convert to cm
    //             vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
    //             // rotate to body-frame if necessary
    //             if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
    //                 blimp.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
    //             }
    //         }

    //         // prepare yaw
    //         float yaw_cd = 0.0f;
    //         bool yaw_relative = false;
    //         float yaw_rate_cds = 0.0f;
    //         if (!yaw_ignore) {
    //             yaw_cd = ToDeg(packet.yaw) * 100.0f;
    //             yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
    //         }
    //         if (!yaw_rate_ignore) {
    //             yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
    //         }

    //         // send request
    //         if (!pos_ignore && !vel_ignore) {
    //             blimp.mode_guided.set_destination_posvel(pos_vector, vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
    //         } else if (pos_ignore && !vel_ignore) {
    //             blimp.mode_guided.set_velocity(vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
    //         } else if (!pos_ignore && vel_ignore) {
    //             blimp.mode_guided.set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
    //         }

    //         break;
    //     }

    //     case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
    //     {
    //         // decode packet
    //         mavlink_set_position_target_global_int_t packet;
    //         mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

    //         // exit if vehicle is not in Guided mode or Auto-Guided mode
    //         if (!blimp.flightmode->in_guided_mode()) {
    //             break;
    //         }

    //         bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    //         bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
    //         bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
    //         bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
    //         bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

    //         // exit immediately if acceleration provided
    //         if (!acc_ignore) {
    //             break;
    //         }

    //         // extract location from message
    //         Location loc;
    //         if (!pos_ignore) {
    //             // sanity check location
    //             if (!check_latlng(packet.lat_int, packet.lon_int)) {
    //                 break;
    //             }
    //             Location::AltFrame frame;
    //             if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.coordinate_frame, frame)) {
    //                 // unknown coordinate frame
    //                 break;
    //             }
    //             loc = {packet.lat_int, packet.lon_int, int32_t(packet.alt*100), frame};
    //         }

    //         // prepare yaw
    //         float yaw_cd = 0.0f;
    //         bool yaw_relative = false;
    //         float yaw_rate_cds = 0.0f;
    //         if (!yaw_ignore) {
    //             yaw_cd = ToDeg(packet.yaw) * 100.0f;
    //             yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
    //         }
    //         if (!yaw_rate_ignore) {
    //             yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
    //         }

    //         // send targets to the appropriate guided mode controller
    //         if (!pos_ignore && !vel_ignore) {
    //             // convert Location to vector from ekf origin for posvel controller
    //             if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
    //                 // posvel controller does not support alt-above-terrain
    //                 break;
    //             }
    //             Vector3f pos_neu_cm;
    //             if (!loc.get_vector_from_origin_NEU(pos_neu_cm)) {
    //                 break;
    //             }
    //             blimp.mode_guided.set_destination_posvel(pos_neu_cm, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
    //         } else if (pos_ignore && !vel_ignore) {
    //             blimp.mode_guided.set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
    //         } else if (!pos_ignore && vel_ignore) {
    //             blimp.mode_guided.set_destination(loc, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
    //         }

    //         break;
    //     }
    // #endif

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS: {     // MAV ID: 109
        handle_radio_status(msg, blimp.should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION: {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(&msg, &packet);
        if ((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            if (!blimp.set_home_to_current_location(true)) {
                // silently ignored
            }
        } else {
            Location new_home_loc;
            new_home_loc.lat = packet.latitude;
            new_home_loc.lng = packet.longitude;
            new_home_loc.alt = packet.altitude / 10;
            if (!blimp.set_home(new_home_loc, true)) {
                // silently ignored
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
        break;

    default:
        handle_common_message(msg);
        break;
    }     // end switch
} // end handle mavlink


MAV_RESULT GCS_MAVLINK_Blimp::handle_flight_termination(const mavlink_command_long_t &packet)
{
    MAV_RESULT result = MAV_RESULT_FAILED;
    if (packet.param1 > 0.5f) {
        blimp.arming.disarm(AP_Arming::Method::TERMINATION);
        result = MAV_RESULT_ACCEPTED;
    }
    return result;
}

float GCS_MAVLINK_Blimp::vfr_hud_alt() const
{
    if (blimp.g2.dev_options.get() & DevOptionVFR_HUDRelativeAlt) {
        // compatibility option for older mavlink-aware devices that
        // assume Blimp returns a relative altitude in VFR_HUD.alt
        return blimp.current_loc.alt * 0.01f;
    }
    return GCS_MAVLINK::vfr_hud_alt();
}

uint64_t GCS_MAVLINK_Blimp::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
            GCS_MAVLINK::capabilities());
}

MAV_LANDED_STATE GCS_MAVLINK_Blimp::landed_state() const
{
    if (blimp.ap.land_complete) {
        return MAV_LANDED_STATE_ON_GROUND;
    }
    if (blimp.flightmode->is_landing()) {
        return MAV_LANDED_STATE_LANDING;
    }
    // if (blimp.flightmode->is_taking_off()) {
    //     return MAV_LANDED_STATE_TAKEOFF;
    // }
    return MAV_LANDED_STATE_IN_AIR;
}

void GCS_MAVLINK_Blimp::send_wind() const
{
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a
        // valid wind estimate on blimps
        return;
    }
    const Vector3f wind = AP::ahrs().wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)),
        wind.length(),
        wind.z);
}
