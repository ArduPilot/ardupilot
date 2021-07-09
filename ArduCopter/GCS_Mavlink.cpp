#include "Copter.h"

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

MAV_TYPE GCS_Copter::frame_type() const
{
    if (copter.motors == nullptr) {
        return MAV_TYPE_GENERIC;
    }
    return copter.motors->get_frame_mav_type();
}

MAV_MODE GCS_MAVLINK_Copter::base_mode() const
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
    switch (copter.flightmode->mode_number()) {
    case Mode::Number::AUTO:
    case Mode::Number::RTL:
    case Mode::Number::LOITER:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::FOLLOW:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::POSHOLD:
    case Mode::Number::BRAKE:
    case Mode::Number::SMART_RTL:
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

    // we are armed if we are not initialising
    if (copter.motors != nullptr && copter.motors->armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return (MAV_MODE)_base_mode;
}

uint32_t GCS_Copter::custom_mode() const
{
    return (uint32_t)copter.flightmode->mode_number();
}

MAV_STATE GCS_MAVLINK_Copter::vehicle_system_status() const
{
    // set system as critical if any failsafe have triggered
    if (copter.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    if (copter.ap.land_complete) {
        return MAV_STATE_STANDBY;
    }

    return MAV_STATE_ACTIVE;
}


void GCS_MAVLINK_Copter::send_position_target_global_int()
{
    Location target;
    if (!copter.flightmode->get_wp(target)) {
        return;
    }

    // convert altitude frame to AMSL (this may use the terrain database)
    if (!target.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
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

void GCS_MAVLINK_Copter::send_position_target_local_ned()
{
#if MODE_GUIDED_ENABLED == ENABLED
    if (!copter.flightmode->in_guided_mode()) {
        return;
    }

    const ModeGuided::SubMode guided_mode = copter.mode_guided.submode();
    Vector3f target_pos;
    Vector3f target_vel;
    Vector3f target_accel;
    uint16_t type_mask = 0;

    switch (guided_mode) {
    case ModeGuided::SubMode::Angle:
        // we don't have a local target when in angle mode
        return;
    case ModeGuided::SubMode::TakeOff:
    case ModeGuided::SubMode::WP:
        type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_FORCE_SET | POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position
        target_pos = copter.mode_guided.get_target_pos().tofloat() * 0.01; // convert to metres
        break;
    case ModeGuided::SubMode::PosVelAccel:
        type_mask = POSITION_TARGET_TYPEMASK_FORCE_SET | POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position, velocity & acceleration
        target_pos = copter.mode_guided.get_target_pos().tofloat() * 0.01; // convert to metres
        target_vel = copter.mode_guided.get_target_vel() * 0.01f; // convert to metres/s
        target_accel = copter.mode_guided.get_target_accel() * 0.01f; // convert to metres/s/s
        break;
    case ModeGuided::SubMode::VelAccel:
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_FORCE_SET | POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except velocity & acceleration
        target_vel = copter.mode_guided.get_target_vel() * 0.01f; // convert to metres/s
        target_accel = copter.mode_guided.get_target_accel() * 0.01f; // convert to metres/s/s
        break;
    case ModeGuided::SubMode::Accel:
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_FORCE_SET | POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except velocity & acceleration
        target_accel = copter.mode_guided.get_target_accel() * 0.01f; // convert to metres/s/s
        break;
    }

    mavlink_msg_position_target_local_ned_send(
        chan,
        AP_HAL::millis(), // time boot ms
        MAV_FRAME_LOCAL_NED, 
        type_mask,
        target_pos.x,   // x in metres
        target_pos.y,   // y in metres
        -target_pos.z,  // z in metres NED frame
        target_vel.x,   // vx in m/s
        target_vel.y,   // vy in m/s
        -target_vel.z,  // vz in m/s NED frame
        target_accel.x, // afx in m/s/s
        target_accel.y, // afy in m/s/s
        -target_accel.z,// afz in m/s/s NED frame
        0.0f, // yaw
        0.0f); // yaw_rate
#endif
}

void GCS_MAVLINK_Copter::send_nav_controller_output() const
{
    if (!copter.ap.initialised) {
        return;
    }
    const Vector3f &targets = copter.attitude_control->get_att_target_euler_cd();
    const Mode *flightmode = copter.flightmode;
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        flightmode->wp_bearing() * 1.0e-2f,
        MIN(flightmode->wp_distance() * 1.0e-2f, UINT16_MAX),
        copter.pos_control->get_pos_error_z_cm() * 1.0e-2f,
        0,
        flightmode->crosstrack_error() * 1.0e-2f);
}

float GCS_MAVLINK_Copter::vfr_hud_airspeed() const
{
    Vector3f airspeed_vec_bf;
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // we are running the EKF3 wind estimation code which can give
        // us an airspeed estimate
        return airspeed_vec_bf.length();
    }
    return AP::gps().ground_speed();
}

int16_t GCS_MAVLINK_Copter::vfr_hud_throttle() const
{
    if (copter.motors == nullptr) {
        return 0;
    }
    return (int16_t)(copter.motors->get_throttle() * 100);
}

/*
  send PID tuning message
 */
void GCS_MAVLINK_Copter::send_pid_tuning()
{
    static const PID_TUNING_AXIS axes[] = {
        PID_TUNING_ROLL,
        PID_TUNING_PITCH,
        PID_TUNING_YAW,
        PID_TUNING_ACCZ
    };
    for (uint8_t i=0; i<ARRAY_SIZE(axes); i++) {
        if (!(copter.g.gcs_pid_mask & (1<<(axes[i]-1)))) {
            continue;
        }
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
        const AP_Logger::PID_Info *pid_info = nullptr;
        switch (axes[i]) {
        case PID_TUNING_ROLL:
            pid_info = &copter.attitude_control->get_rate_roll_pid().get_pid_info();
            break;
        case PID_TUNING_PITCH:
            pid_info = &copter.attitude_control->get_rate_pitch_pid().get_pid_info();
            break;
        case PID_TUNING_YAW:
            pid_info = &copter.attitude_control->get_rate_yaw_pid().get_pid_info();
            break;
        case PID_TUNING_ACCZ:
            pid_info = &copter.pos_control->get_accel_z_pid().get_pid_info();
            break;
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

// send winch status message
void GCS_MAVLINK_Copter::send_winch_status() const
{
#if WINCH_ENABLED == ENABLED
    AP_Winch *winch = AP::winch();
    if (winch == nullptr) {
        return;
    }
    winch->send_status(*this);
#endif
}

uint8_t GCS_MAVLINK_Copter::sysid_my_gcs() const
{
    return copter.g.sysid_my_gcs;
}
bool GCS_MAVLINK_Copter::sysid_enforce() const
{
    return copter.g2.sysid_enforce;
}

uint32_t GCS_MAVLINK_Copter::telem_delay() const
{
    return (uint32_t)(copter.g.telem_delay);
}

bool GCS_Copter::vehicle_initialised() const {
    return copter.ap.initialised;
}

// try to send a message, return false if it wasn't sent
bool GCS_MAVLINK_Copter::try_send_message(enum ap_message id)
{
    switch(id) {

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        copter.terrain.send_request(chan);
#endif
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_SERVO_OUT:
    case MSG_AOA_SSA:
    case MSG_LANDING:
        // unused
        break;

    case MSG_ADSB_VEHICLE: {
#if HAL_ADSB_ENABLED
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        copter.adsb.send_adsb_vehicle(chan);
#endif
#if AC_OAPATHPLANNER_ENABLED == ENABLED
        AP_OADatabase *oadb = AP_OADatabase::get_singleton();
        if (oadb != nullptr) {
            CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
            uint16_t interval_ms = 0;
            if (get_ap_message_interval(id, interval_ms)) {
                oadb->send_adsb_vehicle(chan, interval_ms);
            }
        }
#endif
        break;
    }

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
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, POWER_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, and FENCE_STATUS to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK_Parameters, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK_Parameters, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Unused
    // @Description: Unused
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK_Parameters, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK_Parameters, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE, SIMSTATE (SITL only), AHRS2 and PID_TUNING to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK_Parameters, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, SYSTEM_TIME, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, BATTERY2, MOUNT_STATUS, OPTICAL_FLOW, GIMBAL_REPORT, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION and RPM to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK_Parameters, streamRates[8],  0),

    // @Param: ADSB
    // @DisplayName: ADSB stream rate to ground station
    // @Description: ADSB stream rate to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK_Parameters, streamRates[9],  0),
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
    MSG_RPM,
    MSG_ESC_TELEMETRY,
    MSG_GENERATOR_STATUS,
    MSG_WINCH_STATUS,
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

bool GCS_MAVLINK_Copter::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
#if MODE_AUTO_ENABLED == ENABLED
    return copter.mode_auto.do_guided(cmd);
#else
    return false;
#endif
}

void GCS_MAVLINK_Copter::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    // add home alt if needed
    if (cmd.content.location.relative_alt) {
        cmd.content.location.alt += copter.ahrs.get_home().alt;
    }

    // To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
}

void GCS_MAVLINK_Copter::packetReceived(const mavlink_status_t &status,
                                        const mavlink_message_t &msg)
{
#if HAL_ADSB_ENABLED
    if (copter.g2.dev_options.get() & DevOptionADSBMAVLink) {
        // optional handling of GLOBAL_POSITION_INT as a MAVLink based avoidance source
        copter.avoidance_adsb.handle_msg(msg);
    }
#endif
#if MODE_FOLLOW_ENABLED == ENABLED
    // pass message to follow library
    copter.g2.follow.handle_msg(msg);
#endif
    GCS_MAVLINK::packetReceived(status, msg);
}

bool GCS_MAVLINK_Copter::params_ready() const
{
    if (AP_BoardConfig::in_config_error()) {
        // we may never have parameters "initialised" in this case
        return true;
    }
    // if we have not yet initialised (including allocating the motors
    // object) we drop this request. That prevents the GCS from getting
    // a confusing parameter count during bootup
    return copter.ap.initialised_params;
}

void GCS_MAVLINK_Copter::send_banner()
{
    GCS_MAVLINK::send_banner();
    if (copter.motors == nullptr) {
        send_text(MAV_SEVERITY_INFO, "motors not allocated");
        return;
    }
    char frame_and_type_string[30];
    copter.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    send_text(MAV_SEVERITY_INFO, "%s", frame_and_type_string);
}

void GCS_MAVLINK_Copter::handle_command_ack(const mavlink_message_t &msg)
{
    copter.command_ack_counter++;
    GCS_MAVLINK::handle_command_ack(msg);
}

/*
  handle a LANDING_TARGET command. The timestamp has been jitter corrected
*/
void GCS_MAVLINK_Copter::handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
#if PRECISION_LANDING == ENABLED
    copter.precland.handle_msg(packet, timestamp_ms);
#endif
}

MAV_RESULT GCS_MAVLINK_Copter::_handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    if (is_equal(packet.param6,1.0f)) {
        // compassmot calibration
        return copter.mavlink_compassmot(*this);
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet);
}


MAV_RESULT GCS_MAVLINK_Copter::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    copter.flightmode->auto_yaw.set_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Copter::handle_preflight_reboot(const mavlink_command_long_t &packet)
{
    // reject reboot if user has also specified they want the "Auto" ESC calibration on next reboot
    if (copter.g.esc_calibrate == (uint8_t)Copter::ESCCalibrationModes::ESCCAL_AUTO) {
        send_text(MAV_SEVERITY_CRITICAL, "Reboot rejected, ESC cal on reboot");
        return MAV_RESULT_FAILED;
    }

    // call parent
    return GCS_MAVLINK::handle_preflight_reboot(packet);
}

bool GCS_MAVLINK_Copter::set_home_to_current_location(bool _lock) {
    return copter.set_home_to_current_location(_lock);
}
bool GCS_MAVLINK_Copter::set_home(const Location& loc, bool _lock) {
    return copter.set_home(loc, _lock);
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!copter.flightmode->in_guided_mode() && !change_modes) {
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

    if (request_location.sanitize(copter.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    // we need to do this first, as we don't want to change the flight mode unless we can also set the target
    if (!copter.mode_guided.set_destination(request_location, false, 0, false, 0)) {
        return MAV_RESULT_FAILED;
    }

    if (!copter.flightmode->in_guided_mode()) {
        if (!copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        // the position won't have been loaded if we had to change the flight mode, so load it again
        if (!copter.mode_guided.set_destination(request_location, false, 0, false, 0)) {
            return MAV_RESULT_FAILED;
        }
    }

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    switch(packet.command) {
    case MAV_CMD_DO_FOLLOW:
#if MODE_FOLLOW_ENABLED == ENABLED
        // param1: sysid of target to follow
        if ((packet.param1 > 0) && (packet.param1 <= 255)) {
            copter.g2.follow.set_target_sysid((uint8_t)packet.param1);
            return MAV_RESULT_ACCEPTED;
        }
#endif
        return MAV_RESULT_UNSUPPORTED;

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet);
    }
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_mount(const mavlink_command_long_t &packet)
{
    switch (packet.command) {
#if HAL_MOUNT_ENABLED
    case MAV_CMD_DO_MOUNT_CONTROL:
        // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
        if ((copter.camera_mount.get_mount_type() != copter.camera_mount.MountType::Mount_Type_None) &&
            !copter.camera_mount.has_pan_control()) {
            copter.flightmode->auto_yaw.set_yaw_angle_rate(
                (float)packet.param3 * 0.01f,
                0.0f);
        }
        break;
#endif
    default:
        break;
    }
    return GCS_MAVLINK::handle_command_mount(packet);
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    switch(packet.command) {

    case MAV_CMD_NAV_TAKEOFF: {
        // param3 : horizontal navigation by pilot acceptable
        // param4 : yaw angle   (not supported)
        // param5 : latitude    (not supported)
        // param6 : longitude   (not supported)
        // param7 : altitude [metres]

        float takeoff_alt = packet.param7 * 100;      // Convert m to cm

        if (!copter.flightmode->do_user_takeoff(takeoff_alt, is_zero(packet.param3))) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
    }

#if MODE_AUTO_ENABLED == ENABLED
    case MAV_CMD_DO_LAND_START:
        if (copter.mode_auto.mission.jump_to_landing_sequence() && copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

    case MAV_CMD_NAV_LOITER_UNLIM:
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        if (!copter.set_mode(Mode::Number::RTL, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_LAND:
        if (!copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

#if MODE_FOLLOW_ENABLED == ENABLED
    case MAV_CMD_DO_FOLLOW:
        // param1: sysid of target to follow
        if ((packet.param1 > 0) && (packet.param1 <= 255)) {
            copter.g2.follow.set_target_sysid((uint8_t)packet.param1);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

    case MAV_CMD_CONDITION_YAW:
        // param1 : target angle [0-360]
        // param2 : speed during change [deg per second]
        // param3 : direction (-1:ccw, +1:cw)
        // param4 : relative offset (1) or absolute angle (0)
        if ((packet.param1 >= 0.0f)   &&
            (packet.param1 <= 360.0f) &&
            (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
            copter.flightmode->auto_yaw.set_fixed_yaw(
                packet.param1,
                packet.param2,
                (int8_t)packet.param3,
                is_positive(packet.param4));
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_CHANGE_SPEED:
        // param1 : Speed type (0 or 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
        // param2 : new speed in m/s
        // param3 : unused
        // param4 : unused
        if (packet.param2 > 0.0f) {
            if (packet.param1 > 2.9f) { // 3 = speed down
                copter.wp_nav->set_speed_down(packet.param2 * 100.0f);
            } else if (packet.param1 > 1.9f) { // 2 = speed up
                copter.wp_nav->set_speed_up(packet.param2 * 100.0f);
            } else {
                copter.wp_nav->set_speed_xy(packet.param2 * 100.0f);
            }
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

#if MODE_AUTO_ENABLED == ENABLED
    case MAV_CMD_MISSION_START:
        if (copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) {
            copter.set_auto_armed(true);
            if (copter.mode_auto.mission.state() != AP_Mission::MISSION_RUNNING) {
                copter.mode_auto.mission.start_or_resume();
            }
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // configure or release parachute
        switch ((uint16_t)packet.param1) {
        case PARACHUTE_DISABLE:
            copter.parachute.enabled(false);
            AP::logger().Write_Event(LogEvent::PARACHUTE_DISABLED);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_ENABLE:
            copter.parachute.enabled(true);
            AP::logger().Write_Event(LogEvent::PARACHUTE_ENABLED);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_RELEASE:
            // treat as a manual release which performs some additional check of altitude
            copter.parachute_manual_release();
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

    case MAV_CMD_DO_MOTOR_TEST:
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        // param5 : num_motors (in sequence)
        // param6 : motor test order
        return copter.mavlink_motor_test_start(*this,
                                               (uint8_t)packet.param1,
                                               (uint8_t)packet.param2,
                                               packet.param3,
                                               packet.param4,
                                               (uint8_t)packet.param5);

#if WINCH_ENABLED == ENABLED
    case MAV_CMD_DO_WINCH:
        // param1 : winch number (ignored)
        // param2 : action (0=relax, 1=relative length control, 2=rate control). See WINCH_ACTIONS enum.
        if (!copter.g2.winch.enabled()) {
            return MAV_RESULT_FAILED;
        }
        switch ((uint8_t)packet.param2) {
        case WINCH_RELAXED:
            copter.g2.winch.relax();
            return MAV_RESULT_ACCEPTED;
        case WINCH_RELATIVE_LENGTH_CONTROL: {
            copter.g2.winch.release_length(packet.param3);
            return MAV_RESULT_ACCEPTED;
        }
        case WINCH_RATE_CONTROL:
            copter.g2.winch.set_desired_rate(packet.param4);
            return MAV_RESULT_ACCEPTED;
        default:
            break;
        }
        return MAV_RESULT_FAILED;
#endif

#if LANDING_GEAR_ENABLED == ENABLED
        case MAV_CMD_AIRFRAME_CONFIGURATION: {
            // Param 1: Select which gear, not used in ArduPilot
            // Param 2: 0 = Deploy, 1 = Retract
            // For safety, anything other than 1 will deploy
            switch ((uint8_t)packet.param2) {
                case 1:
                    copter.landinggear.set_position(AP_LandingGear::LandingGear_Retract);
                    return MAV_RESULT_ACCEPTED;
                default:
                    copter.landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
                    return MAV_RESULT_ACCEPTED;
            }
            return MAV_RESULT_FAILED;
        }
#endif

        /* Solo user presses Fly button */
    case MAV_CMD_SOLO_BTN_FLY_CLICK: {
        if (copter.failsafe.radio) {
            return MAV_RESULT_ACCEPTED;
        }

        // set mode to Loiter or fall back to AltHold
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
        }
        return MAV_RESULT_ACCEPTED;
    }

        /* Solo user holds down Fly button for a couple of seconds */
    case MAV_CMD_SOLO_BTN_FLY_HOLD: {
        if (copter.failsafe.radio) {
            return MAV_RESULT_ACCEPTED;
        }

        if (!copter.motors->armed()) {
            // if disarmed, arm motors
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        } else if (copter.ap.land_complete) {
            // if armed and landed, takeoff
            if (copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
                copter.flightmode->do_user_takeoff(packet.param1*100, true);
            }
        } else {
            // if flying, land
            copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
        }
        return MAV_RESULT_ACCEPTED;
    }

        /* Solo user presses pause button */
    case MAV_CMD_SOLO_BTN_PAUSE_CLICK: {
        if (copter.failsafe.radio) {
            return MAV_RESULT_ACCEPTED;
        }

        if (copter.motors->armed()) {
            if (copter.ap.land_complete) {
                // if landed, disarm motors
                copter.arming.disarm(AP_Arming::Method::SOLOPAUSEWHENLANDED);
            } else {
                // assume that shots modes are all done in guided.
                // NOTE: this may need to change if we add a non-guided shot mode
                bool shot_mode = (!is_zero(packet.param1) && (copter.flightmode->mode_number() == Mode::Number::GUIDED || copter.flightmode->mode_number() == Mode::Number::GUIDED_NOGPS));

                if (!shot_mode) {
#if MODE_BRAKE_ENABLED == ENABLED
                    if (copter.set_mode(Mode::Number::BRAKE, ModeReason::GCS_COMMAND)) {
                        copter.mode_brake.timeout_to_loiter_ms(2500);
                    } else {
                        copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
                    }
#else
                    copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
#endif
                } else {
                    // SoloLink is expected to handle pause in shots
                }
            }
        }
        return MAV_RESULT_ACCEPTED;
    }

    default:
        return GCS_MAVLINK::handle_command_long_packet(packet);
    }
}

void GCS_MAVLINK_Copter::handle_mount_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
#if HAL_MOUNT_ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
        if ((copter.camera_mount.get_mount_type() != copter.camera_mount.MountType::Mount_Type_None) &&
            !copter.camera_mount.has_pan_control()) {
            copter.flightmode->auto_yaw.set_yaw_angle_rate(
                mavlink_msg_mount_control_get_input_c(&msg) * 0.01f,
                0.0f);

            break;
        }
#endif
    }
    GCS_MAVLINK::handle_mount_message(msg);
}

void GCS_MAVLINK_Copter::handleMessage(const mavlink_message_t &msg)
{
    // for mavlink SET_POSITION_TARGET messages
    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE =
        POSITION_TARGET_TYPEMASK_X_IGNORE |
        POSITION_TARGET_TYPEMASK_Y_IGNORE |
        POSITION_TARGET_TYPEMASK_Z_IGNORE;

    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE =
        POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE;

    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE =
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE;

    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE =
        POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE =
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    switch (msg.msgid) {

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        if (msg.sysid != copter.g.sysid_my_gcs) {
            break; // only accept control from our gcs
        }

        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(&msg, &packet);

        if (packet.target != copter.g.sysid_this_mav) {
            break; // only accept control aimed at us
        }

        if (packet.z < 0) { // Copter doesn't do negative thrust
            break;
        }

        uint32_t tnow = AP_HAL::millis();

        manual_override(copter.channel_roll, packet.y, 1000, 2000, tnow);
        manual_override(copter.channel_pitch, packet.x, 1000, 2000, tnow, true);
        manual_override(copter.channel_throttle, packet.z, 0, 1000, tnow);
        manual_override(copter.channel_yaw, packet.r, 1000, 2000, tnow);

        // a manual control message is considered to be a 'heartbeat'
        // from the ground station for failsafe purposes
        gcs().sysid_myggcs_seen(tnow);
        break;
    }

#if MODE_GUIDED_ENABLED == ENABLED
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:   // MAV ID: 82
    {
        // decode packet
        mavlink_set_attitude_target_t packet;
        mavlink_msg_set_attitude_target_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode()) {
            break;
        }

        // ensure type_mask specifies to use attitude and thrust
        if ((packet.type_mask & ((1<<7)|(1<<6))) != 0) {
            break;
        }

        // check if the message's thrust field should be interpreted as a climb rate or as thrust
        const bool use_thrust = copter.mode_guided.set_attitude_target_provides_thrust();

        float climb_rate_or_thrust;
        if (use_thrust) {
            // interpret thrust as thrust
            climb_rate_or_thrust = constrain_float(packet.thrust, -1.0f, 1.0f);
        } else {
            // convert thrust to climb rate
            packet.thrust = constrain_float(packet.thrust, 0.0f, 1.0f);
            if (is_equal(packet.thrust, 0.5f)) {
                climb_rate_or_thrust = 0.0f;
            } else if (packet.thrust > 0.5f) {
                // climb at up to WPNAV_SPEED_UP
                climb_rate_or_thrust = (packet.thrust - 0.5f) * 2.0f * copter.wp_nav->get_default_speed_up();
            } else {
                // descend at up to WPNAV_SPEED_DN
                climb_rate_or_thrust = (0.5f - packet.thrust) * 2.0f * -fabsf(copter.wp_nav->get_default_speed_down());
            }
        }

        // if the body_yaw_rate field is ignored, use the commanded yaw position
        // otherwise use the commanded yaw rate
        bool use_yaw_rate = false;
        if ((packet.type_mask & (1<<2)) == 0) {
            use_yaw_rate = true;
        }

        copter.mode_guided.set_angle(Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]),
                climb_rate_or_thrust, use_yaw_rate, packet.body_yaw_rate, use_thrust);

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode()) {
            break;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
            packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
            packet.coordinate_frame != MAV_FRAME_BODY_NED &&
            packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
            // input is not valid so stop
            copter.mode_guided.init(true);
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
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                pos_vector += copter.inertial_nav.get_position();
            }
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // prepare acceleration
        Vector3f accel_vector;
        if (!acc_ignore) {
            // convert to cm
            accel_vector = Vector3f(packet.afx * 100.0f, packet.afy * 100.0f, -packet.afz * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(accel_vector.x, accel_vector.y);
            }
        }

        // prepare yaw
        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = ToDeg(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
        }

        // send request
        if (!pos_ignore && !vel_ignore) {
            copter.mode_guided.set_destination_posvelaccel(pos_vector, vel_vector, accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && !vel_ignore) {
            copter.mode_guided.set_velaccel(vel_vector, accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && vel_ignore && !acc_ignore) {
            copter.mode_guided.set_accel(accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            copter.mode_guided.set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative, false);
        } else {
            // input is not valid so stop
            copter.mode_guided.init(true);
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
    {
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode()) {
            break;
        }

        // todo: do we need to check for supported coordinate frames

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        // extract location from message
        Location loc;
        if (!pos_ignore) {
            // sanity check location
            if (!check_latlng(packet.lat_int, packet.lon_int)) {
                // input is not valid so stop
                copter.mode_guided.init(true);
                break;
            }
            Location::AltFrame frame;
            if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.coordinate_frame, frame)) {
                // unknown coordinate frame
                // input is not valid so stop
                copter.mode_guided.init(true);
                break;
            }
            loc = {packet.lat_int, packet.lon_int, int32_t(packet.alt*100), frame};
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
        }

        // prepare acceleration
        Vector3f accel_vector;
        if (!acc_ignore) {
            // convert to cm
            accel_vector = Vector3f(packet.afx * 100.0f, packet.afy * 100.0f, -packet.afz * 100.0f);
        }

        // prepare yaw
        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = ToDeg(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
        }

        // send targets to the appropriate guided mode controller
        if (!pos_ignore && !vel_ignore) {
            // convert Location to vector from ekf origin for posvel controller
            if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
                // posvel controller does not support alt-above-terrain
                // input is not valid so stop
                copter.mode_guided.init(true);
                break;
            }
            Vector3f pos_neu_cm;
            if (!loc.get_vector_from_origin_NEU(pos_neu_cm)) {
                // input is not valid so stop
                copter.mode_guided.init(true);
                break;
            }
            copter.mode_guided.set_destination_posvel(pos_neu_cm, vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && !vel_ignore) {
            copter.mode_guided.set_velaccel(vel_vector, accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && vel_ignore && !acc_ignore) {
            copter.mode_guided.set_accel(accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            copter.mode_guided.set_destination(loc, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else {
            // input is not valid so stop
            copter.mode_guided.init(true);
        }

        break;
    }
#endif

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:       // MAV ID: 109
    {
        handle_radio_status(msg, copter.should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        copter.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(&msg, &packet);
        if ((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            if (!copter.set_home_to_current_location(true)) {
                // silently ignored
            }
        } else {
            Location new_home_loc;
            new_home_loc.lat = packet.latitude;
            new_home_loc.lng = packet.longitude;
            new_home_loc.alt = packet.altitude / 10;
            if (!copter.set_home(new_home_loc, true)) {
                // silently ignored
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
#if HAL_ADSB_ENABLED
        copter.adsb.handle_message(chan, msg);
#endif
        break;

#if TOY_MODE_ENABLED == ENABLED
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        copter.g2.toy_mode.handle_message(msg);
        break;
#endif
        
    default:
        handle_common_message(msg);
        break;
    }     // end switch
} // end handle mavlink


MAV_RESULT GCS_MAVLINK_Copter::handle_flight_termination(const mavlink_command_long_t &packet) {
#if ADVANCED_FAILSAFE == ENABLED
    if (GCS_MAVLINK::handle_flight_termination(packet) == MAV_RESULT_ACCEPTED) {
        return MAV_RESULT_ACCEPTED;
    }
#endif
    if (packet.param1 > 0.5f) {
        copter.arming.disarm(AP_Arming::Method::TERMINATION);
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_FAILED;
}

float GCS_MAVLINK_Copter::vfr_hud_alt() const
{
    if (copter.g2.dev_options.get() & DevOptionVFR_HUDRelativeAlt) {
        // compatibility option for older mavlink-aware devices that
        // assume Copter returns a relative altitude in VFR_HUD.alt
        return copter.current_loc.alt * 0.01f;
    }
    return GCS_MAVLINK::vfr_hud_alt();
}

uint64_t GCS_MAVLINK_Copter::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
            (copter.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            GCS_MAVLINK::capabilities());
}

MAV_LANDED_STATE GCS_MAVLINK_Copter::landed_state() const
{
    if (copter.ap.land_complete) {
        return MAV_LANDED_STATE_ON_GROUND;
    }
    if (copter.flightmode->is_landing()) {
        return MAV_LANDED_STATE_LANDING;
    }
    if (copter.flightmode->is_taking_off()) {
        return MAV_LANDED_STATE_TAKEOFF;
    }
    return MAV_LANDED_STATE_IN_AIR;
}

void GCS_MAVLINK_Copter::send_wind() const
{
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a
        // valid wind estimate on copters
        return;
    }
    const Vector3f wind = AP::ahrs().wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)),
        wind.length(),
        wind.z);
}
