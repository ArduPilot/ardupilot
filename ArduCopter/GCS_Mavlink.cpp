#include "Copter.h"

#include "GCS_Mavlink.h"

void Copter::gcs_send_heartbeat(void)
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

MAV_TYPE GCS_Copter::frame_type() const
{
    return copter.get_frame_mav_type();
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
    switch (copter.control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case AVOID_ADSB:
    case FOLLOW:
    case GUIDED:
    case CIRCLE:
    case POSHOLD:
    case BRAKE:
    case SMART_RTL:
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

#if HIL_MODE != HIL_MODE_DISABLED
    _base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

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
    return copter.control_mode;
}

MAV_STATE GCS_MAVLINK_Copter::system_status() const
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
    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL_INT, // targets are always global altitude
        0xFFF8, // ignore everything except the x/y/z components
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

void GCS_MAVLINK_Copter::send_nav_controller_output() const
{
    if (!copter.ap.initialised) {
        return;
    }
    const Vector3f &targets = copter.attitude_control->get_att_target_euler_cd();
    const Copter::Mode *flightmode = copter.flightmode;
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        flightmode->wp_bearing() * 1.0e-2f,
        MIN(flightmode->wp_distance() * 1.0e-2f, UINT16_MAX),
        copter.pos_control->get_alt_error() * 1.0e-2f,
        0,
        flightmode->crosstrack_error() * 1.0e-2f);
}

int16_t GCS_MAVLINK_Copter::vfr_hud_throttle() const
{
    return (int16_t)(copter.motors->get_throttle() * 100);
}

/*
  send PID tuning message
 */
void GCS_MAVLINK_Copter::send_pid_tuning()
{
    const Vector3f &gyro = AP::ahrs().get_gyro();
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
        AC_PID &pid = copter.attitude_control->get_rate_roll_pid(); // dummy ref
        float achieved;
        switch (axes[i]) {
        case PID_TUNING_ROLL:
            pid = copter.attitude_control->get_rate_roll_pid();
            achieved = degrees(gyro.x);
            break;
        case PID_TUNING_PITCH:
            pid = copter.attitude_control->get_rate_pitch_pid();
            achieved = degrees(gyro.y);
            break;
        case PID_TUNING_YAW:
            pid = copter.attitude_control->get_rate_yaw_pid();
            achieved = degrees(gyro.z);
            break;
        case PID_TUNING_ACCZ:
            pid = copter.pos_control->get_accel_z_pid();
            achieved = -(AP::ahrs().get_accel_ef_blended().z + GRAVITY_MSS);
            break;
        default:
            continue;
        }
        const AP_Logger::PID_Info &pid_info = pid.get_pid_info();
        mavlink_msg_pid_tuning_send(chan,
                                    axes[i],
                                    pid_info.desired*0.01f,
                                    achieved,
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f);
    }
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
#if HIL_MODE != HIL_MODE_SENSORS
    // if we don't have at least 250 micros remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications

    // the check for nullptr here doesn't just save a nullptr
    // dereference; it means that we send messages out even if we're
    // failing to detect a PX4 board type (see delay(3000) in px_drivers).
    if (copter.motors != nullptr && copter.scheduler.time_available_usec() < 250 && copter.motors->armed()) {
        gcs().set_out_of_time(true);
        return false;
    }
#endif

    switch(id) {

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        copter.terrain.send_request(chan);
#endif
        break;

    case MSG_WIND:
    case MSG_SERVO_OUT:
    case MSG_AOA_SSA:
    case MSG_LANDING:
        // unused
        break;

    case MSG_ADSB_VEHICLE:
#if ADSB_ENABLED == ENABLED
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        copter.adsb.send_adsb_vehicle(chan);
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
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, POWER_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, and FENCE_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
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
    // @Description: Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE, SIMSTATE (SITL only), AHRS2 and PID_TUNING to ground station
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
    // @Description: Stream rate of AHRS, HWSTATUS, SYSTEM_TIME, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, BATTERY2, MOUNT_STATUS, OPTICAL_FLOW, GIMBAL_REPORT, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION and RPM to ground station
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

    // @Param: ADSB
    // @DisplayName: ADSB stream rate to ground station
    // @Description: ADSB stream rate to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK, streamRates[9],  0),
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
    MSG_RADIO_IN // RC_CHANNELS_RAW, RC_CHANNELS
};
static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
    MSG_SIMSTATE,
    MSG_AHRS2,
    MSG_AHRS3,
    MSG_PID_TUNING // Up to four PID_TUNING messages are sent, depending on GCS_PID_MASK parameter
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
    MSG_RPM,
    MSG_ESC_TELEMETRY,
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
                                        mavlink_message_t &msg)
{
#if ADSB_ENABLED == ENABLED
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
    if (AP_BoardConfig::in_sensor_config_error()) {
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
    send_text(MAV_SEVERITY_INFO, "Frame: %s", copter.get_frame_string());
}

// a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
void GCS_MAVLINK_Copter::handle_rc_channels_override(const mavlink_message_t *msg)
{
    copter.failsafe.last_heartbeat_ms = AP_HAL::millis();
    GCS_MAVLINK::handle_rc_channels_override(msg);
}

void GCS_MAVLINK_Copter::handle_command_ack(const mavlink_message_t* msg)
{
    copter.command_ack_counter++;
    GCS_MAVLINK::handle_command_ack(msg);
}

MAV_RESULT GCS_MAVLINK_Copter::_handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    if (is_equal(packet.param6,1.0f)) {
        // compassmot calibration
        return copter.mavlink_compassmot(chan);
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet);
}


MAV_RESULT GCS_MAVLINK_Copter::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!check_latlng(roi_loc)) {
        return MAV_RESULT_FAILED;
    }
    copter.flightmode->auto_yaw.set_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

bool GCS_MAVLINK_Copter::set_home_to_current_location(bool lock) {
    return copter.set_home_to_current_location(lock);
}
bool GCS_MAVLINK_Copter::set_home(const Location& loc, bool lock) {
    return copter.set_home(loc, lock);
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

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet);
    }
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_mount(const mavlink_command_long_t &packet)
{
    // if the mount doesn't do pan control then yaw the entire vehicle instead:
    switch (packet.command) {
#if MOUNT == ENABLED
    case MAV_CMD_DO_MOUNT_CONTROL:
        if(!copter.camera_mount.has_pan_control()) {
            copter.flightmode->auto_yaw.set_fixed_yaw(
                (float)packet.param3 / 100.0f,
                0.0f,
                0,0);
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

    case MAV_CMD_NAV_LOITER_UNLIM:
        if (!copter.set_mode(LOITER, MODE_REASON_GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        if (!copter.set_mode(RTL, MODE_REASON_GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_LAND:
        if (!copter.set_mode(LAND, MODE_REASON_GCS_COMMAND)) {
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
        // param1 : unused
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

    case MAV_CMD_DO_SET_HOME:
        // param1 : use current (1=use current location, 0=use specified location)
        // param5 : latitude
        // param6 : longitude
        // param7 : altitude (absolute)
        if (is_equal(packet.param1,1.0f)) {
            if (copter.set_home_to_current_location(true)) {
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
            if (copter.set_home(new_home_loc, true)) {
                return MAV_RESULT_ACCEPTED;
            }
        }
        return MAV_RESULT_FAILED;

#if MODE_AUTO_ENABLED == ENABLED
    case MAV_CMD_MISSION_START:
        if (copter.motors->armed() && copter.set_mode(AUTO, MODE_REASON_GCS_COMMAND)) {
            copter.set_auto_armed(true);
            if (copter.mode_auto.mission.state() != AP_Mission::MISSION_RUNNING) {
                copter.mode_auto.mission.start_or_resume();
            }
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (is_equal(packet.param1,1.0f)) {
            // attempt to arm and return success or failure
            const bool do_arming_checks = !is_equal(packet.param2,magic_force_arm_value);
            if (copter.init_arm_motors(AP_Arming::Method::MAVLINK, do_arming_checks)) {
                return MAV_RESULT_ACCEPTED;
            }
        } else if (is_zero(packet.param1))  {
            if (copter.ap.land_complete || is_equal(packet.param2,magic_force_disarm_value)) {
                // force disarming by setting param2 = 21196 is deprecated
                copter.init_disarm_motors();
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else {
            return MAV_RESULT_UNSUPPORTED;
        }
        return MAV_RESULT_FAILED;

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // configure or release parachute
        switch ((uint16_t)packet.param1) {
        case PARACHUTE_DISABLE:
            copter.parachute.enabled(false);
            copter.Log_Write_Event(DATA_PARACHUTE_DISABLED);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_ENABLE:
            copter.parachute.enabled(true);
            copter.Log_Write_Event(DATA_PARACHUTE_ENABLED);
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
        // param6 : compass learning (0: disabled, 1: enabled)
        return copter.mavlink_motor_test_start(chan,
                                               (uint8_t)packet.param1,
                                               (uint8_t)packet.param2,
                                               (uint16_t)packet.param3,
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
            copter.Log_Write_Event(DATA_WINCH_RELAXED);
            return MAV_RESULT_ACCEPTED;
        case WINCH_RELATIVE_LENGTH_CONTROL: {
            copter.g2.winch.release_length(packet.param3, fabsf(packet.param4));
            copter.Log_Write_Event(DATA_WINCH_LENGTH_CONTROL);
            return MAV_RESULT_ACCEPTED;
        }
        case WINCH_RATE_CONTROL:
            if (fabsf(packet.param4) <= copter.g2.winch.get_rate_max()) {
                copter.g2.winch.set_desired_rate(packet.param4);
                copter.Log_Write_Event(DATA_WINCH_RATE_CONTROL);
                return MAV_RESULT_ACCEPTED;
            }
            return MAV_RESULT_FAILED;
        default:
            break;
        }
        return MAV_RESULT_FAILED;
#endif

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

        /* Solo user presses Fly button */
    case MAV_CMD_SOLO_BTN_FLY_CLICK: {
        if (copter.failsafe.radio) {
            return MAV_RESULT_ACCEPTED;
        }

        // set mode to Loiter or fall back to AltHold
        if (!copter.set_mode(LOITER, MODE_REASON_GCS_COMMAND)) {
            copter.set_mode(ALT_HOLD, MODE_REASON_GCS_COMMAND);
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
            copter.init_arm_motors(AP_Arming::Method::MAVLINK);
        } else if (copter.ap.land_complete) {
            // if armed and landed, takeoff
            if (copter.set_mode(LOITER, MODE_REASON_GCS_COMMAND)) {
                copter.flightmode->do_user_takeoff(packet.param1*100, true);
            }
        } else {
            // if flying, land
            copter.set_mode(LAND, MODE_REASON_GCS_COMMAND);
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
                copter.init_disarm_motors();
            } else {
                // assume that shots modes are all done in guided.
                // NOTE: this may need to change if we add a non-guided shot mode
                bool shot_mode = (!is_zero(packet.param1) && (copter.control_mode == GUIDED || copter.control_mode == GUIDED_NOGPS));

                if (!shot_mode) {
#if MODE_BRAKE_ENABLED == ENABLED
                    if (copter.set_mode(BRAKE, MODE_REASON_GCS_COMMAND)) {
                        copter.mode_brake.timeout_to_loiter_ms(2500);
                    } else {
                        copter.set_mode(ALT_HOLD, MODE_REASON_GCS_COMMAND);
                    }
#else
                    copter.set_mode(ALT_HOLD, MODE_REASON_GCS_COMMAND);
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

void GCS_MAVLINK_Copter::handle_mount_message(const mavlink_message_t* msg)
{
    switch (msg->msgid) {
#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        if(!copter.camera_mount.has_pan_control()) {
            // if the mount doesn't do pan control then yaw the entire vehicle instead:
            copter.flightmode->auto_yaw.set_fixed_yaw(
                mavlink_msg_mount_control_get_input_c(msg)/100.0f,
                0.0f,
                0,
                0);

            break;
        }
#endif
    }
    GCS_MAVLINK::handle_mount_message(msg);
}

void GCS_MAVLINK_Copter::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:      // MAV ID: 0
    {
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if(msg->sysid != copter.g.sysid_my_gcs) break;
        copter.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        if (msg->sysid != copter.g.sysid_my_gcs) {
            break; // only accept control from our gcs
        }

        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(msg, &packet);

        if (packet.target != copter.g.sysid_this_mav) {
            break; // only accept control aimed at us
        }

        if (packet.z < 0) { // Copter doesn't do negative thrust
            break;
        }

        uint32_t tnow = AP_HAL::millis();

        int16_t roll = (packet.y == INT16_MAX) ? 0 : copter.channel_roll->get_radio_min() + (copter.channel_roll->get_radio_max() - copter.channel_roll->get_radio_min()) * (packet.y + 1000) / 2000.0f;
        int16_t pitch = (packet.x == INT16_MAX) ? 0 : copter.channel_pitch->get_radio_min() + (copter.channel_pitch->get_radio_max() - copter.channel_pitch->get_radio_min()) * (-packet.x + 1000) / 2000.0f;
        int16_t throttle = (packet.z == INT16_MAX) ? 0 : copter.channel_throttle->get_radio_min() + (copter.channel_throttle->get_radio_max() - copter.channel_throttle->get_radio_min()) * (packet.z) / 1000.0f;
        int16_t yaw = (packet.r == INT16_MAX) ? 0 : copter.channel_yaw->get_radio_min() + (copter.channel_yaw->get_radio_max() - copter.channel_yaw->get_radio_min()) * (packet.r + 1000) / 2000.0f;

        RC_Channels::set_override(uint8_t(copter.rcmap.roll() - 1), roll, tnow);
        RC_Channels::set_override(uint8_t(copter.rcmap.pitch() - 1), pitch, tnow);
        RC_Channels::set_override(uint8_t(copter.rcmap.throttle() - 1), throttle, tnow);
        RC_Channels::set_override(uint8_t(copter.rcmap.yaw() - 1), yaw, tnow);

        // a manual control message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        copter.failsafe.last_heartbeat_ms = tnow;
        break;
    }

#if MODE_GUIDED_ENABLED == ENABLED
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:   // MAV ID: 82
    {
        // decode packet
        mavlink_set_attitude_target_t packet;
        mavlink_msg_set_attitude_target_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode()) {
            break;
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
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * copter.wp_nav->get_default_speed_up();
        } else {
            // descend at up to WPNAV_SPEED_DN
            climb_rate_cms = (0.5f - packet.thrust) * 2.0f * -fabsf(copter.wp_nav->get_default_speed_down());
        }

        // if the body_yaw_rate field is ignored, use the commanded yaw position
        // otherwise use the commanded yaw rate
        bool use_yaw_rate = false;
        if ((packet.type_mask & (1<<2)) == 0) {
            use_yaw_rate = true;
        }

        copter.mode_guided.set_angle(Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]),
            climb_rate_cms, use_yaw_rate, packet.body_yaw_rate);

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode()) {
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
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         */

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
            } else {
                // convert from alt-above-home to alt-above-ekf-origin
                if (!AP::ahrs().home_is_set()) {
                    break;
                }
                const Location &origin = copter.inertial_nav.get_origin();
                pos_vector.z += AP::ahrs().get_home().alt;
                pos_vector.z -= origin.alt;
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
        if (!pos_ignore && !vel_ignore && acc_ignore) {
            copter.mode_guided.set_destination_posvel(pos_vector, vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            copter.mode_guided.set_velocity(vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            copter.mode_guided.set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
    {
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode()) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         */

        Vector3f pos_neu_cm;  // position (North, East, Up coordinates) in centimeters

        if(!pos_ignore) {
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

        if (!pos_ignore && !vel_ignore && acc_ignore) {
            copter.mode_guided.set_destination_posvel(pos_neu_cm, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            copter.mode_guided.set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            copter.mode_guided.set_destination(pos_neu_cm, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        }

        break;
    }
#endif

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
    {
        copter.rangefinder.handle_msg(msg);
#if PROXIMITY_ENABLED == ENABLED
        copter.g2.proximity.handle_msg(msg);
#endif
        break;
    }

    case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
    {
#if PROXIMITY_ENABLED == ENABLED
        copter.g2.proximity.handle_msg(msg);
#endif
        break;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:          // MAV ID: 90
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

        AP::baro().setHIL(packet.alt*0.001f);
        copter.compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
        copter.compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);

        break;
    }
#endif //  HIL_MODE != HIL_MODE_DISABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:       // MAV ID: 109
    {
        handle_radio_status(msg, copter.logger, copter.should_log(MASK_LOG_PM));
        break;
    }

#if PRECISION_LANDING == ENABLED
    case MAVLINK_MSG_ID_LANDING_TARGET:
        copter.precland.handle_msg(msg);
        break;
#endif

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
        copter.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(msg, &packet);
        if((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
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
#if ADSB_ENABLED == ENABLED
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


/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Copter::mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs().chan(0).initialised) return;

    logger.EnableWrites(false);

    uint32_t tnow = millis();
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

AP_AdvancedFailsafe *GCS_MAVLINK_Copter::get_advanced_failsafe() const
{
#if ADVANCED_FAILSAFE == ENABLED
    return &copter.g2.afs;
#else
    return nullptr;
#endif
}

MAV_RESULT GCS_MAVLINK_Copter::handle_flight_termination(const mavlink_command_long_t &packet) {
    MAV_RESULT result = MAV_RESULT_FAILED;

#if ADVANCED_FAILSAFE == ENABLED
    if (GCS_MAVLINK::handle_flight_termination(packet) != MAV_RESULT_ACCEPTED) {
#endif
        if (packet.param1 > 0.5f) {
            copter.init_disarm_motors();
            result = MAV_RESULT_ACCEPTED;
        }
#if ADVANCED_FAILSAFE == ENABLED
    } else {
        result = MAV_RESULT_ACCEPTED;
    }
#endif

    return result;
}

bool GCS_MAVLINK_Copter::set_mode(const uint8_t mode)
{
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (copter.failsafe.radio) {
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif
    return copter.set_mode((control_mode_t)mode, MODE_REASON_GCS_COMMAND);
}

float GCS_MAVLINK_Copter::vfr_hud_alt() const
{
    if (copter.g2.dev_options.get() & DevOptionVFR_HUDRelativeAlt) {
        // compatability option for older mavlink-aware devices that
        // assume Copter returns a relative altitude in VFR_HUD.alt
        return copter.current_loc.alt / 100.0f;
    }
    return GCS_MAVLINK::vfr_hud_alt();
}

uint64_t GCS_MAVLINK_Copter::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
            (copter.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
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
