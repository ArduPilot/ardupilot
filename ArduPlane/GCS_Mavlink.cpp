#include "GCS_Mavlink.h"

#include "Plane.h"

MAV_TYPE GCS_Plane::frame_type() const
{
    return plane.quadplane.get_mav_type();
}

MAV_MODE GCS_MAVLINK_Plane::base_mode() const
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
    switch (plane.control_mode->mode_number()) {
    case Mode::Number::MANUAL:
    case Mode::Number::TRAINING:
    case Mode::Number::ACRO:
    case Mode::Number::QACRO:
        _base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case Mode::Number::STABILIZE:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::CRUISE:
    case Mode::Number::QAUTOTUNE:
        _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case Mode::Number::AUTO:
    case Mode::Number::RTL:
    case Mode::Number::LOITER:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::TAKEOFF:
    case Mode::Number::QRTL:
        _base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                     MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case Mode::Number::INITIALISING:
        break;
    }

    if (!plane.training_manual_pitch || !plane.training_manual_roll) {
        _base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;        
    }

    if (plane.control_mode != &plane.mode_manual && plane.control_mode != &plane.mode_initializing) {
        // stabiliser of some form is enabled
        _base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    if (plane.g.stick_mixing != STICK_MIXING_DISABLED && plane.control_mode != &plane.mode_initializing) {
        if ((plane.g.stick_mixing != STICK_MIXING_VTOL_YAW) || (plane.control_mode == &plane.mode_auto)) {
            // all modes except INITIALISING have some form of manual
            // override if stick mixing is enabled
            _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        }
    }

#if HIL_SUPPORT
    if (plane.g.hil_mode == 1) {
        _base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
    }
#endif

    // we are armed if we are not initialising
    if (plane.control_mode != &plane.mode_initializing && plane.arming.is_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return (MAV_MODE)_base_mode;
}

uint32_t GCS_Plane::custom_mode() const
{
    return plane.control_mode->mode_number();
}

MAV_STATE GCS_MAVLINK_Plane::vehicle_system_status() const
{
    if (plane.control_mode == &plane.mode_initializing) {
        return MAV_STATE_CALIBRATING;
    }
    if (plane.any_failsafe_triggered()) {
        return MAV_STATE_CRITICAL;
    }
    if (plane.crash_state.is_crashed) {
        return MAV_STATE_EMERGENCY;
    }
    if (plane.is_flying()) {
        return MAV_STATE_ACTIVE;
    }

    return MAV_STATE_STANDBY;
}


void GCS_MAVLINK_Plane::send_attitude() const
{
    const AP_AHRS &ahrs = AP::ahrs();

    float r = ahrs.roll;
    float p = ahrs.pitch - radians(plane.g.pitch_trim_cd*0.01f);
    float y = ahrs.yaw;
    
    if (plane.quadplane.in_vtol_mode()) {
        r = plane.quadplane.ahrs_view->roll;
        p = plane.quadplane.ahrs_view->pitch;
        y = plane.quadplane.ahrs_view->yaw;
    }
    
    const Vector3f &omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        r,
        p,
        y,
        omega.x,
        omega.y,
        omega.z);
}

void GCS_MAVLINK_Plane::send_aoa_ssa()
{
    AP_AHRS &ahrs = AP::ahrs();

    mavlink_msg_aoa_ssa_send(
        chan,
        micros(),
        ahrs.getAOA(),
        ahrs.getSSA());
}

#if GEOFENCE_ENABLED == ENABLED
void Plane::send_fence_status(mavlink_channel_t chan)
{
    geofence_send_status(chan);
}
#endif


void GCS_MAVLINK_Plane::send_nav_controller_output() const
{
    if (plane.control_mode == &plane.mode_manual) {
        return;
    }
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.in_vtol_mode()) {
        const Vector3f &targets = quadplane.attitude_control->get_att_target_euler_cd();
        bool wp_nav_valid = quadplane.using_wp_nav();

        mavlink_msg_nav_controller_output_send(
            chan,
            targets.x * 1.0e-2f,
            targets.y * 1.0e-2f,
            targets.z * 1.0e-2f,
            wp_nav_valid ? quadplane.wp_nav->get_wp_bearing_to_destination() : 0,
            wp_nav_valid ? MIN(quadplane.wp_nav->get_wp_distance_to_destination(), UINT16_MAX) : 0,
            (plane.control_mode != &plane.mode_qstabilize) ? quadplane.pos_control->get_alt_error() * 1.0e-2f : 0,
            plane.airspeed_error * 100,
            wp_nav_valid ? quadplane.wp_nav->crosstrack_error() : 0);
    } else {
        const AP_Navigation *nav_controller = plane.nav_controller;
        mavlink_msg_nav_controller_output_send(
            chan,
            plane.nav_roll_cd * 0.01f,
            plane.nav_pitch_cd * 0.01f,
            nav_controller->nav_bearing_cd() * 0.01f,
            nav_controller->target_bearing_cd() * 0.01f,
            MIN(plane.auto_state.wp_distance, UINT16_MAX),
            plane.altitude_error_cm * 0.01f,
            plane.airspeed_error * 100,
            nav_controller->crosstrack_error());
    }
}

void GCS_MAVLINK_Plane::send_position_target_global_int()
{
    if (plane.control_mode == &plane.mode_manual) {
        return;
    }
    Location &next_WP_loc = plane.next_WP_loc;
    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL, // targets are always global altitude
        0xFFF8, // ignore everything except the x/y/z components
        next_WP_loc.lat, // latitude as 1e7
        next_WP_loc.lng, // longitude as 1e7
        next_WP_loc.alt * 0.01f, // altitude is sent as a float
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
}


void Plane::send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) / 4500.0f),
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) / 4500.0f),
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 100.0f),
        10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / 4500.0f),
        0,
        0,
        0,
        0,
        rssi.read_receiver_rssi_uint8());
}

float GCS_MAVLINK_Plane::vfr_hud_airspeed() const
{
    // airspeed sensors are best.  While the AHRS airspeed_estimate
    // will use an airspeed sensor, that value is constrained by the
    // ground speed.  When reporting we should send the true airspeed
    // value if possible:
    if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
        return plane.airspeed.get_airspeed();
    }

    // airspeed estimates are OK:
    float aspeed;
    if (AP::ahrs().airspeed_estimate(aspeed)) {
        return aspeed;
    }

    // lying is worst:
    return 0;
}

int16_t GCS_MAVLINK_Plane::vfr_hud_throttle() const
{
    return plane.throttle_percentage();
}

float GCS_MAVLINK_Plane::vfr_hud_climbrate() const
{
#if SOARING_ENABLED == ENABLED
    if (plane.g2.soaring_controller.is_active()) {
        return plane.g2.soaring_controller.get_vario_reading();
    }
#endif
    return AP::baro().get_climb_rate();
}

/*
  keep last HIL_STATE message to allow sending SIM_STATE
 */
#if HIL_SUPPORT
static mavlink_hil_state_t last_hil_state;
#endif

// report simulator state
void GCS_MAVLINK_Plane::send_simstate() const
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    GCS_MAVLINK::send_simstate();
#elif HIL_SUPPORT
    if (plane.g.hil_mode == 1) {
        mavlink_msg_simstate_send(chan,
                                  last_hil_state.roll,
                                  last_hil_state.pitch,
                                  last_hil_state.yaw,
                                  last_hil_state.xacc*0.001f*GRAVITY_MSS,
                                  last_hil_state.yacc*0.001f*GRAVITY_MSS,
                                  last_hil_state.zacc*0.001f*GRAVITY_MSS,
                                  last_hil_state.rollspeed,
                                  last_hil_state.pitchspeed,
                                  last_hil_state.yawspeed,
                                  last_hil_state.lat,
                                  last_hil_state.lon);
    }
#endif
}

void GCS_MAVLINK_Plane::send_wind() const
{
    const Vector3f wind = AP::ahrs().wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)), // use negative, to give
                                          // direction wind is coming from
        wind.length(),
        wind.z);
}

// sends a single pid info over the provided channel
void GCS_MAVLINK_Plane::send_pid_info(const AP_Logger::PID_Info *pid_info,
                          const uint8_t axis, const float achieved)
{
    if (pid_info == nullptr) {
        return;
    }
    if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
        return;
    }
     mavlink_msg_pid_tuning_send(chan, axis,
                                 pid_info->target,
                                 achieved,
                                 pid_info->FF,
                                 pid_info->P,
                                 pid_info->I,
                                 pid_info->D);
}

/*
  send PID tuning message
 */
void GCS_MAVLINK_Plane::send_pid_tuning()
{
    if (plane.control_mode == &plane.mode_manual) {
        // no PIDs should be used in manual
        return;
    }

    const Parameters &g = plane.g;

    const AP_Logger::PID_Info *pid_info;
    if (g.gcs_pid_mask & TUNING_BITS_ROLL) {
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        } else {
            pid_info = &plane.rollController.get_pid_info();
        }
        send_pid_info(pid_info, PID_TUNING_ROLL, pid_info->actual);
    }
    if (g.gcs_pid_mask & TUNING_BITS_PITCH) {
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        } else {
            pid_info = &plane.pitchController.get_pid_info();
        }
        send_pid_info(pid_info, PID_TUNING_PITCH, pid_info->actual);
    }
    if (g.gcs_pid_mask & TUNING_BITS_YAW) {
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        } else {
            pid_info = &plane.yawController.get_pid_info();
        }
        send_pid_info(pid_info, PID_TUNING_YAW, pid_info->actual);
    }
    if (g.gcs_pid_mask & TUNING_BITS_STEER) {
        pid_info = &plane.steerController.get_pid_info();
        send_pid_info(pid_info, PID_TUNING_STEER, pid_info->actual);
    }
    if ((g.gcs_pid_mask & TUNING_BITS_LAND) && (plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND)) {
        AP_AHRS &ahrs = AP::ahrs();
        const Vector3f &gyro = ahrs.get_gyro();
        send_pid_info(plane.landing.get_pid_info(), PID_TUNING_LANDING, degrees(gyro.z));
    }
    if (g.gcs_pid_mask & TUNING_BITS_ACCZ && plane.quadplane.in_vtol_mode()) {
        pid_info = &plane.quadplane.pos_control->get_accel_z_pid().get_pid_info();
        send_pid_info(pid_info, PID_TUNING_ACCZ, pid_info->actual);
    }
 }

uint8_t GCS_MAVLINK_Plane::sysid_my_gcs() const
{
    return plane.g.sysid_my_gcs;
}
bool GCS_MAVLINK_Plane::sysid_enforce() const
{
    return plane.g2.sysid_enforce;
}

uint32_t GCS_MAVLINK_Plane::telem_delay() const
{
    return (uint32_t)(plane.g.telem_delay);
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Plane::try_send_message(enum ap_message id)
{
    switch (id) {

    case MSG_SERVO_OUT:
#if HIL_SUPPORT
        if (plane.g.hil_mode == 1) {
            CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
            plane.send_servo_out(chan);
        }
#endif
        break;

    case MSG_FENCE_STATUS:
#if GEOFENCE_ENABLED == ENABLED
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        plane.send_fence_status(chan);
#endif
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        plane.terrain.send_request(chan);
#endif
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_ADSB_VEHICLE:
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        plane.adsb.send_adsb_vehicle(chan);
        break;

    case MSG_AOA_SSA:
        CHECK_PAYLOAD_SIZE(AOA_SSA);
        send_aoa_ssa();
        break;
    case MSG_LANDING:
        plane.landing.send_landing_message(chan);
        break;
    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}


/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[0],  1),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK_Parameters, streamRates[1],  1),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK_Parameters, streamRates[2],  1),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK_Parameters, streamRates[3],  1),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK_Parameters, streamRates[4],  1),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[5],  1),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK_Parameters, streamRates[6],  1),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[7],  1),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK_Parameters, streamRates[8],  10),

    // @Param: ADSB
    // @DisplayName: ADSB stream rate to ground station
    // @Description: ADSB stream rate to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK_Parameters, streamRates[9],  5),
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
    MSG_POSITION_TARGET_GLOBAL_INT,
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
    MSG_RC_CHANNELS,
    MSG_RC_CHANNELS_RAW, // only sent on a mavlink1 connection
};
static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
    MSG_SIMSTATE,
    MSG_AHRS2,
    MSG_RPM,
    MSG_AOA_SSA,
    MSG_PID_TUNING,
    MSG_LANDING,
    MSG_ESC_TELEMETRY,
    MSG_EFI_STATUS,
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
#if AP_TERRAIN_AVAILABLE
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
    MAV_STREAM_ENTRY(STREAM_RAW_CONTROLLER),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_ENTRY(STREAM_ADSB),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

bool GCS_MAVLINK_Plane::in_hil_mode() const
{
#if HIL_SUPPORT
    return plane.g.hil_mode == 1;
#endif
    return false;
}

/*
  handle a request to switch to guided mode. This happens via a
  callback from handle_mission_item()
 */
bool GCS_MAVLINK_Plane::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    if (plane.control_mode != &plane.mode_guided) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    plane.guided_WP_loc = cmd.content.location;
    
    // add home alt if needed
    if (plane.guided_WP_loc.relative_alt) {
        plane.guided_WP_loc.alt += plane.home.alt;
        plane.guided_WP_loc.relative_alt = 0;
    }

    plane.set_guided_WP();
    return true;
}

/*
  handle a request to change current WP altitude. This happens via a
  callback from handle_mission_item()
 */
void GCS_MAVLINK_Plane::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    plane.next_WP_loc.alt = cmd.content.location.alt;
    if (cmd.content.location.relative_alt) {
        plane.next_WP_loc.alt += plane.home.alt;
    }
    plane.next_WP_loc.relative_alt = false;
    plane.next_WP_loc.terrain_alt = cmd.content.location.terrain_alt;
    plane.reset_offset_altitude();
}


MAV_RESULT GCS_MAVLINK_Plane::handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    plane.in_calibration = true;
    MAV_RESULT ret = GCS_MAVLINK::handle_command_preflight_calibration(packet);
    plane.in_calibration = false;

    return ret;
}

MAV_RESULT GCS_MAVLINK_Plane::_handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    if (is_equal(packet.param4,1.0f)) {
        if (plane.trim_radio()) {
            return MAV_RESULT_ACCEPTED;
        } else {
            return MAV_RESULT_FAILED;
        }
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet);
}

void GCS_MAVLINK_Plane::packetReceived(const mavlink_status_t &status,
                                       const mavlink_message_t &msg)
{
    plane.avoidance_adsb.handle_msg(msg);
    GCS_MAVLINK::packetReceived(status, msg);
}


bool GCS_MAVLINK_Plane::set_home_to_current_location(bool _lock)
{
    if (!plane.set_home_persistently(AP::gps().location())) {
        return false;
    }
    if (_lock) {
        AP::ahrs().lock_home();
    }
    return true;
}
bool GCS_MAVLINK_Plane::set_home(const Location& loc, bool _lock)
{
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }
    if (_lock) {
        AP::ahrs().lock_home();
    }
    return true;
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location requested_position {};
    requested_position.lat = packet.x;
    requested_position.lng = packet.y;

    if (fabsf(packet.z) > LOCATION_ALT_MAX_M) {
        return MAV_RESULT_DENIED;
    }
    Location::AltFrame frame;
    if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.frame, frame)) {
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }
    requested_position.set_alt_cm((int32_t)(packet.z * 100.0f), frame);

    if (is_zero(packet.param4)) {
        requested_position.loiter_ccw = 0;
    } else {
        requested_position.loiter_ccw = 1;
    }

    if (requested_position.sanitize(plane.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED;
    }

    // location is valid load and set
    if (((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) ||
        (plane.control_mode == &plane.mode_guided)) {
        plane.set_mode(plane.mode_guided, ModeReason::GCS_COMMAND);
        plane.guided_WP_loc = requested_position;

        // add home alt if needed
        if (plane.guided_WP_loc.relative_alt) {
            plane.guided_WP_loc.alt += plane.home.alt;
            plane.guided_WP_loc.relative_alt = 0;
        }

        plane.set_guided_WP();

        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

// these are GUIDED mode commands that are RATE or slew enabled, so you can have more powerful control than default controls.
MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_guided_slew_commands(const mavlink_command_int_t &packet)
{
  switch(packet.command) {
    
#if OFFBOARD_GUIDED == ENABLED
    case MAV_CMD_GUIDED_CHANGE_SPEED: {
        // command is only valid in guided mode
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

         // only airspeed commands are supported right now...
        if (int(packet.param1) != SPEED_TYPE_AIRSPEED) {  // since SPEED_TYPE is int in range 0-1 and packet.param1 is a *float* this works.
            return MAV_RESULT_DENIED;
        }

         // reject airspeeds that are outside of the tuning envelope
        if (packet.param2 > plane.aparm.airspeed_max || packet.param2 < plane.aparm.airspeed_min) {
            return MAV_RESULT_DENIED;
        }

         // no need to process any new packet/s with the
         //  same airspeed any further, if we are already doing it.
        float new_target_airspeed_cm = packet.param2 * 100;
        if ( is_equal(new_target_airspeed_cm,plane.guided_state.target_airspeed_cm)) { 
            return MAV_RESULT_ACCEPTED;
        }
        plane.guided_state.target_airspeed_cm = new_target_airspeed_cm;
        plane.guided_state.target_airspeed_time_ms = AP_HAL::millis();

         if (is_zero(packet.param3)) {
            // the user wanted /maximum acceleration, pick a large value as close enough
            plane.guided_state.target_airspeed_accel = 1000.0f;
        } else {
            plane.guided_state.target_airspeed_accel = fabsf(packet.param3);
        }

         // assign an acceleration direction
        if (plane.guided_state.target_airspeed_cm < plane.target_airspeed_cm) {
            plane.guided_state.target_airspeed_accel *= -1.0f;
        }
        return MAV_RESULT_ACCEPTED;
    }

     case MAV_CMD_GUIDED_CHANGE_ALTITUDE: {
        // command is only valid in guided
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

        // disallow default value of -1 and dangerous value of zero
        if (is_equal(packet.z, -1.0f) || is_equal(packet.z, 0.0f)){
            return MAV_RESULT_DENIED;
        }

         // the requested alt data might be relative or absolute
        float new_target_alt = packet.z * 100;
        float new_target_alt_rel = packet.z * 100 + plane.home.alt;

         // only global/relative/terrain frames are supported
        switch(packet.frame) {
            case MAV_FRAME_GLOBAL_RELATIVE_ALT: {
                if   (is_equal(plane.guided_state.target_alt,new_target_alt_rel) ) { // compare two floats as near-enough
                    // no need to process any new packet/s with the same ALT any further, if we are already doing it.
                    return MAV_RESULT_ACCEPTED;
                }
                plane.guided_state.target_alt = new_target_alt_rel;
                break;
            }
            case MAV_FRAME_GLOBAL: {
                if   (is_equal(plane.guided_state.target_alt,new_target_alt) ) {  // compare two floats as near-enough
                    // no need to process any new packet/s with the same ALT any further, if we are already doing it.
                    return MAV_RESULT_ACCEPTED;
                }
                plane.guided_state.target_alt = new_target_alt;
                break;
            }
            default:
                //  MAV_RESULT_DENIED  means Command is invalid (is supported but has invalid parameters).
                return MAV_RESULT_DENIED;
        }

        plane.guided_state.target_alt_frame = packet.frame;
        plane.guided_state.last_target_alt = plane.current_loc.alt; // FIXME: Reference frame is not corrected for here
        plane.guided_state.target_alt_time_ms = AP_HAL::millis();

        if (is_zero(packet.param3)) {
            // the user wanted /maximum acceleration, pick a large value as close enough
            plane.guided_state.target_alt_accel = 1000.0;
        } else {
            plane.guided_state.target_alt_accel = fabsf(packet.param3);
        }

         // assign an acceleration direction
        if (plane.guided_state.target_alt < plane.current_loc.alt) {
            plane.guided_state.target_alt_accel *= -1.0f;
        }
        return MAV_RESULT_ACCEPTED;
    }

     case MAV_CMD_GUIDED_CHANGE_HEADING: {

        // command is only valid in guided mode
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

         // don't accept packets outside of [0-360] degree range
        if (packet.param2 < 0.0f || packet.param2 >= 360.0f) {
            return MAV_RESULT_DENIED;
        }

        float new_target_heading = radians(wrap_180(packet.param2));

        // if packet is requesting us to go to the heading we are already going to, we-re already on it.
        if ( (is_equal(new_target_heading,plane.guided_state.target_heading))) { // compare two floats as near-enough
            return MAV_RESULT_ACCEPTED;
        }

        // course over ground
        if ( int(packet.param1) == HEADING_TYPE_COURSE_OVER_GROUND) { // compare as nearest int
            plane.guided_state.target_heading_type = GUIDED_HEADING_COG;
            plane.prev_WP_loc = plane.current_loc;
        // normal vehicle heading
        } else if (int(packet.param1) == HEADING_TYPE_HEADING) { // compare as nearest int
            plane.guided_state.target_heading_type = GUIDED_HEADING_HEADING;
        } else {
            //  MAV_RESULT_DENIED  means Command is invalid (is supported but has invalid parameters).
            return MAV_RESULT_DENIED;
        }

        plane.g2.guidedHeading.reset_I();

        plane.guided_state.target_heading = new_target_heading;
        plane.guided_state.target_heading_accel_limit = MAX(packet.param3, 0.05f);
        plane.guided_state.target_heading_time_ms = AP_HAL::millis();
        return MAV_RESULT_ACCEPTED;
    }
#endif // OFFBOARD_GUIDED == ENABLED


  }
  // anything else ...
  return MAV_RESULT_UNSUPPORTED;

}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_packet(const mavlink_command_int_t &packet)
{

    plane.Log_Write_MavCmdI(packet);

    switch(packet.command) {

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    // special 'slew-enabled' guided commands here... for speed,alt, and direction commands
    case MAV_CMD_GUIDED_CHANGE_SPEED:
    case MAV_CMD_GUIDED_CHANGE_ALTITUDE:
    case MAV_CMD_GUIDED_CHANGE_HEADING:
        return handle_command_int_guided_slew_commands(packet);

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet);
    }
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    switch(packet.command) {

    case MAV_CMD_DO_CHANGE_SPEED: {
        // if we're in failsafe modes (e.g., RTL, LOITER) or in pilot
        // controlled modes (e.g., MANUAL, TRAINING)
        // this command should be ignored since it comes in from GCS
        // or a companion computer:
        if ((plane.control_mode != &plane.mode_guided) &&
            (plane.control_mode != &plane.mode_auto) &&
            (plane.control_mode != &plane.mode_avoidADSB)) {
            // failed
            return MAV_RESULT_FAILED;
        }

        AP_Mission::Mission_Command cmd;
        if (AP_Mission::mavlink_cmd_long_to_mission_cmd(packet, cmd) == MAV_MISSION_ACCEPTED) {
            plane.do_change_speed(cmd);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
    }

    case MAV_CMD_NAV_LOITER_UNLIM:
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_TAKEOFF: {
        // user takeoff only works with quadplane code for now
        // param7 : altitude [metres]
        float takeoff_alt = packet.param7;
        if (plane.quadplane.available() && plane.quadplane.do_user_takeoff(takeoff_alt)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
    }

    case MAV_CMD_MISSION_START:
        plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_DO_LAND_START:
        // attempt to switch to next DO_LAND_START command in the mission
        if (plane.mission.jump_to_landing_sequence()) {
            plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_GO_AROUND:
        {
            uint16_t mission_id = plane.mission.get_current_nav_cmd().id;
            bool is_in_landing = (plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) ||
                                 (mission_id == MAV_CMD_NAV_LAND) ||
                                 (mission_id == MAV_CMD_NAV_VTOL_LAND);
            if (is_in_landing) {
                // fly a user planned abort pattern if available
                if (plane.mission.jump_to_abort_landing_sequence()) {
                    return MAV_RESULT_ACCEPTED;
                }

                // only fly a fixed wing abort if we aren't doing quadplane stuff, or potentially
                // shooting a quadplane approach
                if ((!plane.quadplane.available()) ||
                    ((!plane.quadplane.in_vtol_auto()) &&
                     (!(plane.quadplane.options & QuadPlane::OPTION_MISSION_LAND_FW_APPROACH)))) {
                    // Initiate an aborted landing. This will trigger a pitch-up and
                    // climb-out to a safe altitude holding heading then one of the
                    // following actions will occur, check for in this order:
                    // - If MAV_CMD_CONTINUE_AND_CHANGE_ALT is next command in mission,
                    //      increment mission index to execute it
                    // - else if DO_LAND_START is available, jump to it
                    // - else decrement the mission index to repeat the landing approach

                    if (!is_zero(packet.param1)) {
                        plane.auto_state.takeoff_altitude_rel_cm = packet.param1 * 100;
                    }
                    if (plane.landing.request_go_around()) {
                        plane.auto_state.next_wp_crosstrack = false;
                        return MAV_RESULT_ACCEPTED;
                    }
                }
            }
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_FENCE_ENABLE:
        if (!plane.geofence_present()) {
            gcs().send_text(MAV_SEVERITY_NOTICE,"Fence not configured");
            return MAV_RESULT_FAILED;
        }
        switch((uint16_t)packet.param1) {
        case 0:
            if (! plane.geofence_set_enabled(false)) {
                return MAV_RESULT_FAILED;
            }
            return MAV_RESULT_ACCEPTED;
        case 1:
            if (! plane.geofence_set_enabled(true)) {
                return MAV_RESULT_FAILED;
            }
            return MAV_RESULT_ACCEPTED;
        case 2: //disable fence floor only
            if (! plane.geofence_set_floor_enabled(false)) {
                return MAV_RESULT_FAILED;
            }
            gcs().send_text(MAV_SEVERITY_NOTICE,"Fence floor disabled");
            return MAV_RESULT_ACCEPTED;
        default:
            break;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_SET_HOME: {
        // param1 : use current (1=use current location, 0=use specified location)
        // param5 : latitude
        // param6 : longitude
        // param7 : altitude (absolute)
        if (is_equal(packet.param1,1.0f)) {
            if (!plane.set_home_persistently(AP::gps().location())) {
                return MAV_RESULT_FAILED;
            }
            AP::ahrs().lock_home();
            return MAV_RESULT_ACCEPTED;
        } else {
            // ensure param1 is zero
            if (!is_zero(packet.param1)) {
                return MAV_RESULT_FAILED;
            }
            Location new_home_loc {};
            new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
            if (!set_home(new_home_loc, true)) {
                return MAV_RESULT_FAILED;
            }
            return MAV_RESULT_ACCEPTED;
        }
        break;
    }

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        // param1 : enable/disable
        plane.autotune_enable(!is_zero(packet.param1));
        return MAV_RESULT_ACCEPTED;

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // configure or release parachute
        switch ((uint16_t)packet.param1) {
        case PARACHUTE_DISABLE:
            plane.parachute.enabled(false);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_ENABLE:
            plane.parachute.enabled(true);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_RELEASE:
            // treat as a manual release which performs some additional check of altitude
            if (plane.parachute.released()) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Parachute already released");
                return MAV_RESULT_FAILED;
            }
            if (!plane.parachute.enabled()) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Parachute not enabled");
                return MAV_RESULT_FAILED;
            }
            if (!plane.parachute_manual_release()) {
                return MAV_RESULT_FAILED;
            }
            return MAV_RESULT_ACCEPTED;
        default:
            break;
        }
        return MAV_RESULT_FAILED;
#endif

    case MAV_CMD_DO_MOTOR_TEST:
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        // param5 : motor count (number of motors to test in sequence)
        return plane.quadplane.mavlink_motor_test_start(chan,
                                                        (uint8_t)packet.param1,
                                                        (uint8_t)packet.param2,
                                                        (uint16_t)packet.param3,
                                                        packet.param4,
                                                        (uint8_t)packet.param5);

    case MAV_CMD_DO_VTOL_TRANSITION:
        if (!plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)packet.param1)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_DO_ENGINE_CONTROL:
        if (!plane.g2.ice_control.engine_control(packet.param1, packet.param2, packet.param3)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    default:
        return GCS_MAVLINK::handle_command_long_packet(packet);
    }
}

void GCS_MAVLINK_Plane::handleMessage(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

#if GEOFENCE_ENABLED == ENABLED
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(&msg, &packet);
        if (plane.g.fence_action != FENCE_ACTION_NONE) {
            send_text(MAV_SEVERITY_WARNING,"Fencing must be disabled");
        } else if (packet.count != plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else if (!check_latlng(packet.lat,packet.lng)) {
            send_text(MAV_SEVERITY_WARNING,"Invalid fence point, lat or lng too large");
        } else {
            plane.set_fence_point_with_index(Vector2l(packet.lat*1.0e7f, packet.lng*1.0e7f), packet.idx);
        }
        break;
    }

    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(&msg, &packet);
        if (packet.idx >= plane.g.fence_total) {
            send_text(MAV_SEVERITY_WARNING,"Bad fence point");
        } else {
            Vector2l point = plane.get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send(chan, msg.sysid, msg.compid, packet.idx, plane.g.fence_total,
                                             point.x*1.0e-7f, point.y*1.0e-7f);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        if (msg.sysid != plane.g.sysid_my_gcs) {
            break; // only accept control from our gcs
        }

        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(&msg, &packet);

        if (packet.target != plane.g.sysid_this_mav) {
            break; // only accept messages aimed at us
        }

        uint32_t tnow = AP_HAL::millis();

        manual_override(plane.channel_roll, packet.y, 1000, 2000, tnow);
        manual_override(plane.channel_pitch, packet.x, 1000, 2000, tnow, true);
        manual_override(plane.channel_throttle, packet.z, 0, 1000, tnow);
        manual_override(plane.channel_rudder, packet.r, 1000, 2000, tnow);

        // a manual control message is considered to be a 'heartbeat' from the ground station for failsafe purposes
        plane.failsafe.last_heartbeat_ms = tnow;
        break;
    }
    
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from
        // our GCS for failsafe purposes
        if (msg.sysid != plane.g.sysid_my_gcs) break;
        plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
        break;
    }

    case MAVLINK_MSG_ID_HIL_STATE:
    {
#if HIL_SUPPORT
        if (plane.g.hil_mode != 1) {
            break;
        }

        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(&msg, &packet);

        // sanity check location
        if (!check_latlng(packet.lat, packet.lon)) {
            break;
        }

        last_hil_state = packet;

        // set gps hil sensor
        const Location loc{packet.lat, packet.lon, packet.alt/10, Location::AltFrame::ABSOLUTE};
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        // setup airspeed pressure based on 3D speed, no wind
        plane.airspeed.setHIL(sq(vel.length()) / 2.0f + 2013);

        plane.gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                         packet.time_usec/1000,
                         loc, vel, 10, 0);

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * GRAVITY_MSS*0.001f;
        accels.y = packet.yacc * GRAVITY_MSS*0.001f;
        accels.z = packet.zacc * GRAVITY_MSS*0.001f;

        plane.ins.set_gyro(0, gyros);
        plane.ins.set_accel(0, accels);

        plane.barometer.setHIL(packet.alt*0.001f);
        plane.compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
        plane.compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);

        // cope with DCM getting badly off due to HIL lag
        if (plane.g.hil_err_limit > 0 &&
            (fabsf(packet.roll - plane.ahrs.roll) > ToRad(plane.g.hil_err_limit) ||
             fabsf(packet.pitch - plane.ahrs.pitch) > ToRad(plane.g.hil_err_limit) ||
             wrap_PI(fabsf(packet.yaw - plane.ahrs.yaw)) > ToRad(plane.g.hil_err_limit))) {
            plane.ahrs.reset_attitude(packet.roll, packet.pitch, packet.yaw);
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        handle_radio_status(msg, plane.should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        plane.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
    {
        // Only allow companion computer (or other external controller) to
        // control attitude in GUIDED mode.  We DON'T want external control
        // in e.g., RTL, CICLE. Specifying a single mode for companion
        // computer control is more safe (even more so when using
        // FENCE_ACTION = 4 for geofence failures).
        if (plane.control_mode != &plane.mode_guided) { // don't screw up failsafes
            break; 
        }

        mavlink_set_attitude_target_t att_target;
        mavlink_msg_set_attitude_target_decode(&msg, &att_target);

        // Mappings: If any of these bits are set, the corresponding input should be ignored.
        // NOTE, when parsing the bits we invert them for easier interpretation but transport has them inverted
        // bit 1: body roll rate
        // bit 2: body pitch rate
        // bit 3: body yaw rate
        // bit 4: unknown
        // bit 5: unknown
        // bit 6: reserved
        // bit 7: throttle
        // bit 8: attitude

        // if not setting all Quaternion values, use _rate flags to indicate which fields.

        // Extract the Euler roll angle from the Quaternion.
        Quaternion q(att_target.q[0], att_target.q[1],
                att_target.q[2], att_target.q[3]);

        // NOTE: att_target.type_mask is inverted for easier interpretation
        att_target.type_mask = att_target.type_mask ^ 0xFF;

        uint8_t attitude_mask = att_target.type_mask & 0b10000111; // q plus rpy

        uint32_t now = AP_HAL::millis();
        if ((attitude_mask & 0b10000001) ||    // partial, including roll
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.x = degrees(q.get_euler_roll()) * 100.0f;

            // Update timer for external roll to the nav control
            plane.guided_state.last_forced_rpy_ms.x = now;
        }

        if ((attitude_mask & 0b10000010) ||    // partial, including pitch
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.y = degrees(q.get_euler_pitch()) * 100.0f;

            // Update timer for external pitch to the nav control
            plane.guided_state.last_forced_rpy_ms.y = now;
        }

        if ((attitude_mask & 0b10000100) ||    // partial, including yaw
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.z = degrees(q.get_euler_yaw()) * 100.0f;

            // Update timer for external yaw to the nav control
            plane.guided_state.last_forced_rpy_ms.z = now;
        }
        if (att_target.type_mask & 0b01000000) { // throttle
            plane.guided_state.forced_throttle = att_target.thrust * 100.0f;

            // Update timer for external throttle
            plane.guided_state.last_forced_throttle_ms = now;
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(&msg, &packet);
        Location new_home_loc {};
        new_home_loc.lat = packet.latitude;
        new_home_loc.lng = packet.longitude;
        new_home_loc.alt = packet.altitude / 10;
        if (!set_home(new_home_loc, false)) {
            // silently fails...
            break;
        }
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode
        if (plane.control_mode != &plane.mode_guided) {
            break;
        }

        // only local moves for now
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED) {
            break;
        }

        // just do altitude for now
        plane.next_WP_loc.alt += -packet.z*100.0;
        gcs().send_text(MAV_SEVERITY_INFO, "Change alt to %.1f",
                        (double)((plane.next_WP_loc.alt - plane.home.alt)*0.01));
        
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
    {
        // Only want to allow companion computer position control when
        // in a certain mode to avoid inadvertently sending these
        // kinds of commands when the autopilot is responding to problems
        // in modes such as RTL, CIRCLE, etc.  Specifying ONLY one mode
        // for companion computer control is more safe (provided
        // one uses the FENCE_ACTION = 4 (RTL) for geofence failures).
        if (plane.control_mode != &plane.mode_guided) {
            //don't screw up failsafes
            break;
        }

        mavlink_set_position_target_global_int_t pos_target;
        mavlink_msg_set_position_target_global_int_decode(&msg, &pos_target);
        // Unexpectedly, the mask is expecting "ones" for dimensions that should
        // be IGNORNED rather than INCLUDED.  See mavlink documentation of the
        // SET_POSITION_TARGET_GLOBAL_INT message, type_mask field.
        const uint16_t alt_mask = 0b1111111111111011; // (z mask at bit 3)
            
        bool msg_valid = true;
        AP_Mission::Mission_Command cmd = {0};
        
        if (pos_target.type_mask & alt_mask)
        {
            cmd.content.location.alt = pos_target.alt * 100;
            cmd.content.location.relative_alt = false;
            cmd.content.location.terrain_alt = false;
            switch (pos_target.coordinate_frame) 
            {
                case MAV_FRAME_GLOBAL:
                case MAV_FRAME_GLOBAL_INT:
                    break; //default to MSL altitude
                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    cmd.content.location.relative_alt = true;
                    break;
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    cmd.content.location.relative_alt = true;
                    cmd.content.location.terrain_alt = true;
                    break;
                default:
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid coord frame in SET_POSTION_TARGET_GLOBAL_INT");
                    msg_valid = false;
                    break;
            }    

            if (msg_valid) {
                handle_change_alt_request(cmd);
            }
        } // end if alt_mask       

        break;
    }

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
        plane.adsb.handle_message(chan, msg);
        break;

    default:
        handle_common_message(msg);
        break;
    } // end switch
} // end handle mavlink

// a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
void GCS_MAVLINK_Plane::handle_rc_channels_override(const mavlink_message_t &msg)
{
    plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
    GCS_MAVLINK::handle_rc_channels_override(msg);
}

void GCS_MAVLINK_Plane::handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg)
{
    plane.auto_state.next_wp_crosstrack = false;
    GCS_MAVLINK::handle_mission_set_current(mission, msg);
    if (plane.control_mode == &plane.mode_auto && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
        plane.mission.resume();
    }
}

uint64_t GCS_MAVLINK_Plane::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
#if AP_TERRAIN_AVAILABLE
            (plane.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            GCS_MAVLINK::capabilities());
}
