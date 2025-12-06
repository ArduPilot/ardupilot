#include "GCS_MAVLink_Plane.h"

#include "Plane.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_Airspeed/AP_Airspeed_config.h>
#include <AP_EFI/AP_EFI_config.h>

MAV_TYPE GCS_Plane::frame_type() const
{
#if HAL_QUADPLANE_ENABLED
    return plane.quadplane.get_mav_type();
#else
    return MAV_TYPE_FIXED_WING;
#endif
}

uint8_t GCS_MAVLINK_Plane::base_mode() const
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
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QACRO:
        _base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
#endif
    case Mode::Number::STABILIZE:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
#endif  // HAL_QUADPLANE_ENABLED
    case Mode::Number::CRUISE:
        _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case Mode::Number::AUTO:
    case Mode::Number::RTL:
    case Mode::Number::LOITER:
    case Mode::Number::THERMAL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::TAKEOFF:
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
#endif
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
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

    if (plane.g.stick_mixing != StickMixing::NONE && plane.control_mode != &plane.mode_initializing) {
        if ((plane.g.stick_mixing != StickMixing::VTOL_YAW) || (plane.control_mode == &plane.mode_auto)) {
            // all modes except INITIALISING have some form of manual
            // override if stick mixing is enabled
            _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        }
    }

    // we are armed if we are not initialising
    if (plane.control_mode != &plane.mode_initializing && plane.arming.is_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return _base_mode;
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

    float r = ahrs.get_roll_rad();
    float p = ahrs.get_pitch_rad();
    float y = ahrs.get_yaw_rad();

    if (!(plane.flight_option_enabled(FlightOptions::GCS_REMOVE_TRIM_PITCH))) {
        p -= radians(plane.g.pitch_trim);
    }

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.show_vtol_view()) {
        r = plane.quadplane.ahrs_view->roll;
        p = plane.quadplane.ahrs_view->pitch;
        y = plane.quadplane.ahrs_view->yaw;
    }
#endif

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

void GCS_MAVLINK_Plane::send_attitude_target() 
{
#if HAL_QUADPLANE_ENABLED
    // Check if the attitude target is valid for reporting
    const uint32_t now = AP_HAL::millis();
    if (now  - plane.quadplane.last_att_control_ms > 100) {
        return;
    }

    const Quaternion quat  = plane.quadplane.attitude_control->get_attitude_target_quat();
    const Vector3f& ang_vel = plane.quadplane.attitude_control->get_attitude_target_ang_vel();
    const float throttle = plane.quadplane.attitude_control->get_throttle_in();

    const float quat_out[4] {quat.q1, quat.q2, quat.q3, quat.q4};

    const uint16_t typemask = 0; 

    mavlink_msg_attitude_target_send(
        chan,
        now,                    // time since boot (ms)
        typemask,               // Bitmask that tells the system what control dimensions should be ignored by the vehicle
        quat_out,               // Target attitude quaternion [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], unit-length
        ang_vel.x,              // bodyframe target roll rate (rad/s)
        ang_vel.y,              // bodyframe target pitch rate (rad/s)
        ang_vel.z,              // bodyframe yaw rate (rad/s)
        throttle);              // Collective thrust, normalized to 0 .. 1

#endif // HAL_QUADPLANE_ENABLED 
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

void GCS_MAVLINK_Plane::send_nav_controller_output() const
{
    if (plane.control_mode == &plane.mode_manual) {
        return;
    }
#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.show_vtol_view() && quadplane.using_wp_nav()) {
        const Vector3f &targets = quadplane.attitude_control->get_att_target_euler_cd();

        const Vector2f& curr_pos_m = quadplane.inertial_nav.get_position_xy_cm() * 0.01;
        const Vector2f& target_pos_m = quadplane.pos_control->get_pos_target_NED_m().xy().tofloat();
        const Vector2f error_m = (target_pos_m - curr_pos_m);

        mavlink_msg_nav_controller_output_send(
            chan,
            targets.x * 0.01,
            targets.y * 0.01,
            targets.z * 0.01,
            degrees(error_m.angle()),
            MIN(error_m.length(), UINT16_MAX),
            (plane.control_mode != &plane.mode_qstabilize) ? -quadplane.pos_control->get_pos_error_D_m() : 0,
            plane.airspeed_error * 100,  // incorrect units; see PR#7933
            quadplane.wp_nav->crosstrack_error_m());
        return;
    }
#endif
    {
        const AP_Navigation *nav_controller = plane.nav_controller;
        mavlink_msg_nav_controller_output_send(
            chan,
            plane.nav_roll_cd * 0.01,
            plane.nav_pitch_cd * 0.01,
            nav_controller->nav_bearing_cd() * 0.01,
            nav_controller->target_bearing_cd() * 0.01,
            MIN(plane.auto_state.wp_distance, UINT16_MAX),
            plane.calc_altitude_error_cm() * 0.01,
            plane.airspeed_error * 100,  // incorrect units; see PR#7933
            nav_controller->crosstrack_error());
    }
}

void GCS_MAVLINK_Plane::send_position_target_global_int()
{
    if (plane.control_mode == &plane.mode_manual) {
        return;
    }
    Location &next_WP_loc = plane.next_WP_loc;
    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;
    int32_t alt = 0;
    if (!next_WP_loc.is_zero()) {
        UNUSED_RESULT(next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt));
    }

    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL, // targets are always global altitude
        TYPE_MASK, // ignore everything except the x/y/z components
        next_WP_loc.lat, // latitude as 1e7
        next_WP_loc.lng, // longitude as 1e7
        alt * 0.01, // altitude is sent as a float
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
}


float GCS_MAVLINK_Plane::vfr_hud_airspeed() const
{
    // airspeed sensors are best.  While the AHRS airspeed_estimate
    // will use an airspeed sensor, that value is constrained by the
    // ground speed.  When reporting we should send the true airspeed
    // value if possible:
#if AP_AIRSPEED_ENABLED
    if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
        return plane.airspeed.get_airspeed();
    }
#endif

    // airspeed estimates are OK:
    float aspeed;
    if (AP::ahrs().airspeed_EAS(aspeed)) {
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
#if HAL_SOARING_ENABLED
    if (plane.g2.soaring_controller.is_active()) {
        return plane.g2.soaring_controller.get_vario_reading();
    }
#endif
    return GCS_MAVLINK::vfr_hud_climbrate();
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
void GCS_MAVLINK_Plane::send_pid_info(const AP_PIDInfo *pid_info,
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
                                 pid_info->D,
                                 pid_info->slew_rate,
                                 pid_info->Dmod);
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

    const AP_PIDInfo *pid_info;
    if (g.gcs_pid_mask & TUNING_BITS_ROLL) {
        pid_info = &plane.rollController.get_pid_info();
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        }
#endif
        send_pid_info(pid_info, PID_TUNING_ROLL, pid_info->actual);
    }
    if (g.gcs_pid_mask & TUNING_BITS_PITCH) {
        pid_info = &plane.pitchController.get_pid_info();
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        }
#endif
        send_pid_info(pid_info, PID_TUNING_PITCH, pid_info->actual);
    }
    if (g.gcs_pid_mask & TUNING_BITS_YAW) {
        pid_info = &plane.yawController.get_pid_info();
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        }
#endif
        send_pid_info(pid_info, PID_TUNING_YAW, pid_info->actual);
    }
    if (g.gcs_pid_mask & TUNING_BITS_STEER) {
        pid_info = &plane.steerController.get_pid_info();
        send_pid_info(pid_info, PID_TUNING_STEER, pid_info->actual);
    }
    if ((g.gcs_pid_mask & TUNING_BITS_LAND) && (plane.flight_stage == AP_FixedWing::FlightStage::LAND)) {
        AP_AHRS &ahrs = AP::ahrs();
        const Vector3f &gyro = ahrs.get_gyro();
        send_pid_info(plane.landing.get_pid_info(), PID_TUNING_LANDING, degrees(gyro.z));
    }
#if HAL_QUADPLANE_ENABLED
    if (g.gcs_pid_mask & TUNING_BITS_ACCZ && plane.quadplane.in_vtol_mode()) {
        pid_info = &plane.quadplane.pos_control->D_get_accel_pid().get_pid_info();
        send_pid_info(pid_info, PID_TUNING_ACCZ, pid_info->actual);
    }
#endif
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Plane::try_send_message(enum ap_message id)
{
    switch (id) {

#if AP_TERRAIN_AVAILABLE
    case MSG_TERRAIN_REQUEST:
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        plane.terrain.send_request(chan);
        break;
    case MSG_TERRAIN_REPORT:
        CHECK_PAYLOAD_SIZE(TERRAIN_REPORT);
        plane.terrain.send_report(chan);
        break;
#endif

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_ADSB_VEHICLE:
#if HAL_ADSB_ENABLED
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        plane.adsb.send_adsb_vehicle(chan);
#endif
        break;

    case MSG_AOA_SSA:
        CHECK_PAYLOAD_SIZE(AOA_SSA);
        send_aoa_ssa();
        break;

#if HAL_LANDING_DEEPSTALL_ENABLED
    case MSG_LANDING:
        plane.landing.send_landing_message(chan);
        break;
#endif  // HAL_LANDING_DEEPSTALL_ENABLED

    case MSG_HYGROMETER:
#if AP_AIRSPEED_HYGROMETER_ENABLE
        CHECK_PAYLOAD_SIZE(HYGROMETER_SENSOR);
        send_hygrometer();
#endif
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

#if AP_AIRSPEED_HYGROMETER_ENABLE
void GCS_MAVLINK_Plane::send_hygrometer()
{
    if (!HAVE_PAYLOAD_SPACE(chan, HYGROMETER_SENSOR)) {
        return;
    }

    const auto *airspeed = AP::airspeed();
    if (airspeed == nullptr) {
        return;
    } 
    const uint32_t now = AP_HAL::millis();

    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        uint8_t idx = (i+last_hygrometer_send_idx+1) % AIRSPEED_MAX_SENSORS;
        float temperature, humidity;
        uint32_t last_sample_ms;
        if (!airspeed->get_hygrometer(idx, last_sample_ms, temperature, humidity)) {
            continue;
        }
        if (now - last_sample_ms > 2000) {
            // not updating, stop sending
            continue;
        }
        if (!HAVE_PAYLOAD_SPACE(chan, HYGROMETER_SENSOR)) {
            return;
        }

        mavlink_msg_hygrometer_sensor_send(
            chan,
            idx,
            int16_t(temperature*100),
            uint16_t(humidity*100));
        last_hygrometer_send_idx = idx;
    }
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE

/*
  handle a request to switch to guided mode. This happens via a
  callback from handle_mission_item()
 */
bool GCS_MAVLINK_Plane::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    return plane.control_mode->handle_guided_request(cmd.content.location);
}

/*
  handle a request to change current WP altitude. This happens via a
  callback from handle_mission_item()
 */
void GCS_MAVLINK_Plane::handle_change_alt_request(Location &location)
{
    plane.fix_terrain_WP(location, __LINE__);

    if (location.terrain_alt) {
        plane.next_WP_loc.copy_alt_from(location);
    } else {
        // convert to absolute alt
        float abs_alt_m;
        if (location.get_alt_m(Location::AltFrame::ABSOLUTE, abs_alt_m)) {
            plane.next_WP_loc.set_alt_m(abs_alt_m, Location::AltFrame::ABSOLUTE);
        }
    }
    plane.reset_offset_altitude();
}


/*
  handle a LANDING_TARGET command. The timestamp has been jitter corrected
*/
void GCS_MAVLINK_Plane::handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
#if AC_PRECLAND_ENABLED
    plane.g2.precland.handle_msg(packet, timestamp_ms);
#endif
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    plane.in_calibration = true;
    MAV_RESULT ret = GCS_MAVLINK::handle_command_preflight_calibration(packet, msg);
    plane.in_calibration = false;

    return ret;
}

void GCS_MAVLINK_Plane::packetReceived(const mavlink_status_t &status,
                                       const mavlink_message_t &msg)
{
#if AP_ADSB_AVOIDANCE_ENABLED
    plane.avoidance_adsb.handle_msg(msg);
#endif
    GCS_MAVLINK::packetReceived(status, msg);
}


bool Plane::set_home_to_current_location(bool _lock)
{
    if (!set_home_persistently(AP::gps().location())) {
        return false;
    }
    if (_lock) {
        AP::ahrs().lock_home();
    }
    if ((control_mode == &mode_rtl)
#if HAL_QUADPLANE_ENABLED
            || (control_mode == &mode_qrtl)
#endif
                                                        ) {
        // if in RTL head to the updated home location
        control_mode->enter();
    }
    return true;
}
bool Plane::set_home(const Location& loc, bool _lock)
{
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }
    if (_lock) {
        AP::ahrs().lock_home();
    }
    if ((control_mode == &mode_rtl)
#if HAL_QUADPLANE_ENABLED
            || (control_mode == &mode_qrtl)
#endif
                                                        ) {
        // if in RTL head to the updated home location
        control_mode->enter();
    }
    return true;
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location requested_position;
    if (!location_from_command_t(packet, requested_position)) {
        return MAV_RESULT_DENIED;
    }
    plane.fix_terrain_WP(requested_position, __LINE__);

    if (isnan(packet.param4) || is_zero(packet.param4)) {
        requested_position.loiter_ccw = 0;
    } else {
        requested_position.loiter_ccw = 1;
    }

    if (requested_position.sanitize(plane.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED;
    }

#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    if (!plane.fence.check_destination_within_fence(requested_position)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        return MAV_RESULT_DENIED;
    }
#endif

    // location is valid load and set
    if (((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) ||
        (plane.control_mode == &plane.mode_guided)) {
        plane.set_mode(plane.mode_guided, ModeReason::GCS_COMMAND);
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        plane.guided_state.target_heading_type = GUIDED_HEADING_NONE;
#endif

        // convert to absolute alt
        if (!requested_position.terrain_alt) {
            requested_position.change_alt_frame(Location::AltFrame::ABSOLUTE);
        }

        plane.set_guided_WP(requested_position);

        // Loiter radius for planes. Positive radius in meters, direction is controlled by Yaw (param4) value, parsed above
        if (!isnan(packet.param3) && packet.param3 > 0) {
            plane.mode_guided.set_radius_and_direction(packet.param3, requested_position.loiter_ccw);
        }

        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_DO_CHANGE_ALTITUDE(const mavlink_command_int_t &packet)
{
    const float alt = packet.param1;
    MAV_FRAME mav_frame = (MAV_FRAME)packet.param2;
    Location::AltFrame alt_frame;
    if (!mavlink_coordinate_frame_to_location_alt_frame(mav_frame, alt_frame)) {
        return MAV_RESULT_DENIED;
    }
    Location loc {
        0,
        0,
        int32_t(alt * 100),  // m -> cm
        alt_frame,
    };
    handle_change_alt_request(loc);
    return MAV_RESULT_ACCEPTED;
}

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
// these are GUIDED mode commands that are RATE or slew enabled, so you can have more powerful control than default controls.
MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_guided_slew_commands(const mavlink_command_int_t &packet)
{
  switch(packet.command) {
    case MAV_CMD_GUIDED_CHANGE_SPEED: {
        // command is only valid in guided mode
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

        // only airspeed commands are supported right now...
        if (int(packet.param1) != SPEED_TYPE_AIRSPEED) {  // since SPEED_TYPE is int in range 0-1 and packet.param1 is a *float* this works.
            return MAV_RESULT_DENIED;
        }

        if (!plane.mode_guided.handle_change_airspeed(packet.param2, packet.param3)) {
            return MAV_RESULT_FAILED;
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

        Location::AltFrame new_target_alt_frame;
        if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.frame, new_target_alt_frame)) {
            return MAV_RESULT_DENIED;
        }
        // keep a copy of what came in via MAVLink - this is needed for logging, but not for anything else
        plane.guided_state.target_mav_frame = packet.frame;

        const int32_t new_target_alt_cm = packet.z * 100;
        plane.guided_state.target_location.set_alt_cm(new_target_alt_cm, new_target_alt_frame); 
        plane.guided_state.target_alt_time_ms = AP_HAL::millis();

        // param3 contains the desired vertical velocity (not acceleration)
        if (is_zero(packet.param3)) {
            // the user wanted /maximum altitude change rate, pick a large value as close enough
            plane.guided_state.target_alt_rate = 1000.0;
        } else {
            plane.guided_state.target_alt_rate = fabsf(packet.param3);
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

        switch(HEADING_TYPE(packet.param1)) {
        case HEADING_TYPE_COURSE_OVER_GROUND:
            // course over ground
            plane.guided_state.target_heading_type = GUIDED_HEADING_COG;
            plane.prev_WP_loc = plane.current_loc;
            break;
        case HEADING_TYPE_HEADING:
            // normal vehicle heading
            plane.guided_state.target_heading_type = GUIDED_HEADING_HEADING;
            break;
        case HEADING_TYPE_DEFAULT:
            plane.guided_state.target_heading_type = GUIDED_HEADING_NONE;
            return MAV_RESULT_ACCEPTED;
        default:
            //  MAV_RESULT_DENIED  means Command is invalid (is supported but has invalid parameters).
            return MAV_RESULT_DENIED;
        }

        plane.g2.guidedHeading.reset_I();

        plane.guided_state.target_heading = new_target_heading;
        plane.guided_state.target_heading_accel_limit = MAX(packet.param3, 0.05f);
        plane.guided_state.target_heading_time_ms = AP_HAL::millis();
        return MAV_RESULT_ACCEPTED;
    }
  }
  // anything else ...
  return MAV_RESULT_UNSUPPORTED;
}
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch(packet.command) {

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        return handle_MAV_CMD_DO_AUTOTUNE_ENABLE(packet);

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    case MAV_CMD_DO_CHANGE_ALTITUDE:
        return handle_command_int_DO_CHANGE_ALTITUDE(packet);

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // special 'slew-enabled' guided commands here... for speed,alt, and direction commands
    case MAV_CMD_GUIDED_CHANGE_SPEED:
    case MAV_CMD_GUIDED_CHANGE_ALTITUDE:
    case MAV_CMD_GUIDED_CHANGE_HEADING:
        return handle_command_int_guided_slew_commands(packet);
#endif

#if AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED
    case MAV_CMD_DO_FOLLOW:
        // param1: sysid of target to follow
        if ((packet.param1 > 0) && (packet.param1 <= 255)) {
            plane.g2.follow.set_target_sysid((uint8_t)packet.param1);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_DENIED;
#endif

#if AP_ICENGINE_ENABLED
    case MAV_CMD_DO_ENGINE_CONTROL:
        if (!plane.g2.ice_control.engine_control(packet.param1, packet.param2, packet.param3, (uint32_t)packet.param4)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
#endif

    case MAV_CMD_DO_CHANGE_SPEED:
        return handle_command_DO_CHANGE_SPEED(packet);

#if HAL_PARACHUTE_ENABLED
    case MAV_CMD_DO_PARACHUTE:
        return handle_MAV_CMD_DO_PARACHUTE(packet);
#endif

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_DO_MOTOR_TEST:
        return handle_MAV_CMD_DO_MOTOR_TEST(packet);

    case MAV_CMD_DO_VTOL_TRANSITION:
        return handle_command_DO_VTOL_TRANSITION(packet);

    case MAV_CMD_NAV_TAKEOFF:
        return handle_command_MAV_CMD_NAV_TAKEOFF(packet);
#endif

    case MAV_CMD_DO_GO_AROUND:
        return plane.trigger_land_abort(packet.param1) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;

    case MAV_CMD_DO_RETURN_PATH_START:
        // attempt to rejoin after the next DO_RETURN_PATH_START command in the mission
        if (plane.have_position && plane.mission.jump_to_closest_mission_leg(plane.current_loc)) {
            plane.mission.set_force_resume(true);
            if (plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND)) {
                return MAV_RESULT_ACCEPTED;
            }
            // mode change failed, revert force resume flag
            plane.mission.set_force_resume(false);
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_LAND_START:
        // attempt to switch to next DO_LAND_START command in the mission
        if (plane.have_position && plane.mission.jump_to_landing_sequence(plane.current_loc)) {
            plane.mission.set_force_resume(true);
            if (plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND)) {
                return MAV_RESULT_ACCEPTED;
            }
            // mode change failed, revert force resume flag
            plane.mission.set_force_resume(false);
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_MISSION_START:
        if (!is_zero(packet.param1) || !is_zero(packet.param2)) {
            // first-item/last item not supported
            return MAV_RESULT_DENIED;
        }
        plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_LOITER_UNLIM:
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
    case MAV_CMD_SET_HAGL:
        plane.handle_external_hagl(packet);
        return MAV_RESULT_ACCEPTED;
#endif
        
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_DO_CHANGE_SPEED(const mavlink_command_int_t &packet)
{
        // if we're in failsafe modes (e.g., RTL, LOITER) or in pilot
        // controlled modes (e.g., MANUAL, TRAINING)
        // this command should be ignored since it comes in from GCS
        // or a companion computer:
        if ((!plane.control_mode->is_guided_mode()) &&
            (plane.control_mode != &plane.mode_auto)) {
            // failed
            return MAV_RESULT_FAILED;
        }

        if (plane.do_change_speed((SPEED_TYPE)packet.param1, packet.param2, packet.param3)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
}

#if HAL_QUADPLANE_ENABLED
#if AP_MAVLINK_COMMAND_LONG_ENABLED
void GCS_MAVLINK_Plane::convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out)
{
    // convert to MAV_FRAME_LOCAL_OFFSET_NED, "NED local tangent frame
    // with origin that travels with the vehicle"
    out = {};
    out.target_system = in.target_system;
    out.target_component = in.target_component;
    out.frame = MAV_FRAME_LOCAL_OFFSET_NED;
    out.command = in.command;
    // out.current = 0;
    // out.autocontinue = 0;
    // out.param1 = in.param1;  // we only use the "z" parameter in this command:
    // out.param2 = in.param2;
    // out.param3 = in.param3;
    // out.param4 = in.param4;
    // out.x = 0;  // we don't handle positioning when doing takeoffs
    // out.y = 0;
    out.z = -in.param7;  // up -> down
}

void GCS_MAVLINK_Plane::convert_COMMAND_LONG_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out, MAV_FRAME frame)
{
    switch (in.command) {
    case MAV_CMD_NAV_TAKEOFF:
        convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT(in, out);
        return;
    }
    return GCS_MAVLINK::convert_COMMAND_LONG_to_COMMAND_INT(in, out, frame);
}
#endif  // AP_MAVLINK_COMMAND_LONG_ENABLED

MAV_RESULT GCS_MAVLINK_Plane::handle_command_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet)
{
    float takeoff_alt = packet.z;
    switch (packet.frame) {
    case MAV_FRAME_LOCAL_OFFSET_NED:  // "NED local tangent frame with origin that travels with the vehicle"
        takeoff_alt = -takeoff_alt;  // down -> up
        break;
    default:
        return MAV_RESULT_DENIED; // "is supported but has invalid parameters"
    }
    if (!plane.quadplane.available()) {
        return MAV_RESULT_FAILED;
    }
    if (!plane.quadplane.do_user_takeoff(takeoff_alt)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}
#endif

MAV_RESULT GCS_MAVLINK_Plane::handle_MAV_CMD_DO_AUTOTUNE_ENABLE(const mavlink_command_int_t &packet)
{
        // param1 : enable/disable
        plane.autotune_enable(!is_zero(packet.param1));
        return MAV_RESULT_ACCEPTED;
}

#if HAL_PARACHUTE_ENABLED
MAV_RESULT GCS_MAVLINK_Plane::handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet)
{
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
}
#endif


#if HAL_QUADPLANE_ENABLED
MAV_RESULT GCS_MAVLINK_Plane::handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet)
{
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
                                                        (uint8_t)packet.x);
}

MAV_RESULT GCS_MAVLINK_Plane::handle_command_DO_VTOL_TRANSITION(const mavlink_command_int_t &packet)
{
        if (!plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)packet.param1)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}
#endif

// this is called on receipt of a MANUAL_CONTROL packet and is
// expected to call manual_override to override RC input on desired
// axes.
void GCS_MAVLINK_Plane::handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow)
{
    manual_override(plane.channel_roll, packet.y, 1000, 2000, tnow);
    manual_override(plane.channel_pitch, packet.x, 1000, 2000, tnow, true);
    manual_override(plane.channel_throttle, packet.z, 0, 1000, tnow);
    manual_override(plane.channel_rudder, packet.r, 1000, 2000, tnow);
}

void GCS_MAVLINK_Plane::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        plane.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        handle_set_attitude_target(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        handle_set_position_target_local_ned(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        handle_set_position_target_global_int(msg);
        break;

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    } // end switch
} // end handle mavlink

void GCS_MAVLINK_Plane::handle_set_attitude_target(const mavlink_message_t &msg)
    {
        // Only allow companion computer (or other external controller) to
        // control attitude in GUIDED mode.  We DON'T want external control
        // in e.g., RTL, CICLE. Specifying a single mode for companion
        // computer control is more safe (even more so when using
        // FENCE_ACTION = 4 for geofence failures).
        if (plane.control_mode != &plane.mode_guided) { // don't screw up failsafes
            return;
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
    }

void GCS_MAVLINK_Plane::handle_set_position_target_local_ned(const mavlink_message_t &msg)
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode
        if (plane.control_mode != &plane.mode_guided) {
            return;
        }

        // only local moves for now
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED) {
            return;
        }

        // just do altitude for now
        plane.next_WP_loc.alt += -packet.z*100.0;
        gcs().send_text(MAV_SEVERITY_INFO, "Change alt to %.1f",
                        (double)((plane.next_WP_loc.alt - plane.home.alt)*0.01));
    }

void GCS_MAVLINK_Plane::handle_set_position_target_global_int(const mavlink_message_t &msg)
    {
        // Only want to allow companion computer position control when
        // in a certain mode to avoid inadvertently sending these
        // kinds of commands when the autopilot is responding to problems
        // in modes such as RTL, CIRCLE, etc.  Specifying ONLY one mode
        // for companion computer control is more safe (provided
        // one uses the FENCE_ACTION = 4 (RTL) for geofence failures).
        if (plane.control_mode != &plane.mode_guided) {
            //don't screw up failsafes
            return;
        }

        mavlink_set_position_target_global_int_t pos_target;
        mavlink_msg_set_position_target_global_int_decode(&msg, &pos_target);

        Location::AltFrame frame;
        if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)pos_target.coordinate_frame, frame)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Invalid coord frame in SET_POSITION_TARGET_GLOBAL_INT");
            // Even though other parts of the command may be valid, reject the whole thing.
            return;
        }

        // Unexpectedly, the mask is expecting "ones" for dimensions that should
        // be IGNORED rather than INCLUDED.  See mavlink documentation of the
        // SET_POSITION_TARGET_GLOBAL_INT message, type_mask field.
        const bool alt_ignore = (pos_target.type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE);
        if (!alt_ignore) {
            Location loc {
                0,  // lat
                0,  // lng
                int32_t(pos_target.alt * 100),  // m -> cm
                frame,
            };
            handle_change_alt_request(loc);
        }
    }

MAV_RESULT GCS_MAVLINK_Plane::handle_command_do_set_mission_current(const mavlink_command_int_t &packet)
{
    const MAV_RESULT result = GCS_MAVLINK::handle_command_do_set_mission_current(packet);
    if (result != MAV_RESULT_ACCEPTED) {
        return result;
    }

    // if you change this you must change handle_mission_set_current
    plane.auto_state.next_wp_crosstrack = false;
    if (plane.control_mode == &plane.mode_auto && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
        plane.mission.resume();
    }

    return result;
}

#if AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
void GCS_MAVLINK_Plane::handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg)
{
    // if you change this you must change handle_command_do_set_mission_current
    plane.auto_state.next_wp_crosstrack = false;
    GCS_MAVLINK::handle_mission_set_current(mission, msg);
    if (plane.control_mode == &plane.mode_auto && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
        plane.mission.resume();
    }
}
#endif

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

#if HAL_HIGH_LATENCY2_ENABLED
int16_t GCS_MAVLINK_Plane::high_latency_target_altitude() const
{
    AP_AHRS &ahrs = AP::ahrs();
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));

#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    //return units are m
    if (quadplane.show_vtol_view()) {
        return (plane.control_mode != &plane.mode_qstabilize) ? (global_position_current.alt * 0.01 - quadplane.pos_control->get_pos_error_D_m()) : 0;
    }
#endif
    return 0.01 * (global_position_current.alt + plane.calc_altitude_error_cm());
}

uint8_t GCS_MAVLINK_Plane::high_latency_tgt_heading() const
{
    // return units are deg/2
#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.show_vtol_view()) {
        const Vector3f &targets = quadplane.attitude_control->get_att_target_euler_cd();
        return ((uint16_t)(targets.z * 0.01)) / 2;
    }
#endif
        const AP_Navigation *nav_controller = plane.nav_controller;
        // need to convert -18000->18000 to 0->360/2
        return wrap_360_cd(nav_controller->target_bearing_cd() ) / 200;
}

// return units are dm
uint16_t GCS_MAVLINK_Plane::high_latency_tgt_dist() const
{
#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.show_vtol_view()) {
        bool wp_nav_valid = quadplane.using_wp_nav();
        return (wp_nav_valid ? MIN(quadplane.wp_nav->get_wp_distance_to_destination_cm(), UINT16_MAX) : 0) / 10;
    }
    #endif

    return MIN(plane.auto_state.wp_distance, UINT16_MAX) / 10;
}

uint8_t GCS_MAVLINK_Plane::high_latency_tgt_airspeed() const
{
    // return units are m/s*5
    return plane.target_airspeed_cm * 0.05;
}

uint8_t GCS_MAVLINK_Plane::high_latency_wind_speed() const
{
    Vector3f wind;
    wind = AP::ahrs().wind_estimate();

    // return units are m/s*5
    return MIN(wind.length() * 5, UINT8_MAX);
}

uint8_t GCS_MAVLINK_Plane::high_latency_wind_direction() const
{
    const Vector3f wind = AP::ahrs().wind_estimate();

    // return units are deg/2
    // need to convert -180->180 to 0->360/2
    return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

MAV_VTOL_STATE GCS_MAVLINK_Plane::vtol_state() const
{
#if !HAL_QUADPLANE_ENABLED
    return MAV_VTOL_STATE_UNDEFINED;
#else
    if (!plane.quadplane.available()) {
        return MAV_VTOL_STATE_UNDEFINED;
    }

    return plane.quadplane.transition->get_mav_vtol_state();
#endif
};

MAV_LANDED_STATE GCS_MAVLINK_Plane::landed_state() const
{
    if (plane.is_flying()) {
        if (plane.is_taking_off()) {
            return MAV_LANDED_STATE_TAKEOFF;
        }
        if (plane.is_landing()) {
            return MAV_LANDED_STATE_LANDING;
        }

        // note that Q-modes almost always consider themselves as flying
        return MAV_LANDED_STATE_IN_AIR;
    }

    return MAV_LANDED_STATE_ON_GROUND;
}

// Send the mode with the given index (not mode number!) return the total number of modes
// Index starts at 1
uint8_t GCS_MAVLINK_Plane::send_available_mode(uint8_t index) const
{
    // Fixed wing modes
    const Mode* fw_modes[] {
        &plane.mode_manual,
        &plane.mode_circle,
        &plane.mode_stabilize,
        &plane.mode_training,
        &plane.mode_acro,
        &plane.mode_fbwa,
        &plane.mode_fbwb,
        &plane.mode_cruise,
        &plane.mode_autotune,
        &plane.mode_auto,
        &plane.mode_rtl,
        &plane.mode_loiter,
#if HAL_ADSB_ENABLED
        &plane.mode_avoidADSB,
#endif
        &plane.mode_guided,
        &plane.mode_initializing,
        &plane.mode_takeoff,
#if HAL_SOARING_ENABLED
        &plane.mode_thermal,
#endif
#if MODE_AUTOLAND_ENABLED
        &plane.mode_autoland,
#endif
    };

    const uint8_t fw_mode_count = ARRAY_SIZE(fw_modes);

    // Fixedwing modes are always present
    uint8_t mode_count = fw_mode_count;

#if HAL_QUADPLANE_ENABLED
    // Quadplane modes
    const Mode* q_modes[] {
        &plane.mode_qstabilize,
        &plane.mode_qhover,
        &plane.mode_qloiter,
        &plane.mode_qland,
        &plane.mode_qrtl,
        &plane.mode_qacro,
        &plane.mode_loiter_qland,
#if QAUTOTUNE_ENABLED
        &plane.mode_qautotune,
#endif
    };

    // Quadplane modes must be enabled
    if (plane.quadplane.available()) {
        mode_count += ARRAY_SIZE(q_modes);
    }
#endif // HAL_QUADPLANE_ENABLED


    // Convert to zero indexed
    const uint8_t index_zero = index - 1;
    if (index_zero >= mode_count) {
        // Mode does not exist!?
        return mode_count;
    }

    // Ask the mode for its name and number
    const char* name;
    uint8_t mode_number;

    if (index_zero < fw_mode_count) {
        // A fixedwing mode
        name = fw_modes[index_zero]->name();
        mode_number = (uint8_t)fw_modes[index_zero]->mode_number();

    } else {
#if HAL_QUADPLANE_ENABLED
        // A Quadplane mode
        const uint8_t q_index = index_zero - fw_mode_count;
        name = q_modes[q_index]->name();
        mode_number = (uint8_t)q_modes[q_index]->mode_number();
#else
        // Should not endup here
        return mode_count;
#endif
    }

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
