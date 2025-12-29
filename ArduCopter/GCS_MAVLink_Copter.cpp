#include "Copter.h"

#include "GCS_MAVLink_Copter.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_EFI/AP_EFI_config.h>

MAV_TYPE GCS_Copter::frame_type() const
{
    /*
      for GCS don't give MAV_TYPE_GENERIC as the GCS would have no
      information and won't display UIs such as flight mode
      selection
    */
#if FRAME_CONFIG == HELI_FRAME
    const MAV_TYPE mav_type_default = MAV_TYPE_HELICOPTER;
#else
    const MAV_TYPE mav_type_default = MAV_TYPE_QUADROTOR;
#endif
    if (copter.motors == nullptr) {
        return mav_type_default;
    }
    MAV_TYPE mav_type = copter.motors->get_frame_mav_type();
    if (mav_type == MAV_TYPE_GENERIC) {
        mav_type = mav_type_default;
    }
    return mav_type;
}

uint8_t GCS_MAVLINK_Copter::base_mode() const
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
    if ((copter.pos_control != nullptr) && copter.pos_control->NE_is_active()) {
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
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

    return _base_mode;
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

    if (!copter.ap.initialised) {
    	return MAV_STATE_BOOT;
    }

    return MAV_STATE_ACTIVE;
}


void GCS_MAVLINK_Copter::send_attitude_target()
{
    const Quaternion quat  = copter.attitude_control->get_attitude_target_quat();
    const Vector3f ang_vel = copter.attitude_control->get_attitude_target_ang_vel();
    const float thrust = copter.attitude_control->get_throttle_in();

    const float quat_out[4] {quat.q1, quat.q2, quat.q3, quat.q4};

    // Note: When sending out the attitude_target info. we send out all of info. no matter the mavlink typemask
    // This way we send out the maximum information that can be used by the sending control systems to adapt their generated trajectories
    const uint16_t typemask = 0;    // Ignore nothing

    mavlink_msg_attitude_target_send(
        chan,
        AP_HAL::millis(),       // time since boot (ms)
        typemask,               // Bitmask that tells the system what control dimensions should be ignored by the vehicle
        quat_out,               // Attitude quaternion [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], unit-length
        ang_vel.x,              // roll rate (rad/s)
        ang_vel.y,              // pitch rate (rad/s)
        ang_vel.z,              // yaw rate (rad/s)
        thrust);                // Collective thrust, normalized to 0 .. 1
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
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;
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
#if MODE_GUIDED_ENABLED
    if (!copter.flightmode->in_guided_mode()) {
        return;
    }

    const ModeGuided::SubMode guided_mode = copter.mode_guided.submode();
    Vector3f target_pos_ned_m;
    Vector3f target_vel_ned_ms;
    Vector3f target_accel_ned_mss;
    uint16_t type_mask = 0;

    switch (guided_mode) {
    case ModeGuided::SubMode::Angle:
        // we don't have a local target when in angle mode
        return;
    case ModeGuided::SubMode::TakeOff:
    case ModeGuided::SubMode::WP:
    case ModeGuided::SubMode::Pos:
        type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position
        target_pos_ned_m = copter.mode_guided.get_target_pos_NED_m().tofloat();
        break;
    case ModeGuided::SubMode::PosVelAccel:
        type_mask = POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position, velocity & acceleration
        target_pos_ned_m = copter.mode_guided.get_target_pos_NED_m().tofloat();
        target_vel_ned_ms = copter.mode_guided.get_target_vel_NED_ms();
        target_accel_ned_mss = copter.mode_guided.get_target_accel_NED_mss();
        break;
    case ModeGuided::SubMode::VelAccel:
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except velocity & acceleration
        target_vel_ned_ms = copter.mode_guided.get_target_vel_NED_ms();
        target_accel_ned_mss = copter.mode_guided.get_target_accel_NED_mss();
        break;
    case ModeGuided::SubMode::Accel:
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except velocity & acceleration
        target_accel_ned_mss = copter.mode_guided.get_target_accel_NED_mss();
        break;
    }

    mavlink_msg_position_target_local_ned_send(
        chan,
        AP_HAL::millis(), // time boot ms
        MAV_FRAME_LOCAL_NED, 
        type_mask,
        target_pos_ned_m.x,     // x in metres
        target_pos_ned_m.y,     // y in metres
        target_pos_ned_m.z,     // z in metres NED frame
        target_vel_ned_ms.x,    // vx in m/s
        target_vel_ned_ms.y,    // vy in m/s
        target_vel_ned_ms.z,    // vz in m/s NED frame
        target_accel_ned_mss.x, // afx in m/s/s
        target_accel_ned_mss.y, // afy in m/s/s
        target_accel_ned_mss.z, // afz in m/s/s NED frame
        0.0f,                   // yaw
        0.0f);                  // yaw_rate
#endif
}

void GCS_MAVLINK_Copter::send_nav_controller_output() const
{
    if (!copter.ap.initialised) {
        return;
    }
    const Vector3f &targets_rad = copter.attitude_control->get_att_target_euler_rad();
    const Mode *flightmode = copter.flightmode;
    mavlink_msg_nav_controller_output_send(
        chan,
        degrees(targets_rad.x),
        degrees(targets_rad.y),
        degrees(targets_rad.z),
        flightmode->wp_bearing_deg(),
        MIN(flightmode->wp_distance_m(), UINT16_MAX),
        copter.pos_control->get_pos_error_D_m(),
        0,
        flightmode->crosstrack_error_m());
}

float GCS_MAVLINK_Copter::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    // airspeed sensors are best. While the AHRS airspeed_estimate
    // will use an airspeed sensor, that value is constrained by the
    // ground speed. When reporting we should send the true airspeed
    // value if possible:
    if (copter.airspeed.enabled() && copter.airspeed.healthy()) {
        return copter.airspeed.get_airspeed();
    }
#endif
    
    Vector3f airspeed_vec_bf;
    if (AP::ahrs().airspeed_vector_TAS(airspeed_vec_bf)) {
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
        const AP_PIDInfo *pid_info = nullptr;
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
            pid_info = &copter.pos_control->D_get_accel_pid().get_pid_info();
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
                                        pid_info->D,
                                        pid_info->slew_rate,
                                        pid_info->Dmod);
        }
    }
}

#if AP_WINCH_ENABLED
// send winch status message
void GCS_MAVLINK_Copter::send_winch_status() const
{
    AP_Winch *winch = AP::winch();
    if (winch == nullptr) {
        return;
    }
    winch->send_status(*this);
}
#endif

bool GCS_Copter::vehicle_initialised() const {
    return copter.ap.initialised;
}

// try to send a message, return false if it wasn't sent
bool GCS_MAVLINK_Copter::try_send_message(enum ap_message id)
{
    switch(id) {

#if AP_TERRAIN_AVAILABLE
    case MSG_TERRAIN_REQUEST:
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        copter.terrain.send_request(chan);
        break;
    case MSG_TERRAIN_REPORT:
        CHECK_PAYLOAD_SIZE(TERRAIN_REPORT);
        copter.terrain.send_report(chan);
        break;
#endif

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_ADSB_VEHICLE: {
#if HAL_ADSB_ENABLED
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        copter.adsb.send_adsb_vehicle(chan);
#endif
#if AP_OAPATHPLANNER_ENABLED
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


MISSION_STATE GCS_MAVLINK_Copter::mission_state(const class AP_Mission &mission) const
{
    if (copter.mode_auto.paused()) {
        return MISSION_STATE_PAUSED;
    }
    return GCS_MAVLINK::mission_state(mission);
}

bool GCS_MAVLINK_Copter::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
#if MODE_AUTO_ENABLED
    return copter.mode_auto.do_guided(cmd);
#else
    return false;
#endif
}

void GCS_MAVLINK_Copter::packetReceived(const mavlink_status_t &status,
                                        const mavlink_message_t &msg)
{
    // we handle these messages here to avoid them being blocked by mavlink routing code
#if AP_ADSB_AVOIDANCE_ENABLED
    if (copter.g2.dev_options.get() & DevOptionADSBMAVLink) {
        // optional handling of GLOBAL_POSITION_INT as a MAVLink based avoidance source
        copter.avoidance_adsb.handle_msg(msg);
    }
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
#if AC_PRECLAND_ENABLED
    copter.precland.handle_msg(packet, timestamp_ms);
#endif
}

MAV_RESULT GCS_MAVLINK_Copter::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    if (packet.y == 1) {
        // compassmot calibration
        return copter.mavlink_compassmot(*this);
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}


MAV_RESULT GCS_MAVLINK_Copter::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    copter.flightmode->auto_yaw.set_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Copter::handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    // reject reboot if user has also specified they want the "Auto" ESC calibration on next reboot
    if (copter.g.esc_calibrate == (uint8_t)Copter::ESCCalibrationModes::ESCCAL_AUTO) {
        send_text(MAV_SEVERITY_CRITICAL, "Reboot rejected, ESC cal on reboot");
        return MAV_RESULT_FAILED;
    }

    // call parent
    return GCS_MAVLINK::handle_preflight_reboot(packet, msg);
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
#if MODE_GUIDED_ENABLED
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!copter.flightmode->in_guided_mode() && !change_modes) {
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
#else
    return MAV_RESULT_UNSUPPORTED;
#endif
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch(packet.command) {

    case MAV_CMD_CONDITION_YAW:
        return handle_MAV_CMD_CONDITION_YAW(packet);

    case MAV_CMD_DO_CHANGE_SPEED:
        return handle_MAV_CMD_DO_CHANGE_SPEED(packet);

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    // pause or resume an auto mission
    case MAV_CMD_DO_PAUSE_CONTINUE:
        return handle_command_pause_continue(packet);

    case MAV_CMD_DO_MOTOR_TEST:
        return handle_MAV_CMD_DO_MOTOR_TEST(packet);

    case MAV_CMD_NAV_TAKEOFF:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return handle_MAV_CMD_NAV_TAKEOFF(packet);

#if HAL_PARACHUTE_ENABLED
    case MAV_CMD_DO_PARACHUTE:
        return handle_MAV_CMD_DO_PARACHUTE(packet);
#endif

#if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
    // Solo user presses pause button
    case MAV_CMD_SOLO_BTN_PAUSE_CLICK:
        return handle_MAV_CMD_SOLO_BTN_PAUSE_CLICK(packet);
    // Solo user presses Fly button:
    case MAV_CMD_SOLO_BTN_FLY_HOLD:
        return handle_MAV_CMD_SOLO_BTN_FLY_HOLD(packet);
    // Solo user holds down Fly button for a couple of seconds
    case MAV_CMD_SOLO_BTN_FLY_CLICK:
        return handle_MAV_CMD_SOLO_BTN_FLY_CLICK(packet);
#endif

#if MODE_AUTO_ENABLED
    case MAV_CMD_MISSION_START:
        return handle_MAV_CMD_MISSION_START(packet);
#endif

#if AP_WINCH_ENABLED
    case MAV_CMD_DO_WINCH:
        return handle_MAV_CMD_DO_WINCH(packet);
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

    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        if (!copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

#if MODE_AUTO_ENABLED
    case MAV_CMD_DO_RETURN_PATH_START:
        if (copter.mode_auto.return_path_start_auto_RTL(ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_LAND_START:
        if (copter.mode_auto.jump_to_landing_sequence_auto_RTL(ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

#if HAL_MOUNT_ENABLED
MAV_RESULT GCS_MAVLINK_Copter::handle_command_mount(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_DO_MOUNT_CONTROL:
        // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
        if (((MAV_MOUNT_MODE)packet.z == MAV_MOUNT_MODE_MAVLINK_TARGETING) &&
            (copter.camera_mount.get_mount_type() != AP_Mount::Type::None) &&
            !copter.camera_mount.has_pan_control()) {
            // Per the handler in AP_Mount, DO_MOUNT_CONTROL yaw angle is in body frame, which is
            // equivalent to an offset to the current yaw demand.
            copter.flightmode->auto_yaw.set_yaw_angle_offset_deg(packet.param3);
        }
        break;
    default:
        break;
    }
    return GCS_MAVLINK::handle_command_mount(packet, msg);
}
#endif

MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet)
{
    if (packet.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT) {
        return MAV_RESULT_DENIED;  // meaning some parameters are bad
    }

        // param3 : horizontal navigation by pilot acceptable
        // param4 : yaw angle   (not supported)
        // param5 : latitude    (not supported)
        // param6 : longitude   (not supported)
        // param7 : altitude [metres]

        float takeoff_alt_m = packet.z;

        if (!copter.flightmode->do_user_takeoff_U_m(takeoff_alt_m, is_zero(packet.param3))) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

#if AP_MAVLINK_COMMAND_LONG_ENABLED
bool GCS_MAVLINK_Copter::mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const
{
    if (packet_command == MAV_CMD_NAV_TAKEOFF ||
        packet_command == MAV_CMD_NAV_VTOL_TAKEOFF) {
        frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        return true;
    }
    return GCS_MAVLINK::mav_frame_for_command_long(frame, packet_command);
}
#endif


MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet)
{
        // param1 : target angle [0-360]
        // param2 : speed during change [deg per second]
        // param3 : direction (-1:ccw, +1:cw)
        // param4 : relative offset (1) or absolute angle (0)
        if ((packet.param1 >= 0.0f)   &&
            (packet.param1 <= 360.0f) &&
            (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
            copter.flightmode->auto_yaw.set_fixed_yaw_rad(
                radians(packet.param1),
                radians(packet.param2),
                (int8_t)packet.param3,
                is_positive(packet.param4));
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
}

MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet)
{
    if (!is_positive(packet.param2)) {
        // Target speed must be larger than zero
        return MAV_RESULT_DENIED;
    }

    const float speed_ms = packet.param2;

    bool success = false;
    switch (SPEED_TYPE(packet.param1)) {
        case SPEED_TYPE_ENUM_END:
            return MAV_RESULT_DENIED;

        case SPEED_TYPE_AIRSPEED: // Airspeed is treated as ground speed for GCS compatibility
        case SPEED_TYPE_GROUNDSPEED:
            success = copter.flightmode->set_speed_NE_ms(speed_ms);
            break;

        case SPEED_TYPE_CLIMB_SPEED:
            success = copter.flightmode->set_speed_up_ms(speed_ms);
            break;

        case SPEED_TYPE_DESCENT_SPEED:
            success = copter.flightmode->set_speed_down_ms(speed_ms);
            break;
    }

    return success ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
}

#if MODE_AUTO_ENABLED
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet)
{
        if (!is_zero(packet.param1) || !is_zero(packet.param2)) {
            // first-item/last item not supported
            return MAV_RESULT_DENIED;
        }
        if (copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) {
            copter.set_auto_armed(true);
            if (copter.mode_auto.mission.state() != AP_Mission::MISSION_RUNNING) {
                copter.mode_auto.mission.start_or_resume();
            }
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
}
#endif



#if HAL_PARACHUTE_ENABLED
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet)
{
        // configure or release parachute
        switch ((uint16_t)packet.param1) {
        case PARACHUTE_DISABLE:
            copter.parachute.enabled(false);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_ENABLE:
            copter.parachute.enabled(true);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_RELEASE:
            // treat as a manual release which performs some additional check of altitude
            copter.parachute_manual_release();
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
}
#endif

MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet)
{
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
                                               (uint8_t)packet.x);
}

#if AP_WINCH_ENABLED
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_DO_WINCH(const mavlink_command_int_t &packet)
{
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
}
#endif  // AP_WINCH_ENABLED

#if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_SOLO_BTN_FLY_CLICK(const mavlink_command_int_t &packet)
{
        if (copter.failsafe.radio) {
            return MAV_RESULT_ACCEPTED;
        }

        // set mode to Loiter or fall back to AltHold
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
        }
        return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_SOLO_BTN_FLY_HOLD(const mavlink_command_int_t &packet)
{
        if (copter.failsafe.radio) {
            return MAV_RESULT_ACCEPTED;
        }

        if (!copter.motors->armed()) {
            // if disarmed, arm motors
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        } else if (copter.ap.land_complete) {
            // if armed and landed, takeoff
            if (copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
                copter.flightmode->do_user_takeoff_U_m(packet.param1, true);
            }
        } else {
            // if flying, land
            copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
        }
        return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_SOLO_BTN_PAUSE_CLICK(const mavlink_command_int_t &packet)
{
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
#if MODE_BRAKE_ENABLED
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
#endif  // AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED

MAV_RESULT GCS_MAVLINK_Copter::handle_command_pause_continue(const mavlink_command_int_t &packet)
{
    // requested pause
    if ((uint8_t) packet.param1 == 0) {
        if (copter.flightmode->pause()) {
            return MAV_RESULT_ACCEPTED;
        }
        send_text(MAV_SEVERITY_INFO, "Failed to pause");
        return MAV_RESULT_FAILED;
    }

    // requested resume
    if ((uint8_t) packet.param1 == 1) {
        if (copter.flightmode->resume()) {
            return MAV_RESULT_ACCEPTED;
        }
        send_text(MAV_SEVERITY_INFO, "Failed to resume");
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_DENIED;
}

// this is called on receipt of a MANUAL_CONTROL packet and is
// expected to call manual_override to override RC input on desired
// axes.
void GCS_MAVLINK_Copter::handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow)
{
    if (packet.z < 0) { // Copter doesn't do negative thrust
        return;
    }

    manual_override(copter.channel_roll, packet.y, 1000, 2000, tnow);
    manual_override(copter.channel_pitch, packet.x, 1000, 2000, tnow, true);
    manual_override(copter.channel_throttle, packet.z, 0, 1000, tnow);
    manual_override(copter.channel_yaw, packet.r, 1000, 2000, tnow);
}

// sanity check velocity or acceleration vector components are numbers
// (e.g. not NaN) and below 1000. vec argument units are in meters/second or
// metres/second/second
bool GCS_MAVLINK_Copter::sane_vel_or_acc_vector(const Vector3f &vec) const
{
    for (uint8_t i=0; i<3; i++) {
        // consider velocity invalid if any component nan or >1000(m/s or m/s/s)
        if (isnan(vec[i]) || fabsf(vec[i]) > 1000) {
            return false;
        }
    }
    return true;
}

#if MODE_GUIDED_ENABLED
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
    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_FORCE_SET =
        POSITION_TARGET_TYPEMASK_FORCE_SET;
#endif

#if MODE_GUIDED_ENABLED
void GCS_MAVLINK_Copter::handle_message_set_attitude_target(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_set_attitude_target_t packet;
    mavlink_msg_set_attitude_target_decode(&msg, &packet);

    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!copter.flightmode->in_guided_mode()) {
        return;
    }

    const bool roll_rate_ignore   = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE;
    const bool pitch_rate_ignore  = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE;
    const bool yaw_rate_ignore    = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE;
    const bool throttle_ignore    = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE;
    const bool attitude_ignore    = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE;

    // ensure thrust field is not ignored
    if (throttle_ignore) {
        // The throttle input is not defined
        copter.mode_guided.init(true);
        return;
    }

    Quaternion attitude_quat;
    if (attitude_ignore) {
        attitude_quat.zero();
    } else {
        attitude_quat = Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]);

        // Do not accept the attitude_quaternion
        // if its magnitude is not close to unit length +/- 1E-3
        // this limit is somewhat greater than sqrt(FLT_EPSL)
        if (!attitude_quat.is_unit_length()) {
            // The attitude quaternion is ill-defined
            copter.mode_guided.init(true);
            return;
        }
    }

    Vector3f ang_vel_body;
    if (!roll_rate_ignore && !pitch_rate_ignore && !yaw_rate_ignore) {
        ang_vel_body.x = packet.body_roll_rate;
        ang_vel_body.y = packet.body_pitch_rate;
        ang_vel_body.z = packet.body_yaw_rate;
    } else if (!(roll_rate_ignore && pitch_rate_ignore && yaw_rate_ignore)) {
        // The body rates are ill-defined
        // input is not valid so stop
        copter.mode_guided.init(true);
        return;
    }

    // check if the message's thrust field should be interpreted as a climb rate or as thrust
    const bool use_thrust = copter.mode_guided.set_attitude_target_provides_thrust();

    float climb_rate_ms_or_thrust;
    if (use_thrust) {
        // interpret thrust as thrust
        climb_rate_ms_or_thrust = constrain_float(packet.thrust, -1.0f, 1.0f);
    } else {
        // convert thrust to climb rate
        packet.thrust = constrain_float(packet.thrust, 0.0f, 1.0f);
        if (is_equal(packet.thrust, 0.5f)) {
            climb_rate_ms_or_thrust = 0.0f;
        } else if (packet.thrust > 0.5f) {
            // climb at up to WPNAV_SPEED_UP
            climb_rate_ms_or_thrust = (packet.thrust - 0.5f) * 2.0f * copter.wp_nav->get_default_speed_up_ms();
        } else {
            // descend at up to WPNAV_SPEED_DN
            climb_rate_ms_or_thrust = (0.5f - packet.thrust) * 2.0f * -copter.wp_nav->get_default_speed_down_ms();
        }
    }

    copter.mode_guided.set_angle(attitude_quat, ang_vel_body,
            climb_rate_ms_or_thrust, use_thrust);
}

void GCS_MAVLINK_Copter::handle_message_set_position_target_local_ned(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_set_position_target_local_ned_t packet;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!copter.flightmode->in_guided_mode()) {
        return;
    }

    // check for supported coordinate frames
    if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
        packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
        packet.coordinate_frame != MAV_FRAME_BODY_NED &&
        packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
        // input is not valid so stop
        copter.mode_guided.init(true);
        return;
    }

    bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
    bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
    bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
    bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
    bool force_set       = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE_SET;

    // Force inputs are not supported
    // Do not accept command if force_set is true and acc_ignore is false
    if (force_set && !acc_ignore) {
        copter.mode_guided.init(true);
        return;
    }

    // prepare position
    Vector3p pos_ned_m;
    if (!pos_ignore) {
        // convert to m
        pos_ned_m = Vector3p{packet.x, packet.y, packet.z};
        // rotate to body-frame if necessary
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
            packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            pos_ned_m.xy() = copter.ahrs.body_to_earth2D_p(pos_ned_m.xy());
        }
        // add body offset if necessary
        if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
            packet.coordinate_frame == MAV_FRAME_BODY_NED ||
            packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            Vector3p rel_pos_ned_m;
            if (!AP::ahrs().get_relative_position_NED_origin(rel_pos_ned_m)) {
                // need position estimate to calculate target position
                copter.mode_guided.init(true);
                return;
            }
            pos_ned_m += rel_pos_ned_m;
        }
    }

    // prepare velocity
    Vector3f vel_ned_ms;
    if (!vel_ignore) {
        vel_ned_ms = Vector3f{packet.vx, packet.vy, packet.vz};
        if (!sane_vel_or_acc_vector(vel_ned_ms)) {
            // input is not valid so stop
            copter.mode_guided.init(true);
            return;
        }
        // rotate to body-frame if necessary
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            vel_ned_ms.xy() = copter.ahrs.body_to_earth2D(vel_ned_ms.xy());
        }
    }

    // prepare acceleration
    Vector3f accel_ned_mss;
    if (!acc_ignore) {
        accel_ned_mss = Vector3f{packet.afx, packet.afy, packet.afz};
        // rotate to body-frame if necessary
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            accel_ned_mss.xy() = copter.ahrs.body_to_earth2D(accel_ned_mss.xy());
        }
    }

    // prepare yaw
    float yaw_rad = 0.0f;
    bool yaw_relative = false;
    float yaw_rate_rads = 0.0f;
    if (!yaw_ignore) {
        yaw_rad = packet.yaw;
        yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
    }
    if (!yaw_rate_ignore) {
        yaw_rate_rads = packet.yaw_rate;
    }

    // send request
    if (!pos_ignore && !vel_ignore) {
        copter.mode_guided.set_pos_vel_accel_NED_m(pos_ned_m, vel_ned_ms, accel_ned_mss, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads, yaw_relative);
    } else if (pos_ignore && !vel_ignore) {
        copter.mode_guided.set_vel_accel_NED_m(vel_ned_ms, accel_ned_mss, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads, yaw_relative);
    } else if (pos_ignore && vel_ignore && !acc_ignore) {
        copter.mode_guided.set_accel_NED_mss(accel_ned_mss, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads, yaw_relative);
    } else if (!pos_ignore && vel_ignore && acc_ignore) {
        copter.mode_guided.set_pos_NED_m(pos_ned_m, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads, yaw_relative, false);
    } else {
        // input is not valid so stop
        copter.mode_guided.init(true);
    }
}

void GCS_MAVLINK_Copter::handle_message_set_position_target_global_int(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_set_position_target_global_int_t packet;
    mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!copter.flightmode->in_guided_mode()) {
        return;
    }

    // todo: do we need to check for supported coordinate frames

    bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
    bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
    bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
    bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
    bool force_set       = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE_SET;

    // Force inputs are not supported
    // Do not accept command if force_set is true and acc_ignore is false
    if (force_set && !acc_ignore) {
        copter.mode_guided.init(true);
        return;
    }

    // extract location from message
    Location loc;
    if (!pos_ignore) {
        // sanity check location
        if (!check_latlng(packet.lat_int, packet.lon_int)) {
            // input is not valid so stop
            copter.mode_guided.init(true);
            return;
        }
        Location::AltFrame frame;
        if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.coordinate_frame, frame)) {
            // unknown coordinate frame
            // input is not valid so stop
            copter.mode_guided.init(true);
            return;
        }
        loc = {packet.lat_int, packet.lon_int, int32_t(packet.alt*100), frame};
    }

    // prepare velocity
    Vector3f vel_ned_ms;
    if (!vel_ignore) {
        vel_ned_ms = Vector3f{packet.vx, packet.vy, packet.vz};
        if (!sane_vel_or_acc_vector(vel_ned_ms)) {
            // input is not valid so stop
            copter.mode_guided.init(true);
            return;
        }
    }

    // prepare acceleration
    Vector3f accel_ned_mss;
    if (!acc_ignore) {
        accel_ned_mss = Vector3f{packet.afx, packet.afy, packet.afz};
    }

    // prepare yaw
    float yaw_rad = 0.0f;
    float yaw_rate_rads = 0.0f;
    if (!yaw_ignore) {
        yaw_rad = packet.yaw;
    }
    if (!yaw_rate_ignore) {
        yaw_rate_rads = packet.yaw_rate;
    }

    // send targets to the appropriate guided mode controller
    if (!pos_ignore && !vel_ignore) {
        // convert Location to vector from ekf origin for posvel controller
        if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
            // posvel controller does not support alt-above-terrain
            // input is not valid so stop
            copter.mode_guided.init(true);
            return;
        }
        Vector3p pos_ned_m;
        if (!loc.get_vector_from_origin_NED_m(pos_ned_m)) {
            // input is not valid so stop
            copter.mode_guided.init(true);
            return;
        }
        copter.mode_guided.set_pos_vel_NED_m(pos_ned_m, vel_ned_ms, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads);
    } else if (pos_ignore && !vel_ignore) {
        copter.mode_guided.set_vel_accel_NED_m(vel_ned_ms, accel_ned_mss, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads);
    } else if (pos_ignore && vel_ignore && !acc_ignore) {
        copter.mode_guided.set_accel_NED_mss(accel_ned_mss, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads);
    } else if (!pos_ignore && vel_ignore && acc_ignore) {
        copter.mode_guided.set_destination(loc, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads);
    } else {
        // input is not valid so stop
        copter.mode_guided.init(true);
    }
}
#endif  // MODE_GUIDED_ENABLED

void GCS_MAVLINK_Copter::handle_message(const mavlink_message_t &msg)
{

    switch (msg.msgid) {
#if MODE_GUIDED_ENABLED
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        handle_message_set_attitude_target(msg);
        break;
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        handle_message_set_position_target_local_ned(msg);
        break;
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        handle_message_set_position_target_global_int(msg);
        break;
#endif
#if AP_TERRAIN_AVAILABLE
    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
        copter.terrain.handle_data(chan, msg);
        break;
#endif
#if TOY_MODE_ENABLED
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        copter.g2.toy_mode.handle_message(msg);
        break;
#endif
    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}

MAV_RESULT GCS_MAVLINK_Copter::handle_flight_termination(const mavlink_command_int_t &packet) {
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
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
#if AP_TERRAIN_AVAILABLE
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
    if (!AP::ahrs().airspeed_vector_TAS(airspeed_vec_bf)) {
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

#if HAL_HIGH_LATENCY2_ENABLED
int16_t GCS_MAVLINK_Copter::high_latency_target_altitude() const
{
    AP_AHRS &ahrs = AP::ahrs();
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));

    //return units are m
    if (copter.ap.initialised) {
        return global_position_current.alt * 0.01 - copter.pos_control->get_pos_error_D_m();
    }
    return 0;
    
}

uint8_t GCS_MAVLINK_Copter::high_latency_tgt_heading() const
{
    if (copter.ap.initialised) {
        // return units are deg/2
        const Mode *flightmode = copter.flightmode;
        // need to convert -180->180 to 0->360/2
        return wrap_360(flightmode->wp_bearing_deg()) * 0.5;
    }
    return 0;     
}
    
uint16_t GCS_MAVLINK_Copter::high_latency_tgt_dist() const
{
    if (copter.ap.initialised) {
        // return units are dm
        const Mode *flightmode = copter.flightmode;
        return MIN(flightmode->wp_distance_m(), UINT16_MAX) / 10;
    }
    return 0;
}

uint8_t GCS_MAVLINK_Copter::high_latency_tgt_airspeed() const
{
    if (copter.ap.initialised) {
        // return units are m/s*5
        return MIN(copter.pos_control->get_vel_target_NED_ms().length() * 5.0, UINT8_MAX);
    }
    return 0;  
}

uint8_t GCS_MAVLINK_Copter::high_latency_wind_speed() const
{
    Vector3f airspeed_vec_bf;
    Vector3f wind;
    // return units are m/s*5
    if (AP::ahrs().airspeed_vector_TAS(airspeed_vec_bf)) {
        wind = AP::ahrs().wind_estimate();
        return wind.length() * 5;
    }
    return 0; 
}

uint8_t GCS_MAVLINK_Copter::high_latency_wind_direction() const
{
    Vector3f airspeed_vec_bf;
    Vector3f wind;
    // return units are deg/2
    if (AP::ahrs().airspeed_vector_TAS(airspeed_vec_bf)) {
        wind = AP::ahrs().wind_estimate();
        // need to convert -180->180 to 0->360/2
        return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
    }
    return 0;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

// Send the mode with the given index (not mode number!) return the total number of modes
// Index starts at 1
uint8_t GCS_MAVLINK_Copter::send_available_mode(uint8_t index) const
{
    const Mode* modes[] {
#if MODE_AUTO_ENABLED
        &copter.mode_auto, // This auto is actually auto RTL!
        &copter.mode_auto, // This one is really is auto!
#endif
#if MODE_ACRO_ENABLED
        &copter.mode_acro,
#endif
        &copter.mode_stabilize,
        &copter.mode_althold,
#if MODE_CIRCLE_ENABLED
        &copter.mode_circle,
#endif
#if MODE_LOITER_ENABLED
        &copter.mode_loiter,
#endif
#if MODE_GUIDED_ENABLED
        &copter.mode_guided,
#endif
        &copter.mode_land,
#if MODE_RTL_ENABLED
        &copter.mode_rtl,
#endif
#if MODE_DRIFT_ENABLED
        &copter.mode_drift,
#endif
#if MODE_SPORT_ENABLED
        &copter.mode_sport,
#endif
#if MODE_FLIP_ENABLED
        &copter.mode_flip,
#endif
#if AUTOTUNE_ENABLED
        &copter.mode_autotune,
#endif
#if MODE_POSHOLD_ENABLED
        &copter.mode_poshold,
#endif
#if MODE_BRAKE_ENABLED
        &copter.mode_brake,
#endif
#if MODE_THROW_ENABLED
        &copter.mode_throw,
#endif
#if AP_ADSB_AVOIDANCE_ENABLED
        &copter.mode_avoid_adsb,
#endif
#if MODE_GUIDED_NOGPS_ENABLED
        &copter.mode_guided_nogps,
#endif
#if MODE_SMARTRTL_ENABLED
        &copter.mode_smartrtl,
#endif
#if MODE_FLOWHOLD_ENABLED
        (Mode*)copter.g2.mode_flowhold_ptr,
#endif
#if MODE_FOLLOW_ENABLED
        &copter.mode_follow,
#endif
#if MODE_ZIGZAG_ENABLED
        &copter.mode_zigzag,
#endif
#if MODE_SYSTEMID_ENABLED
        (Mode *)copter.g2.mode_systemid_ptr,
#endif
#if MODE_AUTOROTATE_ENABLED
        &copter.mode_autorotate,
#endif
#if MODE_TURTLE_ENABLED
        &copter.mode_turtle,
#endif
    };

    const uint8_t base_mode_count = ARRAY_SIZE(modes);
    uint8_t mode_count = base_mode_count;

#if AP_SCRIPTING_ENABLED
    for (uint8_t i = 0; i < ARRAY_SIZE(copter.mode_guided_custom); i++) {
        if (copter.mode_guided_custom[i] != nullptr) {
            mode_count += 1;
        }
    }
#endif

    // Convert to zero indexed
    const uint8_t index_zero = index - 1;
    if (index_zero >= mode_count) {
        // Mode does not exist!?
        return mode_count;
    }

    // Ask the mode for its name and number
    const char* name;
    uint8_t mode_number;

    if (index_zero < base_mode_count) {
        name = modes[index_zero]->name();
        mode_number = (uint8_t)modes[index_zero]->mode_number();

    } else {
#if AP_SCRIPTING_ENABLED
        const uint8_t custom_index = index_zero - base_mode_count;
        if (copter.mode_guided_custom[custom_index] == nullptr) {
            // Invalid index, should not happen
            return mode_count;
        }
        name = copter.mode_guided_custom[custom_index]->name();
        mode_number = (uint8_t)copter.mode_guided_custom[custom_index]->mode_number();
#else
        // Should not endup here
        return mode_count;
#endif
    }

#if MODE_AUTO_ENABLED
    // Auto RTL is odd
    // Have to deal with is separately because its number and name can change depending on if were in it or not
    if (index_zero == 0) {
        mode_number = (uint8_t)Mode::Number::AUTO_RTL;
        name = "AUTO RTL";

    } else if (index_zero == 1) {
        mode_number = (uint8_t)Mode::Number::AUTO;
        name = "AUTO";

    }
#endif

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
