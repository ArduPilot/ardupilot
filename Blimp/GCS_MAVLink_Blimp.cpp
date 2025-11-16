#include "Blimp.h"

#include "GCS_MAVLink_Blimp.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_OpticalFlow/AP_OpticalFlow_config.h>

MAV_TYPE GCS_Blimp::frame_type() const
{
    return blimp.get_frame_mav_type();
}

uint8_t GCS_MAVLINK_Blimp::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    // we are armed if we are not initialising
    if (blimp.motors != nullptr && blimp.motors->armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return _base_mode;
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
    if (!blimp.ap.initialised) {
    	return MAV_STATE_BOOT;
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

void GCS_MAVLINK_Blimp::send_nav_controller_output() const
{

}

float GCS_MAVLINK_Blimp::vfr_hud_airspeed() const
{
    Vector3f airspeed_vec_bf;
    if (AP::ahrs().airspeed_vector_TAS(airspeed_vec_bf)) {
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
    if (blimp.control_mode == Mode::Number::MANUAL || blimp.control_mode == Mode::Number::LAND) {
        //No PIDs are used in Manual or Land mode.
        return;
    }

    static const int8_t axes[] = {
        PID_SEND::VELX,
        PID_SEND::VELY,
        PID_SEND::VELZ,
        PID_SEND::VELYAW,
        PID_SEND::POSX,
        PID_SEND::POSY,
        PID_SEND::POSZ,
        PID_SEND::POSYAW
    };
    for (uint8_t i=0; i<ARRAY_SIZE(axes); i++) {
        if (!(blimp.g.gcs_pid_mask & (1<<(axes[i]-1)))) {
            continue;
        }
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
        const AP_PIDInfo *pid_info = nullptr;
        switch (axes[i]) {
        case PID_SEND::VELX:
            pid_info = &blimp.pid_vel_xy.get_pid_info_x();
            break;
        case PID_SEND::VELY:
            pid_info = &blimp.pid_vel_xy.get_pid_info_y();
            break;
        case PID_SEND::VELZ:
            pid_info = &blimp.pid_vel_z.get_pid_info();
            break;
        case PID_SEND::VELYAW:
            pid_info = &blimp.pid_vel_yaw.get_pid_info();
            break;
        case PID_SEND::POSX:
            pid_info = &blimp.pid_pos_xy.get_pid_info_x();
            break;
        case PID_SEND::POSY:
            pid_info = &blimp.pid_pos_xy.get_pid_info_y();
            break;
        case PID_SEND::POSZ:
            pid_info = &blimp.pid_pos_z.get_pid_info();
            break;
        case PID_SEND::POSYAW:
            pid_info = &blimp.pid_pos_yaw.get_pid_info();
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

    case MSG_ADSB_VEHICLE:
        // unused
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
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

MAV_RESULT GCS_MAVLINK_Blimp::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}


MAV_RESULT GCS_MAVLINK_Blimp::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    // blimp.flightmode->auto_yaw.set_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
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
    if (!location_from_command_t(packet, request_location)) {
        return MAV_RESULT_DENIED;
    }

    if (request_location.sanitize(blimp.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Blimp::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);
    case MAV_CMD_NAV_TAKEOFF:
        return MAV_RESULT_ACCEPTED;
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

#if AP_MAVLINK_COMMAND_LONG_ENABLED
bool GCS_MAVLINK_Blimp::mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const
{
    if (packet_command == MAV_CMD_NAV_TAKEOFF) {
        frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        return true;
    }
    return GCS_MAVLINK::mav_frame_for_command_long(frame, packet_command);
}
#endif

void GCS_MAVLINK_Blimp::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
        break;

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }     // end switch
} // end handle mavlink


MAV_RESULT GCS_MAVLINK_Blimp::handle_flight_termination(const mavlink_command_int_t &packet)
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
    if (!AP::ahrs().airspeed_vector_TAS(airspeed_vec_bf)) {
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

#if HAL_HIGH_LATENCY2_ENABLED
uint8_t GCS_MAVLINK_Blimp::high_latency_wind_speed() const
{
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_TAS(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a
        // valid wind estimate on blimps
        return 0;
    }
    // return units are m/s*5
    const Vector3f wind = AP::ahrs().wind_estimate();
    return wind.length() * 5;
}

uint8_t GCS_MAVLINK_Blimp::high_latency_wind_direction() const
{
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_TAS(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a
        // valid wind estimate on blimps
        return 0;
    }
    const Vector3f wind = AP::ahrs().wind_estimate();
    // need to convert -180->180 to 0->360/2
    return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

// Send the mode with the given index (not mode number!) return the total number of modes
// Index starts at 1
uint8_t GCS_MAVLINK_Blimp::send_available_mode(uint8_t index) const
{
    const Mode* modes[] {
        &blimp.mode_land,
        &blimp.mode_manual,
        &blimp.mode_velocity,
        &blimp.mode_loiter,
        &blimp.mode_rtl,
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
