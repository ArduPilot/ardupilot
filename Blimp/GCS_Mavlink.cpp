#include "Blimp.h"

#include "GCS_Mavlink.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_OpticalFlow/AP_OpticalFlow_config.h>

MAV_TYPE GCS_Blimp::frame_type() const
{
    return blimp.get_frame_mav_type();
}

MAV_MODE GCS_MAVLINK_Blimp::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
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
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, and FENCE_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK_Parameters, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK_Parameters, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Unused
    // @Description: Unused
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK_Parameters, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK_Parameters, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE, SIMSTATE (SITL only), AHRS2 and PID_TUNING to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK_Parameters, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, SYSTEM_TIME, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION and RPM to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @RebootRequired: True
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
};
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
#if HAL_WITH_MCU_MONITORING
    MSG_MCU_STATUS,
#endif
    MSG_MEMINFO,
    MSG_CURRENT_WAYPOINT, // MISSION_CURRENT
    MSG_GPS_RAW,
    MSG_GPS_RTK,
#if GPS_MAX_RECEIVERS > 1
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
#endif
    MSG_NAV_CONTROLLER_OUTPUT,
#if AP_FENCE_ENABLED
    MSG_FENCE_STATUS,
#endif
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
#if AP_SIM_ENABLED
    MSG_SIMSTATE,
#endif
    MSG_AHRS2,
    MSG_PID_TUNING // Up to four PID_TUNING messages are sent, depending on GCS_PID_MASK parameter
};
static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD
};
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_SYSTEM_TIME,
    MSG_WIND,
#if AP_RANGEFINDER_ENABLED
    MSG_RANGEFINDER,
#endif
    MSG_DISTANCE_SENSOR,
#if AP_BATTERY_ENABLED
    MSG_BATTERY_STATUS,
#endif
#if HAL_MOUNT_ENABLED
    MSG_GIMBAL_DEVICE_ATTITUDE_STATUS,
#endif
#if AP_OPTICALFLOW_ENABLED
    MSG_OPTICAL_FLOW,
#endif
#if COMPASS_CAL_ENABLED
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
#endif
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
#if AP_RPM_ENABLED
    MSG_RPM,
#endif
#if HAL_WITH_ESC_TELEM
    MSG_ESC_TELEMETRY,
#endif
#if HAL_GENERATOR_ENABLED
    MSG_GENERATOR_STATUS,
#endif
};
static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};
static const ap_message STREAM_ADSB_msgs[] = {
    MSG_ADSB_VEHICLE,
#if AP_AIS_ENABLED
    MSG_AIS_VESSEL,
#endif
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

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS: {     // MAV ID: 109
#if HAL_LOGGING_ENABLED
        handle_radio_status(msg, blimp.should_log(MASK_LOG_PM));
#else
        handle_radio_status(msg, false);
#endif
        break;
    }

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

#if HAL_HIGH_LATENCY2_ENABLED
uint8_t GCS_MAVLINK_Blimp::high_latency_wind_speed() const
{
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
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
    if (!AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a
        // valid wind estimate on blimps
        return 0;
    }
    const Vector3f wind = AP::ahrs().wind_estimate();
    // need to convert -180->180 to 0->360/2
    return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
}
#endif // HAL_HIGH_LATENCY2_ENABLED
