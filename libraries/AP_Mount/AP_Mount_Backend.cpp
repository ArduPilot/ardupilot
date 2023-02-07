#include "AP_Mount_Backend.h"
#if HAL_MOUNT_ENABLED
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_UPDATE_DT 0.02     // update rate in seconds.  update() should be called at this rate

// get mount's current attitude in euler angles in degrees.  yaw angle is in body-frame
// returns true on success
bool AP_Mount_Backend::get_attitude_euler(float& roll_deg, float& pitch_deg, float& yaw_bf_deg)
{
    // by default re-use get_attitude_quaternion and convert to Euler angles
    Quaternion att_quat;
    if (!get_attitude_quaternion(att_quat)) {
        return false;
    }

    float roll_rad, pitch_rad, yaw_rad;
    att_quat.to_euler(roll_rad, pitch_rad, yaw_rad);
    roll_deg = degrees(roll_rad);
    pitch_deg = degrees(pitch_rad);
    yaw_bf_deg = degrees(yaw_rad);
    return true;
}

// set angle target in degrees
// yaw_is_earth_frame (aka yaw_lock) should be true if yaw angle is earth-frame, false if body-frame
void AP_Mount_Backend::set_angle_target(float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame)
{
    // set angle targets
    mavt_target.target_type = MountTargetType::ANGLE;
    mavt_target.angle_rad.roll = radians(roll_deg);
    mavt_target.angle_rad.pitch = radians(pitch_deg);
    mavt_target.angle_rad.yaw = radians(yaw_deg);
    mavt_target.angle_rad.yaw_is_ef = yaw_is_earth_frame;

    // set the mode to mavlink targeting
    set_mode(MAV_MOUNT_MODE_MAVLINK_TARGETING);
}

// sets rate target in deg/s
// yaw_lock should be true if the yaw rate is earth-frame, false if body-frame (e.g. rotates with body of vehicle)
void AP_Mount_Backend::set_rate_target(float roll_degs, float pitch_degs, float yaw_degs, bool yaw_is_earth_frame)
{
    // set rate targets
    mavt_target.target_type = MountTargetType::RATE;
    mavt_target.rate_rads.roll = radians(roll_degs);
    mavt_target.rate_rads.pitch = radians(pitch_degs);
    mavt_target.rate_rads.yaw = radians(yaw_degs);
    mavt_target.rate_rads.yaw_is_ef = yaw_is_earth_frame;

    // set the mode to mavlink targeting
    set_mode(MAV_MOUNT_MODE_MAVLINK_TARGETING);
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount_Backend::set_roi_target(const Location &target_loc)
{
    // set the target gps location
    _roi_target = target_loc;
    _roi_target_set = true;

    // set the mode to GPS tracking mode
    set_mode(MAV_MOUNT_MODE_GPS_POINT);
}

// set_sys_target - sets system that mount should attempt to point towards
void AP_Mount_Backend::set_target_sysid(uint8_t sysid)
{
    _target_sysid = sysid;

    // set the mode to sysid tracking mode
    set_mode(MAV_MOUNT_MODE_SYSID_TARGET);
}

// process MOUNT_CONFIGURE messages received from GCS. deprecated.
void AP_Mount_Backend::handle_mount_configure(const mavlink_mount_configure_t &packet)
{
    set_mode((MAV_MOUNT_MODE)packet.mount_mode);
}

// send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
void AP_Mount_Backend::send_gimbal_device_attitude_status(mavlink_channel_t chan)
{
    if (suppress_heartbeat()) {
        // block heartbeat from transmitting to the GCS
        GCS_MAVLINK::disable_channel_routing(chan);
    }

    Quaternion att_quat;
    if (!get_attitude_quaternion(att_quat)) {
        return;
    }

    // construct quaternion array
    const float quat_array[4] = {att_quat.q1, att_quat.q2, att_quat.q3, att_quat.q4};

    mavlink_msg_gimbal_device_attitude_status_send(chan,
                                                   0,   // target system
                                                   0,   // target component
                                                   AP_HAL::millis(),    // autopilot system time
                                                   get_gimbal_device_flags(),
                                                   quat_array,    // attitude expressed as quaternion
                                                   std::numeric_limits<double>::quiet_NaN(),    // roll axis angular velocity (NaN for unknown)
                                                   std::numeric_limits<double>::quiet_NaN(),    // pitch axis angular velocity (NaN for unknown)
                                                   std::numeric_limits<double>::quiet_NaN(),    // yaw axis angular velocity (NaN for unknown)
                                                   0);  // failure flags (not supported)
}

// process MOUNT_CONTROL messages received from GCS. deprecated.
void AP_Mount_Backend::handle_mount_control(const mavlink_mount_control_t &packet)
{
    switch (get_mode()) {
    case MAV_MOUNT_MODE_MAVLINK_TARGETING:
        // input_a : Pitch in centi-degrees
        // input_b : Roll in centi-degrees
        // input_c : Yaw in centi-degrees (interpreted as body-frame)
        set_angle_target(packet.input_b * 0.01, packet.input_a * 0.01, packet.input_c * 0.01, false);
        break;

    case MAV_MOUNT_MODE_GPS_POINT: {
        // input_a : lat in degE7
        // input_b : lon in degE7
        // input_c : alt  in cm (interpreted as above home)
        const Location target_location {
            packet.input_a,
            packet.input_b,
            packet.input_c,
            Location::AltFrame::ABOVE_HOME
        };
        set_roi_target(target_location);
        break;
    }

    case MAV_MOUNT_MODE_RETRACT:
    case MAV_MOUNT_MODE_NEUTRAL:
    case MAV_MOUNT_MODE_RC_TARGETING:
    case MAV_MOUNT_MODE_SYSID_TARGET:
    case MAV_MOUNT_MODE_HOME_LOCATION:
    default:
        // no effect in these modes
        break;
    }
}

// handle do_mount_control command.  Returns MAV_RESULT_ACCEPTED on success
MAV_RESULT AP_Mount_Backend::handle_command_do_mount_control(const mavlink_command_long_t &packet)
{
    const MAV_MOUNT_MODE new_mode = (MAV_MOUNT_MODE)packet.param7;

    // interpret message fields based on mode
    switch (new_mode) {
    case MAV_MOUNT_MODE_RETRACT:
    case MAV_MOUNT_MODE_NEUTRAL:
    case MAV_MOUNT_MODE_RC_TARGETING:
    case MAV_MOUNT_MODE_HOME_LOCATION:
        // simply set mode
        set_mode(new_mode);
        return MAV_RESULT_ACCEPTED;

    case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
        // set body-frame target angles (in degrees) from mavlink message
        const float pitch_deg = packet.param1;  // param1: pitch (in degrees)
        const float roll_deg = packet.param2;   // param2: roll in degrees
        const float yaw_deg = packet.param3;    // param3: yaw in degrees

        // warn if angles are invalid to catch angles sent in centi-degrees
        if ((fabsf(pitch_deg) > 90) || (fabsf(roll_deg) > 180) || (fabsf(yaw_deg) > 360)) {
            send_warning_to_GCS("invalid angle targets");
            return MAV_RESULT_FAILED;
        }

        set_angle_target(packet.param2, packet.param1, packet.param3, false);
        return MAV_RESULT_ACCEPTED;
    }

    case MAV_MOUNT_MODE_GPS_POINT: {
        // set lat, lon, alt position targets from mavlink message

        // warn if lat, lon appear to be in param1,2 instead of param5,6 as this indicates
        // sender is relying on a bug in AP-4.2's (and earlier) handling of MAV_CMD_DO_MOUNT_CONTROL
        if (!is_zero(packet.param1) && !is_zero(packet.param2) && is_zero(packet.param5) && is_zero(packet.param6)) {
            send_warning_to_GCS("GPS_POINT target invalid");
            return MAV_RESULT_FAILED;
        }

        // param4: altitude in meters
        // param5: latitude in degrees * 1E7
        // param6: longitude in degrees * 1E7
        const Location target_location {
            (int32_t)packet.param5,         // latitude in degrees * 1E7
            (int32_t)packet.param6,         // longitude in degrees * 1E7
            (int32_t)packet.param4 * 100,   // alt converted from meters to cm
            Location::AltFrame::ABOVE_HOME
        };
        set_roi_target(target_location);
        return MAV_RESULT_ACCEPTED;
    }

    default:
        // invalid mode
        return MAV_RESULT_FAILED;
    }
}

// handle a GLOBAL_POSITION_INT message
bool AP_Mount_Backend::handle_global_position_int(uint8_t msg_sysid, const mavlink_global_position_int_t &packet)
{
    if (_target_sysid != msg_sysid) {
        return false;
    }

    _target_sysid_location.lat = packet.lat;
    _target_sysid_location.lng = packet.lon;
    // global_position_int.alt is *UP*, so is location.
    _target_sysid_location.set_alt_cm(packet.alt*0.1, Location::AltFrame::ABSOLUTE);
    _target_sysid_location_set = true;

    return true;
}

// get pilot input (in the range -1 to +1) received through RC
void AP_Mount_Backend::get_rc_input(float& roll_in, float& pitch_in, float& yaw_in) const
{
    const RC_Channel *roll_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_ROLL : RC_Channel::AUX_FUNC::MOUNT2_ROLL);
    const RC_Channel *pitch_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_PITCH : RC_Channel::AUX_FUNC::MOUNT2_PITCH);
    const RC_Channel *yaw_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_YAW : RC_Channel::AUX_FUNC::MOUNT2_YAW);

    roll_in = 0;
    if ((roll_ch != nullptr) && (roll_ch->get_radio_in() > 0)) {
        roll_in = roll_ch->norm_input_dz();
    }

    pitch_in = 0;
    if ((pitch_ch != nullptr) && (pitch_ch->get_radio_in() > 0)) {
        pitch_in = pitch_ch->norm_input_dz();
    }

    yaw_in = 0;
    if ((yaw_ch != nullptr) && (yaw_ch->get_radio_in() > 0)) {
        yaw_in = yaw_ch->norm_input_dz();
    }
}

// get rate targets (in rad/s) from pilot RC
// returns true on success (RC is providing rate targets), false on failure (RC is providing angle targets)
bool AP_Mount_Backend::get_rc_rate_target(MountTarget& rate_rads) const
{
    // exit immediately if RC is not providing rate targets
    if (_params.rc_rate_max <= 0) {
        return false;
    }

    // get RC input from pilot
    float roll_in, pitch_in, yaw_in;
    get_rc_input(roll_in, pitch_in, yaw_in);

    // calculate rates
    const float rc_rate_max_rads = radians(_params.rc_rate_max.get());
    rate_rads.roll = roll_in * rc_rate_max_rads;
    rate_rads.pitch = pitch_in * rc_rate_max_rads;
    rate_rads.yaw = yaw_in * rc_rate_max_rads;

    // yaw frame
    rate_rads.yaw_is_ef = _yaw_lock;

    return true;
}

// get angle targets (in radians) from pilot RC
// returns true on success (RC is providing angle targets), false on failure (RC is providing rate targets)
bool AP_Mount_Backend::get_rc_angle_target(MountTarget& angle_rad) const
{
    // exit immediately if RC is not providing angle targets
    if (_params.rc_rate_max > 0) {
        return false;
    }

    // get RC input from pilot
    float roll_in, pitch_in, yaw_in;
    get_rc_input(roll_in, pitch_in, yaw_in);

    // roll angle
    angle_rad.roll = radians(((roll_in + 1.0f) * 0.5f * (_params.roll_angle_max - _params.roll_angle_min) + _params.roll_angle_min));

    // pitch angle
    angle_rad.pitch = radians(((pitch_in + 1.0f) * 0.5f * (_params.pitch_angle_max - _params.pitch_angle_min) + _params.pitch_angle_min));

    // yaw angle
    angle_rad.yaw_is_ef = _yaw_lock;
    if (angle_rad.yaw_is_ef) {
        // if yaw is earth-frame pilot yaw input control angle from -180 to +180 deg
        angle_rad.yaw = yaw_in * M_PI;
    } else {
        // yaw target in body frame so apply body frame limits
        angle_rad.yaw = radians(((yaw_in + 1.0f) * 0.5f * (_params.yaw_angle_max - _params.yaw_angle_min) + _params.yaw_angle_min));
    }

    return true;
}

// get angle targets (in radians) to a Location
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_location(const Location &loc, MountTarget& angle_rad) const
{
    // exit immediately if vehicle's location is unavailable
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    // exit immediate if location is invalid
    if (!loc.initialised()) {
        return false;
    }

    const float GPS_vector_x = Location::diff_longitude(loc.lng, current_loc.lng)*cosf(ToRad((current_loc.lat + loc.lat) * 0.00000005f)) * 0.01113195f;
    const float GPS_vector_y = (loc.lat - current_loc.lat) * 0.01113195f;
    int32_t target_alt_cm = 0;
    if (!loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt_cm)) {
        return false;
    }
    int32_t current_alt_cm = 0;
    if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, current_alt_cm)) {
        return false;
    }
    float GPS_vector_z = target_alt_cm - current_alt_cm;
    float target_distance = 100.0f*norm(GPS_vector_x, GPS_vector_y);      // Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.

    // calculate roll, pitch, yaw angles
    angle_rad.roll = 0;
    angle_rad.pitch = atan2f(GPS_vector_z, target_distance);
    angle_rad.yaw = atan2f(GPS_vector_x, GPS_vector_y);
    angle_rad.yaw_is_ef = true;

    return true;
}

// get angle targets (in radians) to ROI location
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_roi(MountTarget& angle_rad) const
{
    if (!_roi_target_set) {
        return false;
    }
    return get_angle_target_to_location(_roi_target, angle_rad);
}

// return body-frame yaw angle from a mount target
float AP_Mount_Backend::get_bf_yaw_angle(const MountTarget& angle_rad) const
{
    if (angle_rad.yaw_is_ef) {
        // convert to body-frame
        return wrap_PI(angle_rad.yaw - AP::ahrs().yaw);
    }

    // target is already body-frame
    return angle_rad.yaw;
}

// return earth-frame yaw angle from a mount target
float AP_Mount_Backend::get_ef_yaw_angle(const MountTarget& angle_rad) const
{
    if (angle_rad.yaw_is_ef) {
        // target is already earth-frame
        return angle_rad.yaw;
    }

    // convert to earth-frame
    return wrap_PI(angle_rad.yaw + AP::ahrs().yaw);
}

// update angle targets using a given rate target
// the resulting angle_rad yaw frame will match the rate_rad yaw frame
// assumes a 50hz update rate
void AP_Mount_Backend::update_angle_target_from_rate(const MountTarget& rate_rad, MountTarget& angle_rad) const
{
    // update roll and pitch angles and apply limits
    angle_rad.roll = constrain_float(angle_rad.roll + rate_rad.roll * AP_MOUNT_UPDATE_DT, radians(_params.roll_angle_min), radians(_params.roll_angle_max));
    angle_rad.pitch = constrain_float(angle_rad.pitch + rate_rad.pitch * AP_MOUNT_UPDATE_DT, radians(_params.pitch_angle_min), radians(_params.pitch_angle_max));

    // ensure angle yaw frames matches rate yaw frame
    if (angle_rad.yaw_is_ef != rate_rad.yaw_is_ef) {
        if (rate_rad.yaw_is_ef) {
            angle_rad.yaw = get_ef_yaw_angle(angle_rad);
        } else {
            angle_rad.yaw = get_bf_yaw_angle(angle_rad);
        }
        angle_rad.yaw_is_ef = rate_rad.yaw_is_ef;
    }

    // update yaw angle target
    angle_rad.yaw = angle_rad.yaw + rate_rad.yaw * AP_MOUNT_UPDATE_DT;
    if (angle_rad.yaw_is_ef) {
        // if earth-frame yaw wraps between += 180 degrees
        angle_rad.yaw = wrap_PI(angle_rad.yaw);
    } else {
        // if body-frame constrain yaw to body-frame limits
        angle_rad.yaw = constrain_float(angle_rad.yaw, radians(_params.yaw_angle_min), radians(_params.yaw_angle_max));
    }
}

// helper function to provide GIMBAL_DEVICE_FLAGS for use in GIMBAL_DEVICE_ATTITUDE_STATUS message
uint16_t AP_Mount_Backend::get_gimbal_device_flags() const
{
    const uint16_t flags = (get_mode() == MAV_MOUNT_MODE_RETRACT ? GIMBAL_DEVICE_FLAGS_RETRACT : 0) |
                           (get_mode() == MAV_MOUNT_MODE_NEUTRAL ? GIMBAL_DEVICE_FLAGS_NEUTRAL : 0) |
                           GIMBAL_DEVICE_FLAGS_ROLL_LOCK | // roll angle is always earth-frame
                           GIMBAL_DEVICE_FLAGS_PITCH_LOCK; // pitch angle is always earth-frame, yaw_angle is always body-frame
    return flags;
}

// get angle targets (in radians) to home location
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_home(MountTarget& angle_rad) const
{
    // exit immediately if home is not set
    if (!AP::ahrs().home_is_set()) {
        return false;
    }
    return get_angle_target_to_location(AP::ahrs().get_home(), angle_rad);
}

// get angle targets (in radians) to a vehicle with sysid of  _target_sysid
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_sysid(MountTarget& angle_rad) const
{
    // exit immediately if sysid is not set or no location available
    if (!_target_sysid_location_set) {
        return false;
    }
    if (!_target_sysid) {
        return false;
    }
    return get_angle_target_to_location(_target_sysid_location, angle_rad);
}

// sent warning to GCS.  Warnings are throttled to at most once every 30 seconds
void AP_Mount_Backend::send_warning_to_GCS(const char* warning_str)
{
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_warning_ms < 30000) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "Mount: %s", warning_str);
    _last_warning_ms = now_ms;
}

#endif // HAL_MOUNT_ENABLED
