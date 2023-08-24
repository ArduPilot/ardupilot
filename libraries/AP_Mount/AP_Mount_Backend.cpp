#include "AP_Mount_Backend.h"
#if HAL_MOUNT_ENABLED
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_UPDATE_DT 0.02     // update rate in seconds.  update() should be called at this rate

// Default init function for every mount
void AP_Mount_Backend::init()
{
    // setting default target sysid from parameters
    _target_sysid = _params.sysid_default.get();
}

// set device id of this instance, for MNTx_DEVID parameter
void AP_Mount_Backend::set_dev_id(uint32_t id)
{
    _params.dev_id.set_and_save(int32_t(id));
}

// return true if this mount accepts roll targets
bool AP_Mount_Backend::has_roll_control() const
{
    return (_params.roll_angle_min < _params.roll_angle_max);
}

// return true if this mount accepts pitch targets
bool AP_Mount_Backend::has_pitch_control() const
{
    return (_params.pitch_angle_min < _params.pitch_angle_max);
}

// set angle target in degrees
// yaw_is_earth_frame (aka yaw_lock) should be true if yaw angle is earth-frame, false if body-frame
void AP_Mount_Backend::set_angle_target(float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame)
{
    // enforce angle limits
    roll_deg = constrain_float(roll_deg, _params.roll_angle_min, _params.roll_angle_max);
    pitch_deg = constrain_float(pitch_deg, _params.pitch_angle_min, _params.pitch_angle_max);
    if (!yaw_is_earth_frame) {
        // only limit yaw if in body-frame.  earth-frame yaw limiting is backend specific
        // custom wrap code (instead of wrap_180) to better handle yaw of <= -180
        if (yaw_deg > 180) {
            yaw_deg -= 360;
        }
        yaw_deg = constrain_float(yaw_deg, _params.yaw_angle_min, _params.yaw_angle_max);
    }

    // set angle targets
    mnt_target.target_type = MountTargetType::ANGLE;
    mnt_target.angle_rad.roll = radians(roll_deg);
    mnt_target.angle_rad.pitch = radians(pitch_deg);
    mnt_target.angle_rad.yaw = radians(yaw_deg);
    mnt_target.angle_rad.yaw_is_ef = yaw_is_earth_frame;

    // set the mode to mavlink targeting
    set_mode(MAV_MOUNT_MODE_MAVLINK_TARGETING);
}

// sets rate target in deg/s
// yaw_lock should be true if the yaw rate is earth-frame, false if body-frame (e.g. rotates with body of vehicle)
void AP_Mount_Backend::set_rate_target(float roll_degs, float pitch_degs, float yaw_degs, bool yaw_is_earth_frame)
{
    // set rate targets
    mnt_target.target_type = MountTargetType::RATE;
    mnt_target.rate_rads.roll = radians(roll_degs);
    mnt_target.rate_rads.pitch = radians(pitch_degs);
    mnt_target.rate_rads.yaw = radians(yaw_degs);
    mnt_target.rate_rads.yaw_is_ef = yaw_is_earth_frame;

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

// clear_roi_target - clears target location that mount should attempt to point towards
void AP_Mount_Backend::clear_roi_target()
{
    // clear the target GPS location
    _roi_target_set = false;

    // reset the mode if in GPS tracking mode
    if (_mode == MAV_MOUNT_MODE_GPS_POINT) {
        MAV_MOUNT_MODE default_mode = (MAV_MOUNT_MODE)_params.default_mode.get();
        set_mode(default_mode);
    }
}

// set_sys_target - sets system that mount should attempt to point towards
void AP_Mount_Backend::set_target_sysid(uint8_t sysid)
{
    _target_sysid = sysid;

    // set the mode to sysid tracking mode
    set_mode(MAV_MOUNT_MODE_SYSID_TARGET);
}

#if AP_MAVLINK_MSG_MOUNT_CONFIGURE_ENABLED
// process MOUNT_CONFIGURE messages received from GCS. deprecated.
void AP_Mount_Backend::handle_mount_configure(const mavlink_mount_configure_t &packet)
{
    set_mode((MAV_MOUNT_MODE)packet.mount_mode);
}
#endif

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
                                                   0,                                           // failure flags (not supported)
                                                   std::numeric_limits<double>::quiet_NaN(),    // delta_yaw (NaN for unknonw)
                                                   std::numeric_limits<double>::quiet_NaN(),    // delta_yaw_velocity (NaN for unknonw)
                                                   _instance + 1);  // gimbal_device_id
}

// return gimbal manager capability flags used by GIMBAL_MANAGER_INFORMATION message
uint32_t AP_Mount_Backend::get_gimbal_manager_capability_flags() const
{
    uint32_t cap_flags = GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT |
                         GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL |
                         GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS |
                         GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL |
                         GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL;

    // roll control
    if (has_roll_control()) {
        cap_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK;
    }

    // pitch control
    if (has_pitch_control()) {
        cap_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK;
    }

    // yaw control
    if (has_pan_control()) {
        cap_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK;
    }

    return cap_flags;
}

// send a GIMBAL_MANAGER_INFORMATION message to GCS
void AP_Mount_Backend::send_gimbal_manager_information(mavlink_channel_t chan)
{
    mavlink_msg_gimbal_manager_information_send(chan,
                                                AP_HAL::millis(),                       // autopilot system time
                                                get_gimbal_manager_capability_flags(),  // bitmap of gimbal manager capability flags
                                                _instance + 1,                          // gimbal device id
                                                radians(_params.roll_angle_min),        // roll_min in radians
                                                radians(_params.roll_angle_max),        // roll_max in radians
                                                radians(_params.pitch_angle_min),       // pitch_min in radians
                                                radians(_params.pitch_angle_max),       // pitch_max in radians
                                                radians(_params.yaw_angle_min),         // yaw_min in radians
                                                radians(_params.yaw_angle_max));        // yaw_max in radians
}

// send a GIMBAL_MANAGER_STATUS message to GCS
void AP_Mount_Backend::send_gimbal_manager_status(mavlink_channel_t chan)
{
    uint32_t flags = GIMBAL_MANAGER_FLAGS_ROLL_LOCK | GIMBAL_MANAGER_FLAGS_PITCH_LOCK;

    if (_yaw_lock) {
        flags |= GIMBAL_MANAGER_FLAGS_YAW_LOCK;
    }

    mavlink_msg_gimbal_manager_status_send(chan,
                                           AP_HAL::millis(),    // autopilot system time
                                           flags,               // bitmap of gimbal manager flags
                                           _instance + 1,       // gimbal device id
                                           mavlink_control_id.sysid,    // primary control system id
                                           mavlink_control_id.compid,   // primary control component id
                                           0,                           // secondary control system id
                                           0);                          // secondary control component id
}

#if AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED
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
#endif

// handle do_mount_control command.  Returns MAV_RESULT_ACCEPTED on success
MAV_RESULT AP_Mount_Backend::handle_command_do_mount_control(const mavlink_command_int_t &packet)
{
    const MAV_MOUNT_MODE new_mode = (MAV_MOUNT_MODE)packet.z;

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

        // warn if lat, lon appear to be in param1,2 instead of param x,y as this indicates
        // sender is relying on a bug in AP-4.2's (and earlier) handling of MAV_CMD_DO_MOUNT_CONTROL
        if (!is_zero(packet.param1) && !is_zero(packet.param2) && packet.x == 0 && packet.y == 0) {
            send_warning_to_GCS("GPS_POINT target invalid");
            return MAV_RESULT_FAILED;
        }

        // param4: altitude in meters
        // x: latitude in degrees * 1E7
        // y: longitude in degrees * 1E7
        const Location target_location {
            packet.x,                       // latitude in degrees * 1E7
            packet.y,                       // longitude in degrees * 1E7
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

// handle do_gimbal_manager_configure.  Returns MAV_RESULT_ACCEPTED on success
// requires original message in order to extract caller's sysid and compid
MAV_RESULT AP_Mount_Backend::handle_command_do_gimbal_manager_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    // sanity check param1 and param2 values
    if ((packet.param1 < -3) || (packet.param1 > UINT8_MAX) || (packet.param2 < -3) || (packet.param2 > UINT8_MAX)) {
        return MAV_RESULT_FAILED;
    }

    // convert negative packet1 and packet2 values
    int16_t new_sysid = packet.param1;
    switch (new_sysid) {
        case -1:
            // leave unchanged
            break;
        case -2:
            // set itself in control
            mavlink_control_id.sysid = msg.sysid;
            mavlink_control_id.compid = msg.compid;
            break;
        case -3:
            // remove control if currently in control
            if ((mavlink_control_id.sysid == msg.sysid) && (mavlink_control_id.compid == msg.compid)) {
                mavlink_control_id.sysid = 0;
                mavlink_control_id.compid = 0;
            }
            break;
        default:
            mavlink_control_id.sysid = packet.param1;
            mavlink_control_id.compid = packet.param2;
            break;
    }

    return MAV_RESULT_ACCEPTED;
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

// write mount log packet
void AP_Mount_Backend::write_log(uint64_t timestamp_us)
{
    // return immediately if no yaw estimate
    float ahrs_yaw = AP::ahrs().yaw;
    if (isnan(ahrs_yaw)) {
        return;
    }

    const auto nanf = AP::logger().quiet_nanf();

    // get_attitude_quaternion and convert to Euler angles
    float roll = nanf;
    float pitch = nanf;
    float yaw_bf = nanf;
    float yaw_ef = nanf;
    if (_frontend.get_attitude_euler(_instance, roll, pitch, yaw_bf)) {
        yaw_ef = wrap_180(yaw_bf + degrees(ahrs_yaw));
    }

    // get mount's target (desired) angles and convert yaw to earth frame
    float target_roll = nanf;
    float target_pitch = nanf;
    float target_yaw = nanf;
    bool target_yaw_is_ef = false;
    IGNORE_RETURN(get_angle_target(target_roll, target_pitch, target_yaw, target_yaw_is_ef));

    // get rangefinder distance
    float rangefinder_dist = nanf;
    IGNORE_RETURN(get_rangefinder_distance(rangefinder_dist));

    const struct log_Mount pkt {
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(LOG_MOUNT_MSG)),
        time_us       : (timestamp_us > 0) ? timestamp_us : AP_HAL::micros64(),
        instance      : _instance,
        desired_roll  : target_roll,
        actual_roll   : roll,
        desired_pitch : target_pitch,
        actual_pitch  : pitch,
        desired_yaw_bf: target_yaw_is_ef ? nanf : target_yaw,
        actual_yaw_bf : yaw_bf,
        desired_yaw_ef: target_yaw_is_ef ? target_yaw : nanf,
        actual_yaw_ef : yaw_ef,
        rangefinder_dist : rangefinder_dist,
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
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

// get angle or rate targets from pilot RC
// target_type will be either ANGLE or RATE, rpy will be the target angle in deg or rate in deg/s
void AP_Mount_Backend::get_rc_target(MountTargetType& target_type, MountTarget& target_rpy) const
{
    // get RC input from pilot
    float roll_in, pitch_in, yaw_in;
    get_rc_input(roll_in, pitch_in, yaw_in);

    // yaw frame
    target_rpy.yaw_is_ef = _yaw_lock;

    // if RC_RATE is zero, targets are angle
    if (_params.rc_rate_max <= 0) {
        target_type = MountTargetType::ANGLE;

        // roll angle
        target_rpy.roll = radians(((roll_in + 1.0f) * 0.5f * (_params.roll_angle_max - _params.roll_angle_min) + _params.roll_angle_min));

        // pitch angle
        target_rpy.pitch = radians(((pitch_in + 1.0f) * 0.5f * (_params.pitch_angle_max - _params.pitch_angle_min) + _params.pitch_angle_min));

        // yaw angle
        if (target_rpy.yaw_is_ef) {
            // if yaw is earth-frame pilot yaw input control angle from -180 to +180 deg
            target_rpy.yaw = yaw_in * M_PI;
        } else {
            // yaw target in body frame so apply body frame limits
            target_rpy.yaw = radians(((yaw_in + 1.0f) * 0.5f * (_params.yaw_angle_max - _params.yaw_angle_min) + _params.yaw_angle_min));
        }
        return;
    }

    // calculate rate targets
    target_type = MountTargetType::RATE;
    const float rc_rate_max_rads = radians(_params.rc_rate_max.get());
    target_rpy.roll = roll_in * rc_rate_max_rads;
    target_rpy.pitch = pitch_in * rc_rate_max_rads;
    target_rpy.yaw = yaw_in * rc_rate_max_rads;
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
float AP_Mount_Backend::MountTarget::get_bf_yaw() const
{
    if (yaw_is_ef) {
        // convert to body-frame
        return wrap_PI(yaw - AP::ahrs().yaw);
    }

    // target is already body-frame
    return yaw;
}

// return earth-frame yaw angle from a mount target
float AP_Mount_Backend::MountTarget::get_ef_yaw() const
{
    if (yaw_is_ef) {
        // target is already earth-frame
        return yaw;
    }

    // convert to earth-frame
    return wrap_PI(yaw + AP::ahrs().yaw);
}

// sets roll, pitch, yaw and yaw_is_ef
void AP_Mount_Backend::MountTarget::set(const Vector3f& rpy, bool yaw_is_ef_in)
{
    roll  = rpy.x;
    pitch = rpy.y;
    yaw   = rpy.z;
    yaw_is_ef = yaw_is_ef_in;
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
            angle_rad.yaw = angle_rad.get_ef_yaw();
        } else {
            angle_rad.yaw = angle_rad.get_bf_yaw();
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

// get target rate in deg/sec. returns true on success
bool AP_Mount_Backend::get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame)
{
    if (mnt_target.target_type == MountTargetType::RATE) {
        roll_degs = degrees(mnt_target.rate_rads.roll);
        pitch_degs = degrees(mnt_target.rate_rads.pitch);
        yaw_degs = degrees(mnt_target.rate_rads.yaw);
        yaw_is_earth_frame = mnt_target.rate_rads.yaw_is_ef;
        return true;
    }
    return false;
}

// get target angle in deg. returns true on success
bool AP_Mount_Backend::get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame)
{
    if (mnt_target.target_type == MountTargetType::ANGLE) {
        roll_deg = degrees(mnt_target.angle_rad.roll);
        pitch_deg = degrees(mnt_target.angle_rad.pitch);
        yaw_deg = degrees(mnt_target.angle_rad.yaw);
        yaw_is_earth_frame = mnt_target.angle_rad.yaw_is_ef;
        return true;
    }
    return false;
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
