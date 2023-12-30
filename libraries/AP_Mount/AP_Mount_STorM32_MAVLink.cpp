
#include "AP_Mount_STorM32_MAVLink.h"

#if HAL_MOUNT_STORM32_MAVLINK_V2_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Notify/AP_Notify.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

//******************************************************
// Quaternion & Euler for Gimbal
//******************************************************
// For most gimbals it is inappropriate to use NED (roll-pitch-yaw) to convert received
// quaternion to Euler angles and vice versa. For most gimbals pitch-roll-yaw is
// appropriate. When the roll angle is zero, both are equivalent, which should be the
// majority of cases. The issue with NED is the gimbal lock at pitch +-90 deg, but pitch
// +-90 deg is a common operation point for gimbals.
// The angles in this driver are thus pitch-roll-yaw Euler.

class GimbalQuaternion : public Quaternion
{
public:
    // inherit constructors
    using Quaternion::Quaternion;

    // create a quaternion from gimbal Euler angles
    void from_gimbal_euler(float roll, float pitch, float yaw);

    // create gimbal Euler angles from a quaternion
    void to_gimbal_euler(float &roll, float &pitch, float &yaw) const;
};

void GimbalQuaternion::from_gimbal_euler(float roll, float pitch, float yaw)
{
    const float cr2 = cosf(roll*0.5f);
    const float cp2 = cosf(pitch*0.5f);
    const float cy2 = cosf(yaw*0.5f);
    const float sr2 = sinf(roll*0.5f);
    const float sp2 = sinf(pitch*0.5f);
    const float sy2 = sinf(yaw*0.5f);

    q1 = cp2*cr2*cy2 - sp2*sr2*sy2;  // ->  cp2*cy2
    q2 = cp2*sr2*cy2 - sp2*cr2*sy2;  // -> -sp2*sy2
    q3 = sp2*cr2*cy2 + cp2*sr2*sy2;  // ->  sp2*cy2
    q4 = sp2*sr2*cy2 + cp2*cr2*sy2;  // ->  cp2*sy2
}

void GimbalQuaternion::to_gimbal_euler(float &roll, float &pitch, float &yaw) const
{
    pitch = atan2f(2.0f*(q1*q3 - q2*q4), 1.0f - 2.0f*(q2*q2 + q3*q3));  // -R31 / R33 = -(-spcr) / cpcr
    roll = safe_asin(2.0f*(q1*q2 + q3*q4));                             // R32 = sr
    yaw = atan2f(2.0f*(q1*q4 - q2*q3), 1.0f - 2.0f*(q2*q2 + q4*q4));    // -R12 / R22 = -(-crsy) / crcy
}

//******************************************************
// AP_Mount_STorM32_MAVLink, main class
//******************************************************

void AP_Mount_STorM32_MAVLink::init()
{
    AP_Mount_Backend::init();

    _mode = MAV_MOUNT_MODE_RC_TARGETING; // irrelevant, will be later set to default in frontend init()

    _flags_from_gimbal_client = UINT32_MAX; // the UINT32_MAX is important!

    _current_angles = {0.0f, 0.0f, 0.0f, NAN}; // the NAN is important!
}

// called by all vehicles with 50 Hz, using the scheduler
// several vehicles do not support fast_update(), so let's go with this
// priority of update() not very high, so no idea how reliable that is, may be not so good
// STorM32-Link wants 25 Hz, so we update at 25 Hz and 12.5 Hz respectively
void AP_Mount_STorM32_MAVLink::update()
{
    update_target_angles(); // update at 50 Hz (RC_TARGETING handling assumes this)

    switch (_task_counter) {
        case TASK_SLOT0:
        case TASK_SLOT2:
            if (_compid) { // we send it as soon as we have found the gimbal
                send_autopilot_state_for_gimbal_device();
            }
            break;

        case TASK_SLOT1:
            if (_initialised) { // we do it when the startup sequence has been fully completed
                send_target_angles();
            }
            break;

        case TASK_SLOT3:
            if (_compid) { // we send it as soon as we have found the gimbal
                if (!_got_radio_rc_channels) { // don't send it if we have seen RADIO_RC_CHANNELS messages
                    send_rc_channels();
                }
            }
            break;
    }

    _task_counter++;
    if (_task_counter > TASK_SLOT3) _task_counter = TASK_SLOT0;

    update_send_banner();

    if (!_initialised) {
        find_gimbal();
        return;
    }

    update_manager_status();

    uint32_t tnow_ms = AP_HAL::millis();

    if ((tnow_ms - _checks_tlast_ms) >= 1000) { // do every 1 sec
        _checks_tlast_ms = tnow_ms;
        update_checks();
    }

    if ((tnow_ms - _send_system_time_tlast_ms) >= 5000) { // every 5 sec is really plenty
        _send_system_time_tlast_ms = tnow_ms;
        send_system_time();
    }
}

//------------------------------------------------------
// Mode handling and targeting functions
//------------------------------------------------------

// flags coming from gimbal manager messages and commands
bool AP_Mount_STorM32_MAVLink::handle_gimbal_manager_flags(uint32_t flags)
{
    // check flags for change to RETRACT
    if (flags & GIMBAL_MANAGER_FLAGS_RETRACT) {
        set_mode(MAV_MOUNT_MODE_RETRACT);
    } else
    // check flags for change to NEUTRAL
    if (flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) {
        set_mode(MAV_MOUNT_MODE_NEUTRAL);
    }

    _flags_from_gimbal_client = flags;

    update_gimbal_device_flags();

    // if not in mavlink targeting don't accept the angle/rate settings
    // this is needed since backend's set_angle_target(), set_rate_target() do set mode to mavlink targeting
    // I think one should change backend's functions, but also somewhat makes sense
    if (get_mode() != MAV_MOUNT_MODE_MAVLINK_TARGETING) {
        return false; // don't accept angle/rate setting
    }

    // driver currently does not support yaw LOCK
    // front-end is digesting GIMBAL_MANAGER_FLAGS_YAW_LOCK to determine yaw_is_earth_frame
    // we could make it to modify the flag, but for moment let's be happy
    if (flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK) {
        return false; // don't accept angle/rate setting
    }

    // STorM32 expects the "new" format, i.e. that only one of them is set, otherwise it rejects
    if (!(flags & GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME) && !(flags & GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME)) {
        return false; // don't accept angle/rate setting
    }

    // STorM32 currently only supports GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME
    if (flags & GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME) {
        return false; // don't accept angle/rate setting
    }

    return true; // accept angle/rate setting
}

void AP_Mount_STorM32_MAVLink::update_gimbal_device_flags()
{
    _flags_for_gimbal_device = 0;

    // map mode

    switch (get_mode()) {
        case MAV_MOUNT_MODE_RETRACT:
            _flags_for_gimbal_device |= GIMBAL_DEVICE_FLAGS_RETRACT;
            break;
        case MAV_MOUNT_MODE_NEUTRAL:
            _flags_for_gimbal_device |= GIMBAL_DEVICE_FLAGS_NEUTRAL;
            break;
        default:
            break;
    }

    // account for gimbal manager flags

    if (_flags_from_gimbal_client != UINT32_MAX) {
        if (_flags_from_gimbal_client & GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE) { // exclusive overrules mixed
            _flags_for_gimbal_device |= GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE;
        } else
        if (_flags_from_gimbal_client & GIMBAL_MANAGER_FLAGS_RC_MIXED) {
            _flags_for_gimbal_device |= GIMBAL_DEVICE_FLAGS_RC_MIXED;
        }
    } else {
        // for as long as no gimbal manager message has been sent to the fc, enable rc mixed.
        // Should avoid user confusion when choosing the gimbal device operation mode.
        _flags_for_gimbal_device |= GIMBAL_DEVICE_FLAGS_RC_MIXED;
    }

    // driver currently does not support pitch,roll follow, only pitch,roll lock
    _flags_for_gimbal_device |= GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK;

    // driver currently does not support yaw lock, only yaw follow
    // -> _flags_for_gimbal_device &=~ GIMBAL_DEVICE_FLAGS_YAW_LOCK;

    // frame flags

    // set either YAW_IN_VEHICLE_FRAME or YAW_IN_EARTH_FRAME, to indicate new message format, STorM32 will reject otherwise
    _flags_for_gimbal_device |= GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
}

void AP_Mount_STorM32_MAVLink::send_target_angles()
{
    // just send stubbornly at 12.5 Hz, no check if get_target_angles() made a change

    update_gimbal_device_flags();

    if (mnt_target.target_type == MountTargetType::RATE) {
        // we ignore it. One may think to just send angle_rad, but if yaw is earth frame
        // this could result in pretty strange behavior. So better ignore.
        // Should happen only in MAV_MOUNT_MODE_RC_TARGETING, so no need to test for this
        // explicitly.
        return;
    }

    if (_protocol == Protocol::GIMBAL_DEVICE) {
        send_gimbal_device_set_attitude();
    } else {
        send_cmd_do_mount_control();
    }
}

// update_angle_target_from_rate() assumes a 50 Hz update rate!
// TODO: one should allow angles outside of +-PI, to go shortest path in case of turn around
void AP_Mount_STorM32_MAVLink::update_target_angles()
{
    // update based on mount mode
    switch ((uint8_t)get_mode()) {

        // move mount to a "retracted" position
        // -> ANGLE
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &target = _params.retract_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            break;
        }

        // move mount to a neutral position, typically pointing forward
        // -> ANGLE
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &target = _params.neutral_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            break;
        }

        // point to the angles given by a mavlink message or mission command
        // -> ANGLE
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // mnt_target should have already been populated by set_angle_target() or set_rate_target().
            // SToRM32 doesn't support rate, so update target angle from rate if necessary.
            if (mnt_target.target_type == MountTargetType::RATE) {
                update_angle_target_from_rate(mnt_target.rate_rads, mnt_target.angle_rad);
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        // update targets using pilot's RC inputs
        // -> can be RATE, will be ignored
        case MAV_MOUNT_MODE_RC_TARGETING: {
            MountTarget rc_target;
            get_rc_target(mnt_target.target_type, rc_target);
            switch (mnt_target.target_type) {
            case MountTargetType::ANGLE:
                mnt_target.angle_rad = rc_target;
                break;
            case MountTargetType::RATE:
                mnt_target.rate_rads = rc_target;
                break;
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        // ATTENTION: angle_rad.yaw_is_ef = true
        // -> ANGLE
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to another vehicle
        // ATTENTION: angle_rad.yaw_is_ef = true
        // -> ANGLE
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to Home location
        // ATTENTION: angle_rad.yaw_is_ef = true
        // -> ANGLE
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // account for range limits
    // only do the yaw axis (should be done by STorM32 supervisor, but doesn't hurt)

    if (_got_device_info) return;
    if (_device_info.cap_flags & GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW) return;
    if (isnan(_device_info.yaw_min) || isnan(_device_info.yaw_max)) return;

    // I believe currently angles are inside +-PI, no wrapPI needed
    // TODO: if copter, copter might yaw by what the gimbal can't do

    if (mnt_target.angle_rad.yaw < _device_info.yaw_min) mnt_target.angle_rad.yaw = _device_info.yaw_min;
    if (mnt_target.angle_rad.yaw > _device_info.yaw_max) mnt_target.angle_rad.yaw = _device_info.yaw_max;
}

//------------------------------------------------------
// Gimbal and protocol discovery
//------------------------------------------------------

void AP_Mount_STorM32_MAVLink::find_gimbal()
{
    // search for gimbal only until vehicle is armed
    if (hal.util->get_soft_armed()) {
        return;
    }

    uint32_t tnow_ms = AP_HAL::millis();

    // search for gimbal in routing table
    if (!_compid) {
        // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
        uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
        bool found = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid, _chan);
        if (!found || (_sysid != mavlink_system.sysid)) {
            // have not yet found a gimbal so return
            return;
        }

        _compid = compid;
        _request_device_info_tlast_ms = (tnow_ms < 900) ? 0 : tnow_ms - 900; // start sending requests in 100 ms
    }

    // request GIMBAL_DEVICE_INFORMATION
    // STorM32 provides this also for mount mode
    if (!_got_device_info) {
        if (tnow_ms - _request_device_info_tlast_ms > 1000) {
            _request_device_info_tlast_ms = tnow_ms;
            send_cmd_request_gimbal_device_information();
        }
        return;
    }

    // we don't know yet what we should do
    if (_protocol == Protocol::UNDEFINED) {
        return;
    }

    _initialised = true;
}

void AP_Mount_STorM32_MAVLink::determine_protocol(const mavlink_message_t &msg)
{
    if (msg.sysid != _sysid || msg.compid != _compid) { // this msg is not from our gimbal
        return;
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_MOUNT_STATUS:
            _protocol = Protocol::MOUNT;
            break;
            case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
            _protocol = Protocol::GIMBAL_DEVICE;
            break;
    }
}

//------------------------------------------------------
// Gimbal control flags
//------------------------------------------------------

bool AP_Mount_STorM32_MAVLink::has_roll_control() const
{
    if (_protocol == Protocol::GIMBAL_DEVICE) {
        return (_device_info.cap_flags & GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS);
    }
    if (_protocol == Protocol::MOUNT) {
        return (_params.roll_angle_min < _params.roll_angle_max);
    }
    return false;
}

bool AP_Mount_STorM32_MAVLink::has_pitch_control() const
{
    if (_protocol == Protocol::GIMBAL_DEVICE) {
        return (_device_info.cap_flags & GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS);
    }
    if (_protocol == Protocol::MOUNT) {
        return (_params.pitch_angle_min < _params.pitch_angle_max);
    }
    return false;
}

bool AP_Mount_STorM32_MAVLink::has_pan_control() const
{
    if (_protocol == Protocol::GIMBAL_DEVICE) {
        return (_device_info.cap_flags & GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS);
    }
    if (_protocol == Protocol::MOUNT) {
        return yaw_range_valid();
    }
    return false;
}

//------------------------------------------------------
// Gimbal attitude and rate
//------------------------------------------------------

bool AP_Mount_STorM32_MAVLink::get_attitude_quaternion(Quaternion &att_quat)
{
    if (!_initialised) {
        return false;
    }

    // we set roll to zero since wrong Euler's
    att_quat.from_euler(0.0f, _current_angles.pitch, _current_angles.yaw_bf);

    return true;
}

bool AP_Mount_STorM32_MAVLink::get_angular_velocity(Vector3f& rates)
{
    if (!_initialised) {
        return false;
    }

    if (isnan(_current_omega.x) || isnan(_current_omega.y) || isnan(_current_omega.z)) return false;

    rates.x = _current_omega.x;
    rates.y = _current_omega.y;
    rates.z = _current_omega.z;

    return true;
}

//------------------------------------------------------
// MAVLink handle functions
//------------------------------------------------------

void AP_Mount_STorM32_MAVLink::handle_gimbal_device_information(const mavlink_message_t &msg)
{
    // this msg is not from our gimbal
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    mavlink_msg_gimbal_device_information_decode(&msg, &_device_info);

    // we could check here for sanity of _device_info.gimbal_device_id, but let's just be happy

    // correct parameters from gimbal information
    if (!isnan(_device_info.roll_min) && !isnan(_device_info.roll_max)) {
        if (degrees(_device_info.roll_min) > _params.roll_angle_min) _params.roll_angle_min.set(degrees(_device_info.roll_min));
        if (degrees(_device_info.roll_max) < _params.roll_angle_max) _params.roll_angle_max.set(degrees(_device_info.roll_max));
    }
    if (!isnan(_device_info.pitch_min) && !isnan(_device_info.pitch_max)) {
        if (degrees(_device_info.pitch_min) > _params.pitch_angle_min) _params.pitch_angle_min.set(degrees(_device_info.pitch_min));
        if (degrees(_device_info.pitch_max) < _params.pitch_angle_max) _params.pitch_angle_max.set(degrees(_device_info.pitch_max));
    }
    if (!isnan(_device_info.yaw_min) && !isnan(_device_info.yaw_max)) {
        if (degrees(_device_info.yaw_min) > _params.yaw_angle_min) _params.yaw_angle_min.set(degrees(_device_info.yaw_min));
        if (degrees(_device_info.yaw_max) < _params.yaw_angle_max) _params.yaw_angle_max.set(degrees(_device_info.yaw_max));
    }

    // extract version int
    // v2.68b <-> firmware_version 00 01(a) 68 2 <-> version int 268 (ignore char part)
    _device_version_int = (_device_info.firmware_version & 0x000000FF) * 100 + ((_device_info.firmware_version & 0x0000FF00) >> 8);

    // mark it as having been found
    _got_device_info = true;

    // display gimbal info to user
    send_banner();
}

void AP_Mount_STorM32_MAVLink::handle_gimbal_device_attitude_status(const mavlink_message_t &msg)
{
    if (_protocol != Protocol::GIMBAL_DEVICE) {
        return;
    }

    // this msg is not from our gimbal
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    mavlink_gimbal_device_attitude_status_t payload;
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &payload);

    // we could check here for sanity of _device_info.gimbal_device_id, but let's just be happy

    // get relevant data
    _device_status.received_flags = payload.flags;
    _device_status.received_failure_flags = payload.failure_flags;

    // used for health check
    _device_status.received_tlast_ms = AP_HAL::millis();

    // Euler angles
    GimbalQuaternion q(payload.q[0], payload.q[1], payload.q[2], payload.q[3]);
    q.to_gimbal_euler(_current_angles.roll, _current_angles.pitch, _current_angles.yaw_bf);

    _current_angles.delta_yaw = payload.delta_yaw;

    _current_omega.x = payload.angular_velocity_x;
    _current_omega.y = payload.angular_velocity_y;
    _current_omega.z = payload.angular_velocity_z;
}

void AP_Mount_STorM32_MAVLink::handle_message_extra(const mavlink_message_t &msg)
{
    if (_protocol == Protocol::UNDEFINED) { // implies !_initialised && _compid
        determine_protocol(msg);
        return;
    }

    // NOTE: This piece of code depends on the fate of the RADIO_RC_CHANNELS message.
    // It is here to show off the intention.
    // listen to RADIO_RC_CHANNELS messages to stop sending RC_CHANNELS
#ifdef MAVLINK_MSG_ID_RADIO_RC_CHANNELS
    if (msg.msgid == MAVLINK_MSG_ID_RADIO_RC_CHANNELS) { // 60045
        _got_radio_rc_channels = true;
    }
#endif

    // this msg is not from our gimbal
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t payload;
            mavlink_msg_heartbeat_decode(&msg, &payload);
            uint8_t storm32_state = (payload.custom_mode & 0xFF);
            _gimbal_armed = ((storm32_state == STorM32State::NORMAL) || (storm32_state == STorM32State::STARTUP_FASTLEVEL));
            if ((payload.custom_mode & 0x80000000) == 0) { // don't follow all changes, but just toggle it to true once
                _gimbal_prearmchecks_ok = true;
            }
            _gimbal_error_flags = (payload.custom_mode & 0x00FFFF00) >> 8;
            break; }

        case MAVLINK_MSG_ID_MOUNT_STATUS: {
            if (_protocol != Protocol::MOUNT) break;
            _mount_status.received_tlast_ms = AP_HAL::millis(); // used for health check
            mavlink_mount_status_t payload;
            mavlink_msg_mount_status_decode(&msg, &payload);
            _current_angles.pitch = radians((float)payload.pointing_a * 0.01f);
            _current_angles.roll = radians((float)payload.pointing_b * 0.01f);
            _current_angles.yaw_bf = radians((float)payload.pointing_c * 0.01f);
            _current_angles.delta_yaw = NAN;
            _current_omega = { NAN, NAN, NAN };
            break; }
    }
}

//------------------------------------------------------
// MAVLink send functions
//------------------------------------------------------

void AP_Mount_STorM32_MAVLink::send_cmd_request_gimbal_device_information()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    mavlink_msg_command_long_send(
        _chan,
        _sysid, _compid,
        MAV_CMD_REQUEST_MESSAGE,                    // command
        0,                                          // confirmation
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,   // param1
        0, 0, 0, 0, 0, 0                            // param2 .. param7
        );
}

// called by send_target_angles()
void AP_Mount_STorM32_MAVLink::send_cmd_do_mount_control()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // send command_long command containing a do_mount_control command
    // Note: pitch and yaw are reversed
    // ATTENTION: uses get_bf_yaw() to ensure body frame, which uses ahrs.yaw, not delta_yaw!
    mavlink_msg_command_long_send(
        _chan,
        _sysid, _compid,
        MAV_CMD_DO_MOUNT_CONTROL,                       // command
        0,                                              // confirmation
        -degrees(mnt_target.angle_rad.pitch),           // param1
        degrees(mnt_target.angle_rad.roll),             // param2
        -degrees(mnt_target.angle_rad.get_bf_yaw()),    // param3
        0, 0, 0,                                        // param4 .. param6
        get_mode()                                      // param7
        );
}

// called by send_target_angles()
// _flags_for_gimbal_device were just updated, so are correct for sure
void AP_Mount_STorM32_MAVLink::send_gimbal_device_set_attitude()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // convert Euler angles to quaternion
    float target_yaw_bf;
    if (mnt_target.angle_rad.yaw_is_ef) {
        if (isnan(_current_angles.delta_yaw)) { // we don't have a valid yaw_ef
            target_yaw_bf = mnt_target.angle_rad.get_bf_yaw();
        } else {
            // TODO: handle turn around
            target_yaw_bf = wrap_PI(mnt_target.angle_rad.yaw - _current_angles.delta_yaw);
        }
    } else {
        target_yaw_bf = mnt_target.angle_rad.yaw;
    }

    GimbalQuaternion q;
    q.from_euler(mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, target_yaw_bf);

    float qa[4] = {q.q1, q.q2, q.q3, q.q4};

    mavlink_msg_gimbal_device_set_attitude_send(
        _chan,
        _sysid, _compid,
        _flags_for_gimbal_device,   // gimbal device flags
        qa,                         // attitude as a quaternion
        NAN, NAN, NAN               // angular velocities
        );
}

void AP_Mount_STorM32_MAVLink::send_autopilot_state_for_gimbal_device()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE)) {
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();

    // get vehicle attitude
    Quaternion q;
    if (!ahrs.get_quaternion(q)) { // it returns a bool, so it's a good idea to consider it
        q.q1 = q.q2 = q.q3 = q.q4 = NAN;
    }

    // get vehicle velocity
    // comment in AP_AHRS.cpp says "Must only be called if have_inertial_nav() is true", but probably not worth checking
    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) { // it returns a bool, so it's a good idea to consider it
        vel.x = vel.y = vel.z = 0.0f; // or NAN ?
    }

    // get vehicle angular velocity z
    float angular_velocity_z = ahrs.get_yaw_rate_earth();

    // get commanded yaw rate
    // see https://github.com/ArduPilot/ardupilot/issues/22564
    float yawrate = NAN;
    const AP_Vehicle *vehicle = AP::vehicle();
    Vector3f rate_ef_targets;
    if ((vehicle != nullptr) && vehicle->get_rate_ef_targets(rate_ef_targets)) {
        yawrate = rate_ef_targets.z;
    }

    // determine estimator status
    uint16_t nav_estimator_status = 0;

    const uint32_t ESTIMATOR_MASK = (
            ESTIMATOR_ATTITUDE |
            ESTIMATOR_VELOCITY_HORIZ | ESTIMATOR_VELOCITY_VERT |
            ESTIMATOR_POS_HORIZ_REL | ESTIMATOR_POS_HORIZ_ABS |
            ESTIMATOR_POS_VERT_ABS | ESTIMATOR_POS_VERT_AGL |
            ESTIMATOR_CONST_POS_MODE |
            ESTIMATOR_PRED_POS_HORIZ_REL | ESTIMATOR_PRED_POS_HORIZ_ABS);

    nav_filter_status nav_status;
    if (ahrs.get_filter_status(nav_status)) {
        nav_estimator_status = (uint16_t)(nav_status.value & ESTIMATOR_MASK);
    }

    uint16_t estimator_status = 0;

    const bool ahrs_healthy = ahrs.healthy(); // it's a bit costly
    if (!_tahrs_healthy_ms && ahrs_healthy) {
        _tahrs_healthy_ms = AP_HAL::millis();
    }

    // delay by 3 sec to get past "quaternion flip"
    if (ahrs_healthy && (nav_estimator_status & ESTIMATOR_ATTITUDE) && ((AP_HAL::millis() - _tahrs_healthy_ms) > 3000)) {
        estimator_status |= ESTIMATOR_ATTITUDE; // -> QFix
        if (ahrs.initialised() && (nav_estimator_status & ESTIMATOR_VELOCITY_VERT) && (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D)) {
            estimator_status |= ESTIMATOR_VELOCITY_VERT; // -> AHRSFix
        }
    }

    // determine landed state
    uint8_t landed_state = (uint8_t)gcs().get_landed_state();

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    // For copter the landed state is modified such as to reflect the 2 sec pre-take-off period.
    if ((landed_state == MAV_LANDED_STATE_ON_GROUND) && AP::notify().flags.armed) {
        landed_state = (_device_version_int < 270) ? 5 : 128;
    }
#endif

    // ready to send
    float qa[4] = {q.q1, q.q2, q.q3, q.q4};

    mavlink_msg_autopilot_state_for_gimbal_device_send(
        _chan,
        _sysid, _compid,
        AP_HAL::micros64(),         // uint64_t time_boot_us
        qa,                         // attitude quaternion
        0,                          // uint32_t q_estimated_delay_us,
        vel.x, vel.y, vel.z,        // angular velocity vx, vy, vz
        0,                          // uint32_t v_estimated_delay_us,
        yawrate,                    // float feed_forward_angular_velocity_z
        estimator_status, landed_state,
        angular_velocity_z          // float angular_velocity_z
        );
}

void AP_Mount_STorM32_MAVLink::send_system_time()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, SYSTEM_TIME)) {
        return;
    }

    uint64_t time_unix = 0;
    AP::rtc().get_utc_usec(time_unix); // may fail, leaving time_unix at 0

    if (!time_unix) return; // no unix time available, so no reason to send

    mavlink_msg_system_time_send(
        _chan,
        time_unix,          // uint64_t time_unix_usec
        AP_HAL::millis()    // uint32_t time_boot_ms
        );
}

void AP_Mount_STorM32_MAVLink::send_rc_channels()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, RC_CHANNELS)) {
        return;
    }

    // rc().channel(ch)->get_radio_in(), RC_Channels::get_radio_in(ch), and so on
    // these are not the same as hal.rcin->read(), since radio_in can be set by override
    // so we use hal.rcin->read()

    #define RCHALIN(ch_index)  hal.rcin->read(ch_index)

    mavlink_msg_rc_channels_send(
        _chan,
        AP_HAL::millis(),   // uint32_t time_boot_ms
        16,                 // uint8_t chancount, STorM32 won't handle more than 16 anyhow
        RCHALIN(0), RCHALIN(1), RCHALIN(2), RCHALIN(3), RCHALIN(4), RCHALIN(5), RCHALIN(6), RCHALIN(7), // uint16_t chan1_raw ..
        RCHALIN(8), RCHALIN(9), RCHALIN(10), RCHALIN(11), RCHALIN(12), RCHALIN(13), RCHALIN(14), RCHALIN(15), // .. chan16_raw
        0, 0,               // uint16_t chan17_raw .. chan18_raw
        255                 // uint8_t rssi, 255: invalid/unknown
        );
}

void AP_Mount_STorM32_MAVLink::send_banner()
{
    // postpone sending by few seconds, to avoid multiple sends
    _request_send_banner_ms = AP_HAL::millis();
}

void AP_Mount_STorM32_MAVLink::update_send_banner()
{
    if (!_request_send_banner_ms) return; // no request

    uint32_t tnow_ms = AP_HAL::millis();
    if ((tnow_ms - _request_send_banner_ms) < 3500) return; // not yet time to send
    _request_send_banner_ms = 0;

    if (_got_device_info) {
        // we have lots of info
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: gimbal at %u", _instance+1, _compid);

        // convert firmware version to STorM32 convention
        char c = (_device_info.firmware_version & 0x00FF0000) >> 16;
        if (c == '\0') c = ' '; else c += 'a' - 1;

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: %s v%u.%u%c",
                _instance + 1,
                _device_info.model_name,
                (unsigned)(_device_info.firmware_version & 0x000000FF),
                (unsigned)((_device_info.firmware_version & 0x0000FF00) >> 8),
                c
                );

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks %s", _instance+1, (_prearmchecks_passed) ? "passed" : "fail");

    } else
    if (_compid) {
        // we have some info
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: gimbal at %u", _instance+1, _compid);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks %s", _instance+1, (_prearmchecks_passed) ? "passed" : "fail");

    } else {
        // we don't know yet anything
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: no gimbal yet", _instance+1);
    }
}

//------------------------------------------------------
// MAVLink gimbal manager send functions
//------------------------------------------------------

// send a GIMBAL_MANAGER_INFORMATION message to GCS
void AP_Mount_STorM32_MAVLink::send_gimbal_manager_information(mavlink_channel_t chan)
{
    // space already checked by streamer

    // The request to send this message should be NACKed for as long as
    // the gimbal device was not found or its device info was not received.
    // Not done currently, so as workaround don't send if not yet available.
    // Should make third parties to repeat request.

    if (!_got_device_info) return;

    // There are few specific gimbal manager capability flags, which are not used.
    // So we simply can carry forward the cap_flags received from the gimbal.

    uint32_t cap_flags = _device_info.cap_flags;

    // This driver does not support all capabilities, so we erase them.
    // Note: This can mean that the gimbal device and gimbal manager capability flags
    // may be different, and any third party which mistakenly thinks it can use those from
    // the gimbal device messages may get confused. Their fault.

    cap_flags &=~ (GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW |
                   GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW |
                   GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK |
                   GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME);

    mavlink_msg_gimbal_manager_information_send(
        chan,
        AP_HAL::millis(),           // autopilot system time
        cap_flags,                  // bitmap of gimbal manager capability flags
        _compid,                    // gimbal device id
        _device_info.roll_min,      // roll_min in radians
        _device_info.roll_max,      // roll_max in radians
        _device_info.pitch_min,     // pitch_min in radians
        _device_info.pitch_max,     // pitch_max in radians
        _device_info.yaw_min,       // yaw_min in radians
        _device_info.yaw_max        // yaw_max in radians
        );
}

// return gimbal device id
uint8_t AP_Mount_STorM32_MAVLink::get_gimbal_device_id() const
{
    if (_instance == 0) {
        return MAV_COMP_ID_GIMBAL;
    } else
    if (_instance <= 5) {
        return MAV_COMP_ID_GIMBAL2 + _instance - 1;
    }
    return MAV_COMP_ID_GIMBAL; // should not happen
}

// return gimbal manager flags. Used by GIMBAL_MANAGER_STATUS message.
uint32_t AP_Mount_STorM32_MAVLink::get_gimbal_manager_flags() const
{
    // There are currently no specific gimbal manager flags. So one simply
    // can carry forward the _flags received from the gimbal.

    // Note: This driver does not support all capabilities, but this
    // should never be a problem since any third party should strictly adhere
    // to the capability flags obtained from the gimbal manager.

    return _device_status.received_flags;
}

//------------------------------------------------------
// Prearm & healthy functions
//------------------------------------------------------

const uint32_t FAILURE_FLAGS =
        GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR;


bool AP_Mount_STorM32_MAVLink::has_failures(char* s)
{
    uint32_t failure_flags = (_protocol == Protocol::GIMBAL_DEVICE) ? _device_status.received_failure_flags : _gimbal_error_flags;

    s[0] = '\0';
    if ((failure_flags & FAILURE_FLAGS) > 0) {
        if (failure_flags & GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR) strcat(s, "mot,");
        if (failure_flags & GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR) strcat(s, "enc,");
        if (failure_flags & GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR) strcat(s, "volt,");
        if (s[0] != '\0') {
            s[strlen(s)-1] = '\0';
        } else {
            strcpy(s, "err flags");
        }
        return true;
    }
    return false;
}

bool AP_Mount_STorM32_MAVLink::is_healthy()
{
    if (_protocol == Protocol::GIMBAL_DEVICE) {
        // unhealthy if attitude status is not received within the last second
        if ((AP_HAL::millis() - _device_status.received_tlast_ms) > 1000) {
            return false;
        }

        // check failure flags
        if ((_device_status.received_failure_flags & FAILURE_FLAGS) > 0) {
            return false;
        }
    } else {
        // unhealthy if mount status is not received within the last second
        if ((AP_HAL::millis() - _mount_status.received_tlast_ms) > 1000) {
            return false;
        }

        // check failure flags // are set since v2.68b
        if ((_gimbal_error_flags & FAILURE_FLAGS) > 0) {
            return false;
        }
    }

    return true;
}

// is called with 1 Hz from update loop
void AP_Mount_STorM32_MAVLink::update_checks()
{
char txt[255];

    if (!(AP::arming().get_enabled_checks() & AP_Arming::ArmingChecks::ARMING_CHECK_ALL ||
          AP::arming().get_enabled_checks() & AP_Arming::ArmingChecks::ARMING_CHECK_CAMERA)) {
        if (_initialised && _gimbal_prearmchecks_ok) {
            if (!_prearmchecks_passed && !_request_send_banner_ms) { // state changed and not going to send soon anyways
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks passed", _instance+1);
            }
            _prearmchecks_passed = true;
        }
        return;
    }

    if (_armingchecks_running) _armingchecks_running--; // count down

    if (_prearmchecks_passed && AP::notify().flags.armed &&
        !AP::arming().option_enabled(AP_Arming::Option::DISABLE_PREARM_DISPLAY)) {
        bool checks = is_healthy();

        if (_checks_last && !checks) { // checks went from true to false
            if (has_failures(txt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: failures: %s", _instance+1, txt);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: failure: gimbal lost", _instance+1);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "MNT%u: not healthy", _instance+1);
        }
        if (!_checks_last && checks) { // checks went from false to true
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: healthy again", _instance+1);
        }
        _checks_last = checks;
    }

    // don't do anything further if arming mechanism is not enabled
    if (!_armingchecks_running) {
        return;
        _healthy = true;
    }

    // do these only at startup, until prearm checks have been passed once
    if (!_prearmchecks_passed) {

        if ((AP_HAL::millis() - _prearmcheck_sendtext_tlast_ms) > 30000) { // we haven't send it for a long while
            _prearmcheck_sendtext_tlast_ms = AP_HAL::millis();
            if (!_initialised || !_gimbal_prearmchecks_ok || !_gimbal_armed) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: not armed", _instance+1);
            } else
            if (has_failures(txt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: %s", _instance+1, txt);
            }
        }

        // unhealthy until gimbal has fully passed the startup sequence
        // _initialised:            -> gimbal found (HB received,_compid != 0)
        //                          -> device info obtained (_got_device_info = true)
        //                          -> status message received, protocol set (_protocol != PROTOCOL_UNDEFINED)
        // _gimbal_prearmchecks_ok: -> gimbal HB reported gimbal's prearmchecks ok
        // _gimbal_armed:           -> gimbal HB reported gimbal is in normal state
        if (!_initialised || !_gimbal_prearmchecks_ok || !_gimbal_armed) {
            _healthy = false;
            return;
        }
    }

    // do these continuously
    // - check failures
    // - check connection to gimbal
    bool checks = is_healthy();

    if (_prearmchecks_passed) { // we are past prearm checks
        if (_checks_last && !checks) { // checks went from true to false
            if (has_failures(txt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: %s", _instance+1, txt);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: gimbal lost", _instance+1);
            }
        }
        if (!_checks_last && checks) { // checks went from false to true
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks passed", _instance+1);
        }
        _checks_last = checks;
        _healthy = checks;
        return;
    }

    _checks_last = checks;

    if (!checks) {
        _healthy = false;
        return;
    }

    // if we get this far in prearm state, then mount is healthy

    // if we got this far the first time we inform the gcs
    if (!_prearmchecks_passed) {
        _prearmchecks_passed = true;
         GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks passed", _instance+1);
    }

    _healthy = true;
}

// return true if healthy
// this is called when ARMING_CHECK_ALL or ARMING_CHECK_CAMERA is set, and if not armed, else not
// is called with 1 Hz
bool AP_Mount_STorM32_MAVLink::healthy() const
{
    _armingchecks_running = 2; // to signal that ArduPilot arming check mechanism is active

    return _healthy;
}

//------------------------------------------------------
// Gimbal manager status function
//------------------------------------------------------

void AP_Mount_STorM32_MAVLink::update_manager_status()
{
    // check if status has changed
    if (_manager_status.flags_last != get_gimbal_manager_flags() ||
        _manager_status.primary_sysid_last != mavlink_control_id.sysid ||
        _manager_status.primary_compid_last != mavlink_control_id.compid) {

        _manager_status.flags_last = get_gimbal_manager_flags();
        _manager_status.primary_sysid_last = mavlink_control_id.sysid;
        _manager_status.primary_compid_last = mavlink_control_id.compid;

        _manager_status.fast = 3;
    }

    uint32_t tnow_ms = AP_HAL::millis();

    if (!_manager_status.fast) {
        if ((tnow_ms - _manager_status.tlast_ms) >= 2000) { // do every 2 sec
            _manager_status.tlast_ms = tnow_ms;
            gcs().send_message(MSG_GIMBAL_MANAGER_STATUS);
        }
        return;
    }

    // we are in fast response

    if (_manager_status.fast >= 3) { // status has just changed, so react immediately
        _manager_status.fast = 2;
        _manager_status.tlast_ms = tnow_ms;
        gcs().send_message(MSG_GIMBAL_MANAGER_STATUS);
    } else
    if ((tnow_ms - _manager_status.tlast_ms) >= 250) { // do every 250 ms
        _manager_status.fast--;
        _manager_status.tlast_ms = tnow_ms;
        gcs().send_message(MSG_GIMBAL_MANAGER_STATUS);
    }
}

//------------------------------------------------------
// MAVLink mount status forwarding
//------------------------------------------------------

// send a MOUNT_STATUS message to GCS
// make MissionPlanner and alike happy and gives parties a chance to know the mode
void AP_Mount_STorM32_MAVLink::send_gimbal_device_attitude_status(mavlink_channel_t chan)
{
    // space already checked by streamer
    // did check for space of GIMBAL_DEVICE_ATTITUDE_STATUS, but MOUNT_STATUS is (much) smaller, so no issue

    if (_compid != MAV_COMP_ID_GIMBAL) { // do it only for the 1st gimbal
        return;
    }

    mavlink_msg_mount_status_send(
        chan,
        0,          // uint8_t target_system
        0,          // uint8_t target_component
        (int32_t)(degrees(_current_angles.pitch) * 100.0f),     // int32_t pointing_a
        (int32_t)(degrees(_current_angles.roll) * 100.0f),      // int32_t pointing_b
        (int32_t)(degrees(_current_angles.yaw_bf) * 100.0f),    // int32_t pointing_c
        get_mode()  // uint8_t mount_mode
        );
}

#endif // HAL_MOUNT_STORM32_MAVLINK_V2_ENABLED


