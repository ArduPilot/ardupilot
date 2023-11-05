#include "AP_Mount_SToRM32_serial.h"

#if HAL_MOUNT_STORM32SERIAL_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_SerialManager/AP_SerialManager.h>

// init - performs any required initialisation for this instance
void AP_Mount_SToRM32_serial::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Gimbal, 0);
    if (_port) {
        _initialised = true;
    }
    AP_Mount_Backend::init();

}

// update mount position - should be called periodically
void AP_Mount_SToRM32_serial::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &target = _params.retract_angles.get();
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &target = _params.neutral_angles.get();
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // mnt_target should have already been filled in by set_angle_target() or set_rate_target()
            if (mnt_target.target_type == MountTargetType::RATE) {
                update_angle_target_from_rate(mnt_target.rate_rads, mnt_target.angle_rad);
            }
            resend_now = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's RC inputs
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
            resend_now = true;
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                resend_now = true;
            }
            break;

        // point mount to Home location
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                resend_now = true;
            }
            break;

        // point mount to another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    resend_now = resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_SERIAL_RESEND_MS);

    if ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_SERIAL_RESEND_MS*2) {
        _reply_type = ReplyType_UNKNOWN;
    }
    if (can_send(resend_now)) {
        if (resend_now) {
            send_target_angles(mnt_target.angle_rad);
            get_angles();
            _reply_type = ReplyType_ACK;
            _reply_counter = 0;
            _reply_length = get_reply_size(_reply_type);
        } else {
            get_angles();
            _reply_type = ReplyType_DATA;
            _reply_counter = 0;
            _reply_length = get_reply_size(_reply_type);
        }
    }
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SToRM32_serial::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(radians(_current_angle.x * 0.01f), radians(_current_angle.y * 0.01f), radians(_current_angle.z * 0.01f));
    return true;
}

bool AP_Mount_SToRM32_serial::can_send(bool with_control) {
    uint16_t required_tx = 1;
    if (with_control) {
        required_tx += sizeof(AP_Mount_SToRM32_serial::cmd_set_angles_struct);
    }
    return (_reply_type == ReplyType_UNKNOWN) && (_port->txspace() >= required_tx);
}


// send_target_angles
void AP_Mount_SToRM32_serial::send_target_angles(const MountTarget& angle_target_rad)
{

    static cmd_set_angles_struct cmd_set_angles_data = {
        0xFA,
        0x0E,
        0x11,
        0, // pitch
        0, // roll
        0, // yaw
        0, // flags
        0, // type
        0, // crc
    };

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if ((size_t)_port->txspace() < sizeof(cmd_set_angles_data)) {
        return;
    }

    // send CMD_SETANGLE (Note: reversed pitch and yaw)
    cmd_set_angles_data.pitch = -degrees(angle_target_rad.pitch);
    cmd_set_angles_data.roll = degrees(angle_target_rad.roll);
    cmd_set_angles_data.yaw = -degrees(angle_target_rad.get_bf_yaw());

    uint8_t* buf = (uint8_t*)&cmd_set_angles_data;

    cmd_set_angles_data.crc = crc_calculate(&buf[1], sizeof(cmd_set_angles_data)-3);

    for (uint8_t i = 0;  i != sizeof(cmd_set_angles_data) ; i++) {
        _port->write(buf[i]);
    }

    // store time of send
    _last_send = AP_HAL::millis();
}

void AP_Mount_SToRM32_serial::get_angles() {
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if (_port->txspace() < 1) {
        return;
    }

    _port->write('d');
};


uint8_t AP_Mount_SToRM32_serial::get_reply_size(ReplyType reply_type) {
    switch (reply_type) {
        case ReplyType_DATA:
            return sizeof(SToRM32_reply_data_struct);
            break;
        case ReplyType_ACK:
            return sizeof(SToRM32_reply_ack_struct);
            break;
        default:
            return 0;
    }
}


void AP_Mount_SToRM32_serial::read_incoming() {
    uint8_t data;
    int16_t numc;

    numc = _port->available();

    if (numc < 0 ) {
        return;
    }

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();
        if (_reply_type == ReplyType_UNKNOWN) {
            continue;
        }

        _buffer[_reply_counter++] = data;
        if (_reply_counter == _reply_length) {
            parse_reply();

            switch (_reply_type) {
                case ReplyType_ACK:
                    _reply_type = ReplyType_DATA;
                    _reply_length = get_reply_size(_reply_type);
                    _reply_counter = 0;
                    break;
                case ReplyType_DATA:
                    _reply_type = ReplyType_UNKNOWN;
                    _reply_length = get_reply_size(_reply_type);
                    _reply_counter = 0;
                    break;
                default:
                    _reply_length = get_reply_size(_reply_type);
                    _reply_counter = 0;
                    break;
            }
        }
    }
}

void AP_Mount_SToRM32_serial::parse_reply() {
    uint16_t crc;
    bool crc_ok;

    switch (_reply_type) {
        case ReplyType_DATA:
            crc = crc_calculate(&_buffer[0], sizeof(_buffer.data) - 3);
            crc_ok = crc == _buffer.data.crc;
            if (!crc_ok) {
                break;
            }

            // Parse angles (Note: reversed pitch and yaw) to match ardupilot coordinate system
            _current_angle.x = _buffer.data.imu1_roll;
            _current_angle.y = -_buffer.data.imu1_pitch;
            _current_angle.z = -_buffer.data.imu1_yaw;
            break;
        default:
            break;
    }
}
#endif // HAL_MOUNT_STORM32SERIAL_ENABLED
