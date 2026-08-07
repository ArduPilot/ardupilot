#include "AP_Mount_config.h"

#if HAL_MOUNT_STORM32SERIAL_ENABLED

#include "AP_Mount_SToRM32_serial.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>

// update mount position - should be called periodically
void AP_Mount_SToRM32_serial::update()
{
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    AP_Mount_Backend::update_mnt_target();

    // send target angles (which may be derived from other target types)
    AP_Mount_Backend::send_target_to_gimbal();

    if ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_SERIAL_TIMEOUT_MS) {
        _reply_type = ReplyType_UNKNOWN;
    }
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SToRM32_serial::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(cd_to_rad(_current_angle.x), cd_to_rad(_current_angle.y), cd_to_rad(_current_angle.z));
    return true;
}

bool AP_Mount_SToRM32_serial::can_send() {
    uint16_t required_tx = 1;
    required_tx += sizeof(AP_Mount_SToRM32_serial::cmd_set_angles_struct);
    return (_reply_type == ReplyType_UNKNOWN) && (_uart->txspace() >= required_tx);
}


// send_target_angles
void AP_Mount_SToRM32_serial::send_target_angles(const MountAngleTarget& angle_target_rad)
{
    if (!can_send()) {
        return;
    }

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

    if ((size_t)_uart->txspace() < sizeof(cmd_set_angles_data)) {
        return;
    }

    // send CMD_SETANGLE (Note: reversed pitch and yaw)
    cmd_set_angles_data.pitch = -degrees(angle_target_rad.pitch);
    cmd_set_angles_data.roll = degrees(angle_target_rad.roll);
    cmd_set_angles_data.yaw = -degrees(angle_target_rad.get_bf_yaw());

    uint8_t* buf = (uint8_t*)&cmd_set_angles_data;

    cmd_set_angles_data.crc = crc_calculate(&buf[1], sizeof(cmd_set_angles_data)-3);

    for (uint8_t i = 0;  i != sizeof(cmd_set_angles_data) ; i++) {
        _uart->write(buf[i]);
    }

    // store time of send
    _last_send = AP_HAL::millis();

    // we pipeline commands.  We have sent in a command to set angles,
    // now fetch data from the device:
    get_angles();
    // we expect an ACK back for the set-angles command.  A state
    // machine in read_incoming will move us to ReplyType_DATA once
    // the ACK has been received.
    _reply_type = ReplyType_ACK;
    _reply_counter = 0;
    _reply_length = get_reply_size(_reply_type);

}

void AP_Mount_SToRM32_serial::get_angles() {
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if (_uart->txspace() < 1) {
        return;
    }

    _uart->write('d');
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

    numc = _uart->available();

    if (numc < 0 ) {
        return;
    }

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _uart->read();
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
