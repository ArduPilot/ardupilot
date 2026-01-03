#include "AP_Mount_config.h"

#if HAL_MOUNT_CADDX_ENABLED

#include "AP_Mount_CADDX.h"
#include <AP_HAL/AP_HAL.h>

#define SET_ATTITUDE_HEADER1 0xA5
#define SET_ATTITUDE_HEADER2 0x5A
#define SET_ATTITUDE_BUF_SIZE 10
#define AXIS_MIN 0
#define AXIS_MAX 4096

// update mount position - should be called periodically
void AP_Mount_CADDX::update()
{
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    AP_Mount_Backend::update_mnt_target();

    // send target angles (which may be derived from other target types)
    AP_Mount_Backend::send_target_to_gimbal();
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_CADDX::get_attitude_quaternion(Quaternion& att_quat)
{
    // gimbal does not provide attitude so simply return targets
    att_quat.from_euler(mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, mnt_target.angle_rad.get_bf_yaw());
    return true;
}

// send_target_angles
void AP_Mount_CADDX::send_target_angles(const MountAngleTarget& angle_target_rad)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // ensure we have enough space to send the packet
    if (_uart->txspace() < SET_ATTITUDE_BUF_SIZE) {
        return;
    }

    // calculate roll, pitch, yaw angles in range 0 to 4096
    const float scalar = AXIS_MAX / M_2PI;
    const uint16_t roll_target_cmd = constrain_uint16(wrap_2PI(angle_target_rad.roll) * scalar, AXIS_MIN, AXIS_MAX);
    const uint16_t pitch_target_cmd = constrain_uint16(wrap_2PI(angle_target_rad.pitch) * scalar, AXIS_MIN, AXIS_MAX);
    const uint16_t yaw_target_cmd = constrain_uint16(wrap_2PI(angle_target_rad.get_bf_yaw()) * scalar, AXIS_MIN, AXIS_MAX);
    
    // prepare packet to send to gimbal
    uint8_t set_attitude_cmd_buf[SET_ATTITUDE_BUF_SIZE] {};

    // first two bytes hold the header
    set_attitude_cmd_buf[0] = SET_ATTITUDE_HEADER1;
    set_attitude_cmd_buf[1] = SET_ATTITUDE_HEADER2;

    // byte 2's lower 3 bits are mode
    // lower 5 bits are sensitivity but always left as zero
    uint8_t mode = 0; //start with axes in bf
    // CADDX yaw lock is not required and is duplication since we get yaw via get_bf_yaw, the gimbal accs would be fighting our yaw target potentially
    // but we need to reset roll and pitch locks to body frame if set by RP_LOCK aux switch or by FPV mnt option
    if (angle_target_rad.pitch_is_ef) {
        mode |= (uint8_t)LockMode::PITCH_LOCK;
    }
    if (angle_target_rad.roll_is_ef) {
        mode |= (uint8_t)LockMode::ROLL_LOCK;
    }
    set_attitude_cmd_buf[2] = mode & 0x07;

    // byte 3's lower 4 bits are reserved
    // upper 4 bits are roll's lower 4 bits
    set_attitude_cmd_buf[3] = (roll_target_cmd << 4) & 0xF0;

    // byte 4 is roll's upper 8 bits
    set_attitude_cmd_buf[4] = (roll_target_cmd >> 4) & 0xFF;

    // byte 5 is pitch's lower 8 bits
    set_attitude_cmd_buf[5] = pitch_target_cmd & 0xFF;

    // byte 6's lower 4 bits are pitch's upper 4 bits
    // upper 4 bits are yaw's lower 4 bits
    set_attitude_cmd_buf[6] = (pitch_target_cmd >> 8) & 0x0F;
    set_attitude_cmd_buf[6] |= (yaw_target_cmd << 4) & 0xF0;

    // byte 7 is yaw's upper 8 bits
    set_attitude_cmd_buf[7] = (yaw_target_cmd >> 4) & 0xFF;

    // calculate CRC
    const uint16_t crc16 = crc16_ccitt(set_attitude_cmd_buf, sizeof(set_attitude_cmd_buf) - 2, 0);
    set_attitude_cmd_buf[8] = HIGHBYTE(crc16);
    set_attitude_cmd_buf[9] = LOWBYTE(crc16);

    // send packet to gimbal
    _uart->write(set_attitude_cmd_buf, sizeof(set_attitude_cmd_buf));
}

#endif // HAL_MOUNT_CADDX_ENABLED
