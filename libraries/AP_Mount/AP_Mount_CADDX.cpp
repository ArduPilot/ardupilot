#include "AP_Mount_CADDX.h"
#include <GCS_MAVLink/GCS.h>
#if HAL_MOUNT_CADDX_ENABLED
#include <AP_HAL/AP_HAL.h>

// update mount position - should be called periodically
void AP_Mount_CADDX::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

     // change to RC_TARGETING mode if RC input has changed
    set_rctargeting_on_rcinput_change();

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
    resend_now = resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_CADDX_RESEND_MS);

        if (resend_now) {
            send_target_angles(mnt_target.angle_rad);
        }
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_CADDX::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(radians(_current_angle.x * 0.01f), radians(_current_angle.y * 0.01f), radians(_current_angle.z * 0.01f));
    return true;
}


// send_target_angles
void AP_Mount_CADDX::send_target_angles(const MountTarget& angle_target_rad)
{

    static cmd_set_angles_struct cmd_set_angles_data = { //init some fields here. all others default to zero
        .sync = {0xA5 , 0x5A},
        .mode = GIMBAL_MODE_DEFAULT,
        .sensitivity = 0,
        .roll = 0,
        .tilt = 0,
        .pan = 0
    };

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if ((size_t)_uart->txspace() < sizeof(cmd_set_angles_data)) {
        return;
    }

    // compute angles
    cmd_set_angles_data.tilt = degrees(angle_target_rad.pitch);
    cmd_set_angles_data.roll = degrees(angle_target_rad.roll);
    cmd_set_angles_data.pan = degrees(angle_target_rad.get_bf_yaw());
    
    //normalize here
    cmd_set_angles_data.tilt = int16_t(linear_interpolate(AXIS_MIN, AXIS_MAX, cmd_set_angles_data.tilt, _params.pitch_angle_min, _params.pitch_angle_max));
    cmd_set_angles_data.roll = int16_t(linear_interpolate(AXIS_MIN, AXIS_MAX, cmd_set_angles_data.roll, _params.roll_angle_min, _params.roll_angle_max));
    cmd_set_angles_data.pan = int16_t(linear_interpolate(AXIS_MIN, AXIS_MAX, cmd_set_angles_data.pan, _params.yaw_angle_min, _params.yaw_angle_max));

    //todo:add mode here
    switch (_mount_lock_mode) {
    case 1:
    cmd_set_angles_data.mode = GIMBAL_MODE_ROLL_LOCK;
    break;
    case 2:
    cmd_set_angles_data.mode = GIMBAL_MODE_TILT_ROLL_LOCK;
    break;
    default:
    cmd_set_angles_data.mode = GIMBAL_MODE_DEFAULT ;
    break;
    }
    
    // CRC here
    uint16_t crc16 = 0;
    uint8_t *b = (uint8_t *)&cmd_set_angles_data;
    for (uint8_t i = 0; i < sizeof(cmd_set_angles_data) - 2; i++) {
        crc16 = crc_xmodem_update(crc16, *(b + i));
    }
    cmd_set_angles_data.crch = (crc16 >> 8) & 0xFF;
    cmd_set_angles_data.crcl = crc16 & 0xFF;
    
    //send to gimbal
    uint8_t* buf = (uint8_t*)&cmd_set_angles_data;    
    for (uint8_t i = 0;  i != sizeof(cmd_set_angles_data) ; i++) {
        _uart->write(buf[i]);
    }

    // store time of send
    _last_send = AP_HAL::millis();
}




#endif // HAL_MOUNT_CADDX_ENABLED
