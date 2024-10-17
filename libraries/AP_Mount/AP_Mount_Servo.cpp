#include "AP_Mount_Servo.h"
#if HAL_MOUNT_SERVO_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;

// init - performs any required initialisation for this instance
void AP_Mount_Servo::init()
{
    if (_instance == 0) {
        _roll_idx = SRV_Channel::k_mount_roll;
        _tilt_idx = SRV_Channel::k_mount_tilt;
        _pan_idx  = SRV_Channel::k_mount_pan;
        _open_idx = SRV_Channel::k_mount_open;
    } else {
        // this must be the 2nd mount
        _roll_idx = SRV_Channel::k_mount2_roll;
        _tilt_idx = SRV_Channel::k_mount2_tilt;
        _pan_idx  = SRV_Channel::k_mount2_pan;
        _open_idx = SRV_Channel::k_mount2_open;
    }
    AP_Mount_Backend::init();
}

// update mount position - should be called periodically
void AP_Mount_Servo::update()
{
    // change to RC_TARGETING mode if RC input has changed
    set_rctargeting_on_rcinput_change();

    // control the open/close servo.
    control_mount_open_servo();

    auto mount_mode = get_mode();
    switch (mount_mode) {
        // move mount to a "retracted position" or to a position where a fourth servo can retract the entire mount into the fuselage
        case MAV_MOUNT_MODE_RETRACT: {
            _angle_bf_output_rad = _params.retract_angles.get() * DEG_TO_RAD;
            mnt_target.angle_rad.set(_angle_bf_output_rad, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            _angle_bf_output_rad = _params.neutral_angles.get() * DEG_TO_RAD;
            mnt_target.angle_rad.set(_angle_bf_output_rad, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
            // mavlink targets are stored while handling the incoming message and considered valid
            break;
        }

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
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to Home location
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        default:
            //do nothing
            break;
    }

    // send target angles or rates depending on the target type
    switch (mnt_target.target_type) {
        case MountTargetType::RATE:
            update_angle_target_from_rate(mnt_target.rate_rads, mnt_target.angle_rad);
            FALLTHROUGH;
        case MountTargetType::ANGLE:
            // update _angle_bf_output_rad based on angle target
            if ((mount_mode != MAV_MOUNT_MODE_RETRACT) & (mount_mode != MAV_MOUNT_MODE_NEUTRAL)) {
                update_angle_outputs(mnt_target.angle_rad);
            }
            break;
    }


    // write the results to the servos
    move_servo(_roll_idx, degrees(_angle_bf_output_rad.x)*10, _params.roll_angle_min*10, _params.roll_angle_max*10);
    move_servo(_tilt_idx, degrees(_angle_bf_output_rad.y)*10, _params.pitch_angle_min*10, _params.pitch_angle_max*10);
    move_servo(_pan_idx,  degrees(_angle_bf_output_rad.z)*10, _params.yaw_angle_min*10, _params.yaw_angle_max*10);
}

// returns true if this mount can control its roll
bool AP_Mount_Servo::has_roll_control() const
{
    return SRV_Channels::function_assigned(_roll_idx) && roll_range_valid();
}

// returns true if this mount can control its tilt
bool AP_Mount_Servo::has_pitch_control() const
{
    return SRV_Channels::function_assigned(_tilt_idx) && pitch_range_valid();
}

// returns true if this mount can control its pan (required for multicopters)
bool AP_Mount_Servo::has_pan_control() const
{
    return SRV_Channels::function_assigned(_pan_idx) && yaw_range_valid();
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Servo::get_attitude_quaternion(Quaternion& att_quat)
{
    // No feedback from gimbal so simply report demanded servo angles (which is
    // not the same as target angles).
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    if (has_roll_control()) {
        roll_rad = constrain_float(_angle_bf_output_rad.x, radians(_params.roll_angle_min), radians(_params.roll_angle_max));
    }
    if (has_pitch_control()) {
        pitch_rad = constrain_float(_angle_bf_output_rad.y, radians(_params.pitch_angle_min), radians(_params.pitch_angle_max));
    }
    if (has_pan_control()) {
        yaw_rad = constrain_float(_angle_bf_output_rad.z, radians(_params.yaw_angle_min), radians(_params.yaw_angle_max));
    }

    // convert to quaternion
    att_quat.from_euler(roll_rad, pitch_rad, yaw_rad);
    return true;
}

// private methods

// update body-frame angle outputs from earth-frame angle targets
void AP_Mount_Servo::update_angle_outputs(const MountTarget& angle_rad)
{
    const AP_AHRS &ahrs = AP::ahrs();

    // get target yaw in body-frame with limits applied
    const float yaw_bf_rad = constrain_float(angle_rad.get_bf_yaw(), radians(_params.yaw_angle_min), radians(_params.yaw_angle_max));

    // default output to target earth-frame roll and pitch angles, body-frame yaw
    _angle_bf_output_rad.x = angle_rad.roll;
    _angle_bf_output_rad.y = angle_rad.pitch;
    _angle_bf_output_rad.z = yaw_bf_rad;

    // this is sufficient for self-stabilising brushless gimbals
    if (!requires_stabilization) {
        return;
    }

    // retrieve lean angles from ahrs
    Vector2f ahrs_angle_rad = {ahrs.get_roll(), ahrs.get_pitch()};

    // rotate ahrs roll and pitch angles to gimbal yaw
    if (has_pan_control()) {
        ahrs_angle_rad.rotate(-yaw_bf_rad);
    }

    // add roll and pitch lean angle correction
    _angle_bf_output_rad.x -= ahrs_angle_rad.x;
    _angle_bf_output_rad.y -= ahrs_angle_rad.y;

    // lead filter
    const Vector3f &gyro = ahrs.get_gyro();

    if (!is_zero(_params.roll_stb_lead) && fabsf(ahrs.get_pitch()) < M_PI/3.0f) {
        // Compute rate of change of euler roll angle
        float roll_rate = gyro.x + (ahrs.sin_pitch() / ahrs.cos_pitch()) * (gyro.y * ahrs.sin_roll() + gyro.z * ahrs.cos_roll());
        _angle_bf_output_rad.x -= roll_rate * _params.roll_stb_lead;
    }

    if (!is_zero(_params.pitch_stb_lead)) {
        // Compute rate of change of euler pitch angle
        float pitch_rate = ahrs.cos_pitch() * gyro.y - ahrs.sin_roll() * gyro.z;
        _angle_bf_output_rad.y -= pitch_rate * _params.pitch_stb_lead;
    }
}

// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
void AP_Mount_Servo::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	SRV_Channels::move_servo((SRV_Channel::Aux_servo_function_t)function_idx, angle, angle_min, angle_max);
}
#endif // HAL_MOUNT_SERVO_ENABLED
