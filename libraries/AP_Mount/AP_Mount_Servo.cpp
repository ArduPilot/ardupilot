#include "AP_Mount_config.h"

#if HAL_MOUNT_SERVO_ENABLED

#include "AP_Mount_Servo.h"

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
    } else {
        // this must be the 2nd mount
        _roll_idx = SRV_Channel::k_mount2_roll;
        _tilt_idx = SRV_Channel::k_mount2_tilt;
        _pan_idx  = SRV_Channel::k_mount2_pan;
    }
    AP_Mount_Backend::init();
}

// update mount position - should be called periodically
void AP_Mount_Servo::update()
{
    AP_Mount_Backend::update();

    update_mnt_target();

    // have our base class call send_target_angles to command the gimbal:
    send_target_to_gimbal();
}

// called by the backend to set the servo angles:
void AP_Mount_Servo::send_target_angles(const MountAngleTarget& angle_rad)
{
    update_angle_outputs(mnt_target.angle_rad);

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
void AP_Mount_Servo::update_angle_outputs(const MountAngleTarget& angle_rad)
{
    const AP_AHRS &ahrs = AP::ahrs();

    // get target yaw in body-frame with limits applied
    const float yaw_bf_rad = constrain_float(angle_rad.get_bf_yaw(), radians(_params.yaw_angle_min), radians(_params.yaw_angle_max));

    // default output to target earth-frame roll and pitch angles, body-frame yaw
    _angle_bf_output_rad.x = angle_rad.roll;
    _angle_bf_output_rad.y = angle_rad.pitch;
    _angle_bf_output_rad.z = yaw_bf_rad;

    // do no stabilization in retract or neutral:
    switch (mnt_target.target_type) {
    case MountTargetType::NEUTRAL:
    case MountTargetType::RETRACTED:
        return;
    case MountTargetType::ANGLE:
    case MountTargetType::RATE:
        break;
    }

    // only have to adjust roll/pitch for body frame in self-stabilising brushless gimbals
    if (!requires_stabilization) {
        //since this is a shared backend, must call this directly
        AP_Mount_Backend::adjust_mnt_target_if_RP_locked();
        return;
    }

    // retrieve lean angles from ahrs
    Vector2f ahrs_angle_rad = {ahrs.get_roll_rad(), ahrs.get_pitch_rad()};

    // rotate ahrs roll and pitch angles to gimbal yaw
    if (has_pan_control()) {
        ahrs_angle_rad.rotate(-yaw_bf_rad);
    }

    // add roll and pitch lean angle correction for earth frame
    if (angle_rad.roll_is_ef){
        _angle_bf_output_rad.x -= ahrs_angle_rad.x;
    }
    
    if (angle_rad.pitch_is_ef){
        _angle_bf_output_rad.y -= ahrs_angle_rad.y;
    } 

    // lead filter
    const Vector3f &gyro = ahrs.get_gyro();

    if (!is_zero(_params.roll_stb_lead) && fabsf(ahrs.get_pitch_rad()) < M_PI/3.0f) {
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
	SRV_Channels::move_servo((SRV_Channel::Function)function_idx, angle, angle_min, angle_max);
}
#endif // HAL_MOUNT_SERVO_ENABLED
