// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_Mount_Servo.h"

extern const AP_HAL::HAL& hal;

// init - performs any required initialisation for this instance
void AP_Mount_Servo::init(const AP_SerialManager& serial_manager)
{
    if (_instance == 0) {
        _roll_idx = RC_Channel_aux::k_mount_roll;
        _tilt_idx = RC_Channel_aux::k_mount_tilt;
        _pan_idx  = RC_Channel_aux::k_mount_pan;
        _open_idx = RC_Channel_aux::k_mount_open;
    } else {
        // this must be the 2nd mount
        _roll_idx = RC_Channel_aux::k_mount2_roll;
        _tilt_idx = RC_Channel_aux::k_mount2_tilt;
        _pan_idx  = RC_Channel_aux::k_mount2_pan;
        _open_idx = RC_Channel_aux::k_mount2_open;
    }

    // check which servos have been assigned
    check_servo_map();
}

// update mount position - should be called periodically
void AP_Mount_Servo::update()
{
    static bool mount_open = 0;     // 0 is closed

    // check servo map every three seconds to allow users to modify parameters
    uint32_t now = hal.scheduler->millis();
    if (now - _last_check_servo_map_ms > 3000) {
        check_servo_map();
        _last_check_servo_map_ms = now;
    }

    switch(get_mode()) {
        // move mount to a "retracted position" or to a position where a fourth servo can retract the entire mount into the fuselage
        case MAV_MOUNT_MODE_RETRACT:
        {
            _angle_bf_output_deg = _state._retract_angles.get();
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
        {
            _angle_bf_output_deg = _state._neutral_angles.get();
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
        {
            // earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            stabilize();
            break;
        }

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
        {
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            stabilize();
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
        {
            if(_frontend._ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, _flags.tilt_control, _flags.pan_control);
                stabilize();
            }
            break;
        }

        default:
            //do nothing
            break;
    }

    // move mount to a "retracted position" into the fuselage with a fourth servo
    bool mount_open_new = (get_mode() == MAV_MOUNT_MODE_RETRACT) ? 0 : 1;
    if (mount_open != mount_open_new) {
        mount_open = mount_open_new;
        move_servo(_open_idx, mount_open_new, 0, 1);
    }

    // write the results to the servos
    move_servo(_roll_idx, _angle_bf_output_deg.x*10, _state._roll_angle_min*0.1f, _state._roll_angle_max*0.1f);
    move_servo(_tilt_idx, _angle_bf_output_deg.y*10, _state._tilt_angle_min*0.1f, _state._tilt_angle_max*0.1f);
    move_servo(_pan_idx,  _angle_bf_output_deg.z*10, _state._pan_angle_min*0.1f, _state._pan_angle_max*0.1f);
}

// set_mode - sets mount's mode
void AP_Mount_Servo::set_mode(enum MAV_MOUNT_MODE mode)
{
    // record the mode change and return success
    _state._mode = mode;
}

// private methods

// check_servo_map - detects which axis we control using the functions assigned to the servos in the RC_Channel_aux
//  should be called periodically (i.e. 1hz or less)
void AP_Mount_Servo::check_servo_map()
{
    _flags.roll_control = RC_Channel_aux::function_assigned(_roll_idx);
    _flags.tilt_control = RC_Channel_aux::function_assigned(_tilt_idx);
    _flags.pan_control = RC_Channel_aux::function_assigned(_pan_idx);
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Servo::status_msg(mavlink_channel_t chan)
{
    mavlink_msg_mount_status_send(chan, 0, 0, _angle_bf_output_deg.y*100, _angle_bf_output_deg.x*100, _angle_bf_output_deg.z*100);
}

// stabilize - stabilizes the mount relative to the Earth's frame
//  input: _angle_ef_target_rad (earth frame targets in radians)
//  output: _angle_bf_output_deg (body frame angles in degrees)
void AP_Mount_Servo::stabilize()
{
    // only do the full 3D frame transform if we are doing pan control
    if (_state._stab_pan) {
        Matrix3f m;                         ///< holds 3 x 3 matrix, var is used as temp in calcs
        Matrix3f cam;                       ///< Rotation matrix earth to camera. Desired camera from input.
        Matrix3f gimbal_target;             ///< Rotation matrix from plane to camera. Then Euler angles to the servos.
        m = _frontend._ahrs.get_dcm_matrix();
        m.transpose();
        cam.from_euler(_angle_ef_target_rad.x, _angle_ef_target_rad.y, _angle_ef_target_rad.z);
        gimbal_target = m * cam;
        gimbal_target.to_euler(&_angle_bf_output_deg.x, &_angle_bf_output_deg.y, &_angle_bf_output_deg.z);
        _angle_bf_output_deg.x  = _state._stab_roll ? degrees(_angle_bf_output_deg.x) : degrees(_angle_ef_target_rad.x);
        _angle_bf_output_deg.y  = _state._stab_tilt ? degrees(_angle_bf_output_deg.y) : degrees(_angle_ef_target_rad.y);
        _angle_bf_output_deg.z = degrees(_angle_bf_output_deg.z);
    } else {
        // otherwise base mount roll and tilt on the ahrs
        // roll/tilt attitude, plus any requested angle
        _angle_bf_output_deg.x = degrees(_angle_ef_target_rad.x);
        _angle_bf_output_deg.y = degrees(_angle_ef_target_rad.y);
        _angle_bf_output_deg.z = degrees(_angle_ef_target_rad.z);
        if (_state._stab_roll) {
            _angle_bf_output_deg.x -= degrees(_frontend._ahrs.roll);
        }
        if (_state._stab_tilt) {
            _angle_bf_output_deg.y -= degrees(_frontend._ahrs.pitch);
        }

        // lead filter
        const Vector3f &gyro = _frontend._ahrs.get_gyro();

        if (_state._stab_roll && !is_zero(_state._roll_stb_lead) && fabsf(_frontend._ahrs.pitch) < M_PI_F/3.0f) {
            // Compute rate of change of euler roll angle
            float roll_rate = gyro.x + (_frontend._ahrs.sin_pitch() / _frontend._ahrs.cos_pitch()) * (gyro.y * _frontend._ahrs.sin_roll() + gyro.z * _frontend._ahrs.cos_roll());
            _angle_bf_output_deg.x -= degrees(roll_rate) * _state._roll_stb_lead;
        }

        if (_state._stab_tilt && !is_zero(_state._pitch_stb_lead)) {
            // Compute rate of change of euler pitch angle
            float pitch_rate = _frontend._ahrs.cos_pitch() * gyro.y - _frontend._ahrs.sin_roll() * gyro.z;
            _angle_bf_output_deg.y -= degrees(pitch_rate) * _state._pitch_stb_lead;
        }
    }
}

// closest_limit - returns closest angle to 'angle' taking into account limits.  all angles are in degrees * 10
int16_t AP_Mount_Servo::closest_limit(int16_t angle, int16_t angle_min, int16_t angle_max)
{
    // Make sure the angle lies in the interval [-180 .. 180[ degrees
    while (angle < -1800) angle += 3600;
    while (angle >= 1800) angle -= 3600;

    // Make sure the angle limits lie in the interval [-180 .. 180[ degrees
    while (angle_min < -1800) angle_min += 3600;
    while (angle_min >= 1800) angle_min -= 3600;
    while (angle_max < -1800) angle_max += 3600;
    while (angle_max >= 1800) angle_max -= 3600;
    // TODO call this function somehow, otherwise this will never work
    //set_range(min, max);

    // If the angle is outside servo limits, saturate the angle to the closest limit
    // On a circle the closest angular position must be carefully calculated to account for wrap-around
    if ((angle < angle_min) && (angle > angle_max)) {
        // angle error if min limit is used
        int16_t err_min = angle_min - angle + (angle<angle_min ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        // angle error if max limit is used
        int16_t err_max = angle - angle_max + (angle>angle_max ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        angle = err_min<err_max ? angle_min : angle_max;
    }

    return angle;
}

// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
void AP_Mount_Servo::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	// saturate to the closest angle limit if outside of [min max] angle interval
	int16_t servo_out = closest_limit(angle, angle_min, angle_max);
	RC_Channel_aux::move_servo((RC_Channel_aux::Aux_servo_function_t)function_idx, servo_out, angle_min, angle_max);
}
