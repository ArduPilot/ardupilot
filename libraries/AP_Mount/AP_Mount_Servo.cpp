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
}

// update mount position - should be called periodically
void AP_Mount_Servo::update()
{
    switch (get_mode()) {
        // move mount to a "retracted position" or to a position where a fourth servo can retract the entire mount into the fuselage
        case MAV_MOUNT_MODE_RETRACT: {
            _angle_bf_output_deg = _params.retract_angles.get();

            // initialise _angle_rad to smooth transition if user changes to RC_TARGETTING
            _angle_rad.roll = radians(_angle_bf_output_deg.x);
            _angle_rad.pitch = radians(_angle_bf_output_deg.y);
            _angle_rad.yaw = radians(_angle_bf_output_deg.z);
            _angle_rad.yaw_is_ef = false;
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            _angle_bf_output_deg = _params.neutral_angles.get();

            // initialise _angle_rad to smooth transition if user changes to RC_TARGETTING
            _angle_rad.roll = radians(_angle_bf_output_deg.x);
            _angle_rad.pitch = radians(_angle_bf_output_deg.y);
            _angle_rad.yaw = radians(_angle_bf_output_deg.z);
            _angle_rad.yaw_is_ef = false;
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
            switch (mavt_target.target_type) {
            case MountTargetType::ANGLE:
                _angle_rad = mavt_target.angle_rad;
                break;
            case MountTargetType::RATE:
                update_angle_target_from_rate(mavt_target.rate_rads, _angle_rad);
                break;
            }
            // update _angle_bf_output_deg based on angle target
            update_angle_outputs(_angle_rad);
            break;
        }

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's RC inputs
            MountTarget rc_target {};
            if (get_rc_rate_target(rc_target)) {
                update_angle_target_from_rate(rc_target, _angle_rad);
            } else if (get_rc_angle_target(rc_target)) {
                _angle_rad = rc_target;
            }
            // update _angle_bf_output_deg based on angle target
            update_angle_outputs(_angle_rad);
            break;
        }

        // point mount to a GPS location
        case MAV_MOUNT_MODE_GPS_POINT: {
            if (get_angle_target_to_roi(_angle_rad)) {
                update_angle_outputs(_angle_rad);
            }
            break;
        }

        case MAV_MOUNT_MODE_HOME_LOCATION: {
            if (get_angle_target_to_home(_angle_rad)) {
                update_angle_outputs(_angle_rad);
            }
            break;
        }

        case MAV_MOUNT_MODE_SYSID_TARGET: {
            if (get_angle_target_to_sysid(_angle_rad)) {
                update_angle_outputs(_angle_rad);
            }
            break;
        }

        default:
            //do nothing
            break;
    }

    // move mount to a "retracted position" into the fuselage with a fourth servo
    const bool mount_open = (get_mode() == MAV_MOUNT_MODE_RETRACT) ? 0 : 1;
    move_servo(_open_idx, mount_open, 0, 1);

    // write the results to the servos
    move_servo(_roll_idx, _angle_bf_output_deg.x*10, _params.roll_angle_min*10, _params.roll_angle_max*10);
    move_servo(_tilt_idx, _angle_bf_output_deg.y*10, _params.pitch_angle_min*10, _params.pitch_angle_max*10);
    move_servo(_pan_idx,  _angle_bf_output_deg.z*10, _params.yaw_angle_min*10, _params.yaw_angle_max*10);
}

// returns true if this mount can control its pan (required for multicopters)
bool AP_Mount_Servo::has_pan_control() const
{
    return SRV_Channels::function_assigned(_pan_idx) && yaw_range_valid();
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Servo::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(radians(_angle_bf_output_deg.x), radians(_angle_bf_output_deg.y), radians(_angle_bf_output_deg.z));
    return true;
}

// private methods

// update body-frame angle outputs from earth-frame angle targets
void AP_Mount_Servo::update_angle_outputs(const MountTarget& angle_rad)
{
    const AP_AHRS &ahrs = AP::ahrs();

    // roll and pitch are based on the ahrs roll and pitch angle plus any requested angle
    _angle_bf_output_deg.x = degrees(angle_rad.roll);
    _angle_bf_output_deg.y = degrees(angle_rad.pitch);
    _angle_bf_output_deg.z = degrees(get_bf_yaw_angle(angle_rad));
    if (requires_stabilization) {
        _angle_bf_output_deg.x -= degrees(ahrs.roll);
        _angle_bf_output_deg.y -= degrees(ahrs.pitch);
    }

    // lead filter
    const Vector3f &gyro = ahrs.get_gyro();

    if (requires_stabilization && !is_zero(_params.roll_stb_lead) && fabsf(ahrs.pitch) < M_PI/3.0f) {
        // Compute rate of change of euler roll angle
        float roll_rate = gyro.x + (ahrs.sin_pitch() / ahrs.cos_pitch()) * (gyro.y * ahrs.sin_roll() + gyro.z * ahrs.cos_roll());
        _angle_bf_output_deg.x -= degrees(roll_rate) * _params.roll_stb_lead;
    }

    if (requires_stabilization && !is_zero(_params.pitch_stb_lead)) {
        // Compute rate of change of euler pitch angle
        float pitch_rate = ahrs.cos_pitch() * gyro.y - ahrs.sin_roll() * gyro.z;
        _angle_bf_output_deg.y -= degrees(pitch_rate) * _params.pitch_stb_lead;
    }
}

// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
void AP_Mount_Servo::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	SRV_Channels::move_servo((SRV_Channel::Aux_servo_function_t)function_idx, angle, angle_min, angle_max);
}
#endif // HAL_MOUNT_SERVO_ENABLED
