#include "AP_Mount_Scripting.h"

#if HAL_MOUNT_SCRIPTING_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_SCRIPTING_TIMEOUT_MS    1000   // scripting mount becomes unhealthy after 1sec with no updates

#define AP_MOUNT_SCRIPTING_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_SCRIPTING_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Siyi: " fmt, ## args); } } while (0)

// update mount position - should be called periodically
void AP_Mount_Scripting::update()
{
    // update based on mount mode
    switch (get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &angle_bf_target = _params.retract_angles.get();
            target_angle_rad.roll = ToRad(angle_bf_target.x);
            target_angle_rad.pitch = ToRad(angle_bf_target.y);
            target_angle_rad.yaw = ToRad(angle_bf_target.z);
            target_angle_rad.yaw_is_ef = false;
            target_angle_rad_valid = true;

            // mark other targets as invalid
            target_rate_rads_valid = false;
            target_loc_valid = false;
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            target_angle_rad.roll = ToRad(angle_bf_target.x);
            target_angle_rad.pitch = ToRad(angle_bf_target.y);
            target_angle_rad.yaw = ToRad(angle_bf_target.z);
            target_angle_rad.yaw_is_ef = false;
            target_angle_rad_valid = true;

            // mark other targets as invalid
            target_rate_rads_valid = false;
            target_loc_valid = false;
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            switch (mavt_target.target_type) {
            case MountTargetType::ANGLE:
                target_angle_rad = mavt_target.angle_rad;
                target_angle_rad_valid = true;
                target_rate_rads_valid = false;
                target_loc_valid = false;
                break;
            case MountTargetType::RATE:
                target_rate_rads = mavt_target.rate_rads;
                target_rate_rads_valid = true;
                target_angle_rad_valid = false;
                target_loc_valid = false;
                break;
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's rc inputs
            MountTarget rc_target {};
            if (get_rc_rate_target(rc_target)) {
                target_rate_rads = rc_target;
                target_rate_rads_valid = true;
                target_angle_rad_valid = false;
                target_loc_valid = false;
            } else if (get_rc_angle_target(rc_target)) {
                target_angle_rad = rc_target;
                target_angle_rad_valid = true;
                target_rate_rads_valid = false;
                target_loc_valid = false;
            }
            break;
        }

        // point mount towards a GPS point
        case MAV_MOUNT_MODE_GPS_POINT: {
            target_loc_valid = _roi_target_set;
            if (target_loc_valid) {
                target_loc = _roi_target;
                target_angle_rad_valid = get_angle_target_to_location(target_loc, target_angle_rad);
            } else {
                target_angle_rad_valid = false;
            }
            target_rate_rads_valid = false;
            break;
        }

        // point mount towards home
        case MAV_MOUNT_MODE_HOME_LOCATION: {
            target_loc_valid = AP::ahrs().home_is_set();
            if (target_loc_valid) {
                target_loc = AP::ahrs().get_home();
                target_angle_rad_valid = get_angle_target_to_home(target_angle_rad);
            } else {
                target_angle_rad_valid = false;
            }
            target_rate_rads_valid = false;
            break;
        }

        // point mount towards another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET: {
            target_loc_valid = _target_sysid_location_set;
            if (target_loc_valid) {
                target_loc = _target_sysid_location;
                target_angle_rad_valid = get_angle_target_to_location(target_loc, target_angle_rad);
            } else {
                target_angle_rad_valid = false;
            }
            target_rate_rads_valid = false;
            break;
        }

        default:
            // we do not know this mode so raise internal error
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }
}

// return true if healthy
bool AP_Mount_Scripting::healthy() const
{
    // healthy if scripting backend has updated actual angles recently
    return (AP_HAL::millis() - last_update_ms <= AP_MOUNT_SCRIPTING_TIMEOUT_MS);
}

// accessors for scripting backends
bool AP_Mount_Scripting::get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame)
{
    if (target_rate_rads_valid) {
        roll_degs = degrees(target_rate_rads.roll);
        pitch_degs = degrees(target_rate_rads.pitch);
        yaw_degs = degrees(target_rate_rads.yaw);
        yaw_is_earth_frame = target_rate_rads.yaw_is_ef;
        return true;
    }
    return false;
}

bool AP_Mount_Scripting::get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame)
{
    if (target_angle_rad_valid) {
        roll_deg = degrees(target_angle_rad.roll);
        pitch_deg = degrees(target_angle_rad.pitch);
        yaw_deg = degrees(target_angle_rad.yaw);
        yaw_is_earth_frame = target_angle_rad.yaw_is_ef;
        return true;
    }
    return false;
}

// return target location if available
// returns true if a target location is available and fills in target_loc argument
bool AP_Mount_Scripting::get_location_target(Location &_target_loc)
{
    if (target_loc_valid) {
        _target_loc = target_loc;
        return true;
    }
    return false;
}

// update mount's actual angles (to be called by script communicating with mount)
void AP_Mount_Scripting::set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg)
{
    last_update_ms = AP_HAL::millis();
    current_angle_deg.x = roll_deg;
    current_angle_deg.y = pitch_deg;
    current_angle_deg.z = yaw_bf_deg;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Scripting::get_attitude_quaternion(Quaternion& att_quat)
{
    // construct quaternion
    att_quat.from_euler(radians(current_angle_deg.x), radians(current_angle_deg.y), radians(current_angle_deg.z));
    return true;
}

#endif // HAL_MOUNT_SCRIPTING_ENABLED
