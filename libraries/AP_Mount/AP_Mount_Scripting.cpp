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
    // change to RC_TARGETING mode if RC input has changed
    set_rctargeting_on_rcinput_change();

    // update based on mount mode
    switch (get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &angle_bf_target = _params.retract_angles.get();
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            target_loc_valid = false;
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            target_loc_valid = false;
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // mavlink targets should have been already stored while handling the message
            target_loc_valid = false;
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
            target_loc_valid = false;
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
