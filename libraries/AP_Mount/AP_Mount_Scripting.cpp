#include "AP_Mount_config.h"

#if HAL_MOUNT_SCRIPTING_ENABLED

#include "AP_Mount_Scripting.h"

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
    AP_Mount_Backend::update();

    // reset script target type so get_angle_target / get_rate_target return
    // false until send_target_to_gimbal() writes a fresh target this cycle
    _script_target_type = ScriptTargetType::NONE;

    update_mnt_target();

    send_target_to_gimbal();
}

// return true if healthy
bool AP_Mount_Scripting::healthy() const
{
    // healthy if scripting backend has updated actual angles recently
    return (AP_HAL::millis() - last_update_ms <= AP_MOUNT_SCRIPTING_TIMEOUT_MS);
}

// update mount's actual angles (to be called by script communicating with mount)
void AP_Mount_Scripting::set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg)
{
    last_update_ms = AP_HAL::millis();
    current_angle_deg.x = roll_deg;
    current_angle_deg.y = pitch_deg;
    current_angle_deg.z = yaw_bf_deg;
}

// called by send_target_to_gimbal() with the angle target for this cycle.
// Store it so get_angle_target() can return it to the Lua script.
void AP_Mount_Scripting::send_target_angles(const MountAngleTarget &angle_rad)
{
    _angle_target = angle_rad;
    _script_target_type = ScriptTargetType::ANGLE;
}

// called by send_target_to_gimbal() with the rate target for this cycle.
// Store it so get_rate_target() can return it to the Lua script.
void AP_Mount_Scripting::send_target_rates(const MountRateTarget &rate_rads)
{
    _rate_target = rate_rads;
    _script_target_type = ScriptTargetType::RATE;
}

// get target angle in deg. returns true on success
bool AP_Mount_Scripting::get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame)
{
    if (_script_target_type != ScriptTargetType::ANGLE) {
        return false;
    }
    roll_deg = degrees(_angle_target.roll);
    pitch_deg = degrees(_angle_target.pitch);
    yaw_deg = degrees(_angle_target.yaw);
    yaw_is_earth_frame = _angle_target.yaw_is_ef;
    return true;
}

// get target rate in deg/sec. returns true on success
bool AP_Mount_Scripting::get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame)
{
    if (_script_target_type != ScriptTargetType::RATE) {
        return false;
    }
    roll_degs = degrees(_rate_target.roll);
    pitch_degs = degrees(_rate_target.pitch);
    yaw_degs = degrees(_rate_target.yaw);
    yaw_is_earth_frame = _rate_target.yaw_is_ef;
    return true;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Scripting::get_attitude_quaternion(Quaternion& att_quat)
{
    // construct quaternion
    att_quat.from_euler(radians(current_angle_deg.x), radians(current_angle_deg.y), radians(current_angle_deg.z));
    return true;
}

#endif // HAL_MOUNT_SCRIPTING_ENABLED
