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

    update_mnt_target();
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

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Scripting::get_attitude_quaternion(Quaternion& att_quat)
{
    // construct quaternion
    att_quat.from_euler(radians(current_angle_deg.x), radians(current_angle_deg.y), radians(current_angle_deg.z));
    return true;
}

#endif // HAL_MOUNT_SCRIPTING_ENABLED
