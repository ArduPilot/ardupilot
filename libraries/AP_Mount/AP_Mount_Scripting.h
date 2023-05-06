/*
  Scripting mount/gimbal driver
 */

#pragma once

#include "AP_Mount_Backend.h"

#if HAL_MOUNT_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

class AP_Mount_Scripting : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Scripting);

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    // accessors for scripting backends
    bool get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame) override;
    bool get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame) override;
    bool get_location_target(Location& _target_loc) override;
    void set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg) override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // internal variables
    uint32_t last_update_ms;        // system time of last call to one of the get_ methods.  Used for health reporting
    Vector3f current_angle_deg;     // current gimbal angles in degrees (x=roll, y=pitch, z=yaw)

    MountTarget target_rate_rads;   // rate target in rad/s
    bool target_rate_rads_valid;    // true if _target_rate_degs holds a valid rate target

    MountTarget target_angle_rad;   // angle target in radians
    bool target_angle_rad_valid;    // true if _target_rate_degs holds a valid rate target

    Location target_loc;            // target location
    bool target_loc_valid;          // true if target_loc holds a valid target location
};

#endif // HAL_MOUNT_SIYISERIAL_ENABLED
