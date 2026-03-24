/*
  Scripting mount/gimbal driver
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SCRIPTING_ENABLED

#include "AP_Mount_Backend.h"

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
    void set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg) override;

protected:

    // Scripting backends poll get_angle_target / get_rate_target rather than
    // receiving pushed targets, so native support for both types is declared so
    // send_target_to_gimbal() never converts rates to angles on the backend's
    // behalf.  send_target_angles() stamps mnt_target.target_type = ANGLE so
    // that get_angle_target() returns the converted value for non-angle modes
    // (RETRACT, NEUTRAL, LOCATION) that send_target_to_gimbal() converts and
    // stores into mnt_target.angle_rad before calling send_target_angles().
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_AND_RATES_ONLY;
    };
    void send_target_angles(const MountAngleTarget &angle_rad) override;
    void send_target_rates(const MountRateTarget &rate_rads) override {};

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // internal variables
    uint32_t last_update_ms;        // system time of last call to one of the get_ methods.  Used for health reporting
    Vector3f current_angle_deg;     // current gimbal angles in degrees (x=roll, y=pitch, z=yaw)

};

#endif // HAL_MOUNT_SCRIPTING_ENABLED
