/*
  CADDX mount using serial protocol backend class
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_CADDX_ENABLED

#include "AP_Mount_Backend_Serial.h"
#include <AP_Math/quaternion.h>

class AP_Mount_CADDX : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    // has_roll_control - returns true if this mount can control its roll
    bool has_roll_control() const override { return roll_range_valid(); };

    // has_pitch_control - returns true if this mount can control its tilt
    bool has_pitch_control() const override { return pitch_range_valid(); };

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // lock mode enum
    enum class LockMode {
        TILT_LOCK = (1<<0),
        ROLL_LOCK = (1<<1),
        YAW_LOCK  = (1<<2),
    };

    // send_target_angles
    void send_target_angles(const MountTarget& angle_target_rad);

    // internal variables
    uint32_t _last_send_ms;     // system time of last do_mount_control sent to gimbal
};
#endif // HAL_MOUNT_CADDX_ENABLED
