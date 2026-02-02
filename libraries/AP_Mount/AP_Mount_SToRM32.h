/*
  SToRM32 mount backend class
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_STORM32MAVLINK_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

class AP_Mount_SToRM32 : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); }

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal();

    // SToRM32 can only send angles:
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_ONLY;
    };
    
    // allow removing lean angles for pitch and roll locks
    bool apply_bf_roll_pitch_adjustments_in_rc_targeting() const override {
        return true;
    }

    // send_do_mount_control with latest angle targets
    void send_target_angles(const MountAngleTarget& angle_target_rad) override;

    // internal variables
    bool _initialised;              // true once the driver has been initialised
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_channel_t _chan = MAVLINK_COMM_0;        // mavlink channel used to communicate with gimbal
};
#endif // HAL_MOUNT_STORM32MAVLINK_ENABLED
