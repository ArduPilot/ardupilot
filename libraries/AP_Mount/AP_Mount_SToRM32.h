/*
  SToRM32 mount backend class
 */
#pragma once

#include "AP_Mount_Backend.h"

#ifndef HAL_MOUNT_STORM32MAVLINK_ENABLED
#define HAL_MOUNT_STORM32MAVLINK_ENABLED HAL_MOUNT_ENABLED
#endif

#if HAL_MOUNT_STORM32MAVLINK_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

class AP_Mount_SToRM32 : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override {}

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

    // send_do_mount_control with latest angle targets
    void send_do_mount_control(const MountTarget& angle_target_rad);

    // internal variables
    bool _initialised;              // true once the driver has been initialised
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
    MountTarget _angle_rad;         // latest angle target
};
#endif // HAL_MOUNT_STORM32MAVLINK_ENABLED
