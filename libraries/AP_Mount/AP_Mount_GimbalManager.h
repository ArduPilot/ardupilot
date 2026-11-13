/*
  MAVLink gimbal manager client backend class

  sends targets to an external MAVLink gimbal manager instead of
  acting as the gimbal manager for the gimbal device.  The gimbal
  device must be a MAVLink gimbal visible to the autopilot
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_GIMBAL_MANAGER_ENABLED

#include "AP_Mount_MAVLink.h"

class AP_Mount_GimbalManager : public AP_Mount_MAVLink
{

public:
    // Constructor
    using AP_Mount_MAVLink::AP_Mount_MAVLink;

    // the external gimbal manager advertises itself
    void send_gimbal_manager_information(mavlink_channel_t chan) override {}
    void send_gimbal_manager_status(mavlink_channel_t chan) override {}

protected:

    // ROI and neutral are converted to angles by the backend
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_AND_RATES_ONLY | (1U << unsigned(MountTargetType::RETRACTED));
    }

private:

    // send GIMBAL_MANAGER_SET_PITCHYAW to the gimbal manager
    void send_target_retracted() override;
    void send_target_rates(const MountRateTarget &rate_rads) override;
    void send_target_angles(const MountAngleTarget &angle_rad) override;
    void send_pitchyaw(uint32_t flags, float pitch, float yaw, float pitch_rate, float yaw_rate);
};
#endif // HAL_MOUNT_GIMBAL_MANAGER_ENABLED
