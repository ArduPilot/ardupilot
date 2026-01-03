/*
  MAVLink Gimbal Protocol v2 backend class
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_MAVLINK_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Mount_MAVLink : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    // update mount position
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control
    bool has_pan_control() const override { return yaw_range_valid(); }

    // handle GIMBAL_DEVICE_INFORMATION message
    void handle_gimbal_device_information(const mavlink_message_t &msg) override;

    // handle GIMBAL_DEVICE_ATTITUDE_STATUS message
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) override;

protected:

    // MVAVLink can send either rates or angles, and can also be
    // directly commanded to move to a retracted state
    uint8_t natively_supported_mount_target_types() const override {
        return (
            (1U<<unsigned(MountTargetType::ANGLE)) |
            (1U<<unsigned(MountTargetType::RATE)) |
            (1U<<unsigned(MountTargetType::RETRACTED))
            );
    };

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal();

    // request GIMBAL_DEVICE_INFORMATION from gimbal (holds vendor and model name, max lean angles)
    void request_gimbal_device_information() const;

    // start sending ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE to gimbal
    // returns true on success, false on failure to start sending
    bool start_sending_attitude_to_gimbal();

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to command gimbal to retract (aka relax)
    void send_target_retracted() override;

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control rate
    void send_target_rates(const MountRateTarget &rate_rads) override;

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
    void send_target_angles(const MountAngleTarget &angle_rad) override;

    // internal variables
    bool _got_device_info;          // true once gimbal has provided device info
    bool _initialised;              // true once the gimbal has provided a GIMBAL_DEVICE_INFORMATION
    uint32_t _last_devinfo_req_ms;  // system time that GIMBAL_DEVICE_INFORMATION was last requested (used to throttle requests)
    class GCS_MAVLINK *_link;       // link we have found gimbal on; nullptr if not seen yet
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_gimbal_device_attitude_status_t _gimbal_device_attitude_status;  // copy of most recently received gimbal status
    uint32_t _last_attitude_status_ms;  // system time last attitude status was received (used for health reporting)
};
#endif // HAL_MOUNT_MAVLINK_ENABLED
