/*
  Gremsy mount backend class
 */
#pragma once

#include "AP_Mount_Backend.h"

#ifndef HAL_MOUNT_GREMSY_ENABLED
#define HAL_MOUNT_GREMSY_ENABLED (HAL_MOUNT_ENABLED && !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024)
#endif

#if HAL_MOUNT_GREMSY_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Mount_Gremsy : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_Gremsy(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init
    void init() override {}

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
    void send_gimbal_device_retract() const;

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control rate
    // earth_frame should be true if yaw_rads target is an earth frame rate, false if body_frame
    void send_gimbal_device_set_rate(float roll_rads, float pitch_rads, float yaw_rads, bool earth_frame) const;

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
    // earth_frame should be true if yaw_rad target is an earth frame angle, false if body_frame
    void send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame) const;

    // internal variables
    bool _found_gimbal;             // true once a MAVLink enabled gimbal has been found
    bool _got_device_info;          // true once gimbal has provided device info
    bool _initialised;              // true once the gimbal has provided a GIMBAL_DEVICE_INFORMATION
    uint32_t _last_devinfo_req_ms;  // system time that GIMBAL_DEVICE_INFORMATION was last requested (used to throttle requests)
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_gimbal_device_attitude_status_t _gimbal_device_attitude_status;  // copy of most recently received gimbal status
    uint32_t _last_attitude_status_ms;  // system time last attitude status was received (used for health reporting)
    uint32_t _sent_gimbal_device_attitude_status_ms;    // time_boot_ms field of gimbal_device_status message last forwarded to the GCS (used to prevent sending duplicates)
};
#endif // HAL_MOUNT_GREMSY_ENABLED
