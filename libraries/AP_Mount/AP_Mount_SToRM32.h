/*
  SToRM32 mount backend class
 */
#pragma once

#include "AP_Mount_Backend.h"

#if HAL_MOUNT_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_AHRS/AP_AHRS.h>

class AP_Mount_SToRM32 : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override {}

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override;

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override;

    // send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void send_mount_status(mavlink_channel_t chan) override;

private:

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal();

    // send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
    void send_do_mount_control(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode);

    // internal variables
    bool _initialised;              // true once the driver has been initialised
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
};
#endif // HAL_MOUNT_ENABLED
