// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  SToRM32 mount backend class
 */

#ifndef __AP_MOUNT_STORM32_H__
#define __AP_MOUNT_STORM32_H__

#include <AP_HAL.h>
#include <AP_AHRS.h>

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>
#include <AP_Mount_Backend.h>

#define AP_MOUNT_STORM32_SYSID      71      // hardcoded system id
#define AP_MOUNT_STORM32_COMPID     67      // hard coded component id for communicating with the gimbal

#define AP_MOUNT_STORM32_RESEND_MS  1000    // resend angle targets to gimbal once per second

class AP_Mount_SToRM32 : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

private:

    // send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
    void send_do_mount_control(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode);

    // internal variables
    bool _initialised;              // true once the driver has been initialised
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal.  Currently hard-coded to Telem2
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
};

#endif // __AP_MOUNT_STORM32_H__
