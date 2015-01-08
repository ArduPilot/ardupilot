// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  MAVLink enabled mount backend class
 */

#ifndef __AP_MOUNT_MAVLINK_H__
#define __AP_MOUNT_MAVLINK_H__

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>
#include <AP_Mount_Backend.h>

class AP_Mount_MAVLink : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_MAVLink(AP_Mount &frontend, uint8_t instance) :
        AP_Mount_Backend(frontend, instance),
        _enabled(false),
        _chan(MAVLINK_COMM_0),
        _sysid(0),
        _compid(0)
    {}

    // init - performs any required initialisation for this instance
    virtual void init();

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // set_roi_target - sets target location that mount should attempt to point towards
    virtual void set_roi_target(const struct Location &target_loc);

    // configure_msg - process MOUNT_CONFIGURE messages received from GCS
    virtual void configure_msg(mavlink_message_t* msg);

    // control_msg - process MOUNT_CONTROL messages received from GCS
    virtual void control_msg(mavlink_message_t* msg);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

private:

    // find_mount - search for MAVLink enabled mount
    void find_mount();

    // send_angle_target - send earth-frame angle targets (in degrees) to mount
    void send_angle_target(const Vector3f& target_deg);

    // internal variables
    bool    _enabled;           // gimbal becomes enabled once the mount has been discovered
    mavlink_channel_t _chan;    // telemetry channel used to communicate with mount
    uint8_t _sysid;             // system id of MAVLink enabled mount
    uint8_t _compid;            // component id of MAVLink enabled mount
};

#endif // __AP_MOUNT_MAVLINK_H__
