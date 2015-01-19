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

#define AP_MOUNT_MAVLINK_COMPID     10      // Use hard-coded component id to communicate with gimbal, sysid is taken from globally defined mavlink_system.sysid

class AP_Mount_MAVLink : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_MAVLink(AP_Mount &frontend, AP_Mount::mount_state state, uint8_t instance) :
        AP_Mount_Backend(frontend, state, instance),
        _initialised(false),
        _chan(MAVLINK_COMM_0),
        _last_mode(MAV_MOUNT_MODE_RETRACT)
    {}

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
    // send_angle_target - send earth-frame angle targets to mount
    //  set target_in_degrees to true to send degree targets, false if target in radians
    void send_angle_target(const Vector3f& target, bool target_in_degrees);

    // internal variables
    bool _initialised;              // true once the driver has been initialised
    mavlink_channel_t _chan;        // telemetry channel used to communicate with mount
    enum MAV_MOUNT_MODE _last_mode; // last mode sent to mount
    Vector3f _last_angle_target;    // last angle target sent to mount
};

#endif // __AP_MOUNT_MAVLINK_H__
