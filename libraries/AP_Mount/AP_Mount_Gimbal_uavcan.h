/*
    Up&Above mount using uavcan protocol backend class
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_Mount_Backend.h"

#if HAL_MOUNT_ENABLED

#define AP_MOUNT_UAVCAN_RESEND_MS  1000    // resend angle targets to gimbal once per second
#define AP_MOUNT_UAVCAN_SEARCH_MS  60000   // search for gimbal for 1 minute after startup

class AP_Mount_Gimbal_uavcan : public AP_Mount_Backend
{

public:

    // Constructor
    /*
    AP_Mount_Gimbal_uavcan(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance)
    {}
    */
    AP_Mount_Gimbal_uavcan(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override;
    
    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override;

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override;

    // send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void send_mount_status(mavlink_channel_t chan) override;
    
    // Added Up&Above gimbal
    // configure - allows to configure mount control modes
    virtual void configure(enum MAV_MOUNT_MODE mount_mode, uint8_t stab_roll, uint8_t stab_pitch, uint8_t stab_yaw, enum AP_Mount::ControlMode roll_mode, enum AP_Mount::ControlMode pitch_mode, enum AP_Mount::ControlMode yaw_mode) override;



private:

        bool _initialised;              // true once the driver has been initialised
        
        //Added Up&Above gimbal
        void find_gimbal(); // search for gimbal
        void send_mount_control();
        uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
        enum AP_Mount::ControlMode _control_mode; //Which control mode currently is used

};
#endif // HAL_MOUNT_ENABLED
