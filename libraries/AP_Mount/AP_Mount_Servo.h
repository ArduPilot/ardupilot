/*
  Servo controlled mount backend class
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_Mount_Backend.h"

class AP_Mount_Servo : public AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Servo(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance),
        _roll_idx(SRV_Channel::k_none),
        _tilt_idx(SRV_Channel::k_none),
        _pan_idx(SRV_Channel::k_none),
        _open_idx(SRV_Channel::k_none)
    {
    }

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override { return _flags.pan_control; }

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override;

    // send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void send_mount_status(mavlink_channel_t chan) override;

private:

    // flags structure
    struct {
        bool roll_control   :1; // true if mount has roll control
        bool tilt_control   :1; // true if mount has tilt control
        bool pan_control    :1; // true if mount has pan control
    } _flags;

    // check_servo_map - detects which axis we control (i.e. _flags) using the functions assigned to the servos in the SRV_Channel
    //  should be called periodically (i.e. 1hz or less)
    void    check_servo_map();

    // stabilize - stabilizes the mount relative to the Earth's frame
    void stabilize();

    // closest_limit - returns closest angle to 'angle' taking into account limits.  all angles are in degrees * 10
    int16_t closest_limit(int16_t angle, int16_t angle_min, int16_t angle_max);

    /// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
    void move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);

    // SRV_Channel - different id numbers are used depending upon the instance number
    SRV_Channel::Aux_servo_function_t    _roll_idx;  // SRV_Channel mount roll function index
    SRV_Channel::Aux_servo_function_t    _tilt_idx;  // SRV_Channel mount tilt function index
    SRV_Channel::Aux_servo_function_t    _pan_idx;   // SRV_Channel mount pan  function index
    SRV_Channel::Aux_servo_function_t    _open_idx;  // SRV_Channel mount open function index

    Vector3f _angle_bf_output_deg;  // final body frame output angle in degrees

    uint32_t _last_check_servo_map_ms;  // system time of latest call to check_servo_map function
};
