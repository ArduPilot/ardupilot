// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Servo controlled mount backend class
 */

#ifndef __AP_MOUNT_SERVO_H__
#define __AP_MOUNT_SERVO_H__

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>
#include <RC_Channel_aux.h>
#include "AP_Mount_Backend.h"

class AP_Mount_Servo : public AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Servo(AP_Mount &frontend, uint8_t instance):
        AP_Mount_Backend(frontend, instance),
        _roll_idx(RC_Channel_aux::k_none),
        _tilt_idx(RC_Channel_aux::k_none),
        _pan_idx(RC_Channel_aux::k_none),
        _open_idx(RC_Channel_aux::k_none)
    {
        // init to no axis being controlled
        _flags.roll_control = false;
        _flags.tilt_control = false;
        _flags.pan_control = false;
    }

    // init - performs any required initialisation for this instance
    virtual void init();

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const { return _flags.pan_control; }

    // set_roi_target - sets target location that mount should attempt to point towards
    virtual void set_roi_target(const struct Location &target_loc);

    // configure_msg - process MOUNT_CONFIGURE messages received from GCS
    virtual void configure_msg(mavlink_message_t* msg);

    // control_msg - process MOUNT_CONTROL messages received from GCS
    virtual void control_msg(mavlink_message_t* msg);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

private:

    // flags structure
    struct {
        bool roll_control   :1; // true if mount has roll control
        bool tilt_control   :1; // true if mount has tilt control
        bool pan_control    :1; // true if mount has pan control
    } _flags;

    // check_servo_map - detects which axis we control (i.e. _flags) using the functions assigned to the servos in the RC_Channel_aux
    //  should be called periodically (i.e. 1hz or less)
    void    check_servo_map();

    // calc_angle_to_location - calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
    void calc_angle_to_location(const struct Location &target, Vector3f& angles_to_target_rad);

    // stabilize - stabilizes the mount relative to the Earth's frame
    void stabilize();

    // angle_input, angle_input_rad - convert RC input into an earth-frame target angle
    int32_t angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max);
    float angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max);

    // closest_limit - returns closest angle to 'angle' taking into account limits.  all angles are in degrees * 10
    int16_t closest_limit(int16_t angle, int16_t angle_min, int16_t angle_max);

    /// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
    void move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);

    // RC_Channel_aux - different id numbers are used depending upon the instance number
    RC_Channel_aux::Aux_servo_function_t    _roll_idx;  // RC_Channel_aux mount roll function index
    RC_Channel_aux::Aux_servo_function_t    _tilt_idx;  // RC_Channel_aux mount tilt function index
    RC_Channel_aux::Aux_servo_function_t    _pan_idx;   // RC_Channel_aux mount pan  function index
    RC_Channel_aux::Aux_servo_function_t    _open_idx;  // RC_Channel_aux mount open function index

    Vector3f _angle_ef_target_rad;  // desired earth-frame roll, tilt and pan angles in radians
    Vector3f _angle_bf_output_deg;  // final body frame output angle in degres
};

#endif // __AP_MOUNT_SERVO_H__
