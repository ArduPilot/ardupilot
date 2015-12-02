// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Servo controlled mount backend class
 */

#ifndef __AP_MOUNT_SERVO_H__
#define __AP_MOUNT_SERVO_H__

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel_aux.h>
#include "AP_Mount_Backend.h"

class AP_Mount_Servo : public AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Servo(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance),
        _roll_idx(RC_Channel_aux::k_none),
        _tilt_idx(RC_Channel_aux::k_none),
        _pan_idx(RC_Channel_aux::k_none),
        _open_idx(RC_Channel_aux::k_none),
        _last_check_servo_map_ms(0)
    {
        // init to no axis being controlled
        _flags.roll_control = false;
        _flags.tilt_control = false;
        _flags.pan_control = false;
    }

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const { return _flags.pan_control; }

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

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

    // stabilize - stabilizes the mount relative to the Earth's frame
    void stabilize();

    // closest_limit - returns closest angle to 'angle' taking into account limits.  all angles are in degrees * 10
    int16_t closest_limit(int16_t angle, int16_t angle_min, int16_t angle_max);

    /// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
    void move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);

    // RC_Channel_aux - different id numbers are used depending upon the instance number
    RC_Channel_aux::Aux_servo_function_t    _roll_idx;  // RC_Channel_aux mount roll function index
    RC_Channel_aux::Aux_servo_function_t    _tilt_idx;  // RC_Channel_aux mount tilt function index
    RC_Channel_aux::Aux_servo_function_t    _pan_idx;   // RC_Channel_aux mount pan  function index
    RC_Channel_aux::Aux_servo_function_t    _open_idx;  // RC_Channel_aux mount open function index

    Vector3f _angle_bf_output_deg;  // final body frame output angle in degrees

    uint32_t _last_check_servo_map_ms;  // system time of latest call to check_servo_map function
};

#endif // __AP_MOUNT_SERVO_H__
