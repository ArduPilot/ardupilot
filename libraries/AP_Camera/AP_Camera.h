// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_Camera.h
/// @brief	Photo or video camera manager, with EEPROM-backed storage of constants.
/// @author Amilcar Lucas

#ifndef AP_CAMERA_H
#define AP_CAMERA_H

#include <AP_Param.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>

#define AP_CAMERA_TRIGGER_TYPE_SERVO                0
#define AP_CAMERA_TRIGGER_TYPE_RELAY                1
#define AP_CAMERA_TRIGGER_TYPE_THROTTLE_OFF_TIME    2
#define AP_CAMERA_TRIGGER_TYPE_WP_DISTANCE          3
#define AP_CAMERA_TRIGGER_TYPE_TRANSISTOR           4

#define AP_CAMERA_TRANSISTOR_PIN    83              // PK6 chosen as it not near anything so safer for soldering

#define AP_CAMERA_WP_DISTANCE       3               // trigger camera shutter when within this many meters of target.  Unfortunately this variable is in meter for ArduPlane and cm for ArduCopter so it will not work for ArduCopter

#define AP_CAMERA_TRIGGER_DEFAULT_TRIGGER_TYPE  AP_CAMERA_TRIGGER_TYPE_SERVO    // default is to use servo to trigger camera

#define AP_CAMERA_TRIGGER_DEFAULT_DURATION  10      // default duration servo or relay is held open in 10ths of a second (i.e. 10 = 1 second)

#define AP_CAMERA_SERVO_ON_PWM              1300    // default PWM value to move servo to when shutter is activated
#define AP_CAMERA_SERVO_OFF_PWM             1100    // default PWM value to move servo to when shutter is deactivated

/// @class	Camera
/// @brief	Object managing a Photo or video camera
class AP_Camera {

public:
    /// Constructor
    ///
    AP_Camera() :
        _trigger_counter(0),            // count of number of cycles shutter has been held open
        _thr_pic_counter(0)             // timer variable for throttle_pic
    {
    }

    // single entry point to take pictures
    void            trigger_pic();

    // de-activate the trigger after some delay, but without using a delay() function
    // should be called at 50hz from main program
    void            trigger_pic_cleanup();

    // MAVLink methods
    void            configure_msg(mavlink_message_t* msg);
    void            control_msg(mavlink_message_t* msg);

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Int8         _trigger_type;      // 0:Servo,1:Relay,2:Servo and turn off throttle,3:Servo when 3m from waypoint,4:transistor
    AP_Int8         _trigger_duration;  // duration in 10ths of a second that the camera shutter is held open
    AP_Int16        _servo_on_pwm;      // PWM value to move servo to when shutter is activated
    AP_Int16        _servo_off_pwm;     // PWM value to move servo to when shutter is deactivated
    uint8_t         _trigger_counter;   // count of number of cycles shutter has been held open
    uint8_t         _thr_pic_counter;   // timer variable for throttle_pic

    void            servo_pic();        // Servo operated camera
    void            relay_pic();        // basic relay activation
    void            throttle_pic();     // pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
    void            distance_pic();     // pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
    void            transistor_pic();   // hacked the circuit to run a transistor? use this trigger to send output.

};

#endif /* AP_CAMERA_H */
