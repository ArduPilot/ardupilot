// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_Camera.h
/// @brief	Photo or video camera manager, with EEPROM-backed storage of constants.
/// @author Amilcar Lucas

#ifndef AP_CAMERA_H
#define AP_CAMERA_H

#include <AP_Common.h>
#include <GCS_MAVLink.h>

/// @class	Camera
/// @brief	Object managing a Photo or video camera
class AP_Camera {

public:
    /// Constructor
    ///
    AP_Camera() :
        picture_time    (0),                            // waypoint trigger variable
        wp_distance_min (10),
        keep_cam_trigg_active_cycles (0),
        thr_pic                 (0),                            // timer variable for throttle_pic
        camtrig                 (83)                            // PK6 chosen as it not near anything so safer for soldering
    {
    }

    // single entry point to take pictures
    void            trigger_pic();

    // de-activate the trigger after some delay, but without using a delay() function
    void            trigger_pic_cleanup();

    // MAVLink methods
    void            configure_msg(mavlink_message_t* msg);
    void            control_msg(mavlink_message_t* msg);

    int16_t         picture_time;                               ///< waypoint trigger variable
    int32_t         wp_distance_min;                            ///< take picture if distance to WP is smaller than this

    static const struct AP_Param::GroupInfo        var_info[];

private:
    uint8_t         keep_cam_trigg_active_cycles; ///< event loop cycles to keep trigger active
    int16_t         thr_pic;                                            ///< timer variable for throttle_pic
    int16_t         camtrig;                                            ///< PK6 chosen as it not near anything so safer for soldering

    AP_Int8         trigger_type;       ///< 0=Servo, 1=relay, 2=throttle_off time, 3=throttle_off waypoint, 4=transistor

    void            servo_pic();        // Servo operated camera
    void            relay_pic();        // basic relay activation
    void            throttle_pic(); // pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
    void            distance_pic(); // pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
    void            NPN_pic();          // hacked the circuit to run a transistor? use this trigger to send output.

};

#endif /* AP_CAMERA_H */
