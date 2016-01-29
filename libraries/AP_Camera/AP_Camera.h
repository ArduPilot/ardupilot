// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Camera.h
/// @brief	Photo or video camera manager, with EEPROM-backed storage of constants.

#ifndef AP_CAMERA_H
#define AP_CAMERA_H

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mission/AP_Mission.h>

#define AP_CAMERA_TRIGGER_TYPE_SERVO                0
#define AP_CAMERA_TRIGGER_TYPE_RELAY                1

#define AP_CAMERA_TRIGGER_DEFAULT_TRIGGER_TYPE  AP_CAMERA_TRIGGER_TYPE_SERVO    // default is to use servo to trigger camera

#define AP_CAMERA_TRIGGER_DEFAULT_DURATION  10      // default duration servo or relay is held open in 10ths of a second (i.e. 10 = 1 second)

#define AP_CAMERA_SERVO_ON_PWM              1300    // default PWM value to move servo to when shutter is activated
#define AP_CAMERA_SERVO_OFF_PWM             1100    // default PWM value to move servo to when shutter is deactivated

#define AP_CAMERA_FEEDBACK_DEFAULT_FEEDBACK_PIN -1  // default is to not use camera feedback pin

/// @class	Camera
/// @brief	Object managing a Photo or video camera
class AP_Camera {

public:
    /// Constructor
    ///
    AP_Camera(AP_Relay *obj_relay) :
        _trigger_counter(0),    // count of number of cycles shutter has been held open
        _image_index(0)
    {
		AP_Param::setup_object_defaults(this, var_info);
        _apm_relay = obj_relay;
    }

    // single entry point to take pictures
    //  set send_mavlink_msg to true to send DO_DIGICAM_CONTROL message to all components
    void            trigger_pic(bool send_mavlink_msg);

    // de-activate the trigger after some delay, but without using a delay() function
    // should be called at 50hz from main program
    void            trigger_pic_cleanup();

    // MAVLink methods
    void            control_msg(mavlink_message_t* msg);
    void            send_feedback(mavlink_channel_t chan, AP_GPS &gps, const AP_AHRS &ahrs, const Location &current_loc);

    // Command processing
    void            configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time);
    // handle camera control. Return true if picture was triggered
    bool            control(float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id);

    // set camera trigger distance in a mission
    void            set_trigger_distance(uint32_t distance_m) { _trigg_dist.set(distance_m); }

    // Update location of vehicle and return true if a picture should be taken
    bool update_location(const struct Location &loc, const AP_AHRS &ahrs);

    // check if trigger pin has fired
    bool check_trigger_pin(void);

    // return true if we are using a feedback pin
    bool using_feedback_pin(void) const { return _feedback_pin > 0; }
    
    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Int8         _trigger_type;      // 0:Servo,1:Relay
    AP_Int8         _trigger_duration;  // duration in 10ths of a second that the camera shutter is held open
    AP_Int8         _relay_on;          // relay value to trigger camera
    AP_Int16        _servo_on_pwm;      // PWM value to move servo to when shutter is activated
    AP_Int16        _servo_off_pwm;     // PWM value to move servo to when shutter is deactivated
    uint8_t         _trigger_counter;   // count of number of cycles shutter has been held open
    AP_Relay       *_apm_relay;         // pointer to relay object from the base class Relay.

    void            servo_pic();        // Servo operated camera
    void            relay_pic();        // basic relay activation
    void            feedback_pin_timer();
    
    AP_Float        _trigg_dist;        // distance between trigger points (meters)
    AP_Int16        _min_interval;      // Minimum time between shots required by camera
    AP_Int16        _max_roll;          // Maximum acceptable roll angle when trigging camera
    uint32_t        _last_photo_time;   // last time a photo was taken
    struct Location _last_location;
    uint16_t        _image_index;       // number of pictures taken since boot

    // pin number for accurate camera feedback messages
    AP_Int8         _feedback_pin;
    AP_Int8         _feedback_polarity;

    // this is set to 1 when camera trigger pin has fired
    volatile bool   _camera_triggered;
    bool            _timer_installed:1;
    uint8_t         _last_pin_state;
};

#endif /* AP_CAMERA_H */
