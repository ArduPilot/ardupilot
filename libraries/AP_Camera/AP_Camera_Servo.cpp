#include "AP_Camera_Servo.h"

#if AP_CAMERA_SERVO_ENABLED

#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// update - should be called at 50hz
void AP_Camera_Servo::update()
{
    // shutter counter
    if (trigger_counter > 0) {
        trigger_counter--;
    } else {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_trigger, _params.servo_off_pwm);
    }

    // iso counter
    if (iso_counter > 0) {
        iso_counter--;
    } else {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_iso, _params.servo_off_pwm);
    }

    // call parent update
    AP_Camera_Backend::update();
}

// entry point to actually take a picture.  returns true on success
bool AP_Camera_Servo::trigger_pic()
{
    // fail if have not completed previous picture
    if (trigger_counter > 0) {
        return false;
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_cam_trigger, _params.servo_on_pwm);

    // set counter to move servo to off position after this many iterations of update (assumes 50hz update rate)
    trigger_counter = constrain_float(_params.trigger_duration * 50, 0, UINT16_MAX);

    return true;
}

// configure camera
void AP_Camera_Servo::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{
    // designed to control Blackmagic Micro Cinema Camera (BMMCC) cameras
    // if the message contains non zero values then use them for the below functions
    if (ISO > 0) {
        // set a trigger for the iso function that is flip controlled
        iso_counter = constrain_float(_params.trigger_duration * 50, 0, UINT16_MAX);
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_iso, _params.servo_on_pwm);
    }

    if (aperture > 0) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_aperture, (uint16_t)aperture);
    }

    if (shutter_speed > 0) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_shutter_speed, (uint16_t)shutter_speed);
    }

    // Use the shooting mode PWM value for the BMMCC as the focus control - no need to modify or create a new MAVlink message type.
    if (shooting_mode > 0) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_cam_focus, (uint16_t)shooting_mode);
    }
}

#endif // AP_CAMERA_SERVO_ENABLED
