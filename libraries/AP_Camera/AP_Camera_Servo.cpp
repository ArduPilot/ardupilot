#include "AP_Camera_Servo.h"

#if AP_CAMERA_SERVO_ENABLED

#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// initialize the AP_Camera_Servo driver
void AP_Camera_Servo::init()
{
    // set the zoom and focus to the trim point
    SRV_Channels::set_output_scaled(SRV_Channel::k_cam_zoom, 500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_cam_focus, 500);
}

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
    float current_zoom = SRV_Channels::get_output_scaled(SRV_Channel::k_cam_zoom);
    float new_zoom = constrain_float(current_zoom + zoom_current_rate, 0, 1000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_cam_zoom, new_zoom);

    float current_focus = SRV_Channels::get_output_scaled(SRV_Channel::k_cam_focus);
    float new_focus = constrain_float(current_focus + focus_current_rate, 0, 1000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_cam_focus, new_focus);

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


bool AP_Camera_Servo::set_zoom(ZoomType zoom_type, float zoom_value)
{
    switch (zoom_type) {
        case ZoomType::RATE:
            zoom_current_rate = zoom_value;
            return true;
        case ZoomType::PCT:
            // expects to receive a value between 0 and 100
            // This maps it to our 0-1000 range
            SRV_Channels::set_output_scaled(SRV_Channel::k_cam_zoom, constrain_float(zoom_value * 10, 0, 1000));
            return true;
    }
    return false;
}

// set focus specified as rate
SetFocusResult AP_Camera_Servo::set_focus(FocusType focus_type, float focus_value)
{
    switch (focus_type) {
        case FocusType::RATE:
            focus_current_rate = focus_value;
            return SetFocusResult::ACCEPTED;
        case FocusType::PCT:
            // expects to receive a value between 0 and 100
            // This maps it to our 0-1000 range
            SRV_Channels::set_output_scaled(SRV_Channel::k_cam_focus, constrain_float(focus_value * 10, 0, 1000));
            return SetFocusResult::ACCEPTED;
        case FocusType::AUTO:
            return SetFocusResult::UNSUPPORTED;
    }
    return SetFocusResult::UNSUPPORTED;
}

// configure camera
void AP_Camera_Servo::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time)
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

// send camera settings message to GCS
void AP_Camera_Servo::send_camera_settings(mavlink_channel_t chan) const
{
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        CAMERA_MODE_IMAGE, // camera mode (0:image, 1:video, 2:image survey)
        SRV_Channels::get_output_scaled(SRV_Channel::k_cam_zoom) / 10.0f,     // zoomLevel float, percentage from 0 to 100, 0 if unassigned
        SRV_Channels::get_output_scaled(SRV_Channel::k_cam_focus) / 10.0f);   // focusLevel float, percentage from 0 to 100, 0 if unassigned
}

#endif // AP_CAMERA_SERVO_ENABLED
