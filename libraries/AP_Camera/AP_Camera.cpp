// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <AP_Camera.h>
#include <AP_Relay.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;
extern int32_t wp_distance;     // Note: unfortunately this variable is in meter for ArduPlane and cm for ArduCopter
extern AP_Relay relay;

// ------------------------------
#define CAM_DEBUG DISABLED

const AP_Param::GroupInfo AP_Camera::var_info[] PROGMEM = {
    // @Param: TRIGG_TYPE
    // @DisplayName: Camera shutter (trigger) type
    // @Description: how to trigger the camera to take a picture
    // @Values: 0:Servo,1:Relay,2:Servo and turn off throttle,3:Servo when 3m from waypoint,4:transistor
    // @User: Standard
    AP_GROUPINFO("TRIGG_TYPE",  0, AP_Camera, _trigger_type, AP_CAMERA_TRIGGER_DEFAULT_TRIGGER_TYPE),

    // @Param: DURATION
    // @DisplayName: Duration that shutter is held open
    // @Description: How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("DURATION",    1, AP_Camera, _trigger_duration, AP_CAMERA_TRIGGER_DEFAULT_DURATION),

    // @Param: SERVO_ON
    // @DisplayName: Servo ON PWM value
    // @Description: PWM value to move servo to when shutter is activated
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_ON",    2, AP_Camera, _servo_on_pwm, AP_CAMERA_SERVO_ON_PWM),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: PWM value to move servo to when shutter is deactivated
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF",   3, AP_Camera, _servo_off_pwm, AP_CAMERA_SERVO_OFF_PWM),

    AP_GROUPEND
};


/// Servo operated camera
void
AP_Camera::servo_pic()
{
	RC_Channel_aux::set_radio(RC_Channel_aux::k_cam_trigger, _servo_on_pwm);

	// leave a message that it should be active for this many loops (assumes 50hz loops)
	_trigger_counter = constrain_int16(_trigger_duration*5,0,255);
}

/// basic relay activation
void
AP_Camera::relay_pic()
{
    _apm_relay->on();

    // leave a message that it should be active for this many loops (assumes 50hz loops)
    _trigger_counter = constrain_int16(_trigger_duration*5,0,255);
}

/// pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
void
AP_Camera::throttle_pic()
{
// TODO find a way to do this without using the global parameter g
//	g.channel_throttle.radio_out = g.throttle_min;
    if (_thr_pic_counter == 10) {
        servo_pic();            // triggering method
        _thr_pic_counter = 0;
//		g.channel_throttle.radio_out = g.throttle_cruise;
    }
    _thr_pic_counter++;
}

/// distance_pic - triggers picture when within 3m of waypoint
void
AP_Camera::distance_pic()
{
    if (wp_distance < AP_CAMERA_WP_DISTANCE) {
        servo_pic();            // triggering method
    }
}

/// hacked the circuit to run a transistor? use this trigger to send output.
void
AP_Camera::transistor_pic()
{
    // TODO: Assign pin spare pin for output
    hal.gpio->write(AP_CAMERA_TRANSISTOR_PIN,1);

    // leave a message that it should be active for two event loop cycles
    _trigger_counter = 1;
}

/// single entry point to take pictures
void
AP_Camera::trigger_pic()
{
    switch (_trigger_type)
    {
    case AP_CAMERA_TRIGGER_TYPE_SERVO:
        servo_pic();                    // Servo operated camera
        break;
    case AP_CAMERA_TRIGGER_TYPE_RELAY:
        relay_pic();                    // basic relay activation
        break;
    case AP_CAMERA_TRIGGER_TYPE_THROTTLE_OFF_TIME:
        throttle_pic();                 // pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
        break;
    case AP_CAMERA_TRIGGER_TYPE_WP_DISTANCE:
        distance_pic();                 // pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
        break;
    case AP_CAMERA_TRIGGER_TYPE_TRANSISTOR:
        transistor_pic();                              // hacked the circuit to run a transistor? use this trigger to send output.
        break;
    }
}

/// de-activate the trigger after some delay, but without using a delay() function
/// should be called at 50hz
void
AP_Camera::trigger_pic_cleanup()
{
    if (_trigger_counter) {
        _trigger_counter--;
    } else {
        switch (_trigger_type) {
            case AP_CAMERA_TRIGGER_TYPE_SERVO:
            case AP_CAMERA_TRIGGER_TYPE_THROTTLE_OFF_TIME:
            case AP_CAMERA_TRIGGER_TYPE_WP_DISTANCE:
                RC_Channel_aux::set_radio(RC_Channel_aux::k_cam_trigger, _servo_off_pwm);
                break;
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
            case AP_CAMERA_TRIGGER_TYPE_RELAY:
                _apm_relay->off();
                break;
#endif
            case AP_CAMERA_TRIGGER_TYPE_TRANSISTOR:
                hal.gpio->write(AP_CAMERA_TRANSISTOR_PIN, 0);
                break;
        }
    }
}

/// decode MavLink that configures camera
void
AP_Camera::configure_msg(mavlink_message_t* msg)
{
    __mavlink_digicam_configure_t packet;
    mavlink_msg_digicam_configure_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        // not for us
        return;
    }
    // This values may or not be used by APM
    // They are bypassed as "echo" to a external specialized board
    /*
     *  packet.aperture
     *  packet.command_id
     *  packet.engine_cut_off
     *  packet.exposure_type
     *  packet.extra_param
     *  packet.extra_value
     *  packet.iso
     *  packet.mode
     *  packet.shutter_speed
     */
    // echo the message to the ArduCam OSD camera control board
    // for more info see: http://code.google.com/p/arducam-osd/
    // TODO is it connected to MAVLINK_COMM_3 ?
    mavlink_msg_digicam_configure_send(MAVLINK_COMM_3, packet.target_system, packet.target_component, packet.mode, packet.shutter_speed, packet.aperture, packet.iso, packet.exposure_type, packet.command_id, packet.engine_cut_off, packet.extra_param, packet.extra_value);
}

/// decode MavLink that controls camera
void
AP_Camera::control_msg(mavlink_message_t* msg)
{
    __mavlink_digicam_control_t packet;
    mavlink_msg_digicam_control_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        // not for us
        return;
    }
    // This values may or not be used by APM (the shot is)
    // They are bypassed as "echo" to a external specialized board
    /*
     *  packet.command_id
     *  packet.extra_param
     *  packet.extra_value
     *  packet.focus_lock
     *  packet.session
     *  packet.shot
     *  packet.zoom_pos
     *  packet.zoom_step
     */
    if (packet.shot)
    {
        trigger_pic();
    }
    // echo the message to the ArduCam OSD camera control board
    // for more info see: http://code.google.com/p/arducam-osd/
    // TODO is it connected to MAVLINK_COMM_3 ?
    mavlink_msg_digicam_control_send(MAVLINK_COMM_3, packet.target_system, packet.target_component, packet.session, packet.zoom_pos, packet.zoom_step, packet.focus_lock, packet.shot, packet.command_id, packet.extra_param, packet.extra_value);
}


