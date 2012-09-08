// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <AP_Camera.h>
#include <AP_Relay.h>
#include <../RC_Channel/RC_Channel_aux.h>

extern int32_t wp_distance;
extern AP_Relay relay;

// ------------------------------
#define CAM_DEBUG DISABLED

const AP_Param::GroupInfo AP_Camera::var_info[] PROGMEM = {
    // @Param: TRIGG_TYPE
    // @DisplayName: Camera shutter (trigger) type
    // @Description: how to trigger the camera to take a picture
    // @Values: 0:Servo,1:relay,2:throttle_off_time,3:throttle_off_waypoint,4:transistor
    // @User: Standard
    AP_GROUPINFO("TRIGG_TYPE",  0, AP_Camera, trigger_type, 0),
    AP_GROUPEND
};


/// Servo operated camera
void
AP_Camera::servo_pic()
{
	RC_Channel_aux::set_radio_to_max(RC_Channel_aux::k_cam_trigger);
	// leave a message that it should be active for two event loop cycles
	keep_cam_trigg_active_cycles = 2;
}

/// basic relay activation
void
AP_Camera::relay_pic()
{
    relay.on();
    keep_cam_trigg_active_cycles = 2;           // leave a message that it should be active for two event loop cycles
}

/// pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
void
AP_Camera::throttle_pic()
{
// TODO find a way to do this without using the global parameter g
//	g.channel_throttle.radio_out = g.throttle_min;
    if (thr_pic == 10) {
        servo_pic();            // triggering method
        thr_pic = 0;
//		g.channel_throttle.radio_out = g.throttle_cruise;
    }
    thr_pic++;
}

/// pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
void
AP_Camera::distance_pic()
{
// TODO find a way to do this without using the global parameter g
//	g.channel_throttle.radio_out = g.throttle_min;
    if (wp_distance < 3) {
        servo_pic();            // triggering method
//		g.channel_throttle.radio_out = g.throttle_cruise;
    }
}

/// hacked the circuit to run a transistor? use this trigger to send output.
void
AP_Camera::NPN_pic()
{
    // TODO: Assign pin spare pin for output
    digitalWrite(camtrig, HIGH);
    keep_cam_trigg_active_cycles = 1;           // leave a message that it should be active for two event loop cycles
}

/// single entry point to take pictures
void
AP_Camera::trigger_pic()
{
    switch (trigger_type)
    {
    case 0:
        servo_pic();                    // Servo operated camera
        break;
    case 1:
        relay_pic();                    // basic relay activation
        break;
    case 2:
        throttle_pic();                 // pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
        break;
    case 3:
        distance_pic();                 // pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
        break;
    case 4:
        NPN_pic();                              // hacked the circuit to run a transistor? use this trigger to send output.
        break;
    }
}

/// de-activate the trigger after some delay, but without using a delay() function
void
AP_Camera::trigger_pic_cleanup()
{
    if (keep_cam_trigg_active_cycles)
    {
        keep_cam_trigg_active_cycles--;
    }
    else
    {
        switch (trigger_type)
        {
        case 0:
        case 2:
        case 3:
			RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_cam_trigger);
            break;
        case 1:
            relay.off();
            break;
        case 4:
            digitalWrite(camtrig, LOW);
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
    // TODO do something with these values
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
    // TODO do something with these values
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


