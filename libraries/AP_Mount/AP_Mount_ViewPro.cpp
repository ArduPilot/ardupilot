#include "AP_Mount_ViewPro.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_SerialManager/AP_SerialManager.h>



#define CAM_SPD_MAX 30.0
#define CAM_SPD_MIN 2.0

extern const AP_HAL::HAL& hal;

AP_Mount_ViewPro::AP_Mount_ViewPro(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _reply_type(ReplyType_UNKNOWN)
{}

// init - performs any required initialisation for this instance
void AP_Mount_ViewPro::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ViewPro, 0);
    if (_port) {
        _initialised = true;
        set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
        _previous_mode = get_mode();
        _zooming_state_change = false;

    }

    is_recording = false;
    is_tracking = false;
    pip_state = 0;
    color_state = 0;
    state_is_video = true;
    is_connected = false;

    yaw_timeout_counter = 0;


	command_flags.change_state = false;
	command_flags.take_picture = false;
	command_flags.center_yaw = false;
	command_flags.toggle_video = false;
	command_flags.stop_video = false;
	command_flags.color_change = false;
	command_flags.IR_zoom = false;
	command_flags.toggle_video_tracking = false;
	command_flags.toggle_tracking = false;
	command_flags.toggle_tracking_video_state = false;
	command_flags.toggle_pip = false;
	command_flags.toggle_track = false;
	command_flags.flip_image = false;
	ir_zoom_level = 1;
	mode_change_flag = false;
	yaw_timeout = false;
	yaw_center_reset_flag = false;

	pip_change_hold = false;
	color_change_hold = false;

	current_rec_state = REC_OFF;

	tracking_camera_on = false;
	tracking_camera_off = false;
	tracking_camera_pic = false;

}

// update mount position - should be called periodically
void AP_Mount_ViewPro::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

//Center yaw if switched to manual flight (!_RC_control_enable) and you are not currently tracking a GPS point

    if((get_mode() == MAV_MOUNT_MODE_RC_TARGETING) and !_RC_control_enable){
    	//set command if we just switched from RC control and to RC_targeting with the gimbal
    	if(yaw_center_reset_flag){
    		command_flags.center_yaw = true;
    	}
    	//set reset flag false so we don't keep sending
    	yaw_center_reset_flag = false;

    }else{
    	yaw_center_reset_flag = true;
    }


    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
            {
            const Vector3f &target = _state._retract_angles.get();
            _angle_ef_target_deg.x = target.x;
            _angle_ef_target_deg.y = target.y;
            _angle_ef_target_deg.z = target.z;
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_deg.x = target.x;
            _angle_ef_target_deg.y = target.y;
            _angle_ef_target_deg.z = target.z;

            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:

        	// update gimbal motion speed targets using pilot's rc inputs
        	update_target_spd_from_rc();
        	update_zoom_focus_from_rc();
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
        {
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_deg, true, true);
            }


            //while in GPS POINT, can switch to RC_Targeting if any gimbal motion controls are used
        	const RC_Channel *pan_ch = rc().channel(CH_1);
        	const RC_Channel *tilt_ch = rc().channel(CH_2);
        	const RC_Channel *tilt_wheel_ch = rc().channel(CH_6);

            if(!is_zero(tilt_wheel_ch->norm_input_dz()) or (_RC_control_enable and (!is_zero(pan_ch->norm_input_dz()) or !is_zero(tilt_ch->norm_input_dz())))){
            	set_mode(MAV_MOUNT_MODE_RC_TARGETING);
            }

            update_zoom_focus_from_rc();

            break;
        }

        default:
            // we do not know this mode so do nothing
            break;
    }


    //Debugging Code
/*
    if((AP_HAL::millis() - _last_send) > 100){

    	hal.console->print("\n");
    	hal.console->print("\n");
    //	hal.console->printf("zoom:%ul", _zoom_level)  ;
    //	hal.console->print("   ");
    //	hal.console->printf("tilt:%f", _camera_tilt_angle)  ;
    //	hal.console->print("   ");
    	hal.console->printf("pan:%f", _camera_pan_angle)  ;
    	hal.console->print("\n");
    	hal.console->print("\n");

    }
*/


    if(get_mode() == MAV_MOUNT_MODE_RC_TARGETING){

    	if((AP_HAL::millis() - _last_send) > 50){
			send_targeting_cmd();
    		}


			return;

    }else if(get_mode() == MAV_MOUNT_MODE_MAVLINK_TARGETING or get_mode() == MAV_MOUNT_MODE_NEUTRAL){

    	if((AP_HAL::millis() - _last_send) > 100){
    		send_targeting_cmd();
    	}

    	return;

    }else if(get_mode() == MAV_MOUNT_MODE_GPS_POINT){

    	if((AP_HAL::millis() - _last_send) > 100){
    		send_targeting_cmd();
    	}

    }

}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_ViewPro::has_pan_control() const
{
    // we do not have yaw control
    return true;
}

// set_mode - sets mount's mode
void AP_Mount_ViewPro::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_ViewPro::send_mount_status(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.
    mavlink_msg_mount_status_send(chan, 0, 0, _camera_tilt_angle, 0, _camera_pan_angle);
}

bool AP_Mount_ViewPro::can_send(bool with_control) {
    uint16_t required_tx = 1;
    if (with_control) {
        required_tx += sizeof(AP_Mount_ViewPro::cmd_set_angles_struct);
    }
    return (_reply_type == ReplyType_UNKNOWN) && (_port->txspace() >= required_tx);
}



// send_target_angles
void AP_Mount_ViewPro::send_targeting_cmd()
{

	////////////////////////////////////////
	/// Send any open command requests//////
	////////////////////////////////////////

	//one time commands trigger flags that persist until they are sent.
	//only one command is sent per cycle and no gimbal motion commands are sent.

	uint8_t* buf_cmd;
	uint8_t buf_size;

	//toggle between picture mode or record mode
	if(command_flags.change_state){

		static cmd_6_byte_struct cmd_change_video_state;

		cmd_change_video_state.byte1 = 0x81;
		cmd_change_video_state.byte2 = 0x01;
		cmd_change_video_state.byte3 = 0x04;
		cmd_change_video_state.byte4 = 0x68;
		cmd_change_video_state.byte5 = 0x05;
		cmd_change_video_state.byte6 = 0xFF;

		buf_size = sizeof(cmd_change_video_state);
		buf_cmd = (uint8_t*)&cmd_change_video_state;

		if ((size_t)_port->txspace() < buf_size) {
			return;
		}

	    for (uint8_t i = 0;  i != buf_size ; i++) {
	        _port->write(buf_cmd[i]);
	    }

	    //Don't repeat command
	    command_flags.change_state = false;

	   // command_flags.toggle_tracking_video_state = true;
		//tracking_camera_off = false;
		//tracking_camera_on = false;
		//tracking_camera_pic = true;


		//setup command for tracking cameras
		command_flags.toggle_tracking_video_state = true;

		if(current_rec_state == REC_OFF){
			tracking_camera_off = false;
			tracking_camera_on = true;
			tracking_camera_pic = false;
		}else{
			tracking_camera_off = true;
			tracking_camera_on = false;
			tracking_camera_pic = false;
		}

	    _last_send = AP_HAL::millis();

	    return;

	}else if(command_flags.turn_camera_off){


		static cmd_6_byte_struct cmd_camera_off;

		cmd_camera_off.byte1 = 0x81;
		cmd_camera_off.byte2 = 0x01;
		cmd_camera_off.byte3 = 0x04;
		cmd_camera_off.byte4 = 0x68;
		cmd_camera_off.byte5 = 0x03;
		cmd_camera_off.byte6 = 0xFF;

		buf_size = sizeof(cmd_camera_off);
		buf_cmd = (uint8_t*)&cmd_camera_off;

		if ((size_t)_port->txspace() < buf_size) {
			return;
		}

		for (uint8_t i = 0;  i != buf_size ; i++) {
			_port->write(buf_cmd[i]);
		}

		command_flags.turn_camera_off = false;


		//setup command for tracking cameras
		command_flags.toggle_tracking_video_state = true;
		tracking_camera_off = true;
		tracking_camera_on = false;
		tracking_camera_pic = false;
		_last_send = AP_HAL::millis();
		return;



	}else if(command_flags.toggle_tracking_video_state){


		static cmd_48_byte_struct cmd_toggle_tracking_video_state;

		cmd_toggle_tracking_video_state.byte1 = 0x7E;
		cmd_toggle_tracking_video_state.byte2 = 0x7E;
		cmd_toggle_tracking_video_state.byte3 = 0x44;
		cmd_toggle_tracking_video_state.byte4 = 0x00;
		cmd_toggle_tracking_video_state.byte5 = 0x00;
		cmd_toggle_tracking_video_state.byte6 = 0x7C;
		cmd_toggle_tracking_video_state.byte7 = 0x00;
		cmd_toggle_tracking_video_state.byte8 = 0x00;
		cmd_toggle_tracking_video_state.byte9 = 0x00;
		cmd_toggle_tracking_video_state.byte10 = 0x00;
		cmd_toggle_tracking_video_state.byte11 = 0x00;
		cmd_toggle_tracking_video_state.byte12 = 0x00;
		cmd_toggle_tracking_video_state.byte13 = 0x00;
		cmd_toggle_tracking_video_state.byte14 = 0x00;
		cmd_toggle_tracking_video_state.byte15 = 0x00;
		cmd_toggle_tracking_video_state.byte16 = 0x00;
		cmd_toggle_tracking_video_state.byte17 = 0x00;
		cmd_toggle_tracking_video_state.byte18 = 0x00;
		cmd_toggle_tracking_video_state.byte19 = 0x00;
		cmd_toggle_tracking_video_state.byte20 = 0x00;
		cmd_toggle_tracking_video_state.byte21 = 0x00;
		cmd_toggle_tracking_video_state.byte22 = 0x00;
		cmd_toggle_tracking_video_state.byte23 = 0x00;
		cmd_toggle_tracking_video_state.byte24 = 0x00;
		cmd_toggle_tracking_video_state.byte25 = 0x00;
		cmd_toggle_tracking_video_state.byte26 = 0x00;
		cmd_toggle_tracking_video_state.byte27 = 0x00;
		cmd_toggle_tracking_video_state.byte28 = 0x00;
		cmd_toggle_tracking_video_state.byte29 = 0x00;
		cmd_toggle_tracking_video_state.byte30 = 0x00;
		cmd_toggle_tracking_video_state.byte31 = 0x00;
		cmd_toggle_tracking_video_state.byte32 = 0x00;
		cmd_toggle_tracking_video_state.byte33 = 0x00;
		cmd_toggle_tracking_video_state.byte34 = 0x00;
		cmd_toggle_tracking_video_state.byte35 = 0x00;
		cmd_toggle_tracking_video_state.byte36 = 0x00;
		cmd_toggle_tracking_video_state.byte37 = 0x00;
		cmd_toggle_tracking_video_state.byte38 = 0x00;
		cmd_toggle_tracking_video_state.byte39 = 0x00;
		cmd_toggle_tracking_video_state.byte40 = 0x00;
		cmd_toggle_tracking_video_state.byte41 = 0x00;
		cmd_toggle_tracking_video_state.byte42 = 0x00;
		cmd_toggle_tracking_video_state.byte43 = 0x00;
		cmd_toggle_tracking_video_state.byte44 = 0x00;
		cmd_toggle_tracking_video_state.byte45 = 0x00;
		cmd_toggle_tracking_video_state.byte46 = 0x00;
		cmd_toggle_tracking_video_state.byte47 = 0x00;
		cmd_toggle_tracking_video_state.byte48 = 0x00;

		if(tracking_camera_off){
			cmd_toggle_tracking_video_state.byte7 = 0x00;
			tracking_camera_off = false;
			current_rec_state = REC_OFF;
		}else if(tracking_camera_on){
			cmd_toggle_tracking_video_state.byte7 = 0x01;
			cmd_toggle_tracking_video_state.byte8 = 0x58;
			cmd_toggle_tracking_video_state.byte9 = 0x02;
			tracking_camera_on = false;
			current_rec_state = REC_ON;
		}else if(tracking_camera_pic){
			cmd_toggle_tracking_video_state.byte7 = 0x02;
			tracking_camera_pic = false;
		}

		//Setup buffer and send
		uint8_t* buf_take_picture = (uint8_t*)&cmd_toggle_tracking_video_state;
		uint16_t sum_take_picture = 0;

		for (uint8_t i = 0;  i < 47 ; i++) {
			sum_take_picture += buf_take_picture[i];
		}

		cmd_toggle_tracking_video_state.byte48 = (uint8_t)(sum_take_picture % 256);

		if ((size_t)_port->txspace() < sizeof(cmd_toggle_tracking_video_state)) {
			return;
		}

		for (uint8_t i = 0;  i != sizeof(cmd_toggle_tracking_video_state) ; i++) {
			_port->write(buf_take_picture[i]);
		}

		_last_send = AP_HAL::millis();
		command_flags.toggle_tracking_video_state = false;

		return;


	}else if(command_flags.center_yaw){

		static	cmd_6_byte_struct center_yaw;

		center_yaw.byte1 = 0x3E;
		center_yaw.byte2 = 0x45;
		center_yaw.byte3 = 0x01;
		center_yaw.byte4 = 0x46;
		center_yaw.byte5 = 0x23;
		center_yaw.byte6 = 0x23;

		buf_size = sizeof(center_yaw);
		buf_cmd = (uint8_t*)&center_yaw;

		if ((size_t)_port->txspace() < buf_size) {
			return;
		}

	    for (uint8_t i = 0;  i != buf_size ; i++) {
	        _port->write(buf_cmd[i]);
	    }

	    command_flags.center_yaw = false;
	    _last_send = AP_HAL::millis();
	    return;

	}else if(command_flags.toggle_video){

		static cmd_6_byte_struct cmd_toggle_video_on_off;

		cmd_toggle_video_on_off.byte1 = 0x81;
		cmd_toggle_video_on_off.byte2 = 0x01;
		cmd_toggle_video_on_off.byte3 = 0x04;
		cmd_toggle_video_on_off.byte4 = 0x68;
		cmd_toggle_video_on_off.byte5 = 0x04;
		cmd_toggle_video_on_off.byte6 = 0xFF;

		buf_size = sizeof(cmd_toggle_video_on_off);
		buf_cmd = (uint8_t*)&cmd_toggle_video_on_off;

		if ((size_t)_port->txspace() < buf_size) {
			return;
		}

	    for (uint8_t i = 0;  i != buf_size ; i++) {
	        _port->write(buf_cmd[i]);
	    }

	    command_flags.toggle_video = false;

	   command_flags.toggle_video_tracking = true;
	    _last_send = AP_HAL::millis();
	    return;

	} else if(command_flags.toggle_video_tracking){


		static cmd_48_byte_struct cmd_toggle_tracking_video_on_off;

			cmd_toggle_tracking_video_on_off.byte1 = 0x7E;
			cmd_toggle_tracking_video_on_off.byte2 = 0x7E;
			cmd_toggle_tracking_video_on_off.byte3 = 0x44;
			cmd_toggle_tracking_video_on_off.byte4 = 0x00;
			cmd_toggle_tracking_video_on_off.byte5 = 0x00;
			cmd_toggle_tracking_video_on_off.byte6 = 0x7C;
			cmd_toggle_tracking_video_on_off.byte7 = 0x02;
			cmd_toggle_tracking_video_on_off.byte8 = 0x00;
			cmd_toggle_tracking_video_on_off.byte9 = 0x00;
			cmd_toggle_tracking_video_on_off.byte10 = 0x00;
			cmd_toggle_tracking_video_on_off.byte11 = 0x00;
			cmd_toggle_tracking_video_on_off.byte12 = 0x00;
			cmd_toggle_tracking_video_on_off.byte13 = 0x00;
			cmd_toggle_tracking_video_on_off.byte14 = 0x00;
			cmd_toggle_tracking_video_on_off.byte15 = 0x00;
			cmd_toggle_tracking_video_on_off.byte16 = 0x00;
			cmd_toggle_tracking_video_on_off.byte17 = 0x00;
			cmd_toggle_tracking_video_on_off.byte18 = 0x00;
			cmd_toggle_tracking_video_on_off.byte19 = 0x00;
			cmd_toggle_tracking_video_on_off.byte20 = 0x00;
			cmd_toggle_tracking_video_on_off.byte21 = 0x00;
			cmd_toggle_tracking_video_on_off.byte22 = 0x00;
			cmd_toggle_tracking_video_on_off.byte23 = 0x00;
			cmd_toggle_tracking_video_on_off.byte24 = 0x00;
			cmd_toggle_tracking_video_on_off.byte25 = 0x00;
			cmd_toggle_tracking_video_on_off.byte26 = 0x00;
			cmd_toggle_tracking_video_on_off.byte27 = 0x00;
			cmd_toggle_tracking_video_on_off.byte28 = 0x00;
			cmd_toggle_tracking_video_on_off.byte29 = 0x00;
			cmd_toggle_tracking_video_on_off.byte30 = 0x00;
			cmd_toggle_tracking_video_on_off.byte31 = 0x00;
			cmd_toggle_tracking_video_on_off.byte32 = 0x00;
			cmd_toggle_tracking_video_on_off.byte33 = 0x00;
			cmd_toggle_tracking_video_on_off.byte34 = 0x00;
			cmd_toggle_tracking_video_on_off.byte35 = 0x00;
			cmd_toggle_tracking_video_on_off.byte36 = 0x00;
			cmd_toggle_tracking_video_on_off.byte37 = 0x00;
			cmd_toggle_tracking_video_on_off.byte38 = 0x00;
			cmd_toggle_tracking_video_on_off.byte39 = 0x00;
			cmd_toggle_tracking_video_on_off.byte40 = 0x00;
			cmd_toggle_tracking_video_on_off.byte41 = 0x00;
			cmd_toggle_tracking_video_on_off.byte42 = 0x00;
			cmd_toggle_tracking_video_on_off.byte43 = 0x00;
			cmd_toggle_tracking_video_on_off.byte44 = 0x00;
			cmd_toggle_tracking_video_on_off.byte45 = 0x00;
			cmd_toggle_tracking_video_on_off.byte46 = 0x00;
			cmd_toggle_tracking_video_on_off.byte47 = 0x00;
			cmd_toggle_tracking_video_on_off.byte48 = 0x00;
/*

			if(is_recording){
				cmd_toggle_tracking_video_on_off.byte7 = 0x05;
				is_recording = false;
			}else{
				cmd_toggle_tracking_video_on_off.byte7 = 0x04;
				is_recording = true;
			}
*/

			uint8_t* buf_tracking_video_control = (uint8_t*)&cmd_toggle_tracking_video_on_off;
			uint16_t sum_tracking_video_control = 0;

			for (uint8_t i = 0;  i < 47 ; i++) {
				sum_tracking_video_control += buf_tracking_video_control[i];
			}

			cmd_toggle_tracking_video_on_off.byte48 = (uint8_t)(sum_tracking_video_control % 256);

			if ((size_t)_port->txspace() < sizeof(cmd_toggle_tracking_video_on_off)) {
				return;
			}

			for (uint8_t i = 0;  i != sizeof(cmd_toggle_tracking_video_on_off) ; i++) {
				_port->write(buf_tracking_video_control[i]);
			}

		command_flags.toggle_video_tracking = false;
		_last_send = AP_HAL::millis();
		return;

	}else if(command_flags.toggle_tracking){


		static cmd_48_byte_struct cmd_toggle_tracking_on_off;

		cmd_toggle_tracking_on_off.byte1 = 0x7E;
		cmd_toggle_tracking_on_off.byte2 = 0x7E;
		cmd_toggle_tracking_on_off.byte3 = 0x44;
		cmd_toggle_tracking_on_off.byte4 = 0x00;
		cmd_toggle_tracking_on_off.byte5 = 0x00;
		cmd_toggle_tracking_on_off.byte6 = 0x00;
		cmd_toggle_tracking_on_off.byte7 = 0x00;
		cmd_toggle_tracking_on_off.byte8 = 0x00;
		cmd_toggle_tracking_on_off.byte9 = 0x00;
		cmd_toggle_tracking_on_off.byte10 = 0x00;
		cmd_toggle_tracking_on_off.byte11 = 0x00;
		cmd_toggle_tracking_on_off.byte12 = 0x00;
		cmd_toggle_tracking_on_off.byte13 = 0x00;
		cmd_toggle_tracking_on_off.byte14 = 0x00;
		cmd_toggle_tracking_on_off.byte15 = 0x00;
		cmd_toggle_tracking_on_off.byte16 = 0x00;
		cmd_toggle_tracking_on_off.byte17 = 0x00;
		cmd_toggle_tracking_on_off.byte18 = 0x00;
		cmd_toggle_tracking_on_off.byte19 = 0x00;
		cmd_toggle_tracking_on_off.byte20 = 0x00;
		cmd_toggle_tracking_on_off.byte21 = 0x00;
		cmd_toggle_tracking_on_off.byte22 = 0x00;
		cmd_toggle_tracking_on_off.byte23 = 0x00;
		cmd_toggle_tracking_on_off.byte24 = 0x00;
		cmd_toggle_tracking_on_off.byte25 = 0x00;
		cmd_toggle_tracking_on_off.byte26 = 0x00;
		cmd_toggle_tracking_on_off.byte27 = 0x00;
		cmd_toggle_tracking_on_off.byte28 = 0x00;
		cmd_toggle_tracking_on_off.byte29 = 0x00;
		cmd_toggle_tracking_on_off.byte30 = 0x00;
		cmd_toggle_tracking_on_off.byte31 = 0x00;
		cmd_toggle_tracking_on_off.byte32 = 0x00;
		cmd_toggle_tracking_on_off.byte33 = 0x00;
		cmd_toggle_tracking_on_off.byte34 = 0x00;
		cmd_toggle_tracking_on_off.byte35 = 0x00;
		cmd_toggle_tracking_on_off.byte36 = 0x00;
		cmd_toggle_tracking_on_off.byte37 = 0x00;
		cmd_toggle_tracking_on_off.byte38 = 0x00;
		cmd_toggle_tracking_on_off.byte39 = 0x00;
		cmd_toggle_tracking_on_off.byte40 = 0x00;
		cmd_toggle_tracking_on_off.byte41 = 0x00;
		cmd_toggle_tracking_on_off.byte42 = 0x00;
		cmd_toggle_tracking_on_off.byte43 = 0x00;
		cmd_toggle_tracking_on_off.byte44 = 0x00;
		cmd_toggle_tracking_on_off.byte45 = 0x00;
		cmd_toggle_tracking_on_off.byte46 = 0x00;
		cmd_toggle_tracking_on_off.byte47 = 0x00;
		cmd_toggle_tracking_on_off.byte48 = 0x00;


		if(is_tracking){
			cmd_toggle_tracking_on_off.byte6 = 0x71;
			cmd_toggle_tracking_on_off.byte12 = 0x01;
			cmd_toggle_tracking_on_off.byte14 = 0x38;
			is_tracking = false;
		}else{
			cmd_toggle_tracking_on_off.byte6 = 0x00;
			cmd_toggle_tracking_on_off.byte12 = 0x00;
			cmd_toggle_tracking_on_off.byte14 = 0x00;
			is_tracking = true;
		}


		uint8_t* buf_tracking_control = (uint8_t*)&cmd_toggle_tracking_on_off;
		uint16_t sum_tracking_control = 0;

		for (uint8_t i = 0;  i < 47 ; i++) {
			sum_tracking_control += buf_tracking_control[i];
		}

		cmd_toggle_tracking_on_off.byte48 = (uint8_t)(sum_tracking_control % 256);

		if ((size_t)_port->txspace() < sizeof(cmd_toggle_tracking_on_off)) {
			return;
		}

		for (uint8_t i = 0;  i != sizeof(cmd_toggle_tracking_on_off) ; i++) {
			_port->write(buf_tracking_control[i]);
		}

			command_flags.toggle_tracking = false;
			_last_send = AP_HAL::millis();
			return;

	}else if(command_flags.toggle_pip){


		static cmd_48_byte_struct cmd_toggle_pip;

		cmd_toggle_pip.byte1 = 0x7E;
		cmd_toggle_pip.byte2 = 0x7E;
		cmd_toggle_pip.byte3 = 0x44;
		cmd_toggle_pip.byte4 = 0x00;
		cmd_toggle_pip.byte5 = 0x00;
		cmd_toggle_pip.byte6 = 0x78;
		cmd_toggle_pip.byte7 = 0x00;
		cmd_toggle_pip.byte8 = 0x00;
		cmd_toggle_pip.byte9 = 0x00;
		cmd_toggle_pip.byte10 = 0x00;
		cmd_toggle_pip.byte11 = 0x00;
		cmd_toggle_pip.byte12 = 0x00;
		cmd_toggle_pip.byte13 = 0x00;
		cmd_toggle_pip.byte14 = 0x00;
		cmd_toggle_pip.byte15 = 0x00;
		cmd_toggle_pip.byte16 = 0x00;
		cmd_toggle_pip.byte17 = 0x00;
		cmd_toggle_pip.byte18 = 0x00;
		cmd_toggle_pip.byte19 = 0x00;
		cmd_toggle_pip.byte20 = 0x00;
		cmd_toggle_pip.byte21 = 0x00;
		cmd_toggle_pip.byte22 = 0x00;
		cmd_toggle_pip.byte23 = 0x00;
		cmd_toggle_pip.byte24 = 0x00;
		cmd_toggle_pip.byte25 = 0x00;
		cmd_toggle_pip.byte26 = 0x00;
		cmd_toggle_pip.byte27 = 0x00;
		cmd_toggle_pip.byte28 = 0x00;
		cmd_toggle_pip.byte29 = 0x00;
		cmd_toggle_pip.byte30 = 0x00;
		cmd_toggle_pip.byte31 = 0x00;
		cmd_toggle_pip.byte32 = 0x00;
		cmd_toggle_pip.byte33 = 0x00;
		cmd_toggle_pip.byte34 = 0x00;
		cmd_toggle_pip.byte35 = 0x00;
		cmd_toggle_pip.byte36 = 0x00;
		cmd_toggle_pip.byte37 = 0x00;
		cmd_toggle_pip.byte38 = 0x00;
		cmd_toggle_pip.byte39 = 0x00;
		cmd_toggle_pip.byte40 = 0x00;
		cmd_toggle_pip.byte41 = 0x00;
		cmd_toggle_pip.byte42 = 0x00;
		cmd_toggle_pip.byte43 = 0x00;
		cmd_toggle_pip.byte44 = 0x00;
		cmd_toggle_pip.byte45 = 0x00;
		cmd_toggle_pip.byte46 = 0x00;
		cmd_toggle_pip.byte47 = 0x00;
		cmd_toggle_pip.byte48 = 0x00;



		pip_state++;

		if(pip_state >= 4 ){
			pip_state = 0;
		}


				if(pip_state == 0){
					// EO+IR(pip)

				}else if(pip_state == 1){
					//IR
					cmd_toggle_pip.byte15 = 0x01;

				}else if(pip_state == 2){
					//IR+EO(pip)
					cmd_toggle_pip.byte15 = 0x02;

				}else{
					//EO
					cmd_toggle_pip.byte15 = 0x03;
				}



				if(color_state == 0){
					// Gray

				}else if(color_state == 1){
					//Color 1
					cmd_toggle_pip.byte7 = 0x01;

				}else if(color_state == 2){
					//Color 2
					cmd_toggle_pip.byte7 = 0x02;

				}else if(color_state == 3){
					//Color 3
					cmd_toggle_pip.byte7 = 0x03;

				}else{
					cmd_toggle_pip.byte7 = 0x04;
				}



				uint8_t* buf_pip_control = (uint8_t*)&cmd_toggle_pip;
				uint16_t sum_pip_control = 0;

				for (uint8_t i = 0;  i < 47 ; i++) {
					sum_pip_control += buf_pip_control[i];
				}

				cmd_toggle_pip.byte48 = (uint8_t)(sum_pip_control % 256);

				if ((size_t)_port->txspace() < sizeof(cmd_toggle_pip)) {
					return;
				}

				for (uint8_t i = 0;  i != sizeof(cmd_toggle_pip) ; i++) {
					_port->write(buf_pip_control[i]);
				}




		command_flags.toggle_pip = false;
		_last_send = AP_HAL::millis();
		return;

	}else if(command_flags.color_change){


		static cmd_48_byte_struct cmd_toggle_color;

				cmd_toggle_color.byte1 = 0x7E;
				cmd_toggle_color.byte2 = 0x7E;
				cmd_toggle_color.byte3 = 0x44;
				cmd_toggle_color.byte4 = 0x00;
				cmd_toggle_color.byte5 = 0x00;
				cmd_toggle_color.byte6 = 0x78;
				cmd_toggle_color.byte7 = 0x00;
				cmd_toggle_color.byte8 = 0x00;
				cmd_toggle_color.byte9 = 0x00;
				cmd_toggle_color.byte10 = 0x00;
				cmd_toggle_color.byte11 = 0x00;
				cmd_toggle_color.byte12 = 0x00;
				cmd_toggle_color.byte13 = 0x00;
				cmd_toggle_color.byte14 = 0x00;
				cmd_toggle_color.byte15 = 0x00;
				cmd_toggle_color.byte16 = 0x00;
				cmd_toggle_color.byte17 = 0x00;
				cmd_toggle_color.byte18 = 0x00;
				cmd_toggle_color.byte19 = 0x00;
				cmd_toggle_color.byte20 = 0x00;
				cmd_toggle_color.byte21 = 0x00;
				cmd_toggle_color.byte22 = 0x00;
				cmd_toggle_color.byte23 = 0x00;
				cmd_toggle_color.byte24 = 0x00;
				cmd_toggle_color.byte25 = 0x00;
				cmd_toggle_color.byte26 = 0x00;
				cmd_toggle_color.byte27 = 0x00;
				cmd_toggle_color.byte28 = 0x00;
				cmd_toggle_color.byte29 = 0x00;
				cmd_toggle_color.byte30 = 0x00;
				cmd_toggle_color.byte31 = 0x00;
				cmd_toggle_color.byte32 = 0x00;
				cmd_toggle_color.byte33 = 0x00;
				cmd_toggle_color.byte34 = 0x00;
				cmd_toggle_color.byte35 = 0x00;
				cmd_toggle_color.byte36 = 0x00;
				cmd_toggle_color.byte37 = 0x00;
				cmd_toggle_color.byte38 = 0x00;
				cmd_toggle_color.byte39 = 0x00;
				cmd_toggle_color.byte40 = 0x00;
				cmd_toggle_color.byte41 = 0x00;
				cmd_toggle_color.byte42 = 0x00;
				cmd_toggle_color.byte43 = 0x00;
				cmd_toggle_color.byte44 = 0x00;
				cmd_toggle_color.byte45 = 0x00;
				cmd_toggle_color.byte46 = 0x00;
				cmd_toggle_color.byte47 = 0x00;
				cmd_toggle_color.byte48 = 0x00;


				color_state++;

				if(color_state >= 5 ){
					color_state = 0;
				}


						if(color_state == 0){
							// Gray

						}else if(color_state == 1){
							//Color 1
							cmd_toggle_color.byte7 = 0x01;

						}else if(color_state == 2){
							//Color 2
							cmd_toggle_color.byte7 = 0x02;

						}else if(color_state == 3){
							//Color 3
							cmd_toggle_color.byte7 = 0x03;

						}else{
							cmd_toggle_color.byte7 = 0x04;
						}


						if(pip_state == 0){
							// EO+IR(pip)

						}else if(pip_state == 1){
							//IR
							cmd_toggle_color.byte15 = 0x01;

						}else if(pip_state == 2){
							//IR+EO(pip)
							cmd_toggle_color.byte15 = 0x02;

						}else{
							//EO
							cmd_toggle_color.byte15 = 0x03;
						}



						uint8_t* buf_color_control = (uint8_t*)&cmd_toggle_color;
						uint16_t sum_color_control = 0;

						for (uint8_t i = 0;  i < 47 ; i++) {
							sum_color_control += buf_color_control[i];
						}

						cmd_toggle_color.byte48 = (uint8_t)(sum_color_control % 256);

						if ((size_t)_port->txspace() < sizeof(cmd_toggle_color)) {
							return;
						}

						for (uint8_t i = 0;  i != sizeof(cmd_toggle_color) ; i++) {
							_port->write(buf_color_control[i]);
						}




				command_flags.color_change = false;

				_last_send = AP_HAL::millis();
				return;




	}else if(order_flip){

		static cmd_48_byte_struct cmd_flip_image;

		cmd_flip_image.byte1 = 0x7E;
		cmd_flip_image.byte2 = 0x7E;
		cmd_flip_image.byte3 = 0x44;
		cmd_flip_image.byte4 = 0x00;
		cmd_flip_image.byte5 = 0x00;
		cmd_flip_image.byte6 = 0x91;
		cmd_flip_image.byte7 = 0x00;
		cmd_flip_image.byte8 = 0x00;
		cmd_flip_image.byte9 = 0x00;
		cmd_flip_image.byte10 = 0x00;
		cmd_flip_image.byte11 = 0x00;
		cmd_flip_image.byte12 = 0x00;
		cmd_flip_image.byte13 = 0x00;
		cmd_flip_image.byte14 = 0x00;
		cmd_flip_image.byte15 = 0x00;
		cmd_flip_image.byte16 = 0x00;
		cmd_flip_image.byte17 = 0x00;
		cmd_flip_image.byte18 = 0x00;
		cmd_flip_image.byte19 = 0x00;
		cmd_flip_image.byte20 = 0x00;
		cmd_flip_image.byte21 = 0x00;
		cmd_flip_image.byte22 = 0x00;
		cmd_flip_image.byte23 = 0x00;
		cmd_flip_image.byte24 = 0x00;
		cmd_flip_image.byte25 = 0x00;
		cmd_flip_image.byte26 = 0x00;
		cmd_flip_image.byte27 = 0x00;
		cmd_flip_image.byte28 = 0x00;
		cmd_flip_image.byte29 = 0x00;
		cmd_flip_image.byte30 = 0x00;
		cmd_flip_image.byte31 = 0x00;
		cmd_flip_image.byte32 = 0x00;
		cmd_flip_image.byte33 = 0x00;
		cmd_flip_image.byte34 = 0x00;
		cmd_flip_image.byte35 = 0x00;
		cmd_flip_image.byte36 = 0x00;
		cmd_flip_image.byte37 = 0x00;
		cmd_flip_image.byte38 = 0x00;
		cmd_flip_image.byte39 = 0x00;
		cmd_flip_image.byte40 = 0x00;
		cmd_flip_image.byte41 = 0x00;
		cmd_flip_image.byte42 = 0x00;
		cmd_flip_image.byte43 = 0x00;
		cmd_flip_image.byte44 = 0x00;
		cmd_flip_image.byte45 = 0x00;
		cmd_flip_image.byte46 = 0x00;
		cmd_flip_image.byte47 = 0x00;
		cmd_flip_image.byte48 = 0x00;


		if(command_flags.flip_image){

			cmd_flip_image.byte7 = 0xC0;
			cmd_flip_image.byte48 = 0x91;

		}else{


			cmd_flip_image.byte7 = 0x80;
			cmd_flip_image.byte48 = 0x51;

		}


		uint8_t* buf_image_flip = (uint8_t*)&cmd_flip_image;


		if ((size_t)_port->txspace() < sizeof(cmd_flip_image)) {
			return;
		}

		for (uint8_t i = 0;  i != sizeof(cmd_flip_image) ; i++) {
			_port->write(buf_image_flip[i]);
		}


		static	cmd_6_byte_struct EO_image_flip;

		EO_image_flip.byte1 = 0x81;
		EO_image_flip.byte2 = 0x01;
		EO_image_flip.byte3 = 0x04;
		EO_image_flip.byte4 = 0x66;
		EO_image_flip.byte5 = 0x00;
		EO_image_flip.byte6 = 0xFF;


		if(command_flags.flip_image){

			EO_image_flip.byte5 = 0x02;

		}else{


			EO_image_flip.byte5 = 0x03;

		}


		uint8_t* buf_image_flip_EO = (uint8_t*)&EO_image_flip;


		if ((size_t)_port->txspace() < sizeof(EO_image_flip)) {
			return;
		}

		for (uint8_t i = 0;  i != sizeof(EO_image_flip) ; i++) {
			_port->write(buf_image_flip_EO[i]);
		}


		order_flip = false;


	}else if(_zooming_state_change){

		static cmd_6_byte_struct cmd_set_zoom_data;

		cmd_set_zoom_data.byte1 = 0x81;
		cmd_set_zoom_data.byte2 = 0x01;
		cmd_set_zoom_data.byte3 = 0x04;
		cmd_set_zoom_data.byte4 = 0x07;
		cmd_set_zoom_data.byte6 = 0xFF;

		if(current_zoom_state == ZOOM_IN){
			cmd_set_zoom_data.byte5 = 0x27;
		}else if(current_zoom_state == ZOOM_OUT){
			cmd_set_zoom_data.byte5 = 0x37;
		}else{
			cmd_set_zoom_data.byte5 = 0x00;
		}


	    if ((size_t)_port->txspace() <= sizeof(cmd_set_zoom_data)) {
	        return;
	    }

	    uint8_t* buf_zoom = (uint8_t*)&cmd_set_zoom_data;

	  	    for (uint8_t i = 0;  i != sizeof(cmd_set_zoom_data) ; i++) {
	  	        _port->write(buf_zoom[i]);
	  	    }

	  	  _zooming_state_change = false;
	      command_flags.IR_zoom = true;
	  	  _last_send = AP_HAL::millis();

	  	  return;

	}else if(command_flags.IR_zoom){


		static cmd_48_byte_struct cmd_IR_zoom;

		cmd_IR_zoom.byte1 = 0x7E;
		cmd_IR_zoom.byte2 = 0x7E;
		cmd_IR_zoom.byte3 = 0x44;
		cmd_IR_zoom.byte4 = 0x00;
		cmd_IR_zoom.byte5 = 0x00;
		cmd_IR_zoom.byte6 = 0x7D;
		cmd_IR_zoom.byte7 = 0x83;
		cmd_IR_zoom.byte8 = 0x00;
		cmd_IR_zoom.byte9 = 0x00;
		cmd_IR_zoom.byte10 = 0x00;
		cmd_IR_zoom.byte11 = 0x00;
		cmd_IR_zoom.byte12 = 0x00;
		cmd_IR_zoom.byte13 = 0x00;
		cmd_IR_zoom.byte14 = 0x00;
		cmd_IR_zoom.byte15 = 0x00;
		cmd_IR_zoom.byte16 = 0x00;
		cmd_IR_zoom.byte17 = 0x00;
		cmd_IR_zoom.byte18 = 0x00;
		cmd_IR_zoom.byte19 = 0x00;
		cmd_IR_zoom.byte20 = 0x00;
		cmd_IR_zoom.byte21 = 0x00;
		cmd_IR_zoom.byte22 = 0x00;
		cmd_IR_zoom.byte23 = 0x00;
		cmd_IR_zoom.byte24 = 0x00;
		cmd_IR_zoom.byte25 = 0x00;
		cmd_IR_zoom.byte26 = 0x00;
		cmd_IR_zoom.byte27 = 0x00;
		cmd_IR_zoom.byte28 = 0x00;
		cmd_IR_zoom.byte29 = 0x00;
		cmd_IR_zoom.byte30 = 0x00;
		cmd_IR_zoom.byte31 = 0x00;
		cmd_IR_zoom.byte32 = 0x00;
		cmd_IR_zoom.byte33 = 0x00;
			cmd_IR_zoom.byte34 = 0x00;
			cmd_IR_zoom.byte35 = 0x00;
			cmd_IR_zoom.byte36 = 0x00;
			cmd_IR_zoom.byte37 = 0x00;
			cmd_IR_zoom.byte38 = 0x00;
			cmd_IR_zoom.byte39 = 0x00;
			cmd_IR_zoom.byte40 = 0x00;
			cmd_IR_zoom.byte41 = 0x00;
			cmd_IR_zoom.byte42 = 0x00;
			cmd_IR_zoom.byte43 = 0x00;
			cmd_IR_zoom.byte44 = 0x00;
			cmd_IR_zoom.byte45 = 0x00;
			cmd_IR_zoom.byte46 = 0x00;
			cmd_IR_zoom.byte47 = 0x00;
			cmd_IR_zoom.byte48 = 0x00;


			if(current_zoom_state == ZOOM_IN){

				if(ir_zoom_level == 1){

					cmd_IR_zoom.byte7 = 0x82;
					ir_zoom_level++;

				}else if(ir_zoom_level == 2){

					cmd_IR_zoom.byte7 = 0x83;
					ir_zoom_level++;

				}else if(ir_zoom_level == 3){

					cmd_IR_zoom.byte7 = 0x84;
					ir_zoom_level++;

				}else{

					cmd_IR_zoom.byte7 = 0x84;

				}

			}else if(current_zoom_state == ZOOM_OUT){

				if(ir_zoom_level == 4){

					cmd_IR_zoom.byte7 = 0x83;
					ir_zoom_level--;

				}else if(ir_zoom_level == 3){

					cmd_IR_zoom.byte7 = 0x82;
					ir_zoom_level--;

				}else if(ir_zoom_level == 2){

					cmd_IR_zoom.byte7 = 0x81;
					ir_zoom_level--;

				}else{

					cmd_IR_zoom.byte7 = 0x81;

				}

			}else{

				command_flags.IR_zoom = false;
				return;
			}



			uint16_t ir_sum = 0;
			uint8_t* buf_IR_zoom = (uint8_t*)&cmd_IR_zoom;


			for (uint8_t i = 0;  i < 47 ; i++) {
				ir_sum	+= buf_IR_zoom[i];
			}

			cmd_IR_zoom.byte48 = (uint8_t)(ir_sum % 256);


			if ((size_t)_port->txspace() < sizeof(cmd_IR_zoom)) {
				return;
			}

			for (uint8_t i = 0;  i != sizeof(cmd_IR_zoom) ; i++) {
				_port->write(buf_IR_zoom[i]);
			}


		command_flags.IR_zoom = false;
		return;

	}


	////////////////////////////////////////
	/// Send Gimbal Control Commands ///////
	////////////////////////////////////////

	float _pitch_value = 0;
	float _yaw_value = 0;

	static cmd_set_angles_struct cmd_set_data;

	 switch(get_mode()) {

		case MAV_MOUNT_MODE_RETRACT:
		case MAV_MOUNT_MODE_NEUTRAL:


			cmd_set_data.header1 = 0xFF;
			cmd_set_data.header2 = 0x01;
			cmd_set_data.header3 = 0x0F;
			cmd_set_data.header4 = 0x10;
			cmd_set_data.RM = 0x00;
			cmd_set_data.PM = 0x00;
			cmd_set_data.YM = 0x00;
			cmd_set_data.Rs = (int)0;
			cmd_set_data.Ra = (int)0;
			cmd_set_data.Ps = (int)0;
			cmd_set_data.Pa = (int)0;
			cmd_set_data.Ys = (int)0;
			cmd_set_data.Ya = (int)0;
			cmd_set_data.crc = 0x00;

			break;

		case MAV_MOUNT_MODE_MAVLINK_TARGETING:
		case MAV_MOUNT_MODE_GPS_POINT:

			_pitch_value = _angle_ef_target_deg.y / 0.021972;
			_yaw_value = _angle_ef_target_deg.z / 0.021972;

			cmd_set_data.header1 = 0xFF;
			cmd_set_data.header2 = 0x01;
			cmd_set_data.header3 = 0x0F;
			cmd_set_data.header4 = 0x10;
			cmd_set_data.RM = 0x00;
			cmd_set_data.PM = 0x02;
			cmd_set_data.YM = 0x05;
			cmd_set_data.Rs = (int)0;
			cmd_set_data.Ra = (int)0;
			cmd_set_data.Ps = (int)0;
			cmd_set_data.Pa = (int)_pitch_value;
			cmd_set_data.Ys = (int)0;
			cmd_set_data.Ya = (int)_yaw_value;
			cmd_set_data.crc = 0;

			break;

		case MAV_MOUNT_MODE_RC_TARGETING:

			_pitch_value = _speed_ef_target_deg.y / 0.12207;
			_yaw_value = _speed_ef_target_deg.z / 0.12207;

		    cmd_set_data.header1 = 0xFF;
		    cmd_set_data.header2 = 0x01;
		    cmd_set_data.header3 = 0x0F;
		    cmd_set_data.header4 = 0x10;
		    cmd_set_data.RM = 0x00;
		    cmd_set_data.PM = 0x01;
		    cmd_set_data.YM = 0x01;
		    cmd_set_data.Rs = (int)0;
		    cmd_set_data.Ra = (int)0;
		    cmd_set_data.Ps = (int)_pitch_value;
		    cmd_set_data.Pa = (int)0;
		    cmd_set_data.Ys = (int)_yaw_value;
		    cmd_set_data.Ya = (int)0;
		    cmd_set_data.crc = 0;


		    if(is_zero(_pitch_value) and  !_RC_control_enable){
			    cmd_set_data.PM = 0x00;
			    cmd_set_data.YM = 0x00;
			    cmd_set_data.Rs = (int)0;
			    cmd_set_data.Ra = (int)0;
			    cmd_set_data.Ps = (int)0;
			    cmd_set_data.Pa = (int)0;
			    cmd_set_data.Ys = (int)0;
			    cmd_set_data.Ya = (int)0;
			    cmd_set_data.crc = 0x00;

		    }


			break;

        default:
            // we do not know this mode so do nothing
            break;

	 }


	uint16_t sum = 0;
	uint8_t* buf_gimbal_control = (uint8_t*)&cmd_set_data;


	for (uint8_t i = 4;  i < 19 ; i++) {
		sum	+= buf_gimbal_control[i];
	}

	cmd_set_data.crc = (uint8_t)(sum % 256);

	if ((size_t)_port->txspace() < sizeof(cmd_set_data)) {
		return;
	}

	for (uint8_t i = 0;  i != sizeof(cmd_set_data) ; i++) {
		_port->write(buf_gimbal_control[i]);
	}



	////////////////////////////////////////
	////////Send any open queries///////////
	////////////////////////////////////////

	uint8_t* buf_query;

	if(false){

		//Need Code

	}else if(false){

		//Need Code

	}else if(false){


		//Need Code

	}else if(get_mode() == MAV_MOUNT_MODE_RC_TARGETING){

		//if(( is_zero(_pitch_value) and is_zero(_yaw_value) and current_zoom_state == ZOOM_STOP)){

		if(current_zoom_state == ZOOM_STOP){

			static cmd_5_byte_struct query_angles;

		    query_angles.byte1 = 0x3E;
		    query_angles.byte2 = 0x3D;
		    query_angles.byte3 = 0x00;
		    query_angles.byte4 = 0x3D;
		    query_angles.byte5 = 0x00;

		    buf_size = sizeof(query_angles);
		    buf_query = (uint8_t*)&query_angles;

		    _reply_type =  ReplyType_angle_DATA;
	        _reply_counter = 0;
	        _reply_length = 59;

		}else{

			static cmd_5_byte_struct cmd_ask_zoom_data;

		    cmd_ask_zoom_data.byte1 = 0x81;
		    cmd_ask_zoom_data.byte2 = 0x09;
		    cmd_ask_zoom_data.byte3 = 0x04;
		    cmd_ask_zoom_data.byte4 = 0x47;
		    cmd_ask_zoom_data.byte5 = 0xFF;

		    buf_size = sizeof(cmd_ask_zoom_data);
		    buf_query = (uint8_t*)&cmd_ask_zoom_data;

		    _reply_type =  ReplyType_Zoom_DATA;
	        _reply_counter = 0;
	        _reply_length = 7;
		}


		if ((size_t)_port->txspace() < buf_size) {
			return;
		}


		for (uint8_t i = 0;  i != buf_size ; i++) {
			_port->write(buf_query[i]);
		}


	}


    // store time of send
    _last_send = AP_HAL::millis();

}



void AP_Mount_ViewPro::read_incoming() {
    uint8_t data;
    int16_t numc;

    numc = _port->available();

    if (numc <= 0 ){
        return;
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////Might be a better idea to get the number of bytes on port->available(), then determine the reply type///////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();
        if (_reply_type == ReplyType_UNKNOWN or _reply_length != numc) {
            continue;
        }



        _buffer[_reply_counter++] = data;



        if(!is_connected){
        	if(_reply_counter > 12){  is_connected = true;	}
        }


        if (_reply_counter == _reply_length) {
            parse_reply();

            switch (_reply_type) {
                case ReplyType_Zoom_DATA:
                    _reply_type = ReplyType_UNKNOWN;
                    _reply_length = 0;
                    _reply_counter = 0;
                    break;
                case ReplyType_angle_DATA:
                    _reply_type = ReplyType_UNKNOWN;
                    _reply_length = 0;
                    _reply_counter = 0;
                    break;
                case ReplyType_Rec_State_DATA:
                    _reply_type = ReplyType_UNKNOWN;
                    _reply_length = 0;
                    _reply_counter = 0;
                    break;

                default:
                    _reply_length = 0;
                    _reply_counter = 0;
                    break;
            }
        }
    }
}





void AP_Mount_ViewPro::parse_reply() {

    switch (_reply_type) {
        case ReplyType_Zoom_DATA:

        	_zoom_level =  (_buffer.zoom_data.byte3<<12 | _buffer.zoom_data.byte4<<8 | _buffer.zoom_data.byte5<<4 | _buffer.zoom_data.byte6);

        	/*
        	hal.console->print("\n");
        	hal.console->print("\n");
        	hal.console->printf("byte3:%x", _buffer.zoom_data.byte3)  ;
        	hal.console->print("\n");
        	hal.console->p
        	rintf("byte4:%x", _buffer.zoom_data.byte4)  ;
        	hal.console->print("\n");
        	hal.console->printf("byte5:%x", _buffer.zoom_data.byte5)  ;
        	hal.console->print("\n");
        	hal.console->printf("byte6:%x", _buffer.zoom_data.byte6)  ;

        	hal.console->print("\n");
        	hal.console->printf("zoom:%ul", _zoom_level)  ;
        	hal.console->print("\n");
        	hal.console->print("\n");
*/

            break;

        case ReplyType_angle_DATA:

        	_camera_tilt_angle = (_buffer.angle_data.pitch_ang)*0.0219726;
        	_camera_pan_angle = (_buffer.angle_data.yaw_rel_ang)*0.0219726;

        	//hal.console->print("\n");
        //	hal.console->printf("tilt:%f", _camera_tilt_angle)  ;
        //	hal.console->print("\n");
        //	hal.console->printf("pan:%f", _camera_pan_angle)  ;
        //	hal.console->print("\n");

            break;


        case ReplyType_Rec_State_DATA:

        	if(_buffer.video_state_data.state == 0x00){
        		is_recording = false;
        		is_video_mode = true;
        	}else if(_buffer.video_state_data.state == 0x01){
        		is_recording = true;
        		is_video_mode = true;
        	}else{
        		is_recording = false;
        		is_video_mode = false;
        	}


            break;

        default:
            break;
    }
}


void AP_Mount_ViewPro::update_zoom_focus_from_rc(){

		if(!_RC_control_enable){

			if(_zoom_level != 0){

				_zoom_out = true;
				_zoom_in = false;
				current_zoom_state = ZOOM_OUT;
				_zooming_state_change = true;

			}else{

				_zoom_in = false;
				_zoom_out = false;
				_zooming_state_change = false;
				_zoom_level = 0;
				_zooming_state_change = false;
				current_zoom_state = ZOOM_STOP;

			}

			return;

		}

		const RC_Channel *zoom_ch = rc().channel(CH_3);
		const RC_Channel *pip_color = rc().channel(CH_4);

		float zoom_value = zoom_ch->norm_input_dz();
		float pip_color_value = pip_color->norm_input_dz();

		if(zoom_value < -0.4){
			_zoom_in = false;
			_zoom_out = true;

			//Check if we've changed state
			if(current_zoom_state != ZOOM_OUT){
				_zooming_state_change = true;
			}

			//Don't trigger _zooming_state_change again
			current_zoom_state = ZOOM_OUT;

		}else if(zoom_value > 0.4){
			_zoom_in = true;
			_zoom_out = false;

			//Check if we've changed state
			if(current_zoom_state != ZOOM_IN){
				_zooming_state_change = true;
			}

			//Don't trigger _zooming_state_change again
			current_zoom_state = ZOOM_IN;

		}else{
			_zoom_in = false;
			_zoom_out = false;

			//Check if we've changed state
			if(current_zoom_state != ZOOM_STOP){
				_zooming_state_change = true;
			}

			current_zoom_state = ZOOM_STOP;
		}




		if(pip_color_value < -0.4){


			if(!pip_change_hold){
				command_flags.color_change = true;
			}

			pip_change_hold = true;


		}else if(pip_color_value > 0.4){


			if(!color_change_hold){
			command_flags.toggle_pip = true;

			}

			color_change_hold = true;

		}else{

			color_change_hold = false;
			pip_change_hold = false;

		}






}


void AP_Mount_ViewPro::update_target_spd_from_rc(){

	//Always have scroll wheel availble for tilt control while in 'RC-Targeting' mode
	const RC_Channel *tilt_wheel_ch = rc().channel(CH_6);

	int16_t wheel_radio = tilt_wheel_ch->get_radio_in();

	if(wheel_radio == 0){
		_speed_ef_target_deg.y = 0;
	}else{
		_speed_ef_target_deg.y = -CAM_SPD_MAX + ((tilt_wheel_ch->norm_input_dz() + 1) * CAM_SPD_MAX);
		_speed_ef_target_deg.y = -1.0f* _speed_ef_target_deg.y;

		if(command_flags.flip_image){
			_speed_ef_target_deg.y = -1.0f* _speed_ef_target_deg.y;
		}
	}


	if(!_RC_control_enable){

		///Add angle target for yaw??
		_speed_ef_target_deg.x = 0;
		_speed_ef_target_deg.z = 0;
		return;
	}

	float spd_factor = CAM_SPD_MAX + ((float)_zoom_level * ((CAM_SPD_MIN - CAM_SPD_MAX) / 16384));
	spd_factor = constrain_float(spd_factor, CAM_SPD_MIN, CAM_SPD_MAX);

	//hal.console->print("\n");
	//hal.console->printf("ang_spd:%f", spd_factor)  ;
	//hal.console->print("\n");

	const RC_Channel *pan_ch = rc().channel(CH_1);
	const RC_Channel *tilt_ch = rc().channel(CH_2);

	_speed_ef_target_deg.x = 0;
	_speed_ef_target_deg.z = -spd_factor + ((pan_ch->norm_input_dz() + 1) * spd_factor);


	//Keep scroll wheel available even in autonomous modes
	if(is_zero(tilt_wheel_ch->norm_input_dz())){
		_speed_ef_target_deg.y = -spd_factor + ((tilt_ch->norm_input_dz() + 1) * spd_factor);

		if(command_flags.flip_image){
			_speed_ef_target_deg.y = -1.0f* _speed_ef_target_deg.y;
		}

	}else{

		_speed_ef_target_deg.y = -spd_factor + ((tilt_wheel_ch->norm_input_dz() + 1) * spd_factor);


		if(command_flags.flip_image){
			_speed_ef_target_deg.y = -1.0f* _speed_ef_target_deg.y;
		}


	}

}



void AP_Mount_ViewPro::turn_motors_off(bool en)
{

	cmd_6_byte_struct cmd_save;

	cmd_save.byte1 = 0x3E;
	cmd_save.byte2 = 0x45;
	cmd_save.byte3 = 0x01;
	cmd_save.byte4 = 0x46;

	if(en){
		cmd_save.byte5 = 0x0C;
		cmd_save.byte6 = 0x0C;
	}else{
		cmd_save.byte5 = 0x0B;
		cmd_save.byte6 = 0x0B;
	}

	uint8_t* buf;
	 buf = (uint8_t*)&cmd_save;

	for (uint8_t i = 0;  i != sizeof(cmd_save) ; i++) {
		_port->write(buf[i]);
	}


    // store time of send
    _last_send = AP_HAL::millis();

}



void AP_Mount_ViewPro::enable_follow_yaw(){



	cmd_11_byte_struct enable_follow;
	enable_follow.byte1 = 0x3E;
	enable_follow.byte2 = 0x1F;
	enable_follow.byte3 = 0x06;
	enable_follow.byte4 = 0x25;
	enable_follow.byte5 = 0x01;
	enable_follow.byte6 = 0x1F;
	enable_follow.byte7 = 0x01;
	enable_follow.byte8 = 0x00;
	enable_follow.byte9 = 0x00;
	enable_follow.byte10 = 0x00;
	enable_follow.byte11 = 0x21;

	uint8_t* buf;
		 buf = (uint8_t*)&enable_follow;

		for (uint8_t i = 0;  i != sizeof(enable_follow) ; i++) {
			_port->write(buf[i]);
		}



		cmd_7_byte_struct ask_follow;
		ask_follow.byte1 = 0x3E;
		ask_follow.byte2 = 0x40;
		ask_follow.byte3 = 0x02;
		ask_follow.byte4 = 0x42;
		ask_follow.byte5 = 0x01;
		ask_follow.byte6 = 0x1F;
		ask_follow.byte7 = 0x20;


		uint8_t* buf2;
			 buf2 = (uint8_t*)&ask_follow;

			for (uint8_t i = 0;  i != sizeof(ask_follow) ; i++) {
				_port->write(buf2[i]);
			}


	    // store time of send
	    _last_send = AP_HAL::millis();


}






void AP_Mount_ViewPro::disable_follow_yaw(){


return;

}


void AP_Mount_ViewPro::reset_camera(){


	static	cmd_6_byte_struct reset_camera;

	reset_camera.byte1 = 0x3E;
	reset_camera.byte2 = 0x45;
	reset_camera.byte3 = 0x01;
	reset_camera.byte4 = 0x46;
	reset_camera.byte5 = 0x23;
	reset_camera.byte6 = 0x23;


	uint8_t* buf;
	buf = (uint8_t*)&reset_camera;

	for (uint8_t i = 0;  i != sizeof(reset_camera) ; i++) {
		_port->write(buf[i]);
	}

	static cmd_6_byte_struct cmd_set_zoom_data;

		cmd_set_zoom_data.byte1 = 0x81;
		cmd_set_zoom_data.byte2 = 0x01;
		cmd_set_zoom_data.byte3 = 0x04;
		cmd_set_zoom_data.byte4 = 0x07;
		cmd_set_zoom_data.byte5 = 0x37;
		cmd_set_zoom_data.byte6 = 0xFF;



		uint8_t* buf2;
		buf2 = (uint8_t*)&cmd_set_zoom_data;

		for (uint8_t i = 0;  i != sizeof(cmd_set_zoom_data) ; i++) {
			_port->write(buf2[i]);
		}



	// store time of send
	_last_send = AP_HAL::millis();


}


void AP_Mount_ViewPro::toggle_camera(){

	command_flags.toggle_video = true;
}

