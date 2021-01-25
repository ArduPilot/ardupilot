#include "AP_Mount_ViewPro.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_SerialManager/AP_SerialManager.h>



#define CAM_SPD_MAX 50.0
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
    state_is_video = true;

}

// update mount position - should be called periodically
void AP_Mount_ViewPro::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }


   // _frontend.f_lat = _lat;
   // _frontend.f_long = _long;
   // _frontend.f_roi_pan = _roi_pan;

    read_incoming(); // read the incoming messages from the gimbal


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
            // update targets using pilot's rc inputs
           // update_targets_from_rc();

        		update_target_spd_from_rc();
        		update_zoom_focus_from_rc();

            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_deg, true, true);
            }

            update_zoom_focus_from_rc();

            break;

        default:
            // we do not know this mode so do nothing
            break;

    }



    //Debugging
/*
    if((AP_HAL::millis() - _last_send) > 100){

    	hal.console->print("\n");
    	hal.console->print("\n");
    	hal.console->printf("zoom:%ul", _zoom_level)  ;
    	hal.console->print("   ");
    	hal.console->printf("tilt:%f", _camera_tilt_angle)  ;
    	hal.console->print("   ");
    	hal.console->printf("pan:%f", _camera_pan_angle)  ;
    	hal.console->print("\n");
    	hal.console->print("\n");

    }
*/


    if(get_mode() == MAV_MOUNT_MODE_RC_TARGETING){

    	if((AP_HAL::millis() - _last_send) > 100){
			send_targeting_cmd();
    		}


			return;

    }else if(get_mode() == MAV_MOUNT_MODE_MAVLINK_TARGETING){

    	if((AP_HAL::millis() - _last_send) > 300){
    		//Do Nothing
    	}

    	return;

    }else if(get_mode() == MAV_MOUNT_MODE_GPS_POINT){

    	if((AP_HAL::millis() - _last_send) > 300){
    		send_targeting_cmd();
    	}

    }else if((AP_HAL::millis() - _last_send) > 300){
    	send_targeting_cmd();
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
    mavlink_msg_mount_status_send(chan, 0, 0, _current_angle.y, _current_angle.x, _current_angle.z);
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

	uint8_t* buf_cmd;
	uint8_t buf_size;

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

	    command_flags.change_state = false;

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

	    return;

	} else if(command_flags.toggle_video_tracking){


		static cmd_48_byte_struct cmd_toggle_tracking_video_on_off;

			cmd_toggle_tracking_video_on_off.byte1 = 0x7E;
			cmd_toggle_tracking_video_on_off.byte2 = 0x7E;
			cmd_toggle_tracking_video_on_off.byte3 = 0x44;
			cmd_toggle_tracking_video_on_off.byte4 = 0x00;
			cmd_toggle_tracking_video_on_off.byte5 = 0x00;
			cmd_toggle_tracking_video_on_off.byte6 = 0x7C;
			cmd_toggle_tracking_video_on_off.byte7 = 0x00;
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


			if(is_recording){
				cmd_toggle_tracking_video_on_off.byte7 = 0x04;
				is_recording = false;
			}else{
				cmd_toggle_tracking_video_on_off.byte7 = 0x05;
				is_recording = true;
			}


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
					cmd_toggle_tracking_on_off.byte14 = 0x3C;
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


		pip_state++;

		if(pip_state >= 4 ){
			pip_state = 0;
		}

		command_flags.toggle_pip = false;

		return;

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

		case MAV_MOUNT_MODE_MAVLINK_TARGETING:
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

		if(( is_zero(_pitch_value) and is_zero(_yaw_value) and current_zoom_state == ZOOM_STOP)){

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


/*

// send_target_angles
void AP_Mount_ViewPro::send_zoom()
{

// if we are not zooming and our current state is STOP, do nothing
	if(!_zoom_in and !_zoom_out and current_zoom_state == ZOOM_STOP){
		return;
	}


// if we are currently zooming update zoom position

	 static cmd_5_byte_struct cmd_ask_zoom_data;
	 uint8_t* buf;

	if((_zoom_in or _zoom_out) and (current_zoom_state == ZOOM_IN or current_zoom_state == ZOOM_OUT)){

		//request zoom state
	    cmd_ask_zoom_data.byte1 = 0x81;
	    cmd_ask_zoom_data.byte2 = 0x09;
	    cmd_ask_zoom_data.byte3 = 0x04;
	    cmd_ask_zoom_data.byte4 = 0x47;
	    cmd_ask_zoom_data.byte5 = 0xFF;

	    buf = (uint8_t*)&cmd_ask_zoom_data;

	    for (uint8_t i = 0;  i != sizeof(cmd_ask_zoom_data) ; i++) {
	        _port->write(buf[i]);
	    }

	    //Set reply flags
	    _reply_type =  ReplyType_Zoom_DATA;
        _reply_counter = 0;
        _reply_length = 7;

		return;
	}


//If the zoom command is changing

	static cmd_6_byte_struct cmd_set_zoom_data;

	cmd_set_zoom_data.byte1 = 0x81;
	cmd_set_zoom_data.byte2 = 0x01;
	cmd_set_zoom_data.byte3 = 0x04;
	cmd_set_zoom_data.byte4 = 0x07;
	cmd_set_zoom_data.byte6 = 0xFF;


	if(_zoom_in){
		cmd_set_zoom_data.byte5 = 0x27;
		current_zoom_state = ZOOM_IN;
	}else if(_zoom_out){
		cmd_set_zoom_data.byte5 = 0x37;
		current_zoom_state = ZOOM_OUT;
	}else{
		cmd_set_zoom_data.byte5 = 0x00;
		current_zoom_state = ZOOM_STOP;
	}


    if ((size_t)_port->txspace() <= sizeof(cmd_set_zoom_data)) {
        return;
    }



    buf = (uint8_t*)&cmd_set_zoom_data;

  	    for (uint8_t i = 0;  i != sizeof(cmd_set_zoom_data) ; i++) {
  	        _port->write(buf[i]);
  	    }


    // store time of send
    _last_send = AP_HAL::millis();

}

*/

/*

void AP_Mount_ViewPro::ctrl_camera(){


	 static cmd_7_byte_struct cmd_ask_follow_data;
	uint8_t* buf;


			//request zoom state
	cmd_ask_follow_data.byte1 = 0x3E;
	cmd_ask_follow_data.byte2 = 0x40;
	cmd_ask_follow_data.byte3 = 0x02;
	cmd_ask_follow_data.byte4 = 0x42;
	cmd_ask_follow_data.byte5 = 0x01;
	cmd_ask_follow_data.byte6 = 0x1F;
	cmd_ask_follow_data.byte7 = 0x20;

		    buf = (uint8_t*)&cmd_ask_follow_data;

		    for (uint8_t i = 0;  i != sizeof(cmd_ask_follow_data) ; i++) {
		        _port->write(buf[i]);
		    }

		    //Set reply flags
		  //  _reply_type =  ReplyType_Zoom_DATA;
	      //  _reply_counter = 0;
	      //  _reply_length = 7;



}


*/



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
        	hal.console->printf("byte4:%x", _buffer.zoom_data.byte4)  ;
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
/*
        	hal.console->print("\n");
        	hal.console->printf("tilt:%f", _camera_tilt_angle)  ;
        	hal.console->print("\n");
        	hal.console->printf("pan:%f", _camera_pan_angle)  ;
        	hal.console->print("\n");
*/
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
			_zoom_in = false;
			_zoom_out = false;
			_zooming_state_change =false;
			_zoom_level = 0;
			current_zoom_state = ZOOM_STOP;
			return;
		}

		const RC_Channel *zoom_ch = rc().channel(CH_3);
		//const RC_Channel *focus_ch = rc().channel(CH_4);

		float zoom_value = zoom_ch->norm_input_dz();
		//float focus_value = focus_ch->norm_input_dz();

		if(zoom_value < -0.5){
			_zoom_in = false;
			_zoom_out = true;

			//Check if we've changed state
			if(current_zoom_state != ZOOM_OUT){
				_zooming_state_change = true;
			}

			current_zoom_state = ZOOM_OUT;

		}else if(zoom_value > 0.5){
			_zoom_in = true;
			_zoom_out = false;

			//Check if we've changed state
			if(current_zoom_state != ZOOM_IN){
				_zooming_state_change = true;
			}

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


}



void AP_Mount_ViewPro::update_target_spd_from_rc(){

	//Always have scroll wheel availble for tilt control while in 'RC-Targeting' mode
	const RC_Channel *tilt_wheel_ch = rc().channel(CH_6);

	int16_t wheel_radio = tilt_wheel_ch->get_radio_in();

	if(wheel_radio == 0){
		_speed_ef_target_deg.y = 0;
	}else{
		_speed_ef_target_deg.y = -CAM_SPD_MAX + ((tilt_wheel_ch->norm_input_dz() + 1) * CAM_SPD_MAX);
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
	}else{
		_speed_ef_target_deg.y = -spd_factor + ((tilt_wheel_ch->norm_input_dz() + 1) * spd_factor);
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

