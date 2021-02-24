#include "AP_Mount_ViewPro.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_SerialManager/AP_SerialManager.h>


//Speeds at which the gimbal will move in deg/s
#define CAM_SPD_MAX 25.0 //zoomed out
#define CAM_SPD_MIN 2.0 // zoomed in

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

    // Initialize and reset commands
    is_recording = false;
    state_is_video = true;

	command_flags.change_state = false;
	command_flags.center_yaw = false;
	command_flags.toggle_rec = false;
	command_flags.stop_video = false;
	yaw_center_reset_flag = false;

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
    		command_flags.zero_zoom = true;
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

            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
        {
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_deg, true, true);
            }

            break;
        }

        default:
            // we do not know this mode so do nothing
            break;

    }



    //while in GPS POINT, can switch to RC_Targeting if any gimbal motion controls are used
	if(get_mode() != MAV_MOUNT_MODE_RC_TARGETING){

		const RC_Channel *pan_ch = rc().channel(CH_1);
		const RC_Channel *tilt_ch = rc().channel(CH_2);
		const RC_Channel *tilt_wheel_ch = rc().channel(CH_6);

		if(!is_zero(tilt_wheel_ch->norm_input_dz()) or (_RC_control_enable and (!is_zero(pan_ch->norm_input_dz()) or !is_zero(tilt_ch->norm_input_dz())))){
			set_mode(MAV_MOUNT_MODE_RC_TARGETING);
		}
	}

	update_zoom_focus_from_rc();


    if(get_mode() == MAV_MOUNT_MODE_RC_TARGETING){

    	//Update at 20 Hz
    	if((AP_HAL::millis() - _last_send) > 50){
			send_targeting_cmd();
		}
		return;

    }else if(get_mode() == MAV_MOUNT_MODE_RETRACT or get_mode() == MAV_MOUNT_MODE_NEUTRAL){
    	//Update at 2 Hz
    	if((AP_HAL::millis() - _last_send) > 500){
    		send_targeting_cmd();
    	}
    	return;

    }else if(get_mode() == MAV_MOUNT_MODE_MAVLINK_TARGETING or get_mode() == MAV_MOUNT_MODE_GPS_POINT){
    	//Update at 10 Hz
    	if((AP_HAL::millis() - _last_send) > 100){
    		send_targeting_cmd();
    	}
    	return;
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

	//toggle between picture mode or record mode
	if(command_flags.change_state){

		camera_state(TOGGLE_STATE);

		command_flags.change_state = false;
		_last_send = AP_HAL::millis();
		return;

	}else if(command_flags.toggle_rec){

		camera_state(TOGGLE_REC);

		command_flags.toggle_rec = false;
		_last_send = AP_HAL::millis();
		return;

	}else if(command_flags.turn_camera_off){

		camera_state(TURN_VID_OFF);

		command_flags.turn_camera_off = false;
		_last_send = AP_HAL::millis();
		return;

	}else if(command_flags.center_yaw){

		if(yaw_center()){
			command_flags.center_yaw = false;
		}

		_last_send = AP_HAL::millis();
		return;

	}else if(_zooming_state_change){

		zoom_camera();

		_last_send = AP_HAL::millis();
		return;

	}else if(command_flags.zero_zoom){

		zero_zoom();

		command_flags.zero_zoom = false;
		_last_send = AP_HAL::millis();
		return;

	}


	// If no commands where out going, resume with gimbal control
	command_gimbal();


	// Send any open queries
	uint8_t* buf_query;
	uint8_t buf_size;

	if(false){

		//Need Code

	}else if(false){

		//Need Code

	}else if(false){


		//Need Code

	}else if(get_mode() == MAV_MOUNT_MODE_RC_TARGETING){

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

            break;

        case ReplyType_angle_DATA:

        	_camera_tilt_angle = (_buffer.angle_data.pitch_ang)*0.0219726;
        	_camera_pan_angle = (_buffer.angle_data.yaw_rel_ang)*0.0219726;

        	/*
        	hal.console->printf("%4f", _camera_tilt_angle);
        	hal.console->print("\n");
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

		/*
		if(_zoom_level != 0){
			current_zoom_state = ZOOM_OUT;
			_zooming_state_change = true;

		}else{
			// if at zero zoom
			if(current_zoom_state == ZOOM_OUT){
				_zooming_state_change = true;
			}

			current_zoom_state = ZOOM_STOP;
		}
		*/

		current_zoom_state = ZOOM_STOP;

		return;
	}

	const RC_Channel *zoom_ch = rc().channel(CH_3);
	//const RC_Channel *pip_color = rc().channel(CH_4);

	float zoom_value = zoom_ch->norm_input_dz();
	//float pip_color_value = pip_color->norm_input_dz();

	if(zoom_value < -0.4){

		//Check if we've changed state
		if(current_zoom_state != ZOOM_OUT){
			_zooming_state_change = true;
		}

		//Don't trigger _zooming_state_change again
		current_zoom_state = ZOOM_OUT;

	}else if(zoom_value > 0.4){

		//Check if we've changed state
		if(current_zoom_state != ZOOM_IN){
			_zooming_state_change = true;
		}

		//Don't trigger _zooming_state_change again
		current_zoom_state = ZOOM_IN;

	}else{

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

	_speed_ef_target_deg.x = 0;

	if(tilt_wheel_ch->get_radio_in() == 0 or is_zero(tilt_wheel_ch->norm_input_dz())){
		_speed_ef_target_deg.y = 0;

	}else{
		_speed_ef_target_deg.y = -CAM_SPD_MAX + ((tilt_wheel_ch->norm_input_dz() + 1) * CAM_SPD_MAX);
		_speed_ef_target_deg.y = -1.0f* _speed_ef_target_deg.y;
	}

	if(!_RC_control_enable){
		_speed_ef_target_deg.z = 0;
		return;
	}

	float spd_factor = CAM_SPD_MAX + ((float)_zoom_level * ((CAM_SPD_MIN - CAM_SPD_MAX) / 16384));
	spd_factor = constrain_float(spd_factor, CAM_SPD_MIN, CAM_SPD_MAX);

	const RC_Channel *pan_ch = rc().channel(CH_1);
	const RC_Channel *tilt_ch = rc().channel(CH_2);

	// Pan speed
	_speed_ef_target_deg.z = -spd_factor + ((pan_ch->norm_input_dz() + 1) * spd_factor);

	//Keep scroll wheel available even in autonomous modes
	if(is_zero(tilt_wheel_ch->norm_input_dz())){
		_speed_ef_target_deg.y = -spd_factor + ((tilt_ch->norm_input_dz() + 1) * spd_factor);
	}else{
		_speed_ef_target_deg.y = -spd_factor + ((tilt_wheel_ch->norm_input_dz() + 1) * spd_factor);
		_speed_ef_target_deg.y = -1.0f* _speed_ef_target_deg.y;
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

	// store time of send
	_last_send = AP_HAL::millis();

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



void AP_Mount_ViewPro::command_gimbal(){

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

			// force follow mode by not giving a control mode to Yaw
			if(!_RC_control_enable){
				cmd_set_data.YM = 0x00;
				cmd_set_data.Ys = (int)0;
				cmd_set_data.Ya = (int)0;
			}


			break;

		default:
			// we do not know this mode so do nothing
			break;

	}

	// write the commands
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

}




bool AP_Mount_ViewPro::yaw_center(){

	uint8_t* buf_cmd;
	uint8_t buf_size;

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
		return false;
	}

	for (uint8_t i = 0;  i != buf_size ; i++) {
		_port->write(buf_cmd[i]);
	}

	return true;

}


void AP_Mount_ViewPro::zoom_camera(){

		static cmd_6_byte_struct cmd_set_zoom_data;

		cmd_set_zoom_data.byte1 = 0x81;
		cmd_set_zoom_data.byte2 = 0x01;
		cmd_set_zoom_data.byte3 = 0x04;
		cmd_set_zoom_data.byte4 = 0x07;
		cmd_set_zoom_data.byte5 = 0x00;
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
}




void AP_Mount_ViewPro::zero_zoom(){

		static cmd_9_byte_struct cmd_set_zoom_zero_data;
		cmd_set_zoom_zero_data.byte1 = 0x81;
		cmd_set_zoom_zero_data.byte2 = 0x01;
		cmd_set_zoom_zero_data.byte3 = 0x04;
		cmd_set_zoom_zero_data.byte4 = 0x47;
		cmd_set_zoom_zero_data.byte5 = 0x00;
		cmd_set_zoom_zero_data.byte6 = 0x00;
		cmd_set_zoom_zero_data.byte7 = 0x00;
		cmd_set_zoom_zero_data.byte8 = 0x00;
		cmd_set_zoom_zero_data.byte9 = 0xFF;

		if ((size_t)_port->txspace() <= sizeof(cmd_set_zoom_zero_data)) {
			return;
		}

		uint8_t* buf_zoom = (uint8_t*)&cmd_set_zoom_zero_data;

		for (uint8_t i = 0;  i != sizeof(cmd_set_zoom_zero_data) ; i++) {
			_port->write(buf_zoom[i]);
		}

		_zoom_level = 0;

}


void AP_Mount_ViewPro::camera_state(int camera_cmd){

	static cmd_6_byte_struct cmd_change_video_state;

	uint8_t* buf_cmd;
	uint8_t buf_size;

	cmd_change_video_state.byte1 = 0x81;
	cmd_change_video_state.byte2 = 0x01;
	cmd_change_video_state.byte3 = 0x04;
	cmd_change_video_state.byte4 = 0x68;
	cmd_change_video_state.byte5 = 0x00;
	cmd_change_video_state.byte6 = 0xFF;

	switch(camera_cmd) {

		case TOGGLE_REC:
			cmd_change_video_state.byte5 = 0x04;
			break;

		case TURN_VID_OFF:
			cmd_change_video_state.byte5 = 0x03;
			break;

		case TOGGLE_STATE:
			cmd_change_video_state.byte5 = 0x05;
			break;

		//case TAKE_PIC:
			//cmd_change_video_state.byte5 = 0x01;
			//break;
	}


	buf_size = sizeof(cmd_change_video_state);
	buf_cmd = (uint8_t*)&cmd_change_video_state;

	if ((size_t)_port->txspace() < buf_size) {
		return;
	}

	for (uint8_t i = 0;  i != buf_size ; i++) {
		_port->write(buf_cmd[i]);
	}

}




