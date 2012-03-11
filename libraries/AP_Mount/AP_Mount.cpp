// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <AP_Mount.h>

extern RC_Channel_aux* g_rc_function[RC_Channel_aux::k_nr_aux_servo_functions];	// the aux. servo ch. assigned to each function

AP_Mount::AP_Mount(GPS *gps, AP_AHRS *ahrs)
{
	_ahrs=ahrs;
	_gps=gps;
	//set_mode(MAV_MOUNT_MODE_RETRACT);
	//set_mode(MAV_MOUNT_MODE_RC_TARGETING); // FIXME: This is just to test without mavlink
	set_mode(MAV_MOUNT_MODE_GPS_POINT); // FIXME: this is to test ONLY targeting

	_retract_angles.x=0;
	_retract_angles.y=0;
	_retract_angles.z=0;
}

//sets the servo angles for retraction, note angles are * 100
void AP_Mount::set_retract_angles(int roll, int pitch, int yaw)
{
	_retract_angles.x=roll;
	_retract_angles.y=pitch;
	_retract_angles.z=yaw;
}

//sets the servo angles for neutral, note angles are * 100
void AP_Mount::set_neutral_angles(int roll, int pitch, int yaw)
{
	_neutral_angles.x=roll;
	_neutral_angles.y=pitch;
	_neutral_angles.z=yaw;
}

//sets the servo angles for MAVLink, note angles are * 100
void AP_Mount::set_mavlink_angles(int roll, int pitch, int yaw)
{
	_mavlink_angles.x = roll;
	_mavlink_angles.y = pitch;
	_mavlink_angles.z = yaw;
}

// used to tell the mount to track GPS location
void AP_Mount::set_GPS_target_location(Location targetGPSLocation)
{
	_target_GPS_location=targetGPSLocation;
}

// This one should be called periodically
void AP_Mount::update_mount_position()
{
	Matrix3f m;					//holds 3 x 3 matrix, var is used as temp in calcs
	Vector3f targ;				//holds target vector, var is used as temp in calcs
	Vector3f aux_vec;			//holds target vector, var is used as temp in calcs

	switch(_mount_mode)
	{
	case MAV_MOUNT_MODE_RETRACT:
		roll_angle =100*_retract_angles.x;
		pitch_angle=100*_retract_angles.y;
		yaw_angle  =100*_retract_angles.z;
		break;

	case MAV_MOUNT_MODE_NEUTRAL:
		roll_angle =100*_neutral_angles.x;
		pitch_angle=100*_neutral_angles.y;
		yaw_angle  =100*_neutral_angles.z;
		break;

	case MAV_MOUNT_MODE_MAVLINK_TARGETING:
	{
		aux_vec.x = _mavlink_angles.x;
		aux_vec.y = _mavlink_angles.y;
		aux_vec.z = _mavlink_angles.z;
		m = _ahrs->get_dcm_matrix();
		m.transpose();
		//rotate vector
		targ = m*aux_vec;
		// TODO The next three lines are probably not correct yet
		roll_angle  = _stab_roll? degrees(atan2( targ.y,targ.z))*100:_mavlink_angles.y;	//roll
		pitch_angle = _stab_pitch?degrees(atan2(-targ.x,targ.z))*100:_neutral_angles.x;	//pitch
		yaw_angle   = _stab_yaw?  degrees(atan2(-targ.x,targ.y))*100:_neutral_angles.z;	//yaw
		break;
	}

	case MAV_MOUNT_MODE_RC_TARGETING:  // radio manual control
	{
		// TODO It does work, but maybe is a good idea to replace this simplified implementation with a proper one
		if (_ahrs)
		{
			roll_angle  = -_ahrs->roll_sensor;
			pitch_angle = -_ahrs->pitch_sensor;
			yaw_angle   = -_ahrs->yaw_sensor;
		}
		if (g_rc_function[RC_Channel_aux::k_mount_roll])
			roll_angle  = rc_map(g_rc_function[RC_Channel_aux::k_mount_roll]);
		if (g_rc_function[RC_Channel_aux::k_mount_pitch])
			pitch_angle = rc_map(g_rc_function[RC_Channel_aux::k_mount_pitch]);
		if (g_rc_function[RC_Channel_aux::k_mount_yaw])
			yaw_angle   = rc_map(g_rc_function[RC_Channel_aux::k_mount_yaw]);
		break;
	}

	case MAV_MOUNT_MODE_GPS_POINT:
	{
		if(_gps->fix)
		{
			calc_GPS_target_vector(&_target_GPS_location);
		}
		m = _ahrs->get_dcm_matrix();
		m.transpose();
		targ = m*_GPS_vector;
		/* disable stabilization for now, this will help debug */
		_stab_roll = 0;_stab_pitch=0;_stab_yaw=0;
		/**/
		// TODO The next three lines are probably not correct yet
		roll_angle  = _stab_roll? degrees(atan2( targ.y,targ.z))*100:_GPS_vector.y;	//roll
		pitch_angle = _stab_pitch?degrees(atan2(-targ.x,targ.z))*100:0;	//pitch
		yaw_angle   = _stab_yaw?  degrees(atan2(-targ.x,targ.y))*100:degrees(atan2(-_GPS_vector.x,_GPS_vector.y))*100;	//yaw
		break;
	}
	default:
		//do nothing
		break;
	}

	// write the results to the servos
	// Change scaling to 0.1 degrees in order to avoid overflows in the angle arithmetic
	G_RC_AUX(k_mount_roll)->closest_limit(roll_angle/10);
	G_RC_AUX(k_mount_pitch)->closest_limit(pitch_angle/10);
	G_RC_AUX(k_mount_yaw)->closest_limit(yaw_angle/10);
}	

void AP_Mount::set_mode(enum MAV_MOUNT_MODE mode)
{
	_mount_mode=mode;
}

void AP_Mount::configure_msg(mavlink_message_t* msg)
{
	__mavlink_mount_configure_t packet;
	mavlink_msg_mount_configure_decode(msg, &packet);
	if (mavlink_check_target(packet.target_system, packet.target_component)) {
		// not for us
		return;
	}
	set_mode((enum MAV_MOUNT_MODE)packet.mount_mode);
	_stab_pitch = packet.stab_pitch;
	_stab_roll  = packet.stab_roll;
	_stab_yaw   = packet.stab_yaw;
}

void AP_Mount::control_msg(mavlink_message_t *msg)
{
	__mavlink_mount_control_t packet;
	mavlink_msg_mount_control_decode(msg, &packet);
	if (mavlink_check_target(packet.target_system, packet.target_component)) {
		// not for us
		return;
	}

	switch (_mount_mode)
	{
	case MAV_MOUNT_MODE_RETRACT:  // Load and keep safe position (Roll,Pitch,Yaw) from EEPROM and stop stabilization
		set_retract_angles(packet.input_b, packet.input_a, packet.input_c);
		if (packet.save_position)
		{
			// TODO: Save current trimmed position on EEPROM
		}
		break;

	case MAV_MOUNT_MODE_NEUTRAL:  //  Load and keep neutral position (Roll,Pitch,Yaw) from EEPROM
		set_neutral_angles(packet.input_b, packet.input_a, packet.input_c);
		if (packet.save_position)
		{
			// TODO: Save current trimmed position on EEPROM
		}
		break;

	case MAV_MOUNT_MODE_MAVLINK_TARGETING:  // Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
		set_mavlink_angles(packet.input_b, packet.input_a, packet.input_c);
		break;

	case MAV_MOUNT_MODE_RC_TARGETING:  // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
		break;

	case MAV_MOUNT_MODE_GPS_POINT:  // Load neutral position and start to point to Lat,Lon,Alt
		Location targetGPSLocation;
		targetGPSLocation.lat = packet.input_a;
		targetGPSLocation.lng = packet.input_b;
		targetGPSLocation.alt = packet.input_c;
		set_GPS_target_location(targetGPSLocation);
		break;
	}
}

void AP_Mount::status_msg(mavlink_message_t *msg)
{
	__mavlink_mount_status_t packet;
	mavlink_msg_mount_status_decode(msg, &packet);
	if (mavlink_check_target(packet.target_system, packet.target_component)) {
		// not for us
		return;
	}

	switch (_mount_mode)
	{
	case MAV_MOUNT_MODE_RETRACT:			// safe position (Roll,Pitch,Yaw) from EEPROM and stop stabilization
	case MAV_MOUNT_MODE_NEUTRAL:			// neutral position (Roll,Pitch,Yaw) from EEPROM
	case MAV_MOUNT_MODE_MAVLINK_TARGETING:	// neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
	case MAV_MOUNT_MODE_RC_TARGETING:		// neutral position and start RC Roll,Pitch,Yaw control with stabilization
		packet.pointing_b = roll_angle;		///< degrees*100
		packet.pointing_a = pitch_angle;	///< degrees*100
		packet.pointing_c = yaw_angle;		///< degrees*100
		break;
	case MAV_MOUNT_MODE_GPS_POINT:         // neutral position and start to point to Lat,Lon,Alt
		packet.pointing_a = _target_GPS_location.lat;	///< latitude
		packet.pointing_b = _target_GPS_location.lng;	///< longitude
		packet.pointing_c = _target_GPS_location.alt;	///< altitude
		break;
	}

	// status reply
	// TODO: is COMM_3 correct ?
	mavlink_msg_mount_status_send(MAVLINK_COMM_3, packet.target_system, packet.target_component,
			packet.pointing_a, packet.pointing_b, packet.pointing_c);
}

void AP_Mount::set_roi_cmd()
{
	// TODO get the information out of the mission command and use it
}

void AP_Mount::configure_cmd()
{
	// TODO get the information out of the mission command and use it
}

void AP_Mount::control_cmd()
{
	// TODO get the information out of the mission command and use it
}


void AP_Mount::calc_GPS_target_vector(struct Location *target)
{
	_GPS_vector.x = (target->lng-_gps->longitude) * cos((_gps->latitude+target->lat)/2)*.01113195;
	_GPS_vector.y = (target->lat-_gps->latitude)*.01113195;
	_GPS_vector.z = (_gps->altitude-target->alt);
}

void
AP_Mount::update_mount_type()
{
	// Auto-detect the mount gimbal type depending on the functions assigned to the servos
	if ((g_rc_function[RC_Channel_aux::k_mount_roll] == NULL) && (g_rc_function[RC_Channel_aux::k_mount_pitch] != NULL) && (g_rc_function[RC_Channel_aux::k_mount_yaw] != NULL))
	{
		_mount_type = k_pan_tilt;
	}
	if ((g_rc_function[RC_Channel_aux::k_mount_roll] != NULL) && (g_rc_function[RC_Channel_aux::k_mount_pitch] != NULL) && (g_rc_function[RC_Channel_aux::k_mount_yaw] == NULL))
	{
		_mount_type = k_tilt_roll;
	}
	if ((g_rc_function[RC_Channel_aux::k_mount_roll] != NULL) && (g_rc_function[RC_Channel_aux::k_mount_pitch] != NULL) && (g_rc_function[RC_Channel_aux::k_mount_yaw] != NULL))
	{
		_mount_type = k_pan_tilt_roll;
	}
}

// This function is needed to let the HIL code compile
long
AP_Mount::rc_map(RC_Channel_aux* rc_ch)
{
  return (rc_ch->radio_in - rc_ch->radio_min) * (rc_ch->angle_max - rc_ch->angle_min) / (rc_ch->radio_max - rc_ch->radio_min) + rc_ch->angle_min;
}

