// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <AP_Mount.h>
#include <../RC_Channel/RC_Channel_aux.h>

extern RC_Channel_aux* g_rc_function[RC_Channel_aux::k_nr_aux_servo_functions];	// the aux. servo ch. assigned to each function

AP_Mount::AP_Mount(GPS *gps, AP_DCM *dcm)
{
	_dcm=dcm;
	_gps=gps;
}

void AP_Mount::set_pitch_yaw(int pitchCh, int yawCh)
{
}

void AP_Mount::set_GPS_target(Location targetGPSLocation)
{
	_target_GPS_location=targetGPSLocation;
	
	//set mode
	_mount_mode=k_gps_target;

	//update mount position
	update_mount();
}

void AP_Mount::set_assisted(int roll, int pitch, int yaw)
{
	_assist_angles.x = roll;
	_assist_angles.y = pitch;
	_assist_angles.z = yaw;

	//set mode
	_mount_mode=k_assisted;

	//update mount position
	update_mount();
}

//sets the servo angles for FPV, note angles are * 100
void AP_Mount::set_mount_free_roam(int roll, int pitch, int yaw)
{
	_roam_angles.x=roll;
	_roam_angles.y=pitch;
	_roam_angles.z=yaw;

	//set mode
	_mount_mode=k_roam;

	//now update mount position
	update_mount();
}

//sets the servo angles for landing, note angles are * 100
void AP_Mount::set_mount_landing(int roll, int pitch, int yaw)
{
	_landing_angles.x=roll;
	_landing_angles.y=pitch;
	_landing_angles.z=yaw;

	//set mode
	_mount_mode=k_landing;

	//now update mount position
	update_mount();
}

void AP_Mount::set_none()
{
	//set mode
	_mount_mode=k_none;

	//now update mount position
	update_mount();
}

void AP_Mount::update_mount()
{
	Matrix3f m;					//holds 3 x 3 matrix, var is used as temp in calcs
	Vector3f targ;				//holds target vector, var is used as temp in calcs

	switch(_mount_mode)
	{
		case k_gps_target:
			{
				if(_gps->fix) 
				{
					calc_GPS_target_vector(&_target_GPS_location);
				}
				m = _dcm->get_dcm_transposed();
				targ = m*_GPS_vector;
				roll_angle =   degrees(atan2(targ.y,targ.z))*100;	//roll
				pitch_angle =  degrees(atan2(-targ.x,targ.z))*100;	//pitch 
				break;
			}
		case k_stabilise:
			{
				// TODO replace this simplified implementation with a proper one
				roll_angle  = -_dcm->roll_sensor;
				pitch_angle = -_dcm->pitch_sensor;
				yaw_angle   = -_dcm->yaw_sensor;
				break;
			}
		case k_roam:
			{
				roll_angle=100*_roam_angles.x;
				pitch_angle=100*_roam_angles.y;
				yaw_angle=100*_roam_angles.z;
				break;
			}
		case k_assisted:
			{
				m = _dcm->get_dcm_transposed();    
				//rotate vector
				targ = m*_assist_vector;
				roll_angle =   degrees(atan2(targ.y,targ.z))*100;	//roll
				pitch_angle =  degrees(atan2(-targ.x,targ.z))*100;	//pitch 
				break;
			}
		case k_landing:
			{
				roll_angle=100*_roam_angles.x;
				pitch_angle=100*_roam_angles.y;
				yaw_angle=100*_roam_angles.z;
				break;
			}
		case k_manual:	// radio manual control
			if (g_rc_function[RC_Channel_aux::k_mount_roll])
				roll_angle = map(g_rc_function[RC_Channel_aux::k_mount_roll]->radio_in,
						g_rc_function[RC_Channel_aux::k_mount_roll]->radio_min,
						g_rc_function[RC_Channel_aux::k_mount_roll]->radio_max,
						g_rc_function[RC_Channel_aux::k_mount_roll]->angle_min,
						g_rc_function[RC_Channel_aux::k_mount_roll]->angle_max);
			if (g_rc_function[RC_Channel_aux::k_mount_pitch])
				pitch_angle = map(g_rc_function[RC_Channel_aux::k_mount_pitch]->radio_in,
						g_rc_function[RC_Channel_aux::k_mount_pitch]->radio_min,
						g_rc_function[RC_Channel_aux::k_mount_pitch]->radio_max,
						g_rc_function[RC_Channel_aux::k_mount_pitch]->angle_min,
						g_rc_function[RC_Channel_aux::k_mount_pitch]->angle_max);
			if (g_rc_function[RC_Channel_aux::k_mount_yaw])
				yaw_angle = map(g_rc_function[RC_Channel_aux::k_mount_yaw]->radio_in,
						g_rc_function[RC_Channel_aux::k_mount_yaw]->radio_min,
						g_rc_function[RC_Channel_aux::k_mount_yaw]->radio_max,
						g_rc_function[RC_Channel_aux::k_mount_yaw]->angle_min,
						g_rc_function[RC_Channel_aux::k_mount_yaw]->angle_max);
			break;
		case k_none:
		default:
			{
				//do nothing
				break;
			}			
	}

	// write the results to the servos
	// Change scaling to 0.1 degrees in order to avoid overflows in the angle arithmetic
	if (g_rc_function[RC_Channel_aux::k_mount_roll])
		g_rc_function[RC_Channel_aux::k_mount_roll]->closest_limit(roll_angle/10);
	if (g_rc_function[RC_Channel_aux::k_mount_pitch])
		g_rc_function[RC_Channel_aux::k_mount_pitch]->closest_limit(pitch_angle/10);
	if (g_rc_function[RC_Channel_aux::k_mount_yaw])
		g_rc_function[RC_Channel_aux::k_mount_yaw]->closest_limit(yaw_angle/10);
}	

void AP_Mount::set_mode(MountMode mode)
{
	_mount_mode=mode;
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
