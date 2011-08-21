// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "Camera.h"
#include "../RC_Channel/RC_Channel.h"

void
Camera::move()
{
	Vector3<float>	target_vector(0,0,1);	// x, y, z to target before rotating to planes axis, values are in meters

	//decide what happens to camera depending on camera mode
	switch(mode)
	{
	case 0:
		//do nothing, i.e lock camera in place
		return;
		break;
	case 1:
		//stabilize
		target_vector.x=0;		//east to west gives +tive value (i.e. longitude)
		target_vector.y=0;		//south to north gives +tive value (i.e. latitude)
		target_vector.z=100;	//downwards is +tive
		break;
	case 2:
		//track target
		if(g_gps->fix)
		{
			target_vector=get_location_vector(&current_loc,&camera_target);
		}
		break;
	case 3:		// radio manual control
	case 4: 	// test (level the camera and point north)
		break;  // see code 25 lines bellow
	}

	Matrix3f m = dcm.get_dcm_transposed();
	Vector3<float> targ = m*target_vector;  //to do: find out notion of x y convention
	switch(gimbal_type)
	{
	case 0:		// pitch & roll (tilt & roll)
		cam_pitch =  degrees(atan2(-targ.x, targ.z)); //pitch
		cam_roll =   degrees(atan2(targ.y, targ.z));  //roll
		break;

	case 1:		// yaw & pitch (pan & tilt)
		cam_pitch = atan2((sqrt(sq(targ.y) + sq(targ.x)) * .01113195), targ.z) * -1;
		cam_yaw = 9000 + atan2(-targ.y, targ.x) * 5729.57795;
		break;

/*	case 2:	// pitch, roll & yaw - not started
		cam_ritch = 0;
		cam_yoll = 0;
		cam_paw = 0;
	break; */

	}

	//some camera modes overwrite the gimbal_type calculations
	switch(mode)
	{
	case 3:		// radio manual control
		if (rc_function[CAM_PITCH])
			cam_pitch = map(rc_function[CAM_PITCH]->radio_in,
					rc_function[CAM_PITCH]->radio_min,
					rc_function[CAM_PITCH]->radio_max,
					rc_function[CAM_PITCH]->angle_min,
					rc_function[CAM_PITCH]->radio_max);
		if (rc_function[CAM_ROLL])
			cam_roll = map(rc_function[CAM_ROLL]->radio_in,
					rc_function[CAM_ROLL]->radio_min,
					rc_function[CAM_ROLL]->radio_max,
					rc_function[CAM_ROLL]->angle_min,
					rc_function[CAM_ROLL]->radio_max);
		if (rc_function[CAM_YAW])
			cam_yaw = map(rc_function[CAM_YAW]->radio_in,
					rc_function[CAM_YAW]->radio_min,
					rc_function[CAM_YAW]->radio_max,
					rc_function[CAM_YAW]->angle_min,
					rc_function[CAM_YAW]->radio_max);
		break;
	case 4: 	// test (level the camera and point north)
		cam_pitch = -dcm.pitch_sensor;
		cam_yaw   =  dcm.yaw_sensor;	// do not invert because the servo is mounted upside-down on my system
		// TODO: the "trunk" code can invert using parameters, but this branch still can't
		cam_roll  = -dcm.roll_sensor;
		break;
	}

	#if CAM_DEBUG == ENABLED
	//for debugging purposes
	Serial.println();
	Serial.print("current_loc: lat: ");
	Serial.print(current_loc.lat);
	Serial.print(", lng: ");
	Serial.print(current_loc.lng);
	Serial.print(", alt: ");
	Serial.print(current_loc.alt);
	Serial.println();
	Serial.print("target_loc: lat: ");
	Serial.print(camera_target.lat);
	Serial.print(", lng: ");
	Serial.print(camera_target.lng);
	Serial.print(", alt: ");
	Serial.print(camera_target.alt);
	Serial.print(", distance: ");
	Serial.print(get_distance(&current_loc,&camera_target));
	Serial.print(", bearing: ");
	Serial.print(get_bearing(&current_loc,&camera_target));
	Serial.println();
	Serial.print("dcm_angles: roll: ");
	Serial.print(degrees(dcm.roll));
	Serial.print(", pitch: ");
	Serial.print(degrees(dcm.pitch));
	Serial.print(", yaw: ");
	Serial.print(degrees(dcm.yaw));
	Serial.println();
	Serial.print("target_vector: x: ");
	Serial.print(target_vector.x,2);
	Serial.print(", y: ");
	Serial.print(target_vector.y,2);
	Serial.print(", z: ");
	Serial.print(target_vector.z,2);
	Serial.println();
	Serial.print("rotated_target_vector: x: ");
	Serial.print(targ.x,2);
	Serial.print(", y: ");
	Serial.print(targ.y,2);
	Serial.print(", z: ");
	Serial.print(targ.z,2);
	Serial.println();
	Serial.print("gimbal type 0: roll: ");
	Serial.print(roll);
	Serial.print(", pitch: ");
	Serial.print(pitch);
	Serial.println();
 /* Serial.print("gimbal type 1: pitch: ");
	Serial.print(pan);
	Serial.print(",  roll: ");
	Serial.print(tilt);
	Serial.println(); */
 /* Serial.print("gimbal type 2: pitch: ");
	Serial.print(ritch);
	Serial.print(", roll: ");
	Serial.print(yoll);
	Serial.print(", yaw: ");
	Serial.print(paw);
	Serial.println(); */
#endif
}


void
Camera::set_target(struct Location target)
{
	camera_target = target;
}


void
Camera::update_camera_gimbal_type()
{

	// Auto detect the camera gimbal type depending on the functions assigned to the servos
	if ((rc_function[CAM_YAW] == NULL) && (rc_function[CAM_PITCH] != NULL) && (rc_function[CAM_ROLL] != NULL))
    {
		gimbal_type = 0;
    }
    if ((rc_function[CAM_YAW] != NULL) && (rc_function[CAM_PITCH] != NULL) && (rc_function[CAM_ROLL] == NULL))
    {
    	gimbal_type = 1;
    }
    if ((rc_function[CAM_YAW] != NULL) && (rc_function[CAM_PITCH] != NULL) && (rc_function[CAM_ROLL] != NULL))
    {
    	gimbal_type = 2;
    }
}

void
Camera::servo_pic()		// Servo operated camera
{
	if (rc_function[CAM_TRIGGER])
	{
		cam_trigger = rc_function[CAM_TRIGGER]->radio_max;
		keep_cam_trigg_active_cycles = 2;	// leave a message that it should be active for two event loop cycles
	}
}

void
Camera::relay_pic()		// basic relay activation
{
	relay_on();
	keep_cam_trigg_active_cycles = 2;	// leave a message that it should be active for two event loop cycles
}

void
Camera::throttle_pic()		// pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
{
	g.channel_throttle.radio_out = g.throttle_min;
	if (thr_pic == 10){
		servo_pic();	// triggering method
		thr_pic = 0;
		g.channel_throttle.radio_out = g.throttle_cruise;
	}
	thr_pic++;
}

void
Camera::distance_pic()		// pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
{
	g.channel_throttle.radio_out = g.throttle_min;
	if (wp_distance < 3){
		servo_pic();	// triggering method
		g.channel_throttle.radio_out = g.throttle_cruise;
	}
}

void
Camera::NPN_pic()		// hacked the circuit to run a transistor? use this trigger to send output.
{
	// To Do: Assign pin spare pin for output
	digitalWrite(camtrig, HIGH);
	keep_cam_trigg_active_cycles = 1;	// leave a message that it should be active for two event loop cycles
}

// single entry point to take pictures
void
Camera::trigger_pic()
{
	switch (trigger_type)
	{
	case 0:
		servo_pic();		// Servo operated camera
		break;
	case 1:
		relay_pic();		// basic relay activation
		break;
	case 2:
		throttle_pic();		// pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
		break;
	case 3:
		distance_pic();		// pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
		break;
	case 4:
		NPN_pic();			// hacked the circuit to run a transistor? use this trigger to send output.
		break;
	}
}

// de-activate the trigger after some delay, but without using a delay() function
void
Camera::trigger_pic_cleanup()
{
	if (keep_cam_trigg_active_cycles)
	{
		keep_cam_trigg_active_cycles --;
	}
	else
	{
		switch (trigger_type)
		{
		case 0:
		case 2:
		case 3:
			if (rc_function[CAM_TRIGGER])
			{
				cam_trigger = rc_function[CAM_TRIGGER]->radio_min;
			}
			break;
		case 1:
			relay_off();
			break;
		case 4:
			digitalWrite(camtrig, LOW);
			break;
		}
	}
}
