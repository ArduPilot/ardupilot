/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CAMERA == ENABLED

void init_camera()
{
	g.rc_camera_pitch.set_angle(4500);			// throw of servo?
	g.rc_camera_pitch.radio_min 		= 1000;		// lowest radio input
	g.rc_camera_pitch.radio_trim 		= 1500;		// middle radio input
	g.rc_camera_pitch.radio_max 		= 2000;		// highest radio input

	g.rc_camera_roll.set_angle(4500);
	g.rc_camera_roll.radio_min 		= 1000;
	g.rc_camera_roll.radio_trim 		= 1500;
	g.rc_camera_roll.radio_max 		= 2000;


	//use test target for now
	camera_target = home;

}

void camera()
{
	//decide what happens to camera depending on camera mode
	switch(camera_mode)
	{
	case 0:
		//do nothing, i.e lock camera in place
		break;
	case 1:
		//stabilize
		target_vector.x=0;		//east to west gives +tive value (i.e. longitude)
		target_vector.y=0;		//south to north gives +tive value (i.e. latitude)
		target_vector.z=100;	//downwards is +tive
		camera_move();
		break;
	case 2:
		//track target
		if(g_gps->fix)
		{
			target_vector=get_location_vector(&current_loc,&camera_target);
			camera_move();
		}
		break;
	}
}

void camera_move()
{
	Matrix3f m = dcm.get_dcm_transposed();
	Vector3<float> targ = m*target_vector;  //to do: find out notion of x y convention

	switch(gimbal_mode)
	{
	case 0:		// pitch & roll
		cam_pitch =  degrees(atan2(-targ.x, targ.z)); //pitch
		cam_roll =   degrees(atan2(targ.y, targ.z));  //roll
		// check_limits(pitch);
		// check_limits(roll);
		// camera_out();
		break;

	case 1:		// pitch and yaw
		cam_tilt = atan2((sqrt(sq(targ.y) + sq(targ.x)) * .01113195), targ.z) * -1;
		cam_pan = 9000 + atan2(-targ.y, targ.x) * 5729.57795;
		if (cam_pan < 0) cam_pan += 36000;
		// check_limits(pitch);
		// check_limits(yaw);
		// camera_out();
		break;

	/* case 2:	// pitch, roll & yaw - not started
		float cam_ritch = 100;
		float cam_yoll = 100;
		float cam_paw = 100;
		break; */
	}
}

/* void check_limits(axis,variable)		// Use servo definitions to calculate for all servo throws - TO DO
{
	// find limits of servo range in deg
	track_pan_right = PAN_CENTER + (PAN_RANGE/2);
	track_pan_left = track_pan_right + (360 - PAN_RANGE);
	if (track_pan_left > 360){
		track_pan_left = track_pan_left - 360;
	}
	// check track_bearing is "safe" - not outside pan servo limits
	// if the bearing lies in the servo dead zone change bearing to closet edge
	if (track_bearing < track_pan_left && track_bearing > track_pan_right){
		track_oor_l = abs(track_bearing - track_pan_left);
		track_oor_r = abs(track_bearing - track_pan_right);
		if (track_oor_r > track_oor_l){
			track_bearing = track_pan_right;
		}
		if (track_oor_l > track_oor_r){
			track_bearing = track_pan_left;
		}
	}
	// center bearing to cam_servo center
	track_pan_deg = track_bearing - PAN_CENTER;
	// make negative is left rotation
	if (track_pan_deg > 180){
		track_pan_deg = (180 - (track_pan_deg - 180)) * -1;
	}

} */

void camera_out()
{
	switch(gimbal_mode)
	{
	case 0:		// pitch & roll
		g.rc_camera_pitch.servo_out = cam_pitch;
		g.rc_camera_pitch.calc_pwm();
		g.rc_camera_roll.servo_out = cam_roll;
		g.rc_camera_roll.calc_pwm();
		break;

	case 1:		// pitch and yaw
		g.rc_camera_pitch.servo_out = cam_tilt;
		g.rc_camera_pitch.calc_pwm();
		g.rc_camera_roll.servo_out = cam_pan;		// borrowing roll servo output for pan/yaw
		g.rc_camera_roll.calc_pwm();
		break;

		/*case 2:		// pitch, roll & yaw
		g.rc_camera_pitch.servo_out = cam_ritch;
		g.rc_camera_pitch.calc_pwm();

		g.rc_camera_roll.servo_out = cam_yoll;
		g.rc_camera_roll.calc_pwm();

		g.rc_camera_yaw.servo_out = cam_paw;		// camera_yaw doesn't exist it should unless we use another channel
		g.rc_camera_yaw.calc_pwm();
	break; */
	}
#if defined PITCH_SERVO
	APM_RC.OutputCh(PITCH_SERVO, g.rc_camera_pitch.radio_out);
#endif
#if defined ROLL_SERVO
	APM_RC.OutputCh(ROLL_SERVO, g.rc_camera_roll.radio_out);
#endif
/*#if defined YAW_SERVO
	APM_RC.OutputCh(YAW_SERVO, g.rc_camera_yaw.radio_out);
#endif */

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
#endif
