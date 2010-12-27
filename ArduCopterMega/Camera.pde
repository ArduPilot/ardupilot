void init_camera()
{
	rc_camera_pitch.set_angle(4500);
	rc_camera_pitch.radio_min 	= 1000;
	rc_camera_pitch.radio_trim 	= 1500;
	rc_camera_pitch.radio_max 	= 2000;
	
	rc_camera_yaw.set_angle(4500);
	rc_camera_yaw.radio_min 	= 1000;
	rc_camera_yaw.radio_trim 	= 1500;
	rc_camera_yaw.radio_max 	= 2000;
}

void
camera_stabilization()
{

		rc_camera_pitch.servo_out = pitch_sensor;
		//rc_camera_pitch.servo_out;
		rc_camera_pitch.calc_pwm();
		
		Serial.print(rc_camera_pitch.radio_min,DEC);
		Serial.print(" ");
		Serial.print(rc_camera_pitch.radio_trim,DEC);
		Serial.print(" ");
		Serial.print(rc_camera_pitch.radio_max,DEC);
		Serial.print(" ");
		Serial.print(rc_camera_pitch.servo_out,DEC);
		Serial.print(" ");
		Serial.print(rc_camera_pitch.angle_to_pwm(),DEC);
		Serial.print(" ");
		Serial.println(rc_camera_pitch.radio_out,DEC);

		APM_RC.OutputCh(CH_5,rc_camera_pitch.radio_out);

		//If you want to do control mixing use this function.
		// set servo_out to the control input from radio
		//rc_camera_yaw 	= rc_2.control_mix(pitch_sensor);
		//rc_camera_yaw.calc_pwm();		
}

