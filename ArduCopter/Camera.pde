/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//#if CAMERA_STABILIZER == ENABLED

static void init_camera()
{
	APM_RC.enable_out(CH_CAM_PITCH);
	APM_RC.enable_out(CH_CAM_ROLL);

	// ch 6 high(right) is down.
	g.rc_camera_pitch.set_angle(4500);
	g.rc_camera_roll.set_angle(4500);

	g.rc_camera_roll.set_type(RC_CHANNEL_ANGLE_RAW);
	g.rc_camera_pitch.set_type(RC_CHANNEL_ANGLE_RAW);
}

static void
camera_stabilization()
{
	// PITCH
	// -----
	// Allow user to control camera pitch with channel 6 (mixed with pitch DCM)
	if(g.radio_tuning == 0) {
		g.rc_camera_pitch.set_pwm(APM_RC.InputCh(CH_6));
		g.rc_camera_pitch.servo_out = g.rc_camera_pitch.control_mix(dcm.pitch_sensor);
	}else{
		// unless channel 6 is already being used for tuning
	    g.rc_camera_pitch.servo_out = dcm.pitch_sensor  * -1;
	}
	g.rc_camera_pitch.servo_out	= (float)g.rc_camera_pitch.servo_out * g.camera_pitch_gain;

	// limit
	//g.rc_camera_pitch.servo_out = constrain(g.rc_camera_pitch.servo_out, -4500, 4500);


	// ROLL
	// -----
	// allow control mixing
	/*
	g.rc_camera_roll.set_pwm(APM_RC.InputCh(CH_6)); // I'm using CH 6 input here.
	g.rc_camera_roll.servo_out = g.rc_camera_roll.control_mix(-dcm.roll_sensor);
	*/

	// dont allow control mixing
	g.rc_camera_roll.servo_out	= (float)-dcm.roll_sensor * g.camera_roll_gain;

	// limit
	//g.rc_camera_roll.servo_out = constrain(-dcm.roll_sensor, -4500, 4500);

	// Output
	// ------
	g.rc_camera_pitch.calc_pwm();
	g.rc_camera_roll.calc_pwm();

	APM_RC.OutputCh(CH_CAM_PITCH, g.rc_camera_pitch.radio_out);
	APM_RC.OutputCh(CH_CAM_ROLL , g.rc_camera_roll.radio_out);
	//Serial.printf("c:%d\n",  g.rc_camera_pitch.radio_out);
}

//#endif
