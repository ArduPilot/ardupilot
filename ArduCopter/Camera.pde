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
        int32_t p_sensor_value = g.camera_pitch_continuous ? (ahrs.get_gyro().y * 100) : ahrs.pitch_sensor;
        int32_t r_sensor_value = g.camera_roll_continuous ? (ahrs.get_gyro().x * 100) : ahrs.roll_sensor;

	// PITCH
	// -----
	// Allow user to control camera pitch with channel 6 (mixed with pitch DCM)
	if(g.radio_tuning == 0) {
		g.rc_camera_pitch.set_pwm(g.rc_6.radio_in);
		g.rc_camera_pitch.servo_out = g.rc_camera_pitch.control_mix(p_sensor_value);
	}else{
		// unless channel 6 is already being used for tuning
	    g.rc_camera_pitch.servo_out = p_sensor_value  * -1;
	}
	g.rc_camera_pitch.servo_out	= (float)g.rc_camera_pitch.servo_out * g.camera_pitch_gain;
	
	// limit
	//g.rc_camera_pitch.servo_out = constrain(g.rc_camera_pitch.servo_out, -4500, 4500);


	// ROLL
	// -----
	// allow control mixing
	/*
	g.rc_camera_roll.set_pwm(APM_RC.InputCh(CH_6)); // I'm using CH 6 input here.
	g.rc_camera_roll.servo_out = g.rc_camera_roll.control_mix(-ahrs.roll_sensor);
	*/

	// dont allow control mixing
	g.rc_camera_roll.servo_out	= (float)-r_sensor_value * g.camera_roll_gain;

	// limit
	//g.rc_camera_roll.servo_out = constrain(-ahrs.roll_sensor, -4500, 4500);

	// Output
	// ------
	g.rc_camera_pitch.calc_pwm();
	g.rc_camera_roll.calc_pwm();

	APM_RC.OutputCh(CH_CAM_PITCH, g.rc_camera_pitch.radio_out);
	APM_RC.OutputCh(CH_CAM_ROLL , g.rc_camera_roll.radio_out);
	//Serial.printf("c:%d\n",  g.rc_camera_pitch.radio_out);
}

//#endif
