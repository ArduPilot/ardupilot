/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

void init_camera()
{
	g.rc_camera_pitch.set_angle(4500);
	g.rc_camera_pitch.radio_min 	= 1000;
	g.rc_camera_pitch.radio_trim 	= 1500;
	g.rc_camera_pitch.radio_max 	= 2000;


	g.rc_camera_roll.set_angle(4500);
	g.rc_camera_roll.radio_min 		= 1000;
	g.rc_camera_roll.radio_trim 	= 1500;
	g.rc_camera_roll.radio_max 		= 2000;
}

void
camera_stabilization()
{
	g.rc_camera_pitch.set_pwm(APM_RC.InputCh(CH_6)); // I'm using CH 6 input here.

	// allow control mixing
	g.rc_camera_pitch.servo_out = g.rc_camera_pitch.control_mix(-dcm.pitch_sensor);

	// dont allow control mixing
	//g.rc_camera_pitch.servo_out = dcm.pitch_sensor  * -1;
	g.rc_camera_pitch.calc_pwm();
	APM_RC.OutputCh(CH_5, g.rc_camera_pitch.radio_out);

	g.rc_camera_roll.servo_out = -dcm.roll_sensor;
	g.rc_camera_roll.calc_pwm();
	APM_RC.OutputCh(CH_6, g.rc_camera_roll.radio_out);


	//If you want to do control mixing use this function.
	// set servo_out to the control input from radio
	//rc_camera_roll 	= g.rc_2.control_mix(dcm.pitch_sensor);
	//rc_camera_roll.calc_pwm();
}

