// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

/*
* control_manual.cpp - manual control mode
*/

/*
* update_manual - runs the manual controller
*  called at 50hz while control_mode is 'MANUAL'
*/
void Tracker::update_manual(void)
{
	//Check to limit CR Pitch Servo to prevent hitting limits if needed 
	if ((enum PitchServoType)g.pitch_servo_type.get() == PITCH_SERVO_TYPE_CR)
	{
		int16_t radio_out = limit_cr_pitch_pwm( constrain_int16(g.channel_pitch.radio_in, g.channel_pitch.radio_min, g.channel_pitch.radio_max));

		//Check for Radio center to lock angle
		//Check to see if we should lock the pitch angle
		check_pitch_lock(radio_out);

		if (pitch_lock)
		{
			float pitch = pitch_lock_angle + g.pitch_trim;
			update_pitch_servo(pitch);
		}
		else
		{
			g.channel_pitch.radio_out =  radio_out;
			g.channel_pitch.enable_out();
			g.channel_pitch.output();
		}
	}
	else
	{
		g.channel_pitch.radio_out = constrain_int16(g.channel_pitch.radio_in, g.channel_pitch.radio_min, g.channel_pitch.radio_max);
		g.channel_pitch.enable_out();
		g.channel_pitch.output();
	}

	// copy yaw input to output
	g.channel_yaw.radio_out = constrain_int16(g.channel_yaw.radio_in, g.channel_yaw.radio_min, g.channel_yaw.radio_max);
	//enable out to overcome servo stop not arming again
	g.channel_yaw.enable_out();
	// send output to servos
	g.channel_yaw.output();
}

/*
*  Called to determine if Manual CR Pitch has hit range limits
*/
int16_t Tracker::limit_cr_pitch_pwm(int16_t pitch_pwm)
{
	//Check measured pitch angle of ahrs
	float ahrs_pitch = degrees(ahrs.pitch);
	//float pitch_range = g.pitch_range/2;	

	if (ahrs_pitch >= g.cr_pitch_max) //MAX LIMIT
	{
		//Check if servo is reversed and limit servo in one direction only
		if (!g.channel_pitch.get_reverse() && pitch_pwm < g.channel_pitch.radio_trim) //Normal Servo Direction
			return pitch_pwm;
		else if (g.channel_pitch.get_reverse() && pitch_pwm > g.channel_pitch.radio_trim) //Reversed Servo Direction
			return pitch_pwm;
		else
			return g.channel_pitch.radio_trim; //Stop if not in a direction to clear violation
	}
	else if (ahrs_pitch <= g.cr_pitch_min) //MIN LIMIT
	{
		//Check if servo is reversed and limit servo in one direction only
		if (!g.channel_pitch.get_reverse() && pitch_pwm > g.channel_pitch.radio_trim) //Normal Servo Direction
			return pitch_pwm;
		else if (g.channel_pitch.get_reverse() && pitch_pwm < g.channel_pitch.radio_trim) //Reversed Servo Direction
			return pitch_pwm;
		else
			return g.channel_pitch.radio_trim; //Stop if not in a direction to clear violation
	}
	else
	{
		return pitch_pwm; //Not in pitch violation
	}
}

/*
* update_manual_angle - runs the manual controller in angle mode
* called at 50hz while control_mode is 'MANUAL'
* 
* This function converts RC inputs into nav_pitch and nav_bearing
*/
void Tracker::update_manual_angle(void)
{
	int yaw_in; // fixme: zmienic typ danych
	int pitch_in;

	pitch_in = constrain_int16(g.channel_pitch.radio_in, g.channel_pitch.radio_min, g.channel_pitch.radio_max);
	yaw_in = constrain_int16(g.channel_yaw.radio_in, g.channel_yaw.radio_min, g.channel_yaw.radio_max);

	// rc input to angle conversion
	nav_status.bearing = (float)(yaw_in - g.channel_yaw.radio_min)/(float)(g.channel_yaw.radio_max - g.channel_yaw.radio_min) * 360.0;
	nav_status.pitch = ((float)(pitch_in - g.channel_pitch.radio_min)/(float)(g.channel_pitch.radio_max - g.channel_pitch.radio_min) * 180.0) - 90.0;

	float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100) * 0.01f;
	//float yaw = nav_status.bearing
	//Constrain pitch to range limits
	//float pitch = constrain_float(nav_status.pitch+g.pitch_trim, -90, 90);
	float pitch = constrain_float(nav_status.pitch+g.pitch_trim,-(g.pitch_range/2),g.pitch_range/2);

	update_pitch_servo(pitch);
	update_yaw_servo(yaw);

}

