// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <APM_RC.h>
#include "RC_Channel_aux.h"

const AP_Param::GroupInfo RC_Channel_aux::var_info[] PROGMEM = {
	AP_NESTEDGROUPINFO(RC_Channel, 0),

	// @Param: FUNCTION
	// @DisplayName: APM servo output function
	// @Description: Setting this to Disabled(0) will disable this output, any other value will enable the corresponding function
	// @Values: 0:Disabled,1:Manual,2:Flap,3:Flap_auto,4:Aileron,5:flaperon,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release
	// @User: Standard
	AP_GROUPINFO("FUNCTION",       1, RC_Channel_aux, function),

	// @Param: ANGLE_MIN
	// @DisplayName: Minimum object position
	// @Description: Minimum physical angular position of the object that this servo output controls, this could be for example a camera pan angle, an aileron angle, etc
	// @Units: Degrees
	// @Range: -180 180
	// @Increment: .01
	// @User: Standard
	AP_GROUPINFO("ANGLE_MIN",      2, RC_Channel_aux, angle_min),

	// @Param: ANGLE_MAX
	// @DisplayName: Maximum object position
	// @Description: Maximum physical angular position of the object that this servo output controls, this could be for example a camera pan angle, an aileron angle, etc
	// @Units: Degrees
	// @Range: -180 180
	// @Increment: .01
	// @User: Standard
	AP_GROUPINFO("ANGLE_MAX",      3, RC_Channel_aux, angle_max),
	AP_GROUPEND
};

/// Global pointer array, indexed by a "RC function enum" and points to the RC channel output assigned to that function/operation
RC_Channel_aux* g_rc_function[RC_Channel_aux::k_nr_aux_servo_functions];

/// saturate to the closest angle limit if outside of min max angle interval
int16_t
RC_Channel_aux::closest_limit(int16_t angle)
{
	// Change scaling to 0.1 degrees in order to avoid overflows in the angle arithmetic
	int16_t min = angle_min / 10;
	int16_t max = angle_max / 10;

	// Make sure the angle lies in the interval [-180 .. 180[ degrees
	while (angle < -1800) angle += 3600;
	while (angle >= 1800) angle -= 3600;

	// Make sure the angle limits lie in the interval [-180 .. 180[ degrees
	while (min < -1800) min += 3600;
	while (min >= 1800) min -= 3600;
	while (max < -1800) max += 3600;
	while (max >= 1800) max -= 3600;
	// This is done every time because the user might change the min, max values on the fly
	set_range(min, max);

	// If the angle is outside servo limits, saturate the angle to the closest limit
	// On a circle the closest angular position must be carefully calculated to account for wrap-around
	if ((angle < min) && (angle > max)){
		// angle error if min limit is used
		int16_t err_min = min - angle + (angle<min?0:3600); // add 360 degrees if on the "wrong side"
		// angle error if max limit is used
		int16_t err_max = angle - max + (angle>max?0:3600); // add 360 degrees if on the "wrong side"
		angle = err_min<err_max?min:max;
	}

	servo_out = angle;
	// convert angle to PWM using a linear transformation (ignores trimming because the camera limits might not be symmetric)
	calc_pwm();

	return angle;
}

/// Gets the RC and integrates and then compares with the servo out angles to limit control input to servo travel.
/// That way the user doesn't get lost. Rotationally.
void
RC_Channel_aux::rc_input(float *control_angle, int16_t angle)
{
	if((radio_in < 1480 && angle < angle_max)||(radio_in > 1520 && angle > angle_min)){
		*control_angle += ( 1500 - radio_in ) * .0001; // .0001 is the control speed scaler.
	}
}

/// Takes the desired servo angle(deg) and converts to microSeconds for PWM
/// Like this: 45 deg = 2000 us ; -45 deg/1000 us. 1000us/(90*100 deg) = 0.1111111111111
void
RC_Channel_aux::angle_out(int16_t angle)
{
	if(angle >= angle_max){
		angle = angle_max;
	}
	if(angle <= angle_min){
		angle = angle_min;
	}
	// Convert the angle*100 to pwm microseconds. 45 deg = 500 us.
	radio_out = (/*_reverse * */ angle * 0.1111111) + 1500;
}

/// map a function to a servo channel and output it
void
RC_Channel_aux::output_ch(unsigned char ch_nr)
{
	// take care or two corner cases
	switch(function)
	{
	case k_none: 		// disabled
		return;
		break;
	case k_manual:		// manual
		radio_out = radio_in;
		break;
	}

	_apm_rc->OutputCh(ch_nr, radio_out);
}

/// Update the g_rc_function array of pointers to rc_x channels
/// This is to be done before rc_init so that the channels get correctly initialized.
/// It also should be called periodically because the user might change the configuration and
/// expects the changes to take effect instantly
void update_aux_servo_function(RC_Channel_aux* rc_5, RC_Channel_aux* rc_6, RC_Channel_aux* rc_7, RC_Channel_aux* rc_8)
{
	// positions 0..3 of this array never get used, but this is a stack array, so the entire array gets freed at the end of the function
	RC_Channel_aux::Aux_servo_function_t aux_servo_function[4];
	aux_servo_function[0] = (RC_Channel_aux::Aux_servo_function_t)rc_5->function.get();
	aux_servo_function[1] = (RC_Channel_aux::Aux_servo_function_t)rc_6->function.get();
	aux_servo_function[2] = (RC_Channel_aux::Aux_servo_function_t)rc_7->function.get();
	aux_servo_function[3] = (RC_Channel_aux::Aux_servo_function_t)rc_8->function.get();

	for (uint8_t i = 0; i < 4; i++) {
		if (aux_servo_function[i] >= RC_Channel_aux::k_nr_aux_servo_functions) {
			// invalid setting
			aux_servo_function[i] = RC_Channel_aux::k_none;
		}
	}

	// Assume that no auxiliary function is used
	for (uint8_t i = 0; i < RC_Channel_aux::k_nr_aux_servo_functions ; i++)
	{
		g_rc_function[i] = NULL;
	}

	// assign the RC channel to each function
	g_rc_function[aux_servo_function[0]] = rc_5;
	g_rc_function[aux_servo_function[1]] = rc_6;
	g_rc_function[aux_servo_function[2]] = rc_7;
	g_rc_function[aux_servo_function[3]] = rc_8;

	//set auxiliary ranges
	G_RC_AUX(k_flap)->set_range(0,100);
	G_RC_AUX(k_flap_auto)->set_range(0,100);
	G_RC_AUX(k_aileron)->set_angle(4500);
	G_RC_AUX(k_flaperon)->set_range(0,100);
	G_RC_AUX(k_mount_yaw)->set_range(
				g_rc_function[RC_Channel_aux::k_mount_yaw]->angle_min / 10,
				g_rc_function[RC_Channel_aux::k_mount_yaw]->angle_max / 10);
	G_RC_AUX(k_mount_pitch)->set_range(
				g_rc_function[RC_Channel_aux::k_mount_pitch]->angle_min / 10,
				g_rc_function[RC_Channel_aux::k_mount_pitch]->angle_max / 10);
	G_RC_AUX(k_mount_roll)->set_range(
				g_rc_function[RC_Channel_aux::k_mount_roll]->angle_min / 10,
				g_rc_function[RC_Channel_aux::k_mount_roll]->angle_max / 10);
	G_RC_AUX(k_mount_open)->set_range(0,100);
	G_RC_AUX(k_cam_trigger)->set_range(
				g_rc_function[RC_Channel_aux::k_cam_trigger]->angle_min / 10,
				g_rc_function[RC_Channel_aux::k_cam_trigger]->angle_max / 10);
	G_RC_AUX(k_egg_drop)->set_range(0,100);
}
