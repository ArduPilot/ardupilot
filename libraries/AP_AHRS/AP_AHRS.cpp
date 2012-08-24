/*
	APM_AHRS.cpp

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public License
	as published by the Free Software Foundation; either version 2.1
	of the License, or (at your option) any later version.
*/
#include <FastSerial.h>
#include <AP_AHRS.h>


// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS::var_info[] PROGMEM = {
	// index 0 and 1 are for old parameters that are no longer used

    // @Param: GPS_GAIN
    // @DisplayName: AHRS GPS gain
    // @Description: This controls how how much to use the GPS to correct the attitude
    // @Range: 0.0 1.0
    // @Increment: .01
    AP_GROUPINFO("GPS_GAIN",  2, AP_AHRS, gps_gain, 1.0),

    // @Param: GPS_USE
    // @DisplayName: enable/disable use of GPS for position estimation
    // @Description: This controls how how much to use the GPS to correct the attitude. This is for testing the dead-reckoning code
    // @User: Advanced
    AP_GROUPINFO("GPS_USE",  3, AP_AHRS, _gps_use, 1),

    // @Param: YAW_P
    // @DisplayName: Yaw P
    // @Description: This controls the weight the compass has on the overall heading
    // @Range: 0 .4
    // @Increment: .01
    AP_GROUPINFO("YAW_P", 4,    AP_AHRS, _kp_yaw, 0.4),

    // @Param: RP_P
    // @DisplayName: AHRS RP_P
    // @Description: This controls how fast the accelerometers correct the attitude
    // @Range: 0 .4
    // @Increment: .01
    AP_GROUPINFO("RP_P",  5,    AP_AHRS, _kp, 0.4),

    AP_GROUPEND
};

// get pitch rate in earth frame, in radians/s
float AP_AHRS::get_pitch_rate_earth(void) 
{
	Vector3f omega = get_gyro();
	return cos(roll) * omega.y - sin(roll) * omega.z;
}

// get roll rate in earth frame, in radians/s
float AP_AHRS::get_roll_rate_earth(void)  {
	Vector3f omega = get_gyro();
	return omega.x + tan(pitch)*(omega.y*sin(roll) + omega.z*cos(roll));
}

// return airspeed estimate if available
bool AP_AHRS::airspeed_estimate(float *airspeed_ret)
{
	if (_airspeed && _airspeed->use()) {
		*airspeed_ret = _airspeed->get_airspeed();
		return true;
	}
	return false;
}
