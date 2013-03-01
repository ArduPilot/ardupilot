/*
	APM_AHRS.cpp

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public License
	as published by the Free Software Foundation; either version 2.1
	of the License, or (at your option) any later version.
*/
#include <AP_AHRS.h>
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS::var_info[] PROGMEM = {
	// index 0 and 1 are for old parameters that are no longer used

    // @Param: GPS_GAIN
    // @DisplayName: AHRS GPS gain
    // @Description: This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
    // @Range: 0.0 1.0
    // @Increment: .01
    AP_GROUPINFO("GPS_GAIN",  2, AP_AHRS, gps_gain, 1.0f),

    // @Param: GPS_USE
    // @DisplayName: AHRS use GPS for navigation
    // @Description: This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight.
    // @User: Advanced
    AP_GROUPINFO("GPS_USE",  3, AP_AHRS, _gps_use, 1),

    // @Param: YAW_P
    // @DisplayName: Yaw P
    // @Description: This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.
    // @Range: 0.1 0.4
    // @Increment: .01
    AP_GROUPINFO("YAW_P", 4,    AP_AHRS, _kp_yaw, 0.4f),

    // @Param: RP_P
    // @DisplayName: AHRS RP_P
    // @Description: This controls how fast the accelerometers correct the attitude
    // @Range: 0.1 0.4
    // @Increment: .01
    AP_GROUPINFO("RP_P",  5,    AP_AHRS, _kp, 0.4f),

    // @Param: WIND_MAX
    // @DisplayName: Maximum wind
    // @Description: This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is.
    // @Range: 0 127
    // @Units: m/s
    // @Increment: 1
    AP_GROUPINFO("WIND_MAX",  6,    AP_AHRS, _wind_max, 0.0f),

    // @Param: BARO_USE
    // @DisplayName: AHRS Use Barometer
    // @Description: This controls the use of the barometer for vertical acceleration compensation in AHRS. It is currently recommended that you set this value to zero unless you are a developer experimenting with the AHRS system.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("BARO_USE",  7,    AP_AHRS, _baro_use, 0),

    // @Param: TRIM_X
    // @DisplayName: AHRS Trim Roll
    // @Description: Compensates for the roll angle difference between the control board and the frame
    // @Units: Radians
    // @User: Advanced

    // @Param: TRIM_Y
    // @DisplayName: AHRS Trim Pitch
    // @Description: Compensates for the pitch angle difference between the control board and the frame
    // @Units: Radians
    // @User: Advanced

    // @Param: TRIM_Z
    // @DisplayName: AHRS Trim Yaw
    // @Description: Not Used
    // @Units: Radians
    // @User: Advanced
    AP_GROUPINFO("TRIM", 8, AP_AHRS, _trim, 0),

    // @Param: ORIENTATION
    // @DisplayName: Board Orientation
    // @Description: Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. This option takes affect on next boot. After changing you will need to re-level your vehicle.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw135,19:Roll270,20:Roll270Yaw45,21:Roll270Yaw90,22:Roll270Yaw136,23:Pitch90,24:Pitch270
    // @User: Advanced
    AP_GROUPINFO("ORIENTATION", 9, AP_AHRS, _board_orientation, 0),

    AP_GROUPEND
};

// get pitch rate in earth frame, in radians/s
float AP_AHRS::get_pitch_rate_earth(void) 
{
	Vector3f omega = get_gyro();
	return cosf(roll) * omega.y - sinf(roll) * omega.z;
}

// get roll rate in earth frame, in radians/s
float AP_AHRS::get_roll_rate_earth(void)  {
	Vector3f omega = get_gyro();
	return omega.x + tanf(pitch)*(omega.y*sinf(roll) + omega.z*cosf(roll));
}

// return airspeed estimate if available
bool AP_AHRS::airspeed_estimate(float *airspeed_ret)
{
	if (_airspeed && _airspeed->use()) {
		*airspeed_ret = _airspeed->get_airspeed();
		if (_wind_max > 0 && _gps && _gps->status() == GPS::GPS_OK) {
			// constrain the airspeed by the ground speed
			// and AHRS_WIND_MAX
			*airspeed_ret = constrain(*airspeed_ret, 
						  _gps->ground_speed*0.01f - _wind_max, 
						  _gps->ground_speed*0.01f + _wind_max);
		}
		return true;
	}
	return false;
}

// set_trim
void AP_AHRS::set_trim(Vector3f new_trim)
{
    Vector3f trim;
    trim.x = constrain(new_trim.x, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain(new_trim.y, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    _trim.set_and_save(trim);
}

// add_trim - adjust the roll and pitch trim up to a total of 10 degrees
void AP_AHRS::add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom)
{
    Vector3f trim = _trim.get();

    // add new trim
    trim.x = constrain(trim.x + roll_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain(trim.y + pitch_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));

    // set new trim values
    _trim.set(trim);

    // save to eeprom
    if( save_to_eeprom ) {
        _trim.save();
    }
}
