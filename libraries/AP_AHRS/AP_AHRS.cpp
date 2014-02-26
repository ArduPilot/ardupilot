/*
  APM_AHRS.cpp

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <AP_AHRS.h>
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS::var_info[] PROGMEM = {
	// index 0 and 1 are for old parameters that are no longer not used

    // @Param: GPS_GAIN
    // @DisplayName: AHRS GPS gain
    // @Description: This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
    // @Range: 0.0 1.0
    // @Increment: .01
    AP_GROUPINFO("GPS_GAIN",  2, AP_AHRS, gps_gain, 1.0f),

    // @Param: GPS_USE
    // @DisplayName: AHRS use GPS for navigation
    // @Description: This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("GPS_USE",  3, AP_AHRS, _gps_use, 1),

    // @Param: YAW_P
    // @DisplayName: Yaw P
    // @Description: This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.
    // @Range: 0.1 0.4
    // @Increment: .01
    AP_GROUPINFO("YAW_P", 4,    AP_AHRS, _kp_yaw, 0.2f),

    // @Param: RP_P
    // @DisplayName: AHRS RP_P
    // @Description: This controls how fast the accelerometers correct the attitude
    // @Range: 0.1 0.4
    // @Increment: .01
    AP_GROUPINFO("RP_P",  5,    AP_AHRS, _kp, 0.2f),

    // @Param: WIND_MAX
    // @DisplayName: Maximum wind
    // @Description: This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is.
    // @Range: 0 127
    // @Units: m/s
    // @Increment: 1
    AP_GROUPINFO("WIND_MAX",  6,    AP_AHRS, _wind_max, 0.0f),

    // NOTE: 7 was BARO_USE

    // @Param: TRIM_X
    // @DisplayName: AHRS Trim Roll
    // @Description: Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
    // @Units: Radians
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: User

    // @Param: TRIM_Y
    // @DisplayName: AHRS Trim Pitch
    // @Description: Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
    // @Units: Radians
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: User

    // @Param: TRIM_Z
    // @DisplayName: AHRS Trim Yaw
    // @Description: Not Used
    // @Units: Radians
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("TRIM", 8, AP_AHRS, _trim, 0),

    // @Param: ORIENTATION
    // @DisplayName: Board Orientation
    // @Description: Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. This option takes affect on next boot. After changing you will need to re-level your vehicle.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270
    // @User: Advanced
    AP_GROUPINFO("ORIENTATION", 9, AP_AHRS, _board_orientation, 0),

    // @Param: COMP_BETA
    // @DisplayName: AHRS Velocity Complmentary Filter Beta Coefficient
    // @Description: This controls the time constant for the cross-over frequency used to fuse AHRS (airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.
    // @Range: 0.001 0.5
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("COMP_BETA",  10, AP_AHRS, beta, 0.1f),

    // @Param: GPS_MINSATS
    // @DisplayName: AHRS GPS Minimum satellites
    // @Description: Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_MINSATS", 11, AP_AHRS, _gps_minsats, 6),

    // NOTE: index 12 was for GPS_DELAY, but now removed, fixed delay
    // of 1 was found to be the best choice

#if AP_AHRS_NAVEKF_AVAILABLE
    // @Param: EKF_USE
    // @DisplayName: Use NavEKF Kalman filter for attitude and position estimation
    // @Description: This controls whether the NavEKF Kalman filter is used for attitude and position estimation
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("EKF_USE",  13, AP_AHRS, _ekf_use, 0),
#endif

    AP_GROUPEND
};

// return airspeed estimate if available
bool AP_AHRS::airspeed_estimate(float *airspeed_ret) const
{
	if (_airspeed && _airspeed->use()) {
		*airspeed_ret = _airspeed->get_airspeed();
		if (_wind_max > 0 && _gps && _gps->status() >= GPS::GPS_OK_FIX_2D) {
                    // constrain the airspeed by the ground speed
                    // and AHRS_WIND_MAX
                    float gnd_speed = _gps->ground_speed_cm*0.01f;
                    float true_airspeed = *airspeed_ret * get_EAS2TAS();
                    true_airspeed = constrain_float(true_airspeed,
                                                    gnd_speed - _wind_max, 
                                                    gnd_speed + _wind_max);
                    *airspeed_ret = true_airspeed / get_EAS2TAS();
		}
		return true;
	}
	return false;
}

// set_trim
void AP_AHRS::set_trim(Vector3f new_trim)
{
    Vector3f trim;
    trim.x = constrain_float(new_trim.x, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain_float(new_trim.y, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    _trim.set_and_save(trim);
}

// add_trim - adjust the roll and pitch trim up to a total of 10 degrees
void AP_AHRS::add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom)
{
    Vector3f trim = _trim.get();

    // add new trim
    trim.x = constrain_float(trim.x + roll_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain_float(trim.y + pitch_in_radians, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));

    // set new trim values
    _trim.set(trim);

    // save to eeprom
    if( save_to_eeprom ) {
        _trim.save();
    }
}

// return a ground speed estimate in m/s
Vector2f AP_AHRS::groundspeed_vector(void)
{
    // Generate estimate of ground speed vector using air data system
    Vector2f gndVelADS;
    Vector2f gndVelGPS;
    float airspeed;
    bool gotAirspeed = airspeed_estimate_true(&airspeed);
    bool gotGPS = (_gps && _gps->status() >= GPS::GPS_OK_FIX_2D);
    if (gotAirspeed) {
	    Vector3f wind = wind_estimate();
	    Vector2f wind2d = Vector2f(wind.x, wind.y);
	    Vector2f airspeed_vector = Vector2f(cosf(yaw), sinf(yaw)) * airspeed;
	    gndVelADS = airspeed_vector - wind2d;
    }
    
    // Generate estimate of ground speed vector using GPS
    if (gotGPS) {
	    float cog = radians(_gps->ground_course_cd*0.01f);
	    gndVelGPS = Vector2f(cosf(cog), sinf(cog)) * _gps->ground_speed_cm * 0.01f;
    }
    // If both ADS and GPS data is available, apply a complementary filter
    if (gotAirspeed && gotGPS) {
	    // The LPF is applied to the GPS and the HPF is applied to the air data estimate
	    // before the two are summed
	    //Define filter coefficients
	    // alpha and beta must sum to one
	    // beta = dt/Tau, where
	    // dt = filter time step (0.1 sec if called by nav loop)
	    // Tau = cross-over time constant (nominal 2 seconds)
	    // More lag on GPS requires Tau to be bigger, less lag allows it to be smaller
	    // To-Do - set Tau as a function of GPS lag.
	    const float alpha = 1.0f - beta; 
	    // Run LP filters
	    _lp = gndVelGPS * beta  + _lp * alpha;
	    // Run HP filters
	    _hp = (gndVelADS - _lastGndVelADS) + _hp * alpha;
	    // Save the current ADS ground vector for the next time step
	    _lastGndVelADS = gndVelADS;
	    // Sum the HP and LP filter outputs
	    return _hp + _lp;
    }
    // Only ADS data is available return ADS estimate
    if (gotAirspeed && !gotGPS) {
	    return gndVelADS;
    }
    // Only GPS data is available so return GPS estimate
    if (!gotAirspeed && gotGPS) {
	    return gndVelGPS;
    }
    return Vector2f(0.0f, 0.0f);
}

// update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
//      should be called after _dcm_matrix is updated
void AP_AHRS::update_trig(void)
{
    Vector2f yaw_vector;
    const Matrix3f &temp = get_dcm_matrix();

    // sin_yaw, cos_yaw
    yaw_vector.x = temp.a.x;
    yaw_vector.y = temp.b.x;
    yaw_vector.normalize();
    _sin_yaw = constrain_float(yaw_vector.y, -1.0, 1.0);
    _cos_yaw = constrain_float(yaw_vector.x, -1.0, 1.0);

    // cos_roll, cos_pitch
    _cos_pitch = safe_sqrt(1 - (temp.c.x * temp.c.x));
    _cos_roll = temp.c.z / _cos_pitch;
    _cos_pitch = constrain_float(_cos_pitch, 0, 1.0);
    _cos_roll = constrain_float(_cos_roll, -1.0, 1.0); // this relies on constrain_float() of infinity doing the right thing,which it does do in avr-libc

    // sin_roll, sin_pitch
    _sin_pitch = -temp.c.x;
    _sin_roll = temp.c.y / _cos_pitch;
}
