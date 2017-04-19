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
#include "AP_AHRS.h"
#include "AP_AHRS_View.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS::var_info[] = {
    // index 0 and 1 are for old parameters that are no longer not used

    // @Param: GPS_GAIN
    // @DisplayName: AHRS GPS gain
    // @Description: This controls how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
    // @Range: 0.0 1.0
    // @Increment: .01
    // @User: Advanced
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
    // @User: Advanced
    AP_GROUPINFO("YAW_P", 4,    AP_AHRS, _kp_yaw, 0.2f),

    // @Param: RP_P
    // @DisplayName: AHRS RP_P
    // @Description: This controls how fast the accelerometers correct the attitude
    // @Range: 0.1 0.4
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("RP_P",  5,    AP_AHRS, _kp, 0.2f),

    // @Param: WIND_MAX
    // @DisplayName: Maximum wind
    // @Description: This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is.
    // @Range: 0 127
    // @Units: m/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("WIND_MAX",  6,    AP_AHRS, _wind_max, 0.0f),

    // NOTE: 7 was BARO_USE

    // @Param: TRIM_X
    // @DisplayName: AHRS Trim Roll
    // @Description: Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
    // @Units: Radians
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Standard

    // @Param: TRIM_Y
    // @DisplayName: AHRS Trim Pitch
    // @Description: Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
    // @Units: Radians
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Standard

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
    // @DisplayName: AHRS Velocity Complementary Filter Beta Coefficient
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

    // 13 was the old EKF_USE

#if AP_AHRS_NAVEKF_AVAILABLE
    // @Param: EKF_TYPE
    // @DisplayName: Use NavEKF Kalman filter for attitude and position estimation
    // @Description: This controls which NavEKF Kalman filter version is used for attitude and position estimation
    // @Values: 0:Disabled,2:Enable EKF2,3:Enable EKF3
    // @User: Advanced
    AP_GROUPINFO("EKF_TYPE",  14, AP_AHRS, _ekf_type, 2),
#endif

    AP_GROUPEND
};

// return a smoothed and corrected gyro vector using the latest ins data (which may not have been consumed by the EKF yet)
Vector3f AP_AHRS::get_gyro_latest(void) const
{
    uint8_t primary_gyro = get_primary_gyro_index();
    return get_ins().get_gyro(primary_gyro) + get_gyro_drift();
}

// return airspeed estimate if available
bool AP_AHRS::airspeed_estimate(float *airspeed_ret) const
{
    if (airspeed_sensor_enabled()) {
        *airspeed_ret = _airspeed->get_airspeed();
        if (_wind_max > 0 && _gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
            // constrain the airspeed by the ground speed
            // and AHRS_WIND_MAX
            float gnd_speed = _gps.ground_speed();
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
    bool gotGPS = (_gps.status() >= AP_GPS::GPS_OK_FIX_2D);
    if (gotAirspeed) {
        Vector3f wind = wind_estimate();
        Vector2f wind2d = Vector2f(wind.x, wind.y);
        Vector2f airspeed_vector = Vector2f(cosf(yaw), sinf(yaw)) * airspeed;
        gndVelADS = airspeed_vector - wind2d;
    }

    // Generate estimate of ground speed vector using GPS
    if (gotGPS) {
        float cog = radians(_gps.ground_course_cd()*0.01f);
        gndVelGPS = Vector2f(cosf(cog), sinf(cog)) * _gps.ground_speed();
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

/*
  calculate sin and cos of roll/pitch/yaw from a body_to_ned rotation matrix
 */
void AP_AHRS::calc_trig(const Matrix3f &rot,
                        float &cr, float &cp, float &cy,
                        float &sr, float &sp, float &sy) const
{
    Vector2f yaw_vector;

    yaw_vector.x = rot.a.x;
    yaw_vector.y = rot.b.x;
    if (fabsf(yaw_vector.x) > 0 ||
        fabsf(yaw_vector.y) > 0) {
        yaw_vector.normalize();
    }
    sy = constrain_float(yaw_vector.y, -1.0, 1.0);
    cy = constrain_float(yaw_vector.x, -1.0, 1.0);

    // sanity checks
    if (yaw_vector.is_inf() || yaw_vector.is_nan()) {
        sy = 0.0f;
        cy = 1.0f;
    }
    
    float cx2 = rot.c.x * rot.c.x;
    if (cx2 >= 1.0f) {
        cp = 0;
        cr = 1.0f;
    } else {
        cp = safe_sqrt(1 - cx2);
        cr = rot.c.z / cp;
    }
    cp = constrain_float(cp, 0, 1.0);
    cr = constrain_float(cr, -1.0, 1.0); // this relies on constrain_float() of infinity doing the right thing

    sp = -rot.c.x;

    if (!is_zero(cp)) {
        sr = rot.c.y / cp;
    }

    if (is_zero(cp) || isinf(cr) || isnan(cr) || isinf(sr) || isnan(sr)) {
        float r, p, y;
        rot.to_euler(&r, &p, &y);
        cr = cosf(r);
        sr = sinf(r);
    }
}

// update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
//      should be called after _dcm_matrix is updated
void AP_AHRS::update_trig(void)
{
    if (_last_trim != _trim.get()) {
        _last_trim = _trim.get();
        _rotation_autopilot_body_to_vehicle_body.from_euler(_last_trim.x, _last_trim.y, 0.0f);
        _rotation_vehicle_body_to_autopilot_body = _rotation_autopilot_body_to_vehicle_body.transposed();
    }

    calc_trig(get_rotation_body_to_ned(),
              _cos_roll, _cos_pitch, _cos_yaw,
              _sin_roll, _sin_pitch, _sin_yaw);
}

/*
  update the centi-degree values
 */
void AP_AHRS::update_cd_values(void)
{
    roll_sensor  = degrees(roll) * 100;
    pitch_sensor = degrees(pitch) * 100;
    yaw_sensor   = degrees(yaw) * 100;
    if (yaw_sensor < 0)
        yaw_sensor += 36000;
}

/*
  create a rotated view of AP_AHRS
 */
AP_AHRS_View *AP_AHRS::create_view(enum Rotation rotation)
{
    if (_view != nullptr) {
        // can only have one
        return nullptr;
    }
    _view = new AP_AHRS_View(*this, rotation);
    return _view;
}

/*
 * Update AOA and SSA estimation based on airspeed, velocity vector and wind vector
 *
 * Based on:
 * "On estimation of wind velocity, angle-of-attack and sideslip angle of small UAVs using standard sensors" by
 * Tor A. Johansen, Andrea Cristofaro, Kim Sorensen, Jakob M. Hansen, Thor I. Fossen
 *
 * "Multi-Stage Fusion Algorithm for Estimation of Aerodynamic Angles in Mini Aerial Vehicle" by
 * C.Ramprasadh and Hemendra Arya
 *
 * "ANGLE OF ATTACK AND SIDESLIP ESTIMATION USING AN INERTIAL REFERENCE PLATFORM" by
 * JOSEPH E. ZEIS, JR., CAPTAIN, USAF
 */
void AP_AHRS::update_AOA_SSA(void)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    uint32_t now = AP_HAL::millis();
    if (now - _last_AOA_update_ms < 50) {
        // don't update at more than 20Hz
        return;
    }
    _last_AOA_update_ms = now;
    
    Vector3f aoa_velocity, aoa_wind;

    // get velocity and wind
    if (get_velocity_NED(aoa_velocity) == false) {
        return;
    }

    aoa_wind = wind_estimate();

    // Rotate vectors to the body frame and calculate velocity and wind
    const Matrix3f &rot = get_rotation_body_to_ned();
    aoa_velocity = rot.mul_transpose(aoa_velocity);
    aoa_wind = rot.mul_transpose(aoa_wind);

    // calculate relative velocity in body coordinates
    aoa_velocity = aoa_velocity - aoa_wind;
    float vel_len = aoa_velocity.length();

    // do not calculate if speed is too low
    if (vel_len < 2.0) {
        _AOA = 0;
        _SSA = 0;
        return;
    }

    // Calculate AOA and SSA
    if (aoa_velocity.x > 0) {
        _AOA = degrees(atanf(aoa_velocity.z / aoa_velocity.x));
    } else {
        _AOA = 0;
    }

    _SSA = degrees(safe_asin(aoa_velocity.y / vel_len));
#endif
}

// return current AOA
float AP_AHRS::getAOA(void)
{
    update_AOA_SSA();
    return _AOA;
}

// return calculated SSA
float AP_AHRS::getSSA(void)
{
    update_AOA_SSA();
    return _SSA;
}
