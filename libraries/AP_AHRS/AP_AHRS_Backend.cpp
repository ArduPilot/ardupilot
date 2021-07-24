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
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_NMEA_Output/AP_NMEA_Output.h>

extern const AP_HAL::HAL& hal;

// init sets up INS board orientation
void AP_AHRS_Backend::init()
{
    update_orientation();

#if !HAL_MINIMIZE_FEATURES
    _nmea_out = AP_NMEA_Output::probe();
#endif
}

// return a smoothed and corrected gyro vector using the latest ins data (which may not have been consumed by the EKF yet)
Vector3f AP_AHRS_Backend::get_gyro_latest(void) const
{
    const uint8_t primary_gyro = get_primary_gyro_index();
    return AP::ins().get_gyro(primary_gyro) + get_gyro_drift();
}

// set_trim
void AP_AHRS_Backend::set_trim(const Vector3f &new_trim)
{
    Vector3f trim;
    trim.x = constrain_float(new_trim.x, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    trim.y = constrain_float(new_trim.y, ToRad(-AP_AHRS_TRIM_LIMIT), ToRad(AP_AHRS_TRIM_LIMIT));
    _trim.set_and_save(trim);
}

// add_trim - adjust the roll and pitch trim up to a total of 10 degrees
void AP_AHRS_Backend::add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom)
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

// Set the board mounting orientation, may be called while disarmed
void AP_AHRS_Backend::update_orientation()
{
    const enum Rotation orientation = (enum Rotation)_board_orientation.get();
    if (orientation != ROTATION_CUSTOM) {
        AP::ins().set_board_orientation(orientation);
        if (_compass != nullptr) {
            _compass->set_board_orientation(orientation);
        }
    } else {
        _custom_rotation.from_euler(radians(_custom_roll), radians(_custom_pitch), radians(_custom_yaw));
        AP::ins().set_board_orientation(orientation, &_custom_rotation);
        if (_compass != nullptr) {
            _compass->set_board_orientation(orientation, &_custom_rotation);
        }
    }
}

// return a ground speed estimate in m/s
Vector2f AP_AHRS_Backend::groundspeed_vector(void)
{
    // Generate estimate of ground speed vector using air data system
    Vector2f gndVelADS;
    Vector2f gndVelGPS;
    float airspeed = 0;
    const bool gotAirspeed = airspeed_estimate_true(airspeed);
    const bool gotGPS = (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D);
    if (gotAirspeed) {
        const Vector3f wind = wind_estimate();
        const Vector2f wind2d(wind.x, wind.y);
        const Vector2f airspeed_vector(_cos_yaw * airspeed, _sin_yaw * airspeed);
        gndVelADS = airspeed_vector + wind2d;
    }

    // Generate estimate of ground speed vector using GPS
    if (gotGPS) {
        const float cog = radians(AP::gps().ground_course());
        gndVelGPS = Vector2f(cosf(cog), sinf(cog)) * AP::gps().ground_speed();
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

    if (airspeed > 0) {
        // we have a rough airspeed, and we have a yaw. For
        // dead-reckoning purposes we can create a estimated
        // groundspeed vector
        Vector2f ret(cosf(yaw), sinf(yaw));
        ret *= airspeed;
        // adjust for estimated wind
        const Vector3f wind = wind_estimate();
        ret.x += wind.x;
        ret.y += wind.y;
        return ret;
    }

    return Vector2f(0.0f, 0.0f);
}

/*
  calculate sin and cos of roll/pitch/yaw from a body_to_ned rotation matrix
 */
void AP_AHRS_Backend::calc_trig(const Matrix3f &rot,
                        float &cr, float &cp, float &cy,
                        float &sr, float &sp, float &sy) const
{
    Vector2f yaw_vector(rot.a.x, rot.b.x);

    if (fabsf(yaw_vector.x) > 0 ||
        fabsf(yaw_vector.y) > 0) {
        yaw_vector.normalize();
    }
    sy = constrain_float(yaw_vector.y, -1.0f, 1.0f);
    cy = constrain_float(yaw_vector.x, -1.0f, 1.0f);

    // sanity checks
    if (yaw_vector.is_inf() || yaw_vector.is_nan()) {
        sy = 0.0f;
        cy = 1.0f;
    }

    const float cx2 = rot.c.x * rot.c.x;
    if (cx2 >= 1.0f) {
        cp = 0;
        cr = 1.0f;
    } else {
        cp = safe_sqrt(1 - cx2);
        cr = rot.c.z / cp;
    }
    cp = constrain_float(cp, 0.0f, 1.0f);
    cr = constrain_float(cr, -1.0f, 1.0f); // this relies on constrain_float() of infinity doing the right thing

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
void AP_AHRS_Backend::update_trig(void)
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
void AP_AHRS_Backend::update_cd_values(void)
{
    roll_sensor  = degrees(roll) * 100;
    pitch_sensor = degrees(pitch) * 100;
    yaw_sensor   = degrees(yaw) * 100;
    if (yaw_sensor < 0)
        yaw_sensor += 36000;
}

/*
  create a rotated view of AP_AHRS with optional pitch trim
 */
AP_AHRS_View *AP_AHRS::create_view(enum Rotation rotation, float pitch_trim_deg)
{
    if (_view != nullptr) {
        // can only have one
        return nullptr;
    }
    _view = new AP_AHRS_View(*this, rotation, pitch_trim_deg);
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
void AP_AHRS_Backend::update_AOA_SSA(void)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    const uint32_t now = AP_HAL::millis();
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
    const float vel_len = aoa_velocity.length();

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
float AP_AHRS_Backend::getAOA(void)
{
    update_AOA_SSA();
    return _AOA;
}

// return calculated SSA
float AP_AHRS_Backend::getSSA(void)
{
    update_AOA_SSA();
    return _SSA;
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_Backend::earth_to_body2D(const Vector2f &ef) const
{
    return Vector2f(ef.x * _cos_yaw + ef.y * _sin_yaw,
                    -ef.x * _sin_yaw + ef.y * _cos_yaw);
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_Backend::body_to_earth2D(const Vector2f &bf) const
{
    return Vector2f(bf.x * _cos_yaw - bf.y * _sin_yaw,
                    bf.x * _sin_yaw + bf.y * _cos_yaw);
}

// log ahrs home and EKF origin
void AP_AHRS_Backend::Log_Write_Home_And_Origin()
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }
    Location ekf_orig;
    if (get_origin(ekf_orig)) {
        Write_Origin(LogOriginType::ekf_origin, ekf_orig);
    }

    if (home_is_set()) {
        Write_Origin(LogOriginType::ahrs_home, _home);
    }
}

// get apparent to true airspeed ratio
float AP_AHRS_Backend::get_EAS2TAS(void) const {
    return AP::baro().get_EAS2TAS();
}

void AP_AHRS_Backend::update_nmea_out()
{
#if !HAL_MINIMIZE_FEATURES
    if (_nmea_out != nullptr) {
        _nmea_out->update();
    }
#endif
}

// return current vibration vector for primary IMU
Vector3f AP_AHRS_Backend::get_vibration(void) const
{
    return AP::ins().get_vibration_levels();
}

void AP_AHRS_Backend::set_takeoff_expected(bool b)
{
    _flags.takeoff_expected = b;
    takeoff_expected_start_ms = AP_HAL::millis();
}

void AP_AHRS_Backend::set_touchdown_expected(bool b)
{
    _flags.touchdown_expected = b;
    touchdown_expected_start_ms = AP_HAL::millis();
}

/*
  update takeoff/touchdown flags
 */
void AP_AHRS_Backend::update_flags(void)
{
    const uint32_t timeout_ms = 1000;
    if (_flags.takeoff_expected && AP_HAL::millis() - takeoff_expected_start_ms > timeout_ms) {
        _flags.takeoff_expected = false;
    }
    if (_flags.touchdown_expected && AP_HAL::millis() - touchdown_expected_start_ms > timeout_ms) {
        _flags.touchdown_expected = false;
    }
}
