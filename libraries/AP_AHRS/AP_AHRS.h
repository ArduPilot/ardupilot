/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_AHRS_H__
#define __AP_AHRS_H__
/*
 *  AHRS (Attitude Heading Reference System) interface for ArduPilot
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

#include <AP_Math.h>
#include <inttypes.h>
#include <AP_Compass.h>
#include <AP_Airspeed.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Baro.h>
#include <AP_Param.h>

class AP_AHRS
{
public:
    // Constructor
    AP_AHRS(AP_InertialSensor *ins, GPS *&gps) :
        _ins(ins),
        _gps(gps),
        _barometer(NULL)
    {
        // load default values from var_info table
        AP_Param::setup_object_defaults(this, var_info);

        // base the ki values by the sensors maximum drift
        // rate. The APM2 has gyros which are much less drift
        // prone than the APM1, so we should have a lower ki,
        // which will make us less prone to increasing omegaI
        // incorrectly due to sensor noise
        _gyro_drift_limit = ins->get_gyro_drift_rate();
    }

    // empty init
    virtual void init() {
    };

    // Accessors
    void set_fly_forward(bool b) {
        _fly_forward = b;
    }
    void set_compass(Compass *compass) {
        _compass = compass;
    }
    void set_barometer(AP_Baro *barometer) {
        _barometer = barometer;
    }
    void set_airspeed(AP_Airspeed *airspeed) {
        _airspeed = airspeed;
    }

    AP_InertialSensor* get_ins() {
	    return _ins;
    }

    // accelerometer values in the earth frame in m/s/s
    Vector3f        get_accel_ef(void) { return _accel_ef; }

    // Methods
    virtual void update(void) = 0;

    // Euler angles (radians)
    float roll;
    float pitch;
    float yaw;

    // integer Euler angles (Degrees * 100)
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;

    // roll and pitch rates in earth frame, in radians/s
    float get_pitch_rate_earth(void);
    float get_roll_rate_earth(void);

    // return a smoothed and corrected gyro vector
    virtual Vector3f get_gyro(void) = 0;

    // return the current estimate of the gyro drift
    virtual Vector3f get_gyro_drift(void) = 0;

    // reset the current attitude, used on new IMU calibration
    virtual void reset(bool recover_eulers=false) = 0;

    // how often our attitude representation has gone out of range
    uint8_t renorm_range_count;

    // how often our attitude representation has blown up completely
    uint8_t renorm_blowup_count;

    // return the average size of the roll/pitch error estimate
    // since last call
    virtual float get_error_rp(void) = 0;

    // return the average size of the yaw error estimate
    // since last call
    virtual float get_error_yaw(void) = 0;

    // return a DCM rotation matrix representing our current
    // attitude
    virtual Matrix3f get_dcm_matrix(void) = 0;

    // get our current position, either from GPS or via
    // dead-reckoning. Return true if a position is available,
    // otherwise false. This only updates the lat and lng fields
    // of the Location
    bool get_position(struct Location *loc) {
        if (!_gps || _gps->status() != GPS::GPS_OK) {
            return false;
        }
        loc->lat = _gps->latitude;
        loc->lng = _gps->longitude;
        return true;
    }

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate(void) {
        return Vector3f(0,0,0);
    }

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(float *airspeed_ret);

    // return true if yaw has been initialised
    bool yaw_initialised(void) {
        return _have_initial_yaw;
    }

    // set the fast gains flag
    void set_fast_gains(bool setting) {
        _fast_ground_gains = setting;
    }

    // get strim
    Vector3f                get_trim() { return _trim; }

    // set_trim
    virtual void            set_trim(Vector3f new_trim) { _trim.set_and_save(new_trim); }

    // add_trim - adjust the roll and pitch trim up to a total of 10 degrees
    virtual void            add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);

    // settable parameters
    AP_Float _kp_yaw;
    AP_Float _kp;
    AP_Float gps_gain;
    AP_Int8 _gps_use;
    AP_Int8 _baro_use;
    AP_Int8 _wind_max;

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // whether the yaw value has been intialised with a reference
    bool _have_initial_yaw;

    // pointer to compass object, if available
    Compass         * _compass;

    // pointer to airspeed object, if available
    AP_Airspeed     * _airspeed;

    // time in microseconds of last compass update
    uint32_t _compass_last_update;

    // note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
    //       IMU under us without our noticing.
    AP_InertialSensor   *_ins;
    GPS                 *&_gps;
    AP_Baro             *_barometer;

    // a vector to capture the difference between the controller and body frames
    AP_Vector3f         _trim;

    // should we raise the gain on the accelerometers for faster
    // convergence, used when disarmed for ArduCopter
    bool _fast_ground_gains;

    // true if we can assume the aircraft will be flying forward
    // on its X axis
    bool _fly_forward;

    // the limit of the gyro drift claimed by the sensors, in
    // radians/s/s
    float _gyro_drift_limit;

    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef;

    // acceleration due to gravity in m/s/s
    static constexpr float _gravity = 9.80665;

};

#include <AP_AHRS_DCM.h>
#include <AP_AHRS_MPU6000.h>
#include <AP_AHRS_HIL.h>

#endif // __AP_AHRS_H__
