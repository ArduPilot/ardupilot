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

#define AP_AHRS_TRIM_LIMIT 10.0f        // maximum trim angle in degrees

class AP_AHRS
{
public:
    // Constructor
    AP_AHRS(AP_InertialSensor *ins, GPS *&gps) :
        _ins(ins),
        _gps(gps)
    {
        // load default values from var_info table
        AP_Param::setup_object_defaults(this, var_info);

        // base the ki values by the sensors maximum drift
        // rate. The APM2 has gyros which are much less drift
        // prone than the APM1, so we should have a lower ki,
        // which will make us less prone to increasing omegaI
        // incorrectly due to sensor noise
        _gyro_drift_limit = ins->get_gyro_drift_rate();

        // enable centrifugal correction by default
        _flags.correct_centrifugal = true;
    }

    // init sets up INS board orientation
    virtual void init() {
        set_orientation();
    };

    // Accessors
    void set_fly_forward(bool b) {
        _flags.fly_forward = b;
    }

    void set_wind_estimation(bool b) {
        _flags.wind_estimation = b;
    }

    void set_compass(Compass *compass) {
        _compass = compass;
        set_orientation();
    }


    // allow for runtime change of orientation
    // this makes initial config easier
    void set_orientation() {
        _ins->set_board_orientation((enum Rotation)_board_orientation.get());
        if (_compass != NULL) {
            _compass->set_board_orientation((enum Rotation)_board_orientation.get());
        }
    }

    void set_airspeed(AP_Airspeed *airspeed) {
        _airspeed = airspeed;
    }

    AP_InertialSensor* get_ins() const {
	    return _ins;
    }

    // accelerometer values in the earth frame in m/s/s
    const Vector3f &get_accel_ef(void) const { return _accel_ef; }

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

    // return a smoothed and corrected gyro vector
    virtual const Vector3f get_gyro(void) const = 0;

    // return the current estimate of the gyro drift
    virtual const Vector3f &get_gyro_drift(void) const = 0;

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
    virtual const Matrix3f &get_dcm_matrix(void) const = 0;

    // get our current position, either from GPS or via
    // dead-reckoning. Return true if a position is available,
    // otherwise false. This only updates the lat and lng fields
    // of the Location
    virtual bool get_position(struct Location *loc) {
        if (!_gps || _gps->status() <= GPS::NO_FIX) {
            return false;
        }
        loc->lat = _gps->latitude;
        loc->lng = _gps->longitude;
        return true;
    }

    // get our projected position, based on our GPS position plus
    // heading and ground speed
    bool get_projected_position(struct Location *loc);

    // return a wind estimation vector, in m/s
    virtual Vector3f wind_estimate(void) {
        return Vector3f(0,0,0);
    }

    // return an airspeed estimate if available. return true
    // if we have an estimate
    virtual bool airspeed_estimate(float *airspeed_ret);

    // return a true airspeed estimate (navigation airspeed) if
    // available. return true if we have an estimate
    bool airspeed_estimate_true(float *airspeed_ret) {
        if (!airspeed_estimate(airspeed_ret)) {
            return false;
        }
        *airspeed_ret *= get_EAS2TAS();
        return true;
    }

    // get apparent to true airspeed ratio
    float get_EAS2TAS(void) const {
        if (_airspeed) {
            return _airspeed->get_EAS2TAS();
        }
        return 1.0f;
    }

    // return true if airspeed comes from an airspeed sensor, as
    // opposed to an IMU estimate
    bool airspeed_sensor_enabled(void) const {
        return _airspeed != NULL && _airspeed->use();
    }

    // return a ground vector estimate in meters/second, in North/East order
    Vector2f groundspeed_vector(void);

    // return true if we will use compass for yaw
    virtual bool use_compass(void) const { return _compass && _compass->use_for_yaw(); }

    // return true if yaw has been initialised
    bool yaw_initialised(void) const {
        return _flags.have_initial_yaw;
    }

    // set the fast gains flag
    void set_fast_gains(bool setting) {
        _flags.fast_ground_gains = setting;
    }

    // set the correct centrifugal flag
    // allows arducopter to disable corrections when disarmed
    void set_correct_centrifugal(bool setting) {
        _flags.correct_centrifugal = setting;
    }

    // get trim
    const Vector3f &get_trim() const { return _trim.get(); }

    // set trim
    virtual void            set_trim(Vector3f new_trim);

    // add_trim - adjust the roll and pitch trim up to a total of 10 degrees
    virtual void            add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // these are public for ArduCopter
	AP_Float _kp_yaw;
    AP_Float _kp;
    AP_Float gps_gain;

protected:
    // settable parameters
    AP_Float beta;
    AP_Int8 _gps_use;
    AP_Int8 _wind_max;
    AP_Int8 _board_orientation;
    AP_Int8 _gps_minsats;

    // flags structure
    struct ahrs_flags {
        uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
        uint8_t fast_ground_gains       : 1;    // should we raise the gain on the accelerometers for faster convergence, used when disarmed for ArduCopter
        uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
        uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
        uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
    } _flags;

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

    // a vector to capture the difference between the controller and body frames
    AP_Vector3f         _trim;

    // the limit of the gyro drift claimed by the sensors, in
    // radians/s/s
    float _gyro_drift_limit;

    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef;

	// Declare filter states for HPF and LPF used by complementary
	// filter in AP_AHRS::groundspeed_vector
	Vector2f _lp; // ground vector low-pass filter
	Vector2f _hp; // ground vector high-pass filter
    Vector2f _lastGndVelADS; // previous HPF input		
};

#include <AP_AHRS_DCM.h>
#include <AP_AHRS_MPU6000.h>
#include <AP_AHRS_HIL.h>

#endif // __AP_AHRS_H__
