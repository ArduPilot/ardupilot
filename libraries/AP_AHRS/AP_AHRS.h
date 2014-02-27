/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_AHRS_H__
#define __AP_AHRS_H__
/*
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

/*
 *  AHRS (Attitude Heading Reference System) interface for ArduPilot
 *
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
    AP_AHRS(AP_InertialSensor &ins, AP_Baro &baro, GPS *&gps) :
        _compass(NULL),
        _ins(ins),
        _baro(baro),
        _gps(gps),
        _cos_roll(1.0f),
        _cos_pitch(1.0f),
        _cos_yaw(1.0f),
        _sin_roll(0.0f),
        _sin_pitch(0.0f),
        _sin_yaw(0.0f),
        _active_accel_instance(0)
    {
        // load default values from var_info table
        AP_Param::setup_object_defaults(this, var_info);

        // base the ki values by the sensors maximum drift
        // rate. The APM2 has gyros which are much less drift
        // prone than the APM1, so we should have a lower ki,
        // which will make us less prone to increasing omegaI
        // incorrectly due to sensor noise
        _gyro_drift_limit = ins.get_gyro_drift_rate();

        // enable centrifugal correction by default
        _flags.correct_centrifugal = true;

        // start off with armed flag true
        _flags.armed = true;

        // initialise _home
        _home.id         = MAV_CMD_NAV_WAYPOINT;
        _home.options    = 0;
        _home.p1         = 0;
        _home.alt        = 0;
        _home.lng        = 0;
        _home.lat        = 0;
    }

    // init sets up INS board orientation
    virtual void init() {
        set_orientation();
    };

    // Accessors
    void set_fly_forward(bool b) {
        _flags.fly_forward = b;
    }

    bool get_fly_forward(void) const {
        return _flags.fly_forward;
    }

    void set_wind_estimation(bool b) {
        _flags.wind_estimation = b;
    }

    void set_compass(Compass *compass) {
        _compass = compass;
        set_orientation();
    }

    const Compass* get_compass() const {
        return _compass;
    }
        
    // allow for runtime change of orientation
    // this makes initial config easier
    void set_orientation() {
        _ins.set_board_orientation((enum Rotation)_board_orientation.get());
        if (_compass != NULL) {
            _compass->set_board_orientation((enum Rotation)_board_orientation.get());
        }
    }

    void set_airspeed(AP_Airspeed *airspeed) {
        _airspeed = airspeed;
    }

    const AP_Airspeed *get_airspeed(void) const {
        return _airspeed;
    }

    const GPS *get_gps() const {
        return _gps;
    }

    const AP_InertialSensor &get_ins() const {
	    return _ins;
    }

    const AP_Baro &get_baro() const {
	    return _baro;
    }

    // accelerometer values in the earth frame in m/s/s
    const Vector3f &get_accel_ef(void) const { return _accel_ef[_ins.get_primary_accel()]; }

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

    // reset the current attitude, used on new IMU calibration
    virtual void reset_attitude(const float &roll, const float &pitch, const float &yaw) = 0;

    // return the average size of the roll/pitch error estimate
    // since last call
    virtual float get_error_rp(void) = 0;

    // return the average size of the yaw error estimate
    // since last call
    virtual float get_error_yaw(void) = 0;

    // return a DCM rotation matrix representing our current
    // attitude
    virtual const Matrix3f &get_dcm_matrix(void) const = 0;

    // get our current position estimate. Return true if a position is available,
    // otherwise false. This call fills in lat, lng and alt
    virtual bool get_position(struct Location &loc) = 0;

    // return a wind estimation vector, in m/s
    virtual Vector3f wind_estimate(void) = 0;

    // return an airspeed estimate if available. return true
    // if we have an estimate
    virtual bool airspeed_estimate(float *airspeed_ret) const;

    // return a true airspeed estimate (navigation airspeed) if
    // available. return true if we have an estimate
    bool airspeed_estimate_true(float *airspeed_ret) const {
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
    virtual Vector2f groundspeed_vector(void);

    // return a ground velocity in meters/second, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true 
    virtual bool get_velocity_NED(Vector3f &vec) const { return false; }

    // return a position relative to home in meters, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true 
    virtual bool get_relative_position_NED(Vector3f &vec) const { return false; }

    // return ground speed estimate in meters/second. Used by ground vehicles.
    float groundspeed(void) const {
        if (!_gps || _gps->status() <= GPS::NO_FIX) {
            return 0.0f;
        }
        return _gps->ground_speed_cm * 0.01f;
    }

    // return true if we will use compass for yaw
    virtual bool use_compass(void) { return _compass && _compass->use_for_yaw(); }

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

    // get the correct centrifugal flag
    bool get_correct_centrifugal(void) const {
        return _flags.correct_centrifugal;
    }

    // set the armed flag
    // allows EKF enter static mode when disarmed
    void set_armed(bool setting) {
        _flags.armed = setting;
    }

    // get the armed flag
    bool get_armed(void) const {
        return _flags.armed;
    }

    // get trim
    const Vector3f &get_trim() const { return _trim.get(); }

    // set trim
    virtual void            set_trim(Vector3f new_trim);

    // add_trim - adjust the roll and pitch trim up to a total of 10 degrees
    virtual void            add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);

    // helper trig value accessors
    float cos_roll() const  { return _cos_roll; }
    float cos_pitch() const { return _cos_pitch; }
    float cos_yaw() const   { return _cos_yaw; }
    float sin_roll() const  { return _sin_roll; }
    float sin_pitch() const { return _sin_pitch; }
    float sin_yaw() const   { return _sin_yaw; }

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // these are public for ArduCopter
	AP_Float _kp_yaw;
    AP_Float _kp;
    AP_Float gps_gain;

    // return secondary attitude solution if available, as eulers in radians
    virtual bool get_secondary_attitude(Vector3f &eulers) { return false; }

    // return secondary position solution if available
    virtual bool get_secondary_position(struct Location &loc) { return false; }

    // get the home location. This is const to prevent any changes to
    // home without telling AHRS about the change    
    const struct Location &get_home(void) const { return _home; }

    // set the home location in 10e7 degrees. This should be called
    // when the vehicle is at this position. It is assumed that the
    // current barometer and GPS altitudes correspond to this altitude
    virtual void set_home(int32_t lat, int32_t lon, int32_t alt_cm) = 0;

    // return true if the AHRS object supports inertial navigation,
    // with very accurate position and velocity
    virtual bool have_inertial_nav(void) const { return false; }

    // return the active accelerometer instance
    uint8_t get_active_accel_instance(void) const { return _active_accel_instance; }

protected:
    // settable parameters
    AP_Float beta;
    AP_Int8 _gps_use;
    AP_Int8 _wind_max;
    AP_Int8 _board_orientation;
    AP_Int8 _gps_minsats;
    AP_Int8 _gps_delay;
    AP_Int8 _ekf_use;

    // flags structure
    struct ahrs_flags {
        uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
        uint8_t fast_ground_gains       : 1;    // should we raise the gain on the accelerometers for faster convergence, used when disarmed for ArduCopter
        uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
        uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
        uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
        uint8_t armed                   : 1;    // 1 if we are armed for flight
    } _flags;

    // update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
    //      should be called after _dcm_matrix is updated
    void update_trig(void);

    // pointer to compass object, if available
    Compass         * _compass;

    // pointer to airspeed object, if available
    AP_Airspeed     * _airspeed;

    // time in microseconds of last compass update
    uint32_t _compass_last_update;

    // note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
    //       IMU under us without our noticing.
    AP_InertialSensor   &_ins;
    AP_Baro             &_baro;
    GPS                 *&_gps;

    // a vector to capture the difference between the controller and body frames
    AP_Vector3f         _trim;

    // the limit of the gyro drift claimed by the sensors, in
    // radians/s/s
    float _gyro_drift_limit;

    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef[INS_MAX_INSTANCES];

	// Declare filter states for HPF and LPF used by complementary
	// filter in AP_AHRS::groundspeed_vector
	Vector2f _lp; // ground vector low-pass filter
	Vector2f _hp; // ground vector high-pass filter
    Vector2f _lastGndVelADS; // previous HPF input		

    // reference position for NED positions
    struct Location _home;

    // helper trig variables
    float _cos_roll, _cos_pitch, _cos_yaw;
    float _sin_roll, _sin_pitch, _sin_yaw;

    // which accelerometer instance is active
    uint8_t _active_accel_instance;
};

#include <AP_AHRS_DCM.h>
#include <AP_AHRS_NavEKF.h>

#endif // __AP_AHRS_H__
