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

#include <AP_Math/AP_Math.h>
#include <inttypes.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Param/AP_Param.h>

class OpticalFlow;
#define AP_AHRS_TRIM_LIMIT 10.0f        // maximum trim angle in degrees
#define AP_AHRS_RP_P_MIN   0.05f        // minimum value for AHRS_RP_P parameter
#define AP_AHRS_YAW_P_MIN  0.05f        // minimum value for AHRS_YAW_P parameter

enum AHRS_VehicleClass {
    AHRS_VEHICLE_UNKNOWN,
    AHRS_VEHICLE_GROUND,
    AHRS_VEHICLE_COPTER,
    AHRS_VEHICLE_FIXED_WING,
};


class AP_AHRS
{
public:
    // Constructor
    AP_AHRS(AP_InertialSensor &ins, AP_Baro &baro, AP_GPS &gps) :
        roll(0.0f),
        pitch(0.0f),
        yaw(0.0f),
        roll_sensor(0),
        pitch_sensor(0),
        yaw_sensor(0),
        _vehicle_class(AHRS_VEHICLE_UNKNOWN),
        _compass(NULL),
        _optflow(NULL),
        _airspeed(NULL),
        _compass_last_update(0),
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
        // rate.
        _gyro_drift_limit = ins.get_gyro_drift_rate();

        // enable centrifugal correction by default
        _flags.correct_centrifugal = true;

        // initialise _home
        _home.options    = 0;
        _home.alt        = 0;
        _home.lng        = 0;
        _home.lat        = 0;
    }

    // empty virtual destructor
    virtual ~AP_AHRS() {}

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

    AHRS_VehicleClass get_vehicle_class(void) const {
        return _vehicle_class;
    }

    void set_vehicle_class(AHRS_VehicleClass vclass) {
        _vehicle_class = vclass;
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

    void set_optflow(const OpticalFlow *optflow) {
        _optflow = optflow;
    }

    const OpticalFlow* get_optflow() const {
        return _optflow;
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

    const AP_GPS &get_gps() const {
        return _gps;
    }

    const AP_InertialSensor &get_ins() const {
        return _ins;
    }

    const AP_Baro &get_baro() const {
        return _baro;
    }

    // accelerometer values in the earth frame in m/s/s
    virtual const Vector3f &get_accel_ef(uint8_t i) const {
        return _accel_ef[i];
    }
    virtual const Vector3f &get_accel_ef(void) const {
        return get_accel_ef(_ins.get_primary_accel());
    }

    // blended accelerometer values in the earth frame in m/s/s
    virtual const Vector3f &get_accel_ef_blended(void) const {
        return _accel_ef_blended;
    }

    // get yaw rate in earth frame in radians/sec
    float get_yaw_rate_earth(void) const {
        return get_gyro() * get_rotation_body_to_ned().c;
    }

    // Methods
    virtual void update(void) = 0;

    // report any reason for why the backend is refusing to initialise
    virtual const char *prearm_failure_reason(void) const {
        return nullptr;
    }

    // Euler angles (radians)
    float roll;
    float pitch;
    float yaw;

    // integer Euler angles (Degrees * 100)
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;

    // return a smoothed and corrected gyro vector
    virtual const Vector3f &get_gyro(void) const = 0;

    // return the current estimate of the gyro drift
    virtual const Vector3f &get_gyro_drift(void) const = 0;

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    virtual void reset_gyro_drift(void) = 0;

    // reset the current attitude, used on new IMU calibration
    virtual void reset(bool recover_eulers=false) = 0;

    // reset the current attitude, used on new IMU calibration
    virtual void reset_attitude(const float &roll, const float &pitch, const float &yaw) = 0;

    // return the average size of the roll/pitch error estimate
    // since last call
    virtual float get_error_rp(void) const = 0;

    // return the average size of the yaw error estimate
    // since last call
    virtual float get_error_yaw(void) const = 0;

    // return a DCM rotation matrix representing our current
    // attitude
    virtual const Matrix3f &get_rotation_body_to_ned(void) const = 0;

    // get our current position estimate. Return true if a position is available,
    // otherwise false. This call fills in lat, lng and alt
    virtual bool get_position(struct Location &loc) const = 0;

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
        return _airspeed != NULL && _airspeed->use() && _airspeed->healthy();
    }

    // return a ground vector estimate in meters/second, in North/East order
    virtual Vector2f groundspeed_vector(void);

    // return a ground velocity in meters/second, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true
    virtual bool get_velocity_NED(Vector3f &vec) const {
        return false;
    }

    // returns the expected NED magnetic field
    virtual bool get_expected_mag_field_NED(Vector3f &ret) const {
        return false;
    }

    // returns the estimated magnetic field offsets in body frame
    virtual bool get_mag_field_correction(Vector3f &ret) const {
        return false;
    }

    // return a position relative to home in meters, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true
    virtual bool get_relative_position_NED(Vector3f &vec) const {
        return false;
    }

    // return ground speed estimate in meters/second. Used by ground vehicles.
    float groundspeed(void) const {
        if (_gps.status() <= AP_GPS::NO_FIX) {
            return 0.0f;
        }
        return _gps.ground_speed();
    }

    // return true if we will use compass for yaw
    virtual bool use_compass(void) {
        return _compass && _compass->use_for_yaw();
    }

    // return true if yaw has been initialised
    bool yaw_initialised(void) const {
        return _flags.have_initial_yaw;
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

    // get trim
    const Vector3f &get_trim() const {
        return _trim.get();
    }

    // set trim
    virtual void            set_trim(Vector3f new_trim);

    // add_trim - adjust the roll and pitch trim up to a total of 10 degrees
    virtual void            add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);

    // helper trig value accessors
    float cos_roll() const  {
        return _cos_roll;
    }
    float cos_pitch() const {
        return _cos_pitch;
    }
    float cos_yaw() const   {
        return _cos_yaw;
    }
    float sin_roll() const  {
        return _sin_roll;
    }
    float sin_pitch() const {
        return _sin_pitch;
    }
    float sin_yaw() const   {
        return _sin_yaw;
    }

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // return secondary attitude solution if available, as eulers in radians
    virtual bool get_secondary_attitude(Vector3f &eulers) {
        return false;
    }

    // return secondary position solution if available
    virtual bool get_secondary_position(struct Location &loc) {
        return false;
    }

    // get the home location. This is const to prevent any changes to
    // home without telling AHRS about the change
    const struct Location &get_home(void) const {
        return _home;
    }

    // set the home location in 10e7 degrees. This should be called
    // when the vehicle is at this position. It is assumed that the
    // current barometer and GPS altitudes correspond to this altitude
    virtual void set_home(const Location &loc) = 0;

    // return true if the AHRS object supports inertial navigation,
    // with very accurate position and velocity
    virtual bool have_inertial_nav(void) const {
        return false;
    }

    // return the active accelerometer instance
    uint8_t get_active_accel_instance(void) const {
        return _active_accel_instance;
    }

    // is the AHRS subsystem healthy?
    virtual bool healthy(void) const = 0;

    // true if the AHRS has completed initialisation
    virtual bool initialised(void) const {
        return true;
    };

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    virtual uint32_t getLastYawResetAngle(float &yawAng) const {
        return 0;
    };

    // return the amount of NE position change in metres due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastPosNorthEastReset(Vector2f &pos) const {
        return 0;
    };

    // return the amount of NE velocity change in metres/sec due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastVelNorthEastReset(Vector2f &vel) const {
        return 0;
    };

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    virtual bool resetHeightDatum(void) {
        return false;
    }
    
    // time that the AHRS has been up
    virtual uint32_t uptime_ms(void) const = 0;

    // get the selected ekf type, for allocation decisions
    int8_t get_ekf_type(void) const {
        return _ekf_type;
    }
    
protected:
    AHRS_VehicleClass _vehicle_class;

    // settable parameters
    // these are public for ArduCopter
    AP_Float _kp_yaw;
    AP_Float _kp;
    AP_Float gps_gain;

    AP_Float beta;
    AP_Int8 _gps_use;
    AP_Int8 _wind_max;
    AP_Int8 _board_orientation;
    AP_Int8 _gps_minsats;
    AP_Int8 _gps_delay;
    AP_Int8 _ekf_type;

    // flags structure
    struct ahrs_flags {
        uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
        uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
        uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
        uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
    } _flags;

    // update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
    //      should be called after _dcm_matrix is updated
    void update_trig(void);

    // update roll_sensor, pitch_sensor and yaw_sensor
    void update_cd_values(void);

    // pointer to compass object, if available
    Compass         * _compass;

    // pointer to OpticalFlow object, if available
    const OpticalFlow *_optflow;

    // pointer to airspeed object, if available
    AP_Airspeed     * _airspeed;

    // time in microseconds of last compass update
    uint32_t _compass_last_update;

    // note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
    //       IMU under us without our noticing.
    AP_InertialSensor   &_ins;
    AP_Baro             &_baro;
    const AP_GPS        &_gps;

    // a vector to capture the difference between the controller and body frames
    AP_Vector3f         _trim;

    // the limit of the gyro drift claimed by the sensors, in
    // radians/s/s
    float _gyro_drift_limit;

    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef[INS_MAX_INSTANCES];
    Vector3f        _accel_ef_blended;

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

#include "AP_AHRS_DCM.h"
#include "AP_AHRS_NavEKF.h"

#if AP_AHRS_NAVEKF_AVAILABLE
#define AP_AHRS_TYPE AP_AHRS_NavEKF
#else
#define AP_AHRS_TYPE AP_AHRS
#endif

#endif // __AP_AHRS_H__
