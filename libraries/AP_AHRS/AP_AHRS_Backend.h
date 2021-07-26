#pragma once

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
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>

class OpticalFlow;
#define AP_AHRS_TRIM_LIMIT 10.0f        // maximum trim angle in degrees
#define AP_AHRS_RP_P_MIN   0.05f        // minimum value for AHRS_RP_P parameter
#define AP_AHRS_YAW_P_MIN  0.05f        // minimum value for AHRS_YAW_P parameter

class AP_AHRS_Backend
{
public:

    // Constructor
    AP_AHRS_Backend() {}

    // empty virtual destructor
    virtual ~AP_AHRS_Backend() {}

    // init sets up INS board orientation
    virtual void init();

    // return the index of the primary core or -1 if no primary core selected
    virtual int8_t get_primary_core_index() const { return -1; }

    // get the index of the current primary accelerometer sensor
    virtual uint8_t get_primary_accel_index(void) const {
        return AP::ins().get_primary_accel();
    }

    // get the index of the current primary gyro sensor
    virtual uint8_t get_primary_gyro_index(void) const {
        return AP::ins().get_primary_gyro();
    }

    // accelerometer values in the earth frame in m/s/s
    virtual const Vector3f &get_accel_ef(uint8_t i) const {
        return _accel_ef[i];
    }
    virtual const Vector3f &get_accel_ef(void) const {
        return get_accel_ef(AP::ins().get_primary_accel());
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
    virtual void _update() = 0;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked
    virtual bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const = 0;

    // check all cores providing consistent attitudes for prearm checks
    virtual bool attitudes_consistent(char *failure_msg, const uint8_t failure_msg_len) const { return true; }

    // see if EKF lane switching is possible to avoid EKF failsafe
    virtual void check_lane_switch(void) {}

    // check if non-compass sensor is providing yaw.  Allows compass pre-arm checks to be bypassed
    virtual bool using_noncompass_for_yaw(void) const { return false; }

    // check if external nav is providing yaw
    virtual bool using_extnav_for_yaw(void) const { return false; }

    // request EKF yaw reset to try and avoid the need for an EKF lane switch or failsafe
    virtual void request_yaw_reset(void) {}

    // set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
    virtual void set_posvelyaw_source_set(uint8_t source_set_idx) {}

    // Euler angles (radians)
    float roll;
    float pitch;
    float yaw;

    float get_roll() const { return roll; }
    float get_pitch() const { return pitch; }
    float get_yaw() const { return yaw; }

    // integer Euler angles (Degrees * 100)
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;

    // return a smoothed and corrected gyro vector in radians/second
    virtual const Vector3f &get_gyro(void) const = 0;

    // return primary accels, for lua
    const Vector3f &get_accel(void) const {
        return AP::ins().get_accel();
    }
    
    // return a smoothed and corrected gyro vector in radians/second using the latest ins data (which may not have been consumed by the EKF yet)
    Vector3f get_gyro_latest(void) const;

    // return the current estimate of the gyro drift
    virtual const Vector3f &get_gyro_drift(void) const = 0;

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    virtual void reset_gyro_drift(void) = 0;

    // reset the current attitude, used on new IMU calibration
    virtual void reset(bool recover_eulers=false) = 0;

    // return the average size of the roll/pitch error estimate
    // since last call
    virtual float get_error_rp(void) const = 0;

    // return the average size of the yaw error estimate
    // since last call
    virtual float get_error_yaw(void) const = 0;

    // return a DCM rotation matrix representing our current attitude in NED frame
    virtual const Matrix3f &get_rotation_body_to_ned(void) const = 0;

    // return a Quaternion representing our current attitude in NED frame
    void get_quat_body_to_ned(Quaternion &quat) const {
        quat.from_rotation_matrix(get_rotation_body_to_ned());
    }

    // get rotation matrix specifically from DCM backend (used for compass calibrator)
    virtual const Matrix3f &get_DCM_rotation_body_to_ned(void) const = 0;

    // get our current position estimate. Return true if a position is available,
    // otherwise false. This call fills in lat, lng and alt
    virtual bool get_position(struct Location &loc) const WARN_IF_UNUSED = 0;

    // get latest altitude estimate above ground level in meters and validity flag
    virtual bool get_hagl(float &height) const WARN_IF_UNUSED { return false; }

    // return a wind estimation vector, in m/s
    virtual Vector3f wind_estimate(void) const = 0;

    // return an airspeed estimate if available. return true
    // if we have an estimate
    virtual bool airspeed_estimate(float &airspeed_ret) const WARN_IF_UNUSED = 0;

    // return a true airspeed estimate (navigation airspeed) if
    // available. return true if we have an estimate
    bool airspeed_estimate_true(float &airspeed_ret) const WARN_IF_UNUSED {
        if (!airspeed_estimate(airspeed_ret)) {
            return false;
        }
        airspeed_ret *= get_EAS2TAS();
        return true;
    }

    // return estimate of true airspeed vector in body frame in m/s
    // returns false if estimate is unavailable
    virtual bool airspeed_vector_true(Vector3f &vec) const WARN_IF_UNUSED {
        return false;
    }

    // return a synthetic airspeed estimate (one derived from sensors
    // other than an actual airspeed sensor), if available. return
    // true if we have a synthetic airspeed.  ret will not be modified
    // on failure.
    virtual bool synthetic_airspeed(float &ret) const WARN_IF_UNUSED = 0;

    // get apparent to true airspeed ratio
    float get_EAS2TAS(void) const;

    // return true if airspeed comes from an airspeed sensor, as
    // opposed to an IMU estimate
    bool airspeed_sensor_enabled(void) const {
        const AP_Airspeed *_airspeed = AP::airspeed();
        return _airspeed != nullptr && _airspeed->use() && _airspeed->healthy();
    }

    // return true if airspeed comes from a specific airspeed sensor, as
    // opposed to an IMU estimate
    bool airspeed_sensor_enabled(uint8_t airspeed_index) const {
        const AP_Airspeed *_airspeed = AP::airspeed();
        return _airspeed != nullptr && _airspeed->use(airspeed_index) && _airspeed->healthy(airspeed_index);
    }

    // return a ground vector estimate in meters/second, in North/East order
    virtual Vector2f groundspeed_vector(void) = 0;

    // return a ground velocity in meters/second, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true
    virtual bool get_velocity_NED(Vector3f &vec) const WARN_IF_UNUSED {
        return false;
    }

    // returns the estimated magnetic field offsets in body frame
    virtual bool get_mag_field_correction(Vector3f &ret) const WARN_IF_UNUSED {
        return false;
    }

    // return a position relative to origin in meters, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true
    virtual bool get_relative_position_NED_origin(Vector3f &vec) const WARN_IF_UNUSED {
        return false;
    }

    // return a position relative to origin in meters, North/East
    // order. Return true if estimate is valid
    virtual bool get_relative_position_NE_origin(Vector2f &vecNE) const WARN_IF_UNUSED {
        return false;
    }

    // return a Down position relative to origin in meters
    // Return true if estimate is valid
    virtual bool get_relative_position_D_origin(float &posD) const WARN_IF_UNUSED {
        return false;
    }

    // return ground speed estimate in meters/second. Used by ground vehicles.
    float groundspeed(void) {
        return groundspeed_vector().length();
    }

    // return true if we will use compass for yaw
    virtual bool use_compass(void) = 0;

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

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    virtual bool get_quaternion(Quaternion &quat) const WARN_IF_UNUSED = 0;

    // return secondary attitude solution if available, as eulers in radians
    virtual bool get_secondary_attitude(Vector3f &eulers) const WARN_IF_UNUSED {
        return false;
    }

    // return secondary attitude solution if available, as quaternion
    virtual bool get_secondary_quaternion(Quaternion &quat) const WARN_IF_UNUSED {
        return false;
    }

    // return secondary position solution if available
    virtual bool get_secondary_position(struct Location &loc) const WARN_IF_UNUSED {
        return false;
    }

    // return true if the AHRS object supports inertial navigation,
    // with very accurate position and velocity
    virtual bool have_inertial_nav(void) const {
        return false;
    }

    // is the AHRS subsystem healthy?
    virtual bool healthy(void) const = 0;

    // true if the AHRS has completed initialisation
    virtual bool initialised(void) const {
        return true;
    };

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    virtual uint32_t getLastYawResetAngle(float &yawAng) {
        return 0;
    };

    // return the amount of NE position change in metres due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastPosNorthEastReset(Vector2f &pos) WARN_IF_UNUSED {
        return 0;
    };

    // return the amount of NE velocity change in metres/sec due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastVelNorthEastReset(Vector2f &vel) const WARN_IF_UNUSED {
        return 0;
    };

    // return the amount of vertical position change due to the last reset in meters
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastPosDownReset(float &posDelta) WARN_IF_UNUSED {
        return 0;
    };

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    virtual bool resetHeightDatum(void) WARN_IF_UNUSED {
        return false;
    }

    // return the innovations for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    virtual bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const {
        return false;
    }

    // get_variances - provides the innovations normalised using the innovation variance where a value of 0
    // indicates perfect consistency between the measurement and the EKF solution and a value of of 1 is the maximum
    // inconsistency that will be accepted by the filter
    // boolean false is returned if variances are not available
    virtual bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const {
        return false;
    }

    // get a source's velocity innovations.  source should be from 0 to 7 (see AP_NavEKF_Source::SourceXY)
    // returns true on success and results are placed in innovations and variances arguments
    virtual bool get_vel_innovations_and_variances_for_source(uint8_t source, Vector3f &innovations, Vector3f &variances) const WARN_IF_UNUSED {
        return false;
    }

    // Retrieves the corrected NED delta velocity in use by the inertial navigation
    virtual void getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const {
        ret.zero();
        AP::ins().get_delta_velocity(ret, dt);
    }

    // rotate a 2D vector from earth frame to body frame
    // in result, x is forward, y is right
    Vector2f earth_to_body2D(const Vector2f &ef_vector) const;

    // rotate a 2D vector from earth frame to body frame
    // in input, x is forward, y is right
    Vector2f body_to_earth2D(const Vector2f &bf) const;

    // convert a vector from body to earth frame
    Vector3f body_to_earth(const Vector3f &v) const {
        return v * get_rotation_body_to_ned();
    }

    // convert a vector from earth to body frame
    Vector3f earth_to_body(const Vector3f &v) const {
        return get_rotation_body_to_ned().mul_transpose(v);
    }

    // get_hgt_ctrl_limit - get maximum height to be observed by the
    // control loops in meters and a validity flag.  It will return
    // false when no limiting is required
    virtual bool get_hgt_ctrl_limit(float &limit) const WARN_IF_UNUSED { return false; };

    // Set to true if the terrain underneath is stable enough to be used as a height reference
    // this is not related to terrain following
    virtual void set_terrain_hgt_stable(bool stable) {}

    // Write position and quaternion data from an external navigation system
    virtual void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms) { }

    // Write velocity data from an external navigation system
    virtual void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms) { }

    // return current vibration vector for primary IMU
    Vector3f get_vibration(void) const;

    // set and save the alt noise parameter value
    virtual void set_alt_measurement_noise(float noise) {};

    // allow threads to lock against AHRS update
    HAL_Semaphore &get_semaphore(void) {
        return _rsem;
    }

    // Logging to disk functions
    void Write_AHRS2(void) const;
    void Write_Attitude(const Vector3f &targets) const;
    void Write_Origin(uint8_t origin_type, const Location &loc) const; 
    void Write_POS(void) const;

protected:

    enum class GPSUse : uint8_t {
        Disable = 0,
        Enable  = 1,
        EnableWithHeight = 2,
    };

    AP_Enum<GPSUse> _gps_use;

    // multi-thread access support
    HAL_Semaphore _rsem;

    // calculate sin/cos of roll/pitch/yaw from rotation
    void calc_trig(const Matrix3f &rot,
                   float &cr, float &cp, float &cy,
                   float &sr, float &sp, float &sy) const;

    // update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
    //      should be called after _dcm_matrix is updated
    void update_trig(void);

    // update roll_sensor, pitch_sensor and yaw_sensor
    void update_cd_values(void);

    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef[INS_MAX_INSTANCES];
    Vector3f        _accel_ef_blended;

    // helper trig variables
    float _cos_roll{1.0f};
    float _cos_pitch{1.0f};
    float _cos_yaw{1.0f};
    float _sin_roll;
    float _sin_pitch;
    float _sin_yaw;

    // EAS to TAS calculated on each loop
    float _EAS2TAS{1};
};
