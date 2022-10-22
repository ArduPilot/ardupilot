/*
  24 state EKF based on the derivation in https://github.com/PX4/ecl/
  blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m

  Converted from Matlab to C++ by Paul Riseborough

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
#pragma once


#if !defined(HAL_DEBUG_BUILD) || !HAL_DEBUG_BUILD
    #pragma GCC optimize("O2")
#endif

#include "AP_NavEKF3_feature.h"
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>
#include <AP_NavEKF/AP_NavEKF_core_common.h>
#include <AP_NavEKF/AP_NavEKF_Source.h>
#include <AP_NavEKF/EKF_Buffer.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_DAL/AP_DAL.h>

#include "AP_NavEKF/EKFGSF_yaw.h"

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS      (1<<0)
#define MASK_GPS_HDOP       (1<<1)
#define MASK_GPS_SPD_ERR    (1<<2)
#define MASK_GPS_POS_ERR    (1<<3)
#define MASK_GPS_YAW_ERR 	(1<<4)
#define MASK_GPS_POS_DRIFT  (1<<5)
#define MASK_GPS_VERT_SPD   (1<<6)
#define MASK_GPS_HORIZ_SPD  (1<<7)

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// maximum allowed gyro bias (rad/sec)
#define GYRO_BIAS_LIMIT 0.5f

// initial accel bias uncertainty as a fraction of the state limit
#define ACCEL_BIAS_LIM_SCALER 0.2f

// target update time for the EKF in msec and sec
#define EKF_TARGET_DT_MS 12
#define EKF_TARGET_DT    0.012f

// mag fusion final reset altitude (using NED frame so altitude is negative)
#define EKF3_MAG_FINAL_RESET_ALT 2.5f

// learning rate for mag biases when using GPS yaw
#define EK3_GPS_MAG_LEARN_RATE 0.005f

// learning limit for mag biases when using GPS yaw (Gauss)
#define EK3_GPS_MAG_LEARN_LIMIT 0.02f

// maximum number of yaw resets due to detected magnetic anomaly allowed per flight
#define MAG_ANOMALY_RESET_MAX 2

// number of seconds a request to reset the yaw to the GSF estimate is active before it times out
#define YAW_RESET_TO_GSF_TIMEOUT_MS 5000

// accuracy threshold applied to GSF yaw estimate use
#define GSF_YAW_ACCURACY_THRESHOLD_DEG 15.0f

// number of continuous valid GSF yaw estimates required to confirm valid hostory
#define GSF_YAW_VALID_HISTORY_THRESHOLD 5

// minimum variances allowed for velocity and position states
#define VEL_STATE_MIN_VARIANCE 1E-4
#define POS_STATE_MIN_VARIANCE 1E-4

// maximum number of times the vertical velocity variance can hit the lower limit before the
// associated states, variances and covariances are reset
#define EKF_TARGET_RATE_HZ uint32_t(1.0 / EKF_TARGET_DT)
#define VERT_VEL_VAR_CLIP_COUNT_LIM (5 * EKF_TARGET_RATE_HZ)

// limit on horizontal position states
#if HAL_WITH_EKF_DOUBLE
#define EK3_POSXY_STATE_LIMIT 50.0e6
#else
#define EK3_POSXY_STATE_LIMIT 1.0e6
#endif

// IMU acceleration process noise in m/s/s used when bad vibration affected IMU accel is detected
#define BAD_IMU_DATA_ACC_P_NSE 5.0f

// Number of milliseconds of bad IMU data before a reset to vertical position and velocity height sources is performed
#define BAD_IMU_DATA_TIMEOUT_MS 1000

// number of milliseconds the bad IMU data response settings will be held after the last bad IMU data is detected
#define BAD_IMU_DATA_HOLD_MS 10000

// wind state variance limits
#define WIND_VEL_VARIANCE_MAX 400.0f
#define WIND_VEL_VARIANCE_MIN 0.25f


class NavEKF3_core : public NavEKF_core_common
{
public:
    // Constructor
    NavEKF3_core(class NavEKF3 *_frontend);

    // setup this core backend
    bool setup_core(uint8_t _imu_index, uint8_t _core_index);
    
    // Initialise the states from accelerometer and magnetometer data (if present)
    // This method can only be used when the vehicle is static
    bool InitialiseFilterBootstrap(void);

    // Update Filter States - this should be called whenever new IMU data is available
    // The predict flag is set true when a new prediction cycle can be started
    void UpdateFilter(bool predict);

    // Check basic filter health metrics and return a consolidated health status
    bool healthy(void) const;

    // Return a consolidated error score where higher numbers are less healthy
    // Intended to be used by the front-end to determine which is the primary EKF
    float errorScore(void) const;

    // Write the last calculated NE position relative to the reference point (m).
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosNE(Vector2f &posNE) const;

    // get position D from local origin
    bool getPosD_local(float &posD) const;

    // Write the last calculated D position relative to the public origin
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosD(float &posD) const;

    // return NED velocity in m/s
    void getVelNED(Vector3f &vel) const;

    // return estimate of true airspeed vector in body frame in m/s
    // returns false if estimate is unavailable
    bool getAirSpdVec(Vector3f &vel) const;

    // return the innovation in m/s, innovation variance in (m/s)^2 and age in msec of the last TAS measurement processed
    // returns false if the data is unavailable
    bool getAirSpdHealthData(float &innovation, float &innovationVariance, uint32_t &age_ms) const;

    // Return the rate of change of vertical position in the down direction (dPosD/dt) in m/s
    // This can be different to the z component of the EKF velocity state because it will fluctuate with height errors and corrections in the EKF
    // but will always be kinematically consistent with the z component of the EKF position state
    float getPosDownDerivative(void) const;

    // return body axis gyro bias estimates in rad/sec
    void getGyroBias(Vector3f &gyroBias) const;

    // return accelerometer bias in m/s/s
    void getAccelBias(Vector3f &accelBias) const;

    // reset body axis gyro bias estimates
    void resetGyroBias(void);

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKF origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    bool resetHeightDatum(void);

    // return the horizontal speed limit in m/s set by optical flow sensor limits
    // return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const;

    // return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
    // returns true if wind state estimation is active
    bool getWind(Vector3f &wind) const;

    // return earth magnetic field estimates in measurement units / 1000
    void getMagNED(Vector3f &magNED) const;

    // return body magnetic field estimates in measurement units / 1000
    void getMagXYZ(Vector3f &magXYZ) const;

    // return the index for the active sensors
    uint8_t getActiveAirspeed() const;

    // Return estimated magnetometer offsets
    // Return true if magnetometer offsets are valid
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    // Return the last calculated latitude, longitude and height in WGS-84
    // If a calculated location isn't available, return a raw GPS measurement
    // The status will return true if a calculation or raw measurement is available
    // The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
    bool getLLH(struct Location &loc) const;

    // return the latitude and longitude and height used to set the NED origin
    // All NED positions calculated by the filter are relative to this location
    // Returns false if the origin has not been set
    bool getOriginLLH(struct Location &loc) const;

    // set the latitude and longitude and height used to set the NED origin
    // All NED positions calculated by the filter will be relative to this location
    // returns false if Absolute aiding and GPS is being used or if the origin is already set
    bool setOriginLLH(const Location &loc);

    // return estimated height above ground level
    // return false if ground height is not being estimated.
    bool getHAGL(float &HAGL) const;

    // return the Euler roll, pitch and yaw angle in radians
    void getEulerAngles(Vector3f &eulers) const;

    // return the transformation matrix from XYZ (body) to NED axes
    void getRotationBodyToNED(Matrix3f &mat) const;

    // return the quaternions defining the rotation from NED to XYZ (body) axes
    void getQuaternion(Quaternion &quat) const;

    // return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
    bool getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    // return the synthetic air data drag and sideslip innovations
    void getSynthAirDataInnovations(Vector2f &dragInnov, float &betaInnov) const;

   // return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
    bool getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

    // get a particular source's velocity innovations
    // returns true on success and results are placed in innovations and variances arguments
    bool getVelInnovationsAndVariancesForSource(AP_NavEKF_Source::SourceXY source, Vector3f &innovations, Vector3f &variances) const WARN_IF_UNUSED;

    // should we use the compass? This is public so it can be used for
    // reporting via ahrs.use_compass()
    bool use_compass(void) const;

    // write the raw optical flow measurements
    // rawFlowQuality is a measured of quality between 0 and 255, with 255 being the best quality
    // rawFlowRates are the optical flow rates in rad/sec about the X and Y sensor axes.
    // rawGyroRates are the sensor rotation rates in rad/sec measured by the sensors internal gyro
    // The sign convention is that a RH physical rotation of the sensor about an axis produces both a positive flow and gyro rate
    // msecFlowMeas is the scheduler time in msec when the optical flow data was received from the sensor.
    // posOffset is the XYZ flow sensor position in the body frame in m
    void writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset);

    // retrieve latest corrected optical flow samples (used for calibration)
    bool getOptFlowSample(uint32_t& timeStamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const;

    /*
     * Write body frame linear and angular displacement measurements from a visual odometry sensor
     *
     * quality is a normalised confidence value from 0 to 100
     * delPos is the XYZ change in linear position measured in body frame and relative to the inertial reference at time_ms (m)
     * delAng is the XYZ angular rotation measured in body frame and relative to the inertial reference at time_ms (rad)
     * delTime is the time interval for the measurement of delPos and delAng (sec)
     * timeStamp_ms is the timestamp of the last image used to calculate delPos and delAng (msec)
     * delay_ms is the average delay of external nav system measurements relative to inertial measurements
     * posOffset is the XYZ body frame position of the camera focal point (m)
    */
    void writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset);

    /*
     * Write odometry data from a wheel encoder. The axis of rotation is assumed to be parallel to the vehicle body axis
     *
     * delAng is the measured change in angular position from the previous measurement where a positive rotation is produced by forward motion of the vehicle (rad)
     * delTime is the time interval for the measurement of delAng (sec)
     * timeStamp_ms is the time when the rotation was last measured (msec)
     * posOffset is the XYZ body frame position of the wheel hub (m)
     * radius is the effective rolling radius of the wheel (m)
    */
    void writeWheelOdom(float delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset, float radius);

    /*
     * Return data for debugging body frame odometry fusion:
     *
     * velInnov are the XYZ body frame velocity innovations (m/s)
     * velInnovVar are the XYZ body frame velocity innovation variances (m/s)**2
     *
     * Return the time stamp of the last odometry fusion update (msec)
     */
    uint32_t getBodyFrameOdomDebug(Vector3f &velInnov, Vector3f &velInnovVar);

    /*
     * Writes the measurement from a yaw angle sensor
     *
     * yawAngle: Yaw angle of the vehicle relative to true north in radians where a positive angle is
     * produced by a RH rotation about the Z body axis. The Yaw rotation is the first rotation in a
     * 321 (ZYX) or a 312 (ZXY) rotation sequence as specified by the 'type' argument.
     * yawAngleErr is the 1SD accuracy of the yaw angle measurement in radians.
     * timeStamp_ms: System time in msec when the yaw measurement was taken. This time stamp must include
     * all measurement lag and transmission delays.
     * type: An integer specifying Euler rotation order used to define the yaw angle.
     * type = 1 specifies a 312 (ZXY) rotation order, type = 2 specifies a 321 (ZYX) rotation order.
    */
    void writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type);

    /*
    * Write position and quaternion data from an external navigation system
    *
    * pos        : position in the RH navigation frame. Frame is assumed to be NED if frameIsNED is true. (m)
    * quat       : quaternion desribing the rotation from navigation frame to body frame
    * posErr     : 1-sigma spherical position error (m)
    * angErr     : 1-sigma spherical angle error (rad)
    * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
    * delay_ms   : average delay of external nav system measurements relative to inertial measurements
    * resetTime_ms : system time of the last position reset request (mSec)
    *
    */
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms);

    /*
     * Write velocity data from an external navigation system
     *
     * vel : velocity in NED (m)
     * err : velocity error (m/s)
     * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
     * delay_ms : average delay of external nav system measurements relative to inertial measurements
    */
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms);

    // Set to true if the terrain underneath is stable enough to be used as a height reference
    // in combination with a range finder. Set to false if the terrain underneath the vehicle
    // cannot be used as a height reference. Use to prevent range finder operation otherwise
    // enabled by the combination of EK3_RNG_USE_HGT and EK3_RNG_USE_SPD parameters.
    void setTerrainHgtStable(bool val);

    /*
    return the filter fault status as a bitmasked integer
     0 = quaternions are NaN
     1 = velocities are NaN
     2 = badly conditioned X magnetometer fusion
     3 = badly conditioned Y magnetometer fusion
     5 = badly conditioned Z magnetometer fusion
     6 = badly conditioned airspeed fusion
     7 = badly conditioned synthetic sideslip fusion
     7 = filter is not initialised
    */
    void getFilterFaults(uint16_t &faults) const;

    /*
    Return a filter function status that indicates:
        Which outputs are valid
        If the filter has detected takeoff
        If the filter has activated the mode that mitigates against ground effect static pressure errors
        If GPS data is being used
    */
    void getFilterStatus(nav_filter_status &status) const;

    // send an EKF_STATUS_REPORT message to GCS
    void send_status_report(class GCS_MAVLINK &link) const;

    // provides the height limit to be observed by the control loops
    // returns false if no height limiting is required
    // this is needed to ensure the vehicle does not fly too high when using optical flow navigation
    bool getHeightControlLimit(float &height) const;

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    uint32_t getLastYawResetAngle(float &yawAng) const;

    // return the amount of NE position change due to the last position reset in metres
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &pos) const;

    // return the amount of D position change due to the last position reset in metres
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosDownReset(float &posD) const;

    // return the amount of NE velocity change due to the last velocity reset in metres/sec
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    // report any reason for why the backend is refusing to initialise
    const char *prearm_failure_reason(void) const;

    // report the number of frames lapsed since the last state prediction
    // this is used by other instances to level load
    uint8_t getFramesSincePredict(void) const;

    // get the IMU index. For now we return the gyro index, as that is most
    // critical for use by other subsystems.
    uint8_t getIMUIndex(void) const { return gyro_index_active; }

    // values for EK3_MAG_CAL
    enum class MagCal {
        WHEN_FLYING = 0,
        WHEN_MANOEUVRING = 1,
        NEVER = 2,
        AFTER_FIRST_CLIMB = 3,
        ALWAYS = 4
        // 5 was EXTERNAL_YAW (do not use)
        // 6 was EXTERNAL_YAW_FALLBACK (do not use)
    };

    // are we using (aka fusing) a non-compass yaw?
    bool using_noncompass_for_yaw(void) const;

    // are we using (aka fusing) external nav for yaw?
    bool using_extnav_for_yaw() const;

    // Writes the default equivalent airspeed and 1-sigma uncertainty in m/s to be used in forward flight if a measured airspeed is required and not available.
    void writeDefaultAirSpeed(float airspeed, float uncertainty);

    // request a reset the yaw to the EKF-GSF value
    void EKFGSF_requestYawReset();

    // return true if we are tilt aligned
    bool have_aligned_tilt(void) const {
        return tiltAlignComplete;
    }

    // return true if we are yaw aligned
    bool have_aligned_yaw(void) const {
        return yawAlignComplete;
    }

    void Log_Write(uint64_t time_us);

    // returns true when the state estimates are significantly degraded by vibration
    bool isVibrationAffected() const { return badIMUdata; }

    // get a yaw estimator instance
    const EKFGSF_yaw *get_yawEstimator(void) const { return yawEstimator; }

private:
    EKFGSF_yaw *yawEstimator;
    AP_DAL &dal;

    // Reference to the global EKF frontend for parameters
    class NavEKF3 *frontend;
    uint8_t imu_index; // preferred IMU index
    uint8_t gyro_index_active; // active gyro index (in case preferred fails)
    uint8_t accel_index_active; // active accel index (in case preferred fails)
    uint8_t core_index;
    uint8_t imu_buffer_length;
    uint8_t obs_buffer_length;

#if MATH_CHECK_INDEXES
    typedef VectorN<ftype,2> Vector2;
    typedef VectorN<ftype,3> Vector3;
    typedef VectorN<ftype,4> Vector4;
    typedef VectorN<ftype,5> Vector5;
    typedef VectorN<ftype,6> Vector6;
    typedef VectorN<ftype,7> Vector7;
    typedef VectorN<ftype,8> Vector8;
    typedef VectorN<ftype,9> Vector9;
    typedef VectorN<ftype,10> Vector10;
    typedef VectorN<ftype,11> Vector11;
    typedef VectorN<ftype,13> Vector13;
    typedef VectorN<ftype,14> Vector14;
    typedef VectorN<ftype,15> Vector15;
    typedef VectorN<ftype,21> Vector21;
    typedef VectorN<ftype,22> Vector22;
    typedef VectorN<ftype,23> Vector23;
    typedef VectorN<ftype,24> Vector24;
    typedef VectorN<ftype,25> Vector25;
    typedef VectorN<ftype,31> Vector31;
    typedef VectorN<VectorN<ftype,3>,3> Matrix3;
    typedef VectorN<VectorN<ftype,24>,24> Matrix24;
    typedef VectorN<VectorN<ftype,34>,50> Matrix34_50;
    typedef VectorN<uint32_t,50> Vector_u32_50;
#else
    typedef ftype Vector2[2];
    typedef ftype Vector3[3];
    typedef ftype Vector4[4];
    typedef ftype Vector5[5];
    typedef ftype Vector6[6];
    typedef ftype Vector7[7];
    typedef ftype Vector8[8];
    typedef ftype Vector9[9];
    typedef ftype Vector10[10];
    typedef ftype Vector11[11];
    typedef ftype Vector13[13];
    typedef ftype Vector14[14];
    typedef ftype Vector15[15];
    typedef ftype Vector21[21];
    typedef ftype Vector22[22];
    typedef ftype Vector23[23];
    typedef ftype Vector24[24];
    typedef ftype Vector25[25];
    typedef ftype Matrix3[3][3];
    typedef ftype Matrix24[24][24];
    typedef ftype Matrix34_50[34][50];
    typedef uint32_t Vector_u32_50[50];
#endif

    // the states are available in two forms, either as a Vector24, or
    // broken down as individual elements. Both are equivalent (same
    // memory)
    struct state_elements {
        QuaternionF quat;           // quaternion defining rotation from local NED earth frame to body frame 0..3
        Vector3F    velocity;       // velocity of IMU in local NED earth frame (m/sec) 4..6
        Vector3F    position;       // position of IMU in local NED earth frame (m)     7..9
        Vector3F    gyro_bias;      // body frame delta angle IMU bias vector (rad)     10..12
        Vector3F    accel_bias;     // body frame delta velocity IMU bias vector (m/sec) 13..15
        Vector3F    earth_magfield; // earth frame magnetic field vector (Gauss)         16..18
        Vector3F    body_magfield;  // body frame magnetic field vector (Gauss)          19..21
        Vector2F    wind_vel;       // horizontal North East wind velocity vector in local NED earth frame (m/sec) 22..23
    };

    union {
        Vector24 statesArray;
        struct state_elements stateStruct;
    };

    struct output_elements {
        QuaternionF quat;           // quaternion defining rotation from local NED earth frame to body frame
        Vector3F    velocity;       // velocity of body frame origin in local NED earth frame (m/sec)
        Vector3F    position;       // position of body frame origin in local NED earth frame (m)
    };

    struct imu_elements {
        Vector3F    delAng;         // IMU delta angle measurements in body frame (rad)
        Vector3F    delVel;         // IMU delta velocity measurements in body frame (m/sec)
        ftype       delAngDT;       // time interval over which delAng has been measured (sec)
        ftype       delVelDT;       // time interval over which delVelDT has been measured (sec)
        uint32_t    time_ms;        // measurement timestamp (msec)
        uint8_t     gyro_index;
        uint8_t     accel_index;
    };

    struct gps_elements : EKF_obs_element_t {
        int32_t     lat, lng;       // latitude and longitude in 1e7 degrees
        ftype       hgt;            // height of the GPS antenna in local NED earth frame (m)
        Vector3F    vel;            // velocity of the GPS antenna in local NED earth frame (m/sec)
        uint8_t     sensor_idx;     // unique integer identifying the GPS sensor
        bool        corrected;      // true when the position and velocity have been corrected for sensor position
        bool        have_vz;        // true when vertical velocity is valid
    };

    struct mag_elements : EKF_obs_element_t {
        Vector3F    mag;            // body frame magnetic field measurements (Gauss)
    };

    struct baro_elements : EKF_obs_element_t {
        ftype       hgt;            // height of the pressure sensor in local NED earth frame (m)
    };

    struct range_elements : EKF_obs_element_t {
        ftype       rng;            // distance measured by the range sensor (m)
        uint8_t     sensor_idx;     // integer either 0 or 1 uniquely identifying up to two range sensors
    };

    struct rng_bcn_elements : EKF_obs_element_t {
        ftype       rng;            // range measurement to each beacon (m)
        Vector3F    beacon_posNED;  // NED position of the beacon (m)
        ftype       rngErr;         // range measurement error 1-std (m)
        uint8_t     beacon_ID;      // beacon identification number
    };

    struct tas_elements : EKF_obs_element_t {
        ftype       tas;            // true airspeed measurement (m/sec)
        ftype       tasVariance;    // variance of true airspeed measurement (m/sec)^2
        bool        allowFusion;    // true if measurement can be allowed to modify EKF states.
    };

    struct of_elements : EKF_obs_element_t {
        Vector2F    flowRadXY;      // raw (non motion compensated) optical flow angular rates about the XY body axes (rad/sec)
        Vector2F    flowRadXYcomp;  // motion compensated XY optical flow angular rates about the XY body axes (rad/sec)
        Vector3F    bodyRadXYZ;     // body frame XYZ axis angular rates averaged across the optical flow measurement interval (rad/sec)
        Vector3F    body_offset;    // XYZ position of the optical flow sensor in body frame (m)
    };

    struct vel_odm_elements : EKF_obs_element_t {
        Vector3F        vel;        // XYZ velocity measured in body frame (m/s)
        ftype           velErr;     // velocity measurement error 1-std (m/s)
        Vector3F        body_offset;// XYZ position of the velocity sensor in body frame (m)
        Vector3F        angRate;    // angular rate estimated from odometry (rad/sec)
    };

    struct wheel_odm_elements : EKF_obs_element_t {
        ftype           delAng;     // wheel rotation angle measured in body frame - positive is forward movement of vehicle (rad/s)
        ftype           radius;     // wheel radius (m)
        Vector3F        hub_offset; // XYZ position of the wheel hub in body frame (m)
        ftype           delTime;    // time interval that the measurement was accumulated over (sec)
    };
        
    // Specifies the rotation order used for the Tait-Bryan or Euler angles where alternative rotation orders are available
    enum class rotationOrder {
        TAIT_BRYAN_321=0,
        TAIT_BRYAN_312=1
    };

    struct yaw_elements : EKF_obs_element_t {
        ftype         yawAng;         // yaw angle measurement (rad)
        ftype         yawAngErr;      // yaw angle 1SD measurement accuracy (rad)
        rotationOrder order;          // type specifiying Euler rotation order used, 0 = 321 (ZYX), 1 = 312 (ZXY)
    };

    struct ext_nav_elements : EKF_obs_element_t {
        Vector3F        pos;        // XYZ position measured in a RH navigation frame (m)
        ftype           posErr;     // spherical position measurement error 1-std (m)
        bool            posReset;   // true when the position measurement has been reset
        bool            corrected;  // true when the position has been corrected for sensor position
    };

    struct ext_nav_vel_elements : EKF_obs_element_t {
        Vector3F vel;               // velocity in NED (m/s)
        ftype err;                  // velocity measurement error (m/s)
        bool corrected;             // true when the velocity has been corrected for sensor position
    };

    struct drag_elements : EKF_obs_element_t {
        Vector2f accelXY;       // measured specific force along the X and Y body axes (m/sec**2)
    };

    // bias estimates for the IMUs that are enabled but not being used
    // by this core.
    struct {
        Vector3F gyro_bias;
        Vector3F accel_bias;
    } inactiveBias[INS_MAX_INSTANCES];

    // Specify source of data to be used for a partial state reset
    // Checking the availability and quality of the data source specified is the responsibility of the caller
    enum class resetDataSource {
        DEFAULT=0,      // Use data source selected by reset function internal rules
        GPS=1,          // Use GPS
        RNGBCN=2,       // Use beacon range data
        FLOW=3,         // Use optical flow rates
        BARO=4,         // Use Baro height
        MAG=5,          // Use magnetometer data
        RNGFND=6,       // Use rangefinder data
        EXTNAV=7        // Use external nav data
    };

    // specifies the method to be used when fusing yaw observations
    enum class yawFusionMethod {
	    MAGNETOMETER=0,
	    GPS=1,
        GSF=2,
        STATIC=3,
        PREDICTED=4,
        EXTNAV=5,
    };

    // update the navigation filter status
    void updateFilterStatus(void);

    // update the quaternion, velocity and position states using IMU measurements
    void UpdateStrapdownEquationsNED();

    // calculate the predicted state covariance matrix
    // Argument rotVarVecPtr is pointer to a vector defining the earth frame uncertainty variance of the quaternion states
    // used to perform a reset of the quaternion state covariances only. Set to null for normal operation.
    void CovariancePrediction(Vector3F *rotVarVecPtr);

    // force symmetry on the state covariance matrix
    void ForceSymmetry();

    // constrain variances (diagonal terms) in the state covariance matrix
    void ConstrainVariances();

    // constrain states
    void ConstrainStates();

    // constrain earth field using WMM tables
    void MagTableConstrain(void);

    // fuse selected position, velocity and height measurements
    void FuseVelPosNED();

    // fuse body frame velocity measurements
    void FuseBodyVel();

    // fuse range beacon measurements
    void FuseRngBcn();

    // use range beacon measurements to calculate a static position
    void FuseRngBcnStatic();

    // calculate the offset from EKF vertical position datum to the range beacon system datum
    void CalcRangeBeaconPosDownOffset(ftype obsVar, Vector3F &vehiclePosNED, bool aligning);

    // fuse magnetometer measurements
    void FuseMagnetometer();

    // fuse true airspeed measurements
    void FuseAirspeed();

    // fuse synthetic sideslip measurement of zero
    void FuseSideslip();

    // zero specified range of rows in the state covariance matrix
    void zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last);

    // zero specified range of columns in the state covariance matrix
    void zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last);

    // Reset the stored output history to current data
    void StoreOutputReset(void);

    // Reset the stored output quaternion history to current EKF state
    void StoreQuatReset(void);

    // Rotate the stored output quaternion history through a quaternion rotation
    void StoreQuatRotate(const QuaternionF &deltaQuat);

    // calculate the NED earth spin vector in rad/sec
    void calcEarthRateNED(Vector3F &omega, int32_t latitude) const;

    // initialise the covariance matrix
    void CovarianceInit();

    // helper functions for readIMUData
    bool readDeltaVelocity(uint8_t ins_index, Vector3F &dVel, ftype &dVel_dt);
    bool readDeltaAngle(uint8_t ins_index, Vector3F &dAng, ftype &dAng_dt);

    // helper functions for correcting IMU data
    void correctDeltaAngle(Vector3F &delAng, ftype delAngDT, uint8_t gyro_index);
    void correctDeltaVelocity(Vector3F &delVel, ftype delVelDT, uint8_t accel_index);

    // update IMU delta angle and delta velocity measurements
    void readIMUData();

    // update estimate of inactive bias states
    void learnInactiveBiases();

    // check for new valid GPS data and update stored measurement if available
    void readGpsData();

    // check for new valid GPS yaw data
    void readGpsYawData();

    // check for new altitude measurement data and update stored measurement if available
    void readBaroData();

    // check for new magnetometer data and update store measurements if available
    void readMagData();

    // try changing compasses on compass failure or timeout
    void tryChangeCompass(void);

    // check for new airspeed data and update stored measurements if available
    void readAirSpdData();

    // check for new range beacon data and update stored measurements if available
    void readRngBcnData();

    // determine when to perform fusion of GPS position and  velocity measurements
    void SelectVelPosFusion();

    // determine when to perform fusion of range measurements take relative to a beacon at a known NED position
    void SelectRngBcnFusion();

    // determine when to perform fusion of magnetometer measurements
    void SelectMagFusion();

    // determine when to perform fusion of true airspeed measurements
    void SelectTasFusion();

    // determine when to perform fusion of drag or synthetic sideslip measurements
    void SelectBetaDragFusion();

    // force alignment of the yaw angle using GPS velocity data
    void realignYawGPS();

    // initialise the earth magnetic field states using declination and current attitude and magnetometer measurements

    // align the yaw angle for the quaternion states to the given yaw angle which should be at the fusion horizon
    void alignYawAngle(const yaw_elements &yawAngData);

    // update mag field states and associated variances using magnetomer and declination data
    void resetMagFieldStates();

    // reset yaw based on magnetic field sample
    void setYawFromMag();

    // zero stored variables
    void InitialiseVariables();

    // zero stored variables related to mag
    void InitialiseVariablesMag();

    // reset the horizontal position states uing the last GPS measurement
    void ResetPosition(resetDataSource posResetSource);

    // reset the stateStruct's NE position to the specified position
    void ResetPositionNE(ftype posN, ftype posE);

    // reset the stateStruct's D position
    void ResetPositionD(ftype posD);

    // reset velocity states using the last GPS measurement
    void ResetVelocity(resetDataSource velResetSource);

    // reset the vertical position state using the last height measurement
    void ResetHeight(void);

    // return true if we should use the airspeed sensor
    bool useAirspeed(void) const;

    // return true if the vehicle code has requested the filter to be ready for flight
    bool readyToUseGPS(void) const;

    // return true if the filter to be ready to use the beacon range measurements
    bool readyToUseRangeBeacon(void) const;

    // Check for filter divergence
    void checkDivergence(void);

    // Calculate weighting that is applied to IMU1 accel data to blend data from IMU's 1 and 2
    void calcIMU_Weighting(ftype K1, ftype K2);

    // return true if the filter is ready to start using optical flow measurements for position and velocity estimation
    bool readyToUseOptFlow(void) const;

    // return true if the filter is ready to start using body frame odometry measurements
    bool readyToUseBodyOdm(void) const;

    // return true if the filter to be ready to use external nav data
    bool readyToUseExtNav(void) const;

    // return true if we should use the range finder sensor
    bool useRngFinder(void) const;

    // determine when to perform fusion of optical flow measurements
    void SelectFlowFusion();

    // determine when to perform fusion of body frame odometry measurements
    void SelectBodyOdomFusion();

    // Estimate terrain offset using a single state EKF
    void EstimateTerrainOffset(const of_elements &ofDataDelayed);

    // fuse optical flow measurements into the main filter
    // really_fuse should be true to actually fuse into the main filter, false to only calculate variances
    void FuseOptFlow(const of_elements &ofDataDelayed, bool really_fuse);

    // Control filter mode changes
    void controlFilterModes();

    // Determine if we are flying or on the ground
    void detectFlight();

    // Set inertial navigation aiding mode
    void setAidingMode();

    // Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
    // avoid unnecessary operations
    void setWindMagStateLearningMode();

    // Check the alignmnent status of the tilt attitude
    // Used during initial bootstrap alignment of the filter
    void checkAttitudeAlignmentStatus();

    // Control reset of yaw and magnetic field states
    void controlMagYawReset();

    // set the latitude and longitude and height used to set the NED origin
    // All NED positions calculated by the filter will be relative to this location
    // returns false if the origin has already been set
    bool setOrigin(const Location &loc);

    // Assess GPS data quality and set gpsGoodToAlign
    void calcGpsGoodToAlign(void);

    // set the class variable true if the delta angle bias variances are sufficiently small
    void checkGyroCalStatus(void);

    // update inflight calculaton that determines if GPS data is good enough for reliable navigation
    void calcGpsGoodForFlight(void);

    // Read the range finder and take new measurements if available
    // Apply a median filter to range finder data
    void readRangeFinder();

    // check if the vehicle has taken off during optical flow navigation by looking at inertial and range finder data
    void detectOptFlowTakeoff(void);

    // align the NE earth magnetic field states with the published declination
    void alignMagStateDeclination();

    // Fuse compass measurements using a direct yaw angle observation (doesn't require magnetic field states)
    // Returns true if the fusion was successful
    bool fuseEulerYaw(yawFusionMethod method);

    // return the best Tait-Bryan rotation order to use
    void bestRotationOrder(rotationOrder &order);

    // Fuse declination angle to keep earth field declination from changing when we don't have earth relative observations.
    // Input is 1-sigma uncertainty in published declination
    void FuseDeclination(ftype declErr);

    // return magnetic declination in radians
    ftype MagDeclination(void) const;

    // Propagate PVA solution forward from the fusion time horizon to the current time horizon
    // using a simple observer
    void calcOutputStates();

    // calculate a filtered offset between baro height measurement and EKF height estimate
    void calcFiltBaroOffset();

    // correct the height of the EKF origin to be consistent with GPS Data using a Bayes filter.
    void correctEkfOriginHeight();

    // Select height data to be fused from the available baro, range finder and GPS sources
    void selectHeightForFusion();

    // zero attitude state covariances, but preserve variances
    void zeroAttCovOnly();

    // record a yaw reset event
    void recordYawReset();

    // record a magnetic field state reset event
    void recordMagReset();

    // effective value of MAG_CAL
    MagCal effective_magCal(void) const;

    // calculate the tilt error variance
    void calcTiltErrorVariance(void);
    
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // calculate the tilt error variance using an alternative numerical difference technique
    // and log with value generated by NavEKF3_core::calcTiltErrorVariance()
    void verifyTiltErrorVariance();
#endif

    // update timing statistics structure
    void updateTimingStatistics(void);

    // Update the state index limit based on which states are active
    void updateStateIndexLim(void);

    // correct GPS data for antenna position
    void CorrectGPSForAntennaOffset(gps_elements &gps_data) const;

    // correct external navigation earth-frame position using sensor body-frame offset
    void CorrectExtNavForSensorOffset(ext_nav_elements &ext_nav_data);

    // correct external navigation earth-frame velocity using sensor body-frame offset
    void CorrectExtNavVelForSensorOffset(ext_nav_vel_elements &ext_nav_vel_data) const;

    // calculate velocity variances and innovations
    // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    // variances argument is updated with variances for each axis
    void CalculateVelInnovationsAndVariances(const Vector3F &velocity, ftype noise, ftype accel_scale, Vector3F &innovations, Vector3F &variances) const;

    // Runs the IMU prediction step for an independent GSF yaw estimator algorithm
    // that uses IMU, GPS horizontal velocity and optionally true airspeed data.
    void runYawEstimatorPrediction(void);

    // Run the GPS velocity correction step for the GSF yaw estimator and use the
    // yaw estimate to reset the main EKF yaw if requested
    void runYawEstimatorCorrection(void);

    // reset the quaternion states using the supplied yaw angle, maintaining the previous roll and pitch
    // also reset the body to nav frame rotation matrix
    // reset the quaternion state covariances using the supplied yaw variance
    // yaw          : new yaw angle (rad)
    // yaw_variance : variance of new yaw angle (rad^2)
    // order : enum defining Tait-Bryan rotation order used in calculation of the yaw angle
    void resetQuatStateYawOnly(ftype yaw, ftype yawVariance, rotationOrder order);

    // attempt to reset the yaw to the EKF-GSF value
    // emergency_reset should be true if this reset is triggered by the loss of the yaw estimate
    // returns false if unsuccessful
    bool EKFGSF_resetMainFilterYaw(bool emergency_reset);

    // returns true on success and populates yaw (in radians) and yawVariance (rad^2)
    bool EKFGSF_getYaw(ftype &yaw, ftype &yawVariance) const;

    // Fusion of body frame X and Y axis drag specific forces for multi-rotor wind estimation
    void FuseDragForces();
    void SelectDragFusion();
    void SampleDragData(const imu_elements &imu);

    bool getGPSLLH(struct Location &loc) const;

    // Variables
    bool statesInitialised;         // boolean true when filter states have been initialised
    bool magHealth;                 // boolean true if magnetometer has passed innovation consistency check
    bool velTimeout;                // boolean true if velocity measurements have failed innovation consistency check and timed out
    bool posTimeout;                // boolean true if position measurements have failed innovation consistency check and timed out
    bool hgtTimeout;                // boolean true if height measurements have failed innovation consistency check and timed out
    bool magTimeout;                // boolean true if magnetometer measurements have failed for too long and have timed out
    bool tasTimeout;                // boolean true if true airspeed measurements have failed for too long and have timed out
    bool dragTimeout;               // boolean true if drag measurements have failed for too long and have timed out
    bool badIMUdata;                // boolean true if the bad IMU data is detected
    uint32_t badIMUdata_ms;         // time stamp bad IMU data was last detected
    uint32_t goodIMUdata_ms;        // time stamp good IMU data was last detected
    uint32_t vertVelVarClipCounter; // counter used to control reset of vertical velocity variance following collapse against the lower limit

    ftype gpsNoiseScaler;           // Used to scale the  GPS measurement noise and consistency gates to compensate for operation with small satellite counts
    Matrix24 P;                     // covariance matrix
    EKF_IMU_buffer_t<imu_elements> storedIMU;      // IMU data buffer
    EKF_obs_buffer_t<gps_elements> storedGPS;      // GPS data buffer
    EKF_obs_buffer_t<mag_elements> storedMag;      // Magnetometer data buffer
    EKF_obs_buffer_t<baro_elements> storedBaro;    // Baro data buffer
    EKF_obs_buffer_t<tas_elements> storedTAS;      // TAS data buffer
    EKF_obs_buffer_t<range_elements> storedRange;  // Range finder data buffer
    EKF_IMU_buffer_t<output_elements> storedOutput;// output state buffer
    Matrix3F prevTnb;               // previous nav to body transformation used for INS earth rotation compensation
    ftype accNavMag;                // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
    ftype accNavMagHoriz;           // magnitude of navigation accel in horizontal plane (m/s^2)
    Vector3F earthRateNED;          // earths angular rate vector in NED (rad/s)
    ftype dtIMUavg;                 // expected time between IMU measurements (sec)
    ftype dtEkfAvg;                 // expected time between EKF updates (sec)
    ftype dt;                       // time lapsed since the last covariance prediction (sec)
    ftype hgtRate;                  // state for rate of change of height filter
    bool onGround;                  // true when the flight vehicle is definitely on the ground
    bool prevOnGround;              // value of onGround from previous frame - used to detect transition
    bool inFlight;                  // true when the vehicle is definitely flying
    bool prevInFlight;              // value inFlight from previous frame - used to detect transition
    bool manoeuvring;               // boolean true when the flight vehicle is performing horizontal changes in velocity
    Vector6 innovVelPos;            // innovation output for a group of measurements
    Vector6 varInnovVelPos;         // innovation variance output for a group of measurements
    Vector6 velPosObs;              // observations for combined velocity and positon group of measurements (3x1 m , 3x1 m/s)
    bool fuseVelData;               // this boolean causes the velNED measurements to be fused
    bool fusePosData;               // this boolean causes the posNE measurements to be fused
    bool fuseHgtData;               // this boolean causes the hgtMea measurements to be fused
    Vector3F innovMag;              // innovation output from fusion of X,Y,Z compass measurements
    Vector3F varInnovMag;           // innovation variance output from fusion of X,Y,Z compass measurements
    ftype innovVtas;                // innovation output from fusion of airspeed measurements
    ftype varInnovVtas;             // innovation variance output from fusion of airspeed measurements
    ftype defaultAirSpeed;          // default equivalent airspeed in m/s to be used if the measurement is unavailable. Do not use if not positive.
    ftype defaultAirSpeedVariance;  // default equivalent airspeed variance in (m/s)**2 to be used when defaultAirSpeed is specified. 
    bool magFusePerformed;          // boolean set to true when magnetometer fusion has been perfomred in that time step
    MagCal effectiveMagCal;         // the actual mag calibration being used as the default
    uint32_t prevTasStep_ms;        // time stamp of last TAS fusion step
    uint32_t prevBetaDragStep_ms;   // time stamp of last synthetic sideslip fusion step
    ftype innovBeta;                // synthetic sideslip innovation (rad)
    uint32_t lastMagUpdate_us;      // last time compass was updated in usec
    uint32_t lastMagRead_ms;        // last time compass data was successfully read
    Vector3F velDotNED;             // rate of change of velocity in NED frame
    Vector3F velDotNEDfilt;         // low pass filtered velDotNED
    uint32_t imuSampleTime_ms;      // time that the last IMU value was taken
    bool tasDataToFuse;             // true when new airspeed data is waiting to be fused
    uint32_t lastBaroReceived_ms;   // time last time we received baro height data
    uint16_t hgtRetryTime_ms;       // time allowed without use of height measurements before a height timeout is declared
    uint32_t lastVelPassTime_ms;    // time stamp when GPS velocity measurement last passed innovation consistency check (msec)
    uint32_t lastPosPassTime_ms;    // time stamp when GPS position measurement last passed innovation consistency check (msec)
    uint32_t lastHgtPassTime_ms;    // time stamp when height measurement last passed innovation consistency check (msec)
    uint32_t lastTasPassTime_ms;    // time stamp when airspeed measurement last passed innovation consistency check (msec)
    uint32_t lastTasFailTime_ms;    // time stamp when airspeed measurement last failed innovation consistency check (msec)
    uint32_t lastTimeGpsReceived_ms;// last time we received GPS data
    uint32_t timeAtLastAuxEKF_ms;   // last time the auxiliary filter was run to fuse range or optical flow measurements
    uint32_t lastHealthyMagTime_ms; // time the magnetometer was last declared healthy
    bool allMagSensorsFailed;       // true if all magnetometer sensors have timed out on this flight and we are no longer using magnetometer data
    uint32_t lastSynthYawTime_ms;   // time stamp when yaw observation was last fused (msec)
    uint32_t ekfStartTime_ms;       // time the EKF was started (msec)
    Vector2F lastKnownPositionNE;   // last known position
    float lastKnownPositionD;       // last known height
    uint32_t lastLaunchAccelTime_ms;
    ftype velTestRatio;             // sum of squares of GPS velocity innovation divided by fail threshold
    ftype posTestRatio;             // sum of squares of GPS position innovation divided by fail threshold
    ftype hgtTestRatio;             // sum of squares of baro height innovation divided by fail threshold
    Vector3F magTestRatio;          // sum of squares of magnetometer innovations divided by fail threshold
    ftype tasTestRatio;             // sum of squares of true airspeed innovation divided by fail threshold
    bool inhibitWindStates;         // true when wind states and covariances are to remain constant
    bool windStatesAligned;         // true when wind states have been aligned
    bool inhibitMagStates;          // true when magnetic field states are inactive
    bool lastInhibitMagStates;      // previous inhibitMagStates
    bool needMagBodyVarReset;       // we need to reset mag body variances at next CovariancePrediction
    bool needEarthBodyVarReset;     // we need to reset mag earth variances at next CovariancePrediction
    bool inhibitDelAngBiasStates;   // true when IMU delta angle bias states are inactive
    bool gpsIsInUse;                // bool true when GPS data is being used to correct states estimates
    struct Location EKF_origin;     // LLH origin of the NED axis system, internal only
    struct Location &public_origin; // LLH origin of the NED axis system, public functions
    bool validOrigin;               // true when the EKF origin is valid
    ftype gpsSpdAccuracy;           // estimated speed accuracy in m/s returned by the GPS receiver
    ftype gpsPosAccuracy;           // estimated position accuracy in m returned by the GPS receiver
    ftype gpsHgtAccuracy;           // estimated height accuracy in m returned by the GPS receiver
    uint32_t lastGpsVelFail_ms;     // time of last GPS vertical velocity consistency check fail
    uint32_t lastGpsVelPass_ms;     // time of last GPS vertical velocity consistency check pass
    uint32_t lastGpsAidBadTime_ms;  // time in msec gps aiding was last detected to be bad
    ftype posDownAtTakeoff;         // flight vehicle vertical position sampled at transition from on-ground to in-air and used as a reference (m)
    bool useGpsVertVel;             // true if GPS vertical velocity should be used
    ftype yawResetAngle;            // Change in yaw angle due to last in-flight yaw reset in radians. A positive value means the yaw angle has increased.
    uint32_t lastYawReset_ms;       // System time at which the last yaw reset occurred. Returned by getLastYawResetAngle
    bool tiltAlignComplete;         // true when tilt alignment is complete
    bool yawAlignComplete;          // true when yaw alignment is complete
    bool magStateInitComplete;      // true when the magnetic field states have been initialised
    uint8_t stateIndexLim;          // Max state index used during matrix and array operations
    imu_elements imuDataDelayed;    // IMU data at the fusion time horizon
    imu_elements imuDataNew;        // IMU data at the current time horizon
    imu_elements imuDataDownSampledNew; // IMU data at the current time horizon that has been downsampled to a 100Hz rate
    QuaternionF imuQuatDownSampleNew; // Quaternion obtained by rotating through the IMU delta angles since the start of the current down sampled frame
    baro_elements baroDataNew;      // Baro data at the current time horizon
    baro_elements baroDataDelayed;  // Baro data at the fusion time horizon
    range_elements rangeDataNew;    // Range finder data at the current time horizon
    range_elements rangeDataDelayed;// Range finder data at the fusion time horizon
    tas_elements tasDataNew;        // TAS data at the current time horizon
    tas_elements tasDataDelayed;    // TAS data at the fusion time horizon
    bool usingDefaultAirspeed;      // true when a default airspeed is being used instead of a measured value
    mag_elements magDataDelayed;    // Magnetometer data at the fusion time horizon
    gps_elements gpsDataNew;        // GPS data at the current time horizon
    gps_elements gpsDataDelayed;    // GPS data at the fusion time horizon
    uint8_t last_gps_idx;           // sensor ID of the GPS receiver used for the last fusion or reset
    output_elements outputDataNew;  // output state data at the current time step
    output_elements outputDataDelayed; // output state data at the current time step
    Vector3F delAngCorrection;      // correction applied to delta angles used by output observer to track the EKF
    Vector3F velErrintegral;        // integral of output predictor NED velocity tracking error (m)
    Vector3F posErrintegral;        // integral of output predictor NED position tracking error (m.sec)
    ftype badImuVelErrIntegral;     // integral of output predictor D velocity tracking error when bad IMU data is detected (m)
    ftype innovYaw;                 // compass yaw angle innovation (rad)
    uint32_t timeTasReceived_ms;    // time last TAS data was received (msec)
    bool gpsGoodToAlign;            // true when the GPS quality can be used to initialise the navigation system
    uint32_t magYawResetTimer_ms;   // timer in msec used to track how long good magnetometer data is failing innovation consistency checks
    bool consistentMagData;         // true when the magnetometers are passing consistency checks
    bool motorsArmed;               // true when the motors have been armed
    bool prevMotorsArmed;           // value of motorsArmed from previous frame
    bool posVelFusionDelayed;       // true when the position and velocity fusion has been delayed
    bool optFlowFusionDelayed;      // true when the optical flow fusion has been delayed
    bool airSpdFusionDelayed;       // true when the air speed fusion has been delayed
    bool sideSlipFusionDelayed;     // true when the sideslip fusion has been delayed
    bool airDataFusionWindOnly;     // true when  sideslip and airspeed fusion is only allowed to modify the wind states
    Vector3F lastMagOffsets;        // Last magnetometer offsets from COMPASS_ parameters. Used to detect parameter changes.
    bool lastMagOffsetsValid;       // True when lastMagOffsets has been initialized
    Vector2F posResetNE;            // Change in North/East position due to last in-flight reset in metres. Returned by getLastPosNorthEastReset
    uint32_t lastPosReset_ms;       // System time at which the last position reset occurred. Returned by getLastPosNorthEastReset
    Vector2F velResetNE;            // Change in North/East velocity due to last in-flight reset in metres/sec. Returned by getLastVelNorthEastReset
    uint32_t lastVelReset_ms;       // System time at which the last velocity reset occurred. Returned by getLastVelNorthEastReset
    ftype posResetD;                // Change in Down position due to last in-flight reset in metres. Returned by getLastPosDowntReset
    uint32_t lastPosResetD_ms;      // System time at which the last position reset occurred. Returned by getLastPosDownReset
    ftype yawTestRatio;             // square of magnetometer yaw angle innovation divided by fail threshold
    QuaternionF prevQuatMagReset;    // Quaternion from the last time the magnetic field state reset condition test was performed
    ftype hgtInnovFiltState;        // state used for fitering of the height innovations used for pre-flight checks
    uint8_t magSelectIndex;         // Index of the magnetometer that is being used by the EKF
    bool runUpdates;                // boolean true when the EKF updates can be run
    uint32_t framesSincePredict;    // number of frames lapsed since EKF instance did a state prediction
    bool startPredictEnabled;       // boolean true when the frontend has given permission to start a new state prediciton cycle
    uint8_t localFilterTimeStep_ms; // average number of msec between filter updates
    ftype posDownObsNoise;          // observation noise variance on the vertical position used by the state and covariance update step (m^2)
    Vector3F delAngCorrected;       // corrected IMU delta angle vector at the EKF time horizon (rad)
    Vector3F delVelCorrected;       // corrected IMU delta velocity vector at the EKF time horizon (m/s)
    bool magFieldLearned;           // true when the magnetic field has been learned
    uint32_t wasLearningCompass_ms; // time when we were last waiting for compass learn to complete
    Vector3F earthMagFieldVar;      // NED earth mag field variances for last learned field (mGauss^2)
    Vector3F bodyMagFieldVar;       // XYZ body mag field variances for last learned field (mGauss^2)
    bool delAngBiasLearned;         // true when the gyro bias has been learned
    nav_filter_status filterStatus; // contains the status of various filter outputs
    ftype ekfOriginHgtVar;          // Variance of the EKF WGS-84 origin height estimate (m^2)
    double ekfGpsRefHgt;            // floating point representation of the WGS-84 reference height used to convert GPS height to local height (m)
    uint32_t lastOriginHgtTime_ms;  // last time the ekf's WGS-84 origin height was corrected
    Vector3F outputTrackError;      // attitude (rad), velocity (m/s) and position (m) tracking error magnitudes from the output observer
    Vector3F velOffsetNED;          // This adds to the earth frame velocity estimate at the IMU to give the velocity at the body origin (m/s)
    Vector3F posOffsetNED;          // This adds to the earth frame position estimate at the IMU to give the position at the body origin (m)
    uint32_t firstInitTime_ms;      // First time the initialise function was called (msec)
    uint32_t lastInitFailReport_ms; // Last time the buffer initialisation failure report was sent (msec)
    ftype tiltErrorVariance;        // variance of the angular uncertainty measured perpendicular to the vertical (rad^2)

    // variables used to calculate a vertical velocity that is kinematically consistent with the vertical position
    struct {
        ftype pos;
        ftype vel;
        ftype acc;
    } vertCompFiltState;

    // variables used by the pre-initialisation GPS checks
    struct Location gpsloc_prev;    // LLH location of previous GPS measurement
    uint32_t lastPreAlignGpsCheckTime_ms;   // last time in msec the GPS quality was checked during pre alignment checks
    ftype gpsDriftNE;               // amount of drift detected in the GPS position during pre-flight GPs checks
    ftype gpsVertVelFilt;           // amount of filtered vertical GPS velocity detected during pre-flight GPS checks
    ftype gpsHorizVelFilt;          // amount of filtered horizontal GPS velocity detected during pre-flight GPS checks

    // variable used by the in-flight GPS quality check
    bool gpsSpdAccPass;             // true when reported GPS speed accuracy passes in-flight checks
    bool gpsVertAccPass;            // true when reported GPS vertical accuracy passes in-flight checks
    bool ekfInnovationsPass;        // true when GPS innovations pass in-flight checks
    ftype sAccFilterState1;         // state variable for LPF applied to reported GPS speed accuracy
    ftype sAccFilterState2;         // state variable for peak hold filter applied to reported GPS speed
    uint32_t lastGpsCheckTime_ms;   // last time in msec the GPS quality was checked
    uint32_t lastGpsInnovPassTime_ms;  // last time in msec the GPS innovations passed
    uint32_t lastGpsInnovFailTime_ms;  // last time in msec the GPS innovations failed
    uint32_t lastGpsVertAccPassTime_ms;  // last time in msec the GPS vertical accuracy test passed
    uint32_t lastGpsVertAccFailTime_ms;  // last time in msec the GPS vertical accuracy test failed
    bool gpsAccuracyGood;           // true when the GPS accuracy is considered to be good enough for safe flight.
    bool gpsAccuracyGoodForAltitude; // true when the GPS accuracy is considered to be good enough to use it as an altitude source.
    Vector3F gpsVelInnov;           // gps velocity innovations
    Vector3F gpsVelVarInnov;        // gps velocity innovation variances
    uint32_t gpsVelInnovTime_ms;    // system time that gps velocity innovations were recorded (to detect timeouts)

    // variables added for optical flow fusion
    EKF_obs_buffer_t<of_elements> storedOF;    // OF data buffer
    bool flowDataValid;             // true while optical flow data is still fresh
    Vector2F auxFlowObsInnov;       // optical flow rate innovation from 1-state terrain offset estimator
    uint32_t flowValidMeaTime_ms;   // time stamp from latest valid flow measurement (msec)
    uint32_t rngValidMeaTime_ms;    // time stamp from latest valid range measurement (msec)
    uint32_t flowMeaTime_ms;        // time stamp from latest flow measurement (msec)
    uint32_t gndHgtValidTime_ms;    // time stamp from last terrain offset state update (msec)
    Vector2 flowVarInnov;           // optical flow innovations variances (rad/sec)^2
    Vector2 flowInnov;              // optical flow LOS innovations (rad/sec)
    uint32_t flowInnovTime_ms;      // system time that optical flow innovations and variances were recorded (to detect timeouts)
    ftype Popt;                     // Optical flow terrain height state covariance (m^2)
    ftype terrainState;             // terrain position state (m)
    ftype prevPosN;                 // north position at last measurement
    ftype prevPosE;                 // east position at last measurement
    ftype varInnovRng;              // range finder observation innovation variance (m^2)
    ftype innovRng;                 // range finder observation innovation (m)
    struct {
        uint32_t timestamp_ms;      // system timestamp of last correct optical flow sample (used for calibration)
        Vector2f flowRate;          // latest corrected optical flow flow rate (used for calibration)
        Vector2f bodyRate;          // latest corrected optical flow body rate (used for calibration)
        Vector2f losPred;           // EKF estimated component of flowRate that comes from vehicle movement (not rotation)
    } flowCalSample;

    ftype hgtMea;                   // height measurement derived from either baro, gps or range finder data (m)
    bool inhibitGndState;           // true when the terrain position state is to remain constant
    uint32_t prevFlowFuseTime_ms;   // time both flow measurement components passed their innovation consistency checks
    Vector2 flowTestRatio;          // square of optical flow innovations divided by fail threshold used by main filter where >1.0 is a fail
    Vector2F auxFlowTestRatio;      // sum of squares of optical flow innovation divided by fail threshold used by 1-state terrain offset estimator
    ftype R_LOS;                    // variance of optical flow rate measurements (rad/sec)^2
    ftype auxRngTestRatio;          // square of range finder innovations divided by fail threshold used by main filter where >1.0 is a fail
    Vector2F flowGyroBias;          // bias error of optical flow sensor gyro output
    bool rangeDataToFuse;           // true when valid range finder height data has arrived at the fusion time horizon.
    bool baroDataToFuse;            // true when valid baro height finder data has arrived at the fusion time horizon.
    bool gpsDataToFuse;             // true when valid GPS data has arrived at the fusion time horizon.
    bool magDataToFuse;             // true when valid magnetometer data has arrived at the fusion time horizon
    enum AidingMode {AID_ABSOLUTE=0,    // GPS or some other form of absolute position reference aiding is being used (optical flow may also be used in parallel) so position estimates are absolute.
                     AID_NONE=1,       // no aiding is being used so only attitude and height estimates are available. Either constVelMode or constPosMode must be used to constrain tilt drift.
                     AID_RELATIVE=2    // only optical flow aiding is being used so position estimates will be relative
                    };
    AidingMode PV_AidingMode;       // Defines the preferred mode for aiding of velocity and position estimates from the INS
    AidingMode PV_AidingModePrev;   // Value of PV_AidingMode from the previous frame - used to detect transitions
    bool gndOffsetValid;            // true when the ground offset state can still be considered valid
    Vector3F delAngBodyOF;          // bias corrected delta angle of the vehicle IMU measured summed across the time since the last OF measurement
    ftype delTimeOF;                // time that delAngBodyOF is summed across
    bool flowFusionActive;          // true when optical flow fusion is active

    Vector3F accelPosOffset;        // position of IMU accelerometer unit in body frame (m)

    // Range finder
    ftype baroHgtOffset;                    // offset applied when when switching to use of Baro height
    ftype rngOnGnd;                         // Expected range finder reading in metres when vehicle is on ground
    ftype storedRngMeas[2][3];              // Ringbuffer of stored range measurements for dual range sensors
    uint32_t storedRngMeasTime_ms[2][3];    // Ringbuffers of stored range measurement times for dual range sensors
    uint32_t lastRngMeasTime_ms;            // Timestamp of last range measurement
    uint8_t rngMeasIndex[2];                // Current range measurement ringbuffer index for dual range sensors
    bool terrainHgtStable;                  // true when the terrain height is stable enough to be used as a height reference

    // body frame odometry fusion
#if EK3_FEATURE_BODY_ODOM
    EKF_obs_buffer_t<vel_odm_elements> storedBodyOdm;    // body velocity data buffer
    vel_odm_elements bodyOdmDataNew;       // Body frame odometry data at the current time horizon
    vel_odm_elements bodyOdmDataDelayed;  // Body  frame odometry data at the fusion time horizon
#endif
    uint32_t lastbodyVelPassTime_ms;    // time stamp when the body velocity measurement last passed innovation consistency checks (msec)
    Vector3 bodyVelTestRatio;           // Innovation test ratios for body velocity XYZ measurements
    Vector3 varInnovBodyVel;            // Body velocity XYZ innovation variances (m/sec)^2
    Vector3 innovBodyVel;               // Body velocity XYZ innovations (m/sec)
    uint32_t prevBodyVelFuseTime_ms;    // previous time all body velocity measurement components passed their innovation consistency checks (msec)
    uint32_t bodyOdmMeasTime_ms;        // time body velocity measurements were accepted for input to the data buffer (msec)
    bool bodyVelFusionDelayed;          // true when body frame velocity fusion has been delayed
    bool bodyVelFusionActive;           // true when body frame velocity fusion is active

#if EK3_FEATURE_BODY_ODOM
    // wheel sensor fusion
    EKF_obs_buffer_t<wheel_odm_elements> storedWheelOdm;    // body velocity data buffer
    wheel_odm_elements wheelOdmDataDelayed;   // Body  frame odometry data at the fusion time horizon
#endif

    // GPS yaw sensor fusion
    uint32_t yawMeasTime_ms;            // system time GPS yaw angle was last input to the data buffer
    EKF_obs_buffer_t<yaw_elements> storedYawAng;    // GPS yaw angle buffer
    yaw_elements yawAngDataNew;         // GPS yaw angle at the current time horizon
    yaw_elements yawAngDataDelayed;     // GPS yaw angle at the fusion time horizon
    yaw_elements yawAngDataStatic;      // yaw angle (regardless of yaw source) when the vehicle was last on ground and not moving

    // Range Beacon Sensor Fusion
    EKF_obs_buffer_t<rng_bcn_elements> storedRangeBeacon; // Beacon range buffer
    rng_bcn_elements rngBcnDataDelayed; // Range beacon data at the fusion time horizon
    uint32_t lastRngBcnPassTime_ms;     // time stamp when the range beacon measurement last passed innovation consistency checks (msec)
    ftype rngBcnTestRatio;              // Innovation test ratio for range beacon measurements
    bool rngBcnHealth;                  // boolean true if range beacon measurements have passed innovation consistency check
    ftype varInnovRngBcn;               // range beacon observation innovation variance (m^2)
    ftype innovRngBcn;                  // range beacon observation innovation (m)
    uint32_t lastTimeRngBcn_ms[4];      // last time we received a range beacon measurement (msec)
    bool rngBcnDataToFuse;              // true when there is new range beacon data to fuse
    Vector3F beaconVehiclePosNED;       // NED position estimate from the beacon system (NED)
    ftype beaconVehiclePosErr;          // estimated position error from the beacon system (m)
    uint32_t rngBcnLast3DmeasTime_ms;   // last time the beacon system returned a 3D fix (msec)
    bool rngBcnGoodToAlign;             // true when the range beacon systems 3D fix can be used to align the filter
    uint8_t lastRngBcnChecked;          // index of the last range beacon checked for data
    Vector3F receiverPos;               // receiver NED position (m) - alignment 3 state filter
    ftype receiverPosCov[3][3];         // Receiver position covariance (m^2) - alignment 3 state filter (
    bool rngBcnAlignmentStarted;        // True when the initial position alignment using range measurements has started
    bool rngBcnAlignmentCompleted;      // True when the initial position alignment using range measurements has finished
    uint8_t lastBeaconIndex;            // Range beacon index last read -  used during initialisation of the 3-state filter
    Vector3F rngBcnPosSum;              // Sum of range beacon NED position (m) - used during initialisation of the 3-state filter
    uint8_t numBcnMeas;                 // Number of beacon measurements - used during initialisation of the 3-state filter
    ftype rngSum;                       // Sum of range measurements (m) - used during initialisation of the 3-state filter
    uint8_t N_beacons;                  // Number of range beacons in use
    ftype maxBcnPosD;                   // maximum position of all beacons in the down direction (m)
    ftype minBcnPosD;                   // minimum position of all beacons in the down direction (m)
    bool usingMinHypothesis;            // true when the min beacon constellation offset hypothesis is being used

    ftype bcnPosDownOffsetMax;          // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
    ftype bcnPosOffsetMaxVar;           // Variance of the bcnPosDownOffsetMax state (m)
    ftype maxOffsetStateChangeFilt;     // Filtered magnitude of the change in bcnPosOffsetHigh

    ftype bcnPosDownOffsetMin;          // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
    ftype bcnPosOffsetMinVar;           // Variance of the bcnPosDownOffsetMin state (m)
    ftype minOffsetStateChangeFilt;     // Filtered magnitude of the change in bcnPosOffsetLow

    Vector3F bcnPosOffsetNED;           // NED position of the beacon origin in earth frame (m)
    bool bcnOriginEstInit;              // True when the beacon origin has been initialised

    // Range Beacon Fusion Debug Reporting
    uint8_t rngBcnFuseDataReportIndex;// index of range beacon fusion data last reported
    struct rngBcnFusionReport_t {
        ftype rng;          // measured range to beacon (m)
        ftype innov;        // range innovation (m)
        ftype innovVar;     // innovation variance (m^2)
        ftype testRatio;    // innovation consistency test ratio
        Vector3F beaconPosNED; // beacon NED position
    } *rngBcnFusionReport;

#if EK3_FEATURE_DRAG_FUSION
    // drag fusion for multicopter wind estimation
    EKF_obs_buffer_t<drag_elements> storedDrag;
    drag_elements dragSampleDelayed;
    drag_elements dragDownSampled;	    // down sampled from filter prediction rate to observation rate
    uint8_t dragSampleCount;	        // number of drag specific force samples accumulated at the filter prediction rate
    ftype dragSampleTimeDelta;	        // time integral across all samples used to form _drag_down_sampled (sec)
    Vector2F innovDrag;		            // multirotor drag measurement innovation (m/sec**2)
	Vector2F innovDragVar;	            // multirotor drag measurement innovation variance ((m/sec**2)**2)
	Vector2F dragTestRatio;		        // drag innovation consistency check ratio
#endif
	uint32_t lastDragPassTime_ms;       // system time that drag samples were last successfully fused
    bool dragFusionEnabled;

    // height source selection logic
    AP_NavEKF_Source::SourceZ activeHgtSource;  // active height source
    AP_NavEKF_Source::SourceZ prevHgtSource;    // previous height source used to detect changes in source

    // Movement detector
    bool takeOffDetected;           // true when takeoff for optical flow navigation has been detected
    ftype rngAtStartOfFlight;       // range finder measurement at start of flight
    uint32_t timeAtArming_ms;       // time in msec that the vehicle armed

    // baro ground effect
    ftype meaHgtAtTakeOff;            // height measured at commencement of takeoff

    // control of post takeoff magnetic field and heading resets
    bool finalInflightYawInit;      // true when the final post takeoff initialisation of yaw angle has been performed
    uint8_t magYawAnomallyCount;    // Number of times the yaw has been reset due to a magnetic anomaly during initial ascent
    bool finalInflightMagInit;      // true when the final post takeoff initialisation of magnetic field states been performed
    bool magStateResetRequest;      // true if magnetic field states need to be reset using the magnetomter measurements
    bool magYawResetRequest;        // true if the vehicle yaw and magnetic field states need to be reset using the magnetometer measurements
    bool gpsYawResetRequest;        // true if the vehicle yaw needs to be reset to the GPS course
    ftype posDownAtLastMagReset;    // vertical position last time the mag states were reset (m)
    ftype yawInnovAtLastMagReset;   // magnetic yaw innovation last time the yaw and mag field states were reset (rad)
    QuaternionF quatAtLastMagReset;  // quaternion states last time the mag states were reset

    // Used by on ground movement check required when operating on ground without a yaw reference
    ftype gyro_diff;                    // filtered gyro difference (rad/s)
    ftype accel_diff;                   // filtered acceerometer difference (m/s/s)
    Vector3F gyro_prev;                 // gyro vector from previous time step (rad/s)
    Vector3F accel_prev;                // accelerometer vector from previous time step (m/s/s)
    bool onGroundNotMoving;             // true when on the ground and not moving
    uint32_t lastMoveCheckLogTime_ms;   // last time the movement check data was logged (msec)

	// variables used to inhibit accel bias learning
    bool inhibitDelVelBiasStates;       // true when all IMU delta velocity bias states are de-activated
    bool dvelBiasAxisInhibit[3] {};		// true when IMU delta velocity bias states for a specific axis is de-activated
	Vector3F dvelBiasAxisVarPrev;		// saved delta velocity XYZ bias variances (m/sec)**2

#if EK3_FEATURE_EXTERNAL_NAV
    // external navigation fusion
    EKF_obs_buffer_t<ext_nav_elements> storedExtNav; // external navigation data buffer
    ext_nav_elements extNavDataDelayed; // External nav at the fusion time horizon
    uint32_t extNavMeasTime_ms;         // time external measurements were accepted for input to the data buffer (msec)
    uint32_t extNavLastPosResetTime_ms; // last time the external nav systen performed a position reset (msec)
    bool extNavDataToFuse;              // true when there is new external nav data to fuse
    bool extNavUsedForPos;              // true when the external nav data is being used as a position reference.
    EKF_obs_buffer_t<ext_nav_vel_elements> storedExtNavVel;    // external navigation velocity data buffer
    ext_nav_vel_elements extNavVelDelayed;  // external navigation velocity data at the fusion time horizon.  Already corrected for sensor position
    uint32_t extNavVelMeasTime_ms;      // time external navigation velocity measurements were accepted for input to the data buffer (msec)
    bool extNavVelToFuse;               // true when there is new external navigation velocity to fuse
    Vector3F extNavVelInnov;            // external nav velocity innovations
    Vector3F extNavVelVarInnov;         // external nav velocity innovation variances
    uint32_t extNavVelInnovTime_ms;     // system time that external nav velocity innovations were recorded (to detect timeouts)
    EKF_obs_buffer_t<yaw_elements> storedExtNavYawAng;  // external navigation yaw angle buffer
    yaw_elements extNavYawAngDataDelayed;   // external navigation yaw angle at the fusion time horizon
    uint32_t last_extnav_yaw_fusion_ms; // system time that external nav yaw was last fused
#endif // EK3_FEATURE_EXTERNAL_NAV
    bool useExtNavVel;                  // true if external nav velocity should be used

    // flags indicating severe numerical errors in innovation variance calculation for different fusion operations
    struct {
        bool bad_xmag:1;
        bool bad_ymag:1;
        bool bad_zmag:1;
        bool bad_airspeed:1;
        bool bad_sideslip:1;
        bool bad_nvel:1;
        bool bad_evel:1;
        bool bad_dvel:1;
        bool bad_npos:1;
        bool bad_epos:1;
        bool bad_dpos:1;
        bool bad_yaw:1;
        bool bad_decl:1;
        bool bad_xflow:1;
        bool bad_yflow:1;
        bool bad_rngbcn:1;
        bool bad_xvel:1;
        bool bad_yvel:1;
        bool bad_zvel:1;
    } faultStatus;

    // flags indicating which GPS quality checks are failing
    union {
        struct {
            bool bad_sAcc:1;
            bool bad_hAcc:1;
            bool bad_vAcc:1;
            bool bad_yaw:1;
            bool bad_sats:1;
            bool bad_VZ:1;
            bool bad_horiz_drift:1;
            bool bad_hdop:1;
            bool bad_vert_vel:1;
            bool bad_fix:1;
            bool bad_horiz_vel:1;
        };
        uint16_t value;
    } gpsCheckStatus;

    // states held by magnetometer fusion across time steps
    // magnetometer X,Y,Z measurements are fused across three time steps
    // to level computational load as this is an expensive operation
    struct {
        ftype q0;
        ftype q1;
        ftype q2;
        ftype q3;
        ftype magN;
        ftype magE;
        ftype magD;
        ftype magXbias;
        ftype magYbias;
        ftype magZbias;
        Matrix3F DCM;
        Vector3F MagPred;
        ftype R_MAG;
        Vector9 SH_MAG;
    } mag_state;

    // string representing last reason for prearm failure
    char prearm_fail_string[40];

    // earth field from WMM tables
    bool have_table_earth_field;   // true when we have initialised table_earth_field_ga
    Vector3F table_earth_field_ga; // earth field from WMM tables
    ftype table_declination;       // declination in radians from the tables

    // 1Hz update
    uint32_t last_oneHz_ms;
    void oneHzUpdate(void);

    // move EKF origin at 1Hz
    void moveEKFOrigin(void);

    // handle earth field updates
    void getEarthFieldTable(const Location &loc);
    void checkUpdateEarthField(void);
    
    // timing statistics
    struct ekf_timing timing;

    // when was attitude filter status last non-zero?
    uint32_t last_filter_ok_ms;
    
    // should we assume zero sideslip?
    bool assume_zero_sideslip(void) const;

    // vehicle specific initial gyro bias uncertainty
    ftype InitialGyroBiasUncertainty(void) const;

    /*
      learn magnetometer biases from GPS yaw. Return true if the
      resulting mag vector is close enough to the one predicted by GPS
      yaw to use it for fallback
    */
    bool learnMagBiasFromGPS(void);

    uint32_t last_gps_yaw_ms; // last time the EKF attempted to use the GPS yaw
    uint32_t last_gps_yaw_fuse_ms; // last time the EKF successfully fused the GPS yaw
    bool gps_yaw_mag_fallback_ok;
    bool gps_yaw_mag_fallback_active;
    uint8_t gps_yaw_fallback_good_counter;

    /*
    Update the on ground and not moving check.
    Should be called once per IMU update.
    Only updates when on ground and when operating with an external yaw sensor
    */
    void updateMovementCheck(void);

    // The following declarations are used to control when the main navigation filter resets it's yaw to the estimate provided by the GSF
    uint32_t EKFGSF_yaw_reset_ms;           // timestamp of last emergency yaw reset (uSec)
    uint32_t EKFGSF_yaw_reset_request_ms;   // timestamp of last emergency yaw reset request (uSec)
    uint8_t EKFGSF_yaw_reset_count;         // number of emergency yaw resets performed
    bool EKFGSF_run_filterbank;             // true when the filter bank is active
    uint8_t EKFGSF_yaw_valid_count;         // number of updates since the last invalid yaw estimate

    // logging timestamps
    uint32_t lastLogTime_ms;
    uint32_t lastUpdateTime_ms;
    uint32_t lastEkfStateVarLogTime_ms;
    uint32_t lastTimingLogTime_ms;

    // bits in EK3_AFFINITY
    enum ekf_affinity {
        EKF_AFFINITY_GPS  = (1U<<0),
        EKF_AFFINITY_BARO = (1U<<1),
        EKF_AFFINITY_MAG  = (1U<<2),
        EKF_AFFINITY_ARSP = (1U<<3),
    };

    // update selected_sensors for this core
    void update_sensor_selection(void);
    void update_gps_selection(void);
    void update_mag_selection(void);
    void update_baro_selection(void);
    void update_airspeed_selection(void);

    // selected and preferred sensor instances. We separate selected
    // from preferred so that calcGpsGoodToAlign() can ensure the
    // preferred sensor is ready. Note that magSelectIndex is used for
    // compass selection
    uint8_t selected_gps;
    uint8_t preferred_gps;
    uint8_t selected_baro;
    uint8_t selected_airspeed;

    // source reset handling
    AP_NavEKF_Source::SourceXY posxy_source_last;   // horizontal position source on previous iteration (used to detect a changes)
    bool posxy_source_reset;                        // true when the horizontal position source has changed but the position has not yet been reset
    AP_NavEKF_Source::SourceYaw yaw_source_last;    // yaw source on previous iteration (used to detect a change)
    bool yaw_source_reset;                          // true when the yaw source has changed but the yaw has not yet been reset

    // logging functions shared by cores:
    void Log_Write_XKF1(uint64_t time_us) const;
    void Log_Write_XKF2(uint64_t time_us) const;
    void Log_Write_XKF3(uint64_t time_us) const;
    void Log_Write_XKF4(uint64_t time_us) const;
    void Log_Write_XKF5(uint64_t time_us) const;
    void Log_Write_XKFS(uint64_t time_us) const;
    void Log_Write_Quaternion(uint64_t time_us) const;
    void Log_Write_Beacon(uint64_t time_us);
    void Log_Write_BodyOdom(uint64_t time_us);
    void Log_Write_State_Variances(uint64_t time_us);
    void Log_Write_Timing(uint64_t time_us);
    void Log_Write_GSF(uint64_t time_us);
};
