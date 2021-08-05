/*
  24 state EKF based on the derivation in https://github.com/priseborough/
  InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/
  GenerateNavFilterEquations.m

  Converted from Matlab to C++ by Paul Riseborough

  EKF Tuning parameters refactored by Tom Cauchois

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

#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_NavEKF/AP_Nav_Common.h>

class NavEKF2_core;

class NavEKF2 {
    friend class NavEKF2_core;

public:
    NavEKF2();

    /* Do not allow copies */
    NavEKF2(const NavEKF2 &other) = delete;
    NavEKF2 &operator=(const NavEKF2&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // allow logging to determine the number of active cores
    uint8_t activeCores(void) const {
        return num_cores;
    }

    // Initialise the filter
    bool InitialiseFilter(void);

    // Update Filter States - this should be called whenever new IMU data is available
    void UpdateFilter(void);

    // Check basic filter health metrics and return a consolidated health status
    bool healthy(void) const;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    // returns the index of the primary core
    // return -1 if no primary core selected
    int8_t getPrimaryCoreIndex(void) const;

    // returns the index of the IMU of the primary core
    // return -1 if no primary core selected
    int8_t getPrimaryCoreIMUIndex(void) const;
    
    // Write the last calculated NE position relative to the reference point (m) for the specified instance.
    // An out of range instance (eg -1) returns data for the primary instance
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosNE(int8_t instance, Vector2f &posNE) const;

    // Write the last calculated D position relative to the reference point (m) for the specified instance.
    // An out of range instance (eg -1) returns data for the primary instance
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosD(int8_t instance, float &posD) const;

    // return NED velocity in m/s for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void getVelNED(int8_t instance, Vector3f &vel) const;

    // return estimate of true airspeed vector in body frame in m/s for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    // returns false if estimate is unavailable
    bool getAirSpdVec(int8_t instance, Vector3f &vel) const;

    // Return the rate of change of vertical position in the down direction (dPosD/dt) in m/s for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    // This can be different to the z component of the EKF velocity state because it will fluctuate with height errors and corrections in the EKF
    // but will always be kinematically consistent with the z component of the EKF position state
    float getPosDownDerivative(int8_t instance) const;

    // return body axis gyro bias estimates in rad/sec for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void getGyroBias(int8_t instance, Vector3f &gyroBias) const;

    // return body axis gyro scale factor error as a percentage for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void getGyroScaleErrorPercentage(int8_t instance, Vector3f &gyroScale) const;

    // reset body axis gyro bias estimates
    void resetGyroBias(void);

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    bool resetHeightDatum(void);

    // return the horizontal speed limit in m/s set by optical flow sensor limits
    // return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const;

    // return the Z-accel bias estimate in m/s^2 for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void getAccelZBias(int8_t instance, float &zbias) const;

    // return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
    // An out of range instance (eg -1) returns data for the primary instance
    void getWind(int8_t instance, Vector3f &wind) const;

    // return earth magnetic field estimates in measurement units / 1000 for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void getMagNED(int8_t instance, Vector3f &magNED) const;

    // return body magnetic field estimates in measurement units / 1000 for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void getMagXYZ(int8_t instance, Vector3f &magXYZ) const;

    // Return estimated magnetometer offsets
    // Return true if magnetometer offsets are valid
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    // Return the last calculated latitude, longitude and height in WGS-84
    // If a calculated location isn't available, return a raw GPS measurement
    // The status will return true if a calculation or raw measurement is available
    // The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
    bool getLLH(struct Location &loc) const;

    // Return the latitude and longitude and height used to set the NED origin for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    // All NED positions calculated by the filter are relative to this location
    // Returns false if the origin has not been set
    bool getOriginLLH(int8_t instance, struct Location &loc) const;

    // set the latitude and longitude and height used to set the NED origin
    // All NED positions calculated by the filter will be relative to this location
    // The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
    // Returns false if the filter has rejected the attempt to set the origin
    bool setOriginLLH(const Location &loc);

    // return estimated height above ground level
    // return false if ground height is not being estimated.
    bool getHAGL(float &HAGL) const;

    // return the Euler roll, pitch and yaw angle in radians for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void getEulerAngles(int8_t instance, Vector3f &eulers) const;

    // return the transformation matrix from XYZ (body) to NED axes
    void getRotationBodyToNED(Matrix3f &mat) const;

    // return the transformation matrix from XYZ (body) to NED axes
    void getQuaternionBodyToNED(int8_t instance, Quaternion &quat) const;

    // return the quaternions defining the rotation from NED to autopilot axes
    void getQuaternion(int8_t instance, Quaternion &quat) const;

    // return the innovations for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void  getInnovations(int8_t index, Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    // return the innovation consistency test ratios for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void  getVariances(int8_t instance, float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

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
    void  writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset);

    // Set to true if the terrain underneath is stable enough to be used as a height reference
    // in combination with a range finder. Set to false if the terrain underneath the vehicle
    // cannot be used as a height reference. Use to prevent range finder operation otherwise
    // enabled by the combination of EK2_RNG_AID_HGT and EK2_RNG_USE_SPD parameters.
    void setTerrainHgtStable(bool val);

    /*
    return the filter fault status as a bitmasked integer for the specified instance
    An out of range instance (eg -1) returns data for the primary instance
     0 = quaternions are NaN
     1 = velocities are NaN
     2 = badly conditioned X magnetometer fusion
     3 = badly conditioned Y magnetometer fusion
     4 = badly conditioned Z magnetometer fusion
     5 = badly conditioned airspeed fusion
     6 = badly conditioned synthetic sideslip fusion
     7 = filter is not initialised
    */
    void  getFilterFaults(int8_t instance, uint16_t &faults) const;

    /*
    return filter gps quality check status for the specified instance
    An out of range instance (eg -1) returns data for the primary instance
    */
    void  getFilterGpsStatus(int8_t instance, nav_gps_status &faults) const;

    /*
    return filter status flags for the specified instance
    An out of range instance (eg -1) returns data for the primary instance
    */
    void  getFilterStatus(int8_t instance, nav_filter_status &status) const;

    // send an EKF_STATUS_REPORT message to GCS
    void send_status_report(mavlink_channel_t chan) const;

    // provides the height limit to be observed by the control loops
    // returns false if no height limiting is required
    // this is needed to ensure the vehicle does not fly too high when using optical flow navigation
    bool getHeightControlLimit(float &height) const;

    // return the amount of yaw angle change (in radians) due to the last yaw angle reset or core selection switch
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    uint32_t getLastYawResetAngle(float &yawAngDelta);

    // return the amount of NE position change due to the last position reset in metres
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &posDelta);

    // return the amount of NE velocity change due to the last velocity reset in metres/sec
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    // return the amount of vertical position change due to the last reset in metres
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosDownReset(float &posDelta);

    // set and save the _baroAltNoise parameter
    void set_baro_alt_noise(float noise) { _baroAltNoise.set_and_save(noise); };

    // allow the enable flag to be set by Replay
    void set_enable(bool enable) { _enable.set_enable(enable); }

    /*
     * Write position and quaternion data from an external navigation system
     *
     * pos        : position in the RH navigation frame. Frame is assumed to be NED (m)
     * quat       : quaternion desribing the rotation from navigation frame to body frame
     * posErr     : 1-sigma spherical position error (m)
     * angErr     : 1-sigma spherical angle error (rad)
     * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
     * delay_ms   : average delay of external nav system measurements relative to inertial measurements
     * resetTime_ms : system time of the last position reset request (mSec)
     *
     * Sensor offsets are pulled directly from the AP_VisualOdom library
     *
    */
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms);

    /*
     * Write velocity data from an external navigation system
     * vel : velocity in NED (m)
     * err : velocity error (m/s)
     * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
     * delay_ms   : average delay of external nav system measurements relative to inertial measurements
     */
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms);

    /*
      check if switching lanes will reduce the normalised
      innovations. This is called when the vehicle code is about to
      trigger an EKF failsafe, and it would like to avoid that by
      using a different EKF lane
     */
    void checkLaneSwitch(void);

    /*
      Request a reset of the EKF yaw. This is called when the vehicle code is about to
      trigger an EKF failsafe, and it would like to avoid that.
     */
    void requestYawReset(void);

    // write EKF information to on-board logs
    void Log_Write();

    // check if external navigation is being used for yaw observation
    bool isExtNavUsedForYaw(void) const;

    // check if configured to use GPS for horizontal position estimation
    bool configuredToUseGPSForPosXY(void) const;

    // Writes the default equivalent airspeed in m/s to be used in forward flight if a measured airspeed is required and not available.
    void writeDefaultAirSpeed(float airspeed);

private:
    uint8_t num_cores; // number of allocated cores
    uint8_t primary;   // current primary core
    NavEKF2_core *core = nullptr;
    bool core_malloc_failed;

    uint32_t _frameTimeUsec;        // time per IMU frame
    uint8_t  _framesPerPrediction;  // expected number of IMU frames per prediction

    // EKF Mavlink Tuneable Parameters
    AP_Int8  _enable;               // zero to disable EKF2
    AP_Float _gpsHorizVelNoise;     // GPS horizontal velocity measurement noise : m/s
    AP_Float _gpsVertVelNoise;      // GPS vertical velocity measurement noise : m/s
    AP_Float _gpsHorizPosNoise;     // GPS horizontal position measurement noise m
    AP_Float _baroAltNoise;         // Baro height measurement noise : m
    AP_Float _magNoise;             // magnetometer measurement noise : gauss
    AP_Float _easNoise;             // equivalent airspeed measurement noise : m/s
    AP_Float _windVelProcessNoise;  // wind velocity state process noise : m/s^2
    AP_Float _wndVarHgtRateScale;   // scale factor applied to wind process noise due to height rate
    AP_Float _magEarthProcessNoise; // Earth magnetic field process noise : gauss/sec
    AP_Float _magBodyProcessNoise;  // Body magnetic field process noise : gauss/sec
    AP_Float _gyrNoise;             // gyro process noise : rad/s
    AP_Float _accNoise;             // accelerometer process noise : m/s^2
    AP_Float _gyroBiasProcessNoise; // gyro bias state process noise : rad/s
    AP_Float _accelBiasProcessNoise;// accel bias state process noise : m/s^2
    AP_Int16 _hgtDelay_ms;          // effective average delay of Height measurements relative to inertial measurements (msec)
    AP_Int8  _fusionModeGPS;        // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity, 3 = do not use GPS
    AP_Int16  _gpsVelInnovGate;     // Percentage number of standard deviations applied to GPS velocity innovation consistency check
    AP_Int16  _gpsPosInnovGate;     // Percentage number of standard deviations applied to GPS position innovation consistency check
    AP_Int16  _hgtInnovGate;        // Percentage number of standard deviations applied to height innovation consistency check
    AP_Int16  _magInnovGate;        // Percentage number of standard deviations applied to magnetometer innovation consistency check
    AP_Int16  _tasInnovGate;        // Percentage number of standard deviations applied to true airspeed innovation consistency check
    AP_Int8  _magCal;               // Sets activation condition for in-flight magnetometer calibration
    AP_Int8 _gpsGlitchRadiusMax;    // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m
    AP_Float _flowNoise;            // optical flow rate measurement noise
    AP_Int16  _flowInnovGate;       // Percentage number of standard deviations applied to optical flow innovation consistency check
    AP_Int8  _flowDelay_ms;         // effective average delay of optical flow measurements rel to IMU (msec)
    AP_Int16  _rngInnovGate;        // Percentage number of standard deviations applied to range finder innovation consistency check
    AP_Float _maxFlowRate;          // Maximum flow rate magnitude that will be accepted by the filter
    AP_Int8 _altSource;             // Primary alt source during optical flow navigation. 0 = use Baro, 1 = use range finder, 2 = use GPS, 3 = use Range Beacon
    AP_Float _gyroScaleProcessNoise;// gyro scale factor state process noise : 1/s
    AP_Float _rngNoise;             // Range finder noise : m
    AP_Int8 _gpsCheck;              // Bitmask controlling which preflight GPS checks are bypassed
    AP_Int8 _imuMask;               // Bitmask of IMUs to instantiate EKF2 for
    AP_Int16 _gpsCheckScaler;       // Percentage increase to be applied to GPS pre-flight accuracy and drift thresholds
    AP_Float _noaidHorizNoise;      // horizontal position measurement noise assumed when synthesised zero position measurements are used to constrain attitude drift : m
    AP_Float _yawNoise;             // magnetic yaw measurement noise : rad
    AP_Int16 _yawInnovGate;         // Percentage number of standard deviations applied to magnetic yaw innovation consistency check
    AP_Int8 _tauVelPosOutput;       // Time constant of output complementary filter : csec (centi-seconds)
    AP_Int8 _useRngSwHgt;           // Maximum valid range of the range finder as a percentage of the maximum range specified by the sensor driver
    AP_Float _terrGradMax;          // Maximum terrain gradient below the vehicle
    AP_Float _rngBcnNoise;          // Range beacon measurement noise (m)
    AP_Int16 _rngBcnInnovGate;      // Percentage number of standard deviations applied to range beacon innovation consistency check
    AP_Int8  _rngBcnDelay_ms;       // effective average delay of range beacon measurements rel to IMU (msec)
    AP_Float _useRngSwSpd;          // Maximum horizontal ground speed to use range finder as the primary height source (m/s)
    AP_Int8 _magMask;               // Bitmask forcng specific EKF core instances to use simple heading magnetometer fusion.
    AP_Int8 _originHgtMode;         // Bitmask controlling post alignment correction and reporting of the EKF origin height.
    AP_Int8 _flowUse;               // Controls if the optical flow data is fused into the main navigation estimator and/or the terrain estimator.
    AP_Int16 _mag_ef_limit;         // limit on difference between WMM tables and learned earth field.
    AP_Float _hrt_filt_freq;        // frequency of output observer height rate complementary filter in Hz
    AP_Int8 _gsfRunMask;            // mask controlling which EKF2 instances run a separate EKF-GSF yaw estimator
    AP_Int8 _gsfUseMask;            // mask controlling which EKF2 instances will use EKF-GSF yaw estimator data to assit with yaw resets
    AP_Int8 _gsfResetMaxCount;      // maximum number of times the EKF2 is allowed to reset it's yaw to the EKF-GSF estimate

// Possible values for _flowUse
#define FLOW_USE_NONE    0
#define FLOW_USE_NAV     1
#define FLOW_USE_TERRAIN 2

    // Tuning parameters
    const float gpsNEVelVarAccScale = 0.05f;       // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    const float gpsDVelVarAccScale = 0.07f;        // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    const float extNavVelVarAccScale = 0.05f;      // Scale factor applied to ext nav velocity measurement variance due to manoeuvre acceleration
    const float gpsPosVarAccScale = 0.05f;         // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    const uint8_t magDelay_ms = 60;               // Magnetometer measurement delay (msec)
    const uint8_t tasDelay_ms = 240;              // Airspeed measurement delay (msec)
    const uint16_t tiltDriftTimeMax_ms = 15000;    // Maximum number of ms allowed without any form of tilt aiding (GPS, flow, TAS, etc)
    const uint16_t posRetryTimeUseVel_ms = 10000;  // Position aiding retry time with velocity measurements (msec)
    const uint16_t posRetryTimeNoVel_ms = 7000;    // Position aiding retry time without velocity measurements (msec)
    const uint16_t hgtRetryTimeMode0_ms = 10000;   // Height retry time with vertical velocity measurement (msec)
    const uint16_t hgtRetryTimeMode12_ms = 5000;   // Height retry time without vertical velocity measurement (msec)
    const uint16_t tasRetryTime_ms = 5000;         // True airspeed timeout and retry interval (msec)
    const uint16_t magFailTimeLimit_ms = 10000;    // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    const float magVarRateScale = 0.005f;          // scale factor applied to magnetometer variance due to angular rate
    const float gyroBiasNoiseScaler = 2.0f;        // scale factor applied to gyro bias state process noise when on ground
    const uint8_t hgtAvg_ms = 100;                // average number of msec between height measurements
    const uint8_t betaAvg_ms = 100;               // average number of msec between synthetic sideslip measurements
    const float covTimeStepMax = 0.1f;             // maximum time (sec) between covariance prediction updates
    const float covDelAngMax = 0.05f;              // maximum delta angle between covariance prediction updates
    const float DCM33FlowMin = 0.71f;              // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    const float fScaleFactorPnoise = 1e-10f;       // Process noise added to focal length scale factor state variance at each time step
    const uint8_t flowTimeDeltaAvg_ms = 100;       // average interval between optical flow measurements (msec)
    const uint8_t flowIntervalMax_ms = 100;       // maximum allowable time between flow fusion events
    const float gndEffectBaroScaler = 4.0f;        // scaler applied to the barometer observation variance when ground effect mode is active
    const uint8_t fusionTimeStep_ms = 10;          // The minimum time interval between covariance predictions and measurement fusions in msec
    const float maxYawEstVelInnov = 2.0f;          // Maximum acceptable length of the velocity innovation returned by the EKF-GSF yaw estimator (m/s)

    // origin set by one of the cores
    struct Location common_EKF_origin;
    bool common_origin_valid;

    // time at start of current filter update
    uint64_t imuSampleTime_us;

    // last time of Log_Write
    uint64_t lastLogWrite_us;
    
    struct {
        uint32_t last_function_call;  // last time getLastYawResetAngle was called
        bool core_changed;            // true when a core change happened and hasn't been consumed, false otherwise
        uint32_t last_primary_change; // last time a primary has changed
        float core_delta;             // the amount of yaw change between cores when a change happened
    } yaw_reset_data;

    struct {
        uint32_t last_function_call;  // last time getLastPosNorthEastReset was called
        bool core_changed;            // true when a core change happened and hasn't been consumed, false otherwise
        uint32_t last_primary_change; // last time a primary has changed
        Vector2f core_delta;          // the amount of NE position change between cores when a change happened
    } pos_reset_data;

    struct {
        uint32_t last_function_call;  // last time getLastPosDownReset was called
        bool core_changed;            // true when a core change happened and hasn't been consumed, false otherwise
        uint32_t last_primary_change; // last time a primary has changed
        float core_delta;             // the amount of D position change between cores when a change happened
    } pos_down_reset_data;

    bool runCoreSelection; // true when the primary core has stabilised and the core selection logic can be started

    // time of last lane switch
    uint32_t lastLaneSwitch_ms;

    enum class InitFailures {
        UNKNOWN,
        NO_ENABLE, 
        NO_IMUS, 
        NO_MASK, 
        NO_MEM, 
        NO_SETUP,
        NUM_INIT_FAILURES
    };
    // initialization failure reasons
    const char* initFailureReason[int(InitFailures::NUM_INIT_FAILURES)] {
        "EKF2: unknown initialization failure",
        "EKF2: EK2_enable is false",
        "EKF2: no IMUs available",
        "EKF2: EK2_IMU_MASK is zero",
        "EKF2: insufficient memory available",
        "EKF2: core setup failed"
    };
    InitFailures initFailure;

    // update the yaw reset data to capture changes due to a lane switch
    // new_primary - index of the ekf instance that we are about to switch to as the primary
    // old_primary - index of the ekf instance that we are currently using as the primary
    void updateLaneSwitchYawResetData(uint8_t new_primary, uint8_t old_primary);

    // update the position reset data to capture changes due to a lane switch
    // new_primary - index of the ekf instance that we are about to switch to as the primary
    // old_primary - index of the ekf instance that we are currently using as the primary
    void updateLaneSwitchPosResetData(uint8_t new_primary, uint8_t old_primary);

    // update the position down reset data to capture changes due to a lane switch
    // new_primary - index of the ekf instance that we are about to switch to as the primary
    // old_primary - index of the ekf instance that we are currently using as the primary
    void updateLaneSwitchPosDownResetData(uint8_t new_primary, uint8_t old_primary);

    // return true if a new core has a better score than an existing core, including
    // checks for alignment
    bool coreBetterScore(uint8_t new_core, uint8_t current_core) const;
};
