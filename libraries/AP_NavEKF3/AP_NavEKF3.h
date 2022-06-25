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

#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_NavEKF/AP_NavEKF_Source.h>

class NavEKF3_core;
class EKFGSF_yaw;

class NavEKF3 {
    friend class NavEKF3_core;

public:
    NavEKF3();

    /* Do not allow copies */
    NavEKF3(const NavEKF3 &other) = delete;
    NavEKF3 &operator=(const NavEKF3&) = delete;

    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info2[];

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
    // requires_position should be true if horizontal position configuration should be checked
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const;

    // returns the index of the primary core
    // return -1 if no primary core selected
    int8_t getPrimaryCoreIndex(void) const;

    // returns the index of the IMU of the primary core
    // return -1 if no primary core selected
    int8_t getPrimaryCoreIMUIndex(void) const;

    // Write the last calculated NE position relative to the reference point (m)
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosNE(Vector2f &posNE) const;

    // Write the last calculated D position relative to the reference point (m)
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosD(float &posD) const;

    // return NED velocity in m/s
    void getVelNED(Vector3f &vel) const;

    // return estimate of true airspeed vector in body frame in m/s
    // returns false if estimate is unavailable
    bool getAirSpdVec(Vector3f &vel) const;

    // return the innovation in m/s, innovation variance in (m/s)^2 and age in msec of the last TAS measurement processed
    // returns false if the data is unavilable
    bool getAirSpdHealthData(float &innovation, float &innovationVariance, uint32_t &age_ms) const;

    // Return the rate of change of vertical position in the down direction (dPosD/dt) in m/s
    // This can be different to the z component of the EKF velocity state because it will fluctuate with height errors and corrections in the EKF
    // but will always be kinematically consistent with the z component of the EKF position state
    float getPosDownDerivative() const;

    // return body axis gyro bias estimates in rad/sec for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    void getGyroBias(int8_t instance, Vector3f &gyroBias) const;

    // return accelerometer bias estimate in m/s/s
    // An out of range instance (eg -1) returns data for the primary instance
    void getAccelBias(int8_t instance, Vector3f &accelBias) const;

    //returns index of the active source set used
    uint8_t get_active_source_set() const;

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

    // return the NED wind speed estimates in m/s (positive is air
    // moving in the direction of the axis) returns true if wind state
    // estimation is active
    bool getWind(Vector3f &wind) const;

    // return earth magnetic field estimates in measurement units / 1000
    void getMagNED(Vector3f &magNED) const;

    // return body magnetic field estimates in measurement units / 1000
    void getMagXYZ(Vector3f &magXYZ) const;

    // return the airspeed sensor in use
    uint8_t getActiveAirspeed() const;

    // Return estimated magnetometer offsets
    // Return true if magnetometer offsets are valid
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    // Return the last calculated latitude, longitude and height in WGS-84
    // If a calculated location isn't available, return a raw GPS measurement
    // The status will return true if a calculation or raw measurement is available
    // The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
    bool getLLH(struct Location &loc) const;

    // Return the latitude and longitude and height used to set the NED origin
    // All NED positions calculated by the filter are relative to this location
    // Returns false if the origin has not been set
    bool getOriginLLH(struct Location &loc) const;

    // set the latitude and longitude and height used to set the NED origin
    // All NED positions calculated by the filter will be relative to this location
    // The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
    // Returns false if the filter has rejected the attempt to set the origin
    bool setOriginLLH(const Location &loc);

    // return estimated height above ground level
    // return false if ground height is not being estimated.
    bool getHAGL(float &HAGL) const;

    // return the Euler roll, pitch and yaw angle in radians
    void getEulerAngles(Vector3f &eulers) const;

    // return the transformation matrix from XYZ (body) to NED axes
    void getRotationBodyToNED(Matrix3f &mat) const;

    // return the quaternions defining the rotation from XYZ (body) to NED axes
    void getQuaternionBodyToNED(int8_t instance, Quaternion &quat) const;

    // return the quaternions defining the rotation from NED to XYZ (autopilot) axes
    void getQuaternion(Quaternion &quat) const;

    // return the innovations
    bool getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    // return the innovation consistency test ratios
    bool getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

    // get a source's velocity innovations
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
     * delPos is the XYZ change in linear position measured in body frame and relative to the inertial reference at timeStamp_ms (m)
     * delAng is the XYZ angular rotation measured in body frame and relative to the inertial reference at timeStamp_ms (rad)
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
     * this should not be called at more than the EKF's update rate (50hz or 100hz)
    */
    void writeWheelOdom(float delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset, float radius);

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
     * delay_ms   : average delay of external nav system measurements relative to inertial measurements
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
     4 = badly conditioned Z magnetometer fusion
     5 = badly conditioned airspeed fusion
     6 = badly conditioned synthetic sideslip fusion
     7 = filter is not initialised
    */
    void getFilterFaults(uint16_t &faults) const;

    /*
    return filter status flags
    */
    void getFilterStatus(nav_filter_status &status) const;

    // send an EKF_STATUS_REPORT message to GCS
    void send_status_report(class GCS_MAVLINK &link) const;

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

    // set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
    void setPosVelYawSourceSet(uint8_t source_set_idx);

    // write EKF information to on-board logs
    void Log_Write();

    // are we using (aka fusing) a non-compass yaw?
    bool using_noncompass_for_yaw() const;

    // are we using (aka fusing) external nav for yaw?
    bool using_extnav_for_yaw() const;

    // check if configured to use GPS for horizontal position estimation
    bool configuredToUseGPSForPosXY(void) const;
    
    // Writes the default equivalent airspeed and 1-sigma uncertainty in m/s to be used in forward flight if a measured airspeed is required and not available.
    void writeDefaultAirSpeed(float airspeed, float uncertainty);

    // parameter conversion
    void convert_parameters();

    // returns true when the yaw angle has been aligned
    bool yawAlignmentComplete(void) const;

    // returns true when the state estimates are significantly
    // degraded by vibration
    bool isVibrationAffected() const;

    // get a yaw estimator instance
    const EKFGSF_yaw *get_yawEstimator(void) const;

private:
    uint8_t num_cores; // number of allocated cores
    uint8_t primary;   // current primary core
    NavEKF3_core *core = nullptr;

    uint32_t _frameTimeUsec;        // time per IMU frame
    uint8_t  _framesPerPrediction;  // expected number of IMU frames per prediction
  
    // values for EK3_LOG_LEVEL
    enum class LogLevel {
        ALL = 0,
        XKF4 = 1,
        XKF4_GSF = 2,
        NONE = 3
    };

    // EKF Mavlink Tuneable Parameters
    AP_Int8  _enable;               // zero to disable EKF3
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
    AP_Float _rngNoise;             // Range finder noise : m
    AP_Int8 _gpsCheck;              // Bitmask controlling which preflight GPS checks are bypassed
    AP_Int8 _imuMask;               // Bitmask of IMUs to instantiate EKF3 for
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
    AP_Float _accBiasLim;           // Accelerometer bias limit (m/s/s)
    AP_Int8 _magMask;               // Bitmask forcing specific EKF core instances to use simple heading magnetometer fusion.
    AP_Int8 _originHgtMode;         // Bitmask controlling post alignment correction and reporting of the EKF origin height.
    AP_Float _visOdmVelErrMax;      // Observation 1-STD velocity error assumed for visual odometry sensor at lowest reported quality (m/s)
    AP_Float _visOdmVelErrMin;      // Observation 1-STD velocity error assumed for visual odometry sensor at highest reported quality (m/s)
    AP_Float _wencOdmVelErr;        // Observation 1-STD velocity error assumed for wheel odometry sensor (m/s)
    AP_Int8  _flowUse;              // Controls if the optical flow data is fused into the main navigation estimator and/or the terrain estimator.
    AP_Float _hrt_filt_freq;        // frequency of output observer height rate complementary filter in Hz
    AP_Int16 _mag_ef_limit;         // limit on difference between WMM tables and learned earth field.
    AP_Int8 _gsfRunMask;            // mask controlling which EKF3 instances run a separate EKF-GSF yaw estimator
    AP_Int8 _gsfUseMask;            // mask controlling which EKF3 instances will use EKF-GSF yaw estimator data to assit with yaw resets
    AP_Int8 _gsfResetMaxCount;      // maximum number of times the EKF3 is allowed to reset it's yaw to the EKF-GSF estimate
    AP_Float _err_thresh;           // lanes have to be consistently better than the primary by at least this threshold to reduce their overall relativeCoreError
    AP_Int32 _affinity;             // bitmask of sensor affinity options
    AP_Float _dragObsNoise;         // drag specific force observatoin noise (m/s/s)**2
    AP_Float _ballisticCoef_x;      // ballistic coefficient measured for flow in X body frame directions
    AP_Float _ballisticCoef_y;      // ballistic coefficient measured for flow in Y body frame directions
    AP_Float _momentumDragCoef;     // lift rotor momentum drag coefficient
    AP_Int8 _betaMask;              // Bitmask controlling when sideslip angle fusion is used to estimate non wind states
    AP_Float _ognmTestScaleFactor;  // Scale factor applied to the thresholds used by the on ground not moving test
    AP_Float _baroGndEffectDeadZone;// Dead zone applied to positive baro height innovations when in ground effect (m)
    AP_Int8 _primary_core;          // initial core number
    AP_Enum<LogLevel> _log_level;   // log verbosity level
    AP_Float _gpsVAccThreshold;     // vertical accuracy threshold to use GPS as an altitude source

// Possible values for _flowUse
#define FLOW_USE_NONE    0
#define FLOW_USE_NAV     1
#define FLOW_USE_TERRAIN 2

    // Tuning parameters
    const float gpsNEVelVarAccScale = 0.05f;       // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    const float gpsDVelVarAccScale = 0.07f;        // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    const float gpsPosVarAccScale = 0.05f;         // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    const float extNavVelVarAccScale = 0.05f;      // Scale factor applied to ext nav velocity measurement variance due to manoeuvre acceleration
    const uint16_t magDelay_ms = 60;               // Magnetometer measurement delay (msec)
    const uint16_t tasDelay_ms = 100;              // Airspeed measurement delay (msec)
    const uint16_t tiltDriftTimeMax_ms = 15000;    // Maximum number of ms allowed without any form of tilt aiding (GPS, flow, TAS, etc)
    const uint16_t posRetryTimeUseVel_ms = 10000;  // Position aiding retry time with velocity measurements (msec)
    const uint16_t posRetryTimeNoVel_ms = 7000;    // Position aiding retry time without velocity measurements (msec)
    const uint16_t hgtRetryTimeMode0_ms = 10000;   // Height retry time with vertical velocity measurement (msec)
    const uint16_t hgtRetryTimeMode12_ms = 5000;   // Height retry time without vertical velocity measurement (msec)
    const uint16_t tasRetryTime_ms = 5000;         // True airspeed timeout and retry interval (msec)
    const uint16_t dragFailTimeLimit_ms = 5000;    // Drag timeout (msec)
    const uint32_t magFailTimeLimit_ms = 10000;    // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    const float magVarRateScale = 0.005f;          // scale factor applied to magnetometer variance due to angular rate
    const float gyroBiasNoiseScaler = 2.0f;        // scale factor applied to gyro bias state process noise when on ground
    const uint16_t hgtAvg_ms = 100;                // average number of msec between height measurements
    const uint16_t betaAvg_ms = 100;               // average number of msec between synthetic sideslip measurements
    const float covTimeStepMax = 0.1f;             // maximum time (sec) between covariance prediction updates
    const float covDelAngMax = 0.05f;              // maximum delta angle between covariance prediction updates
    const float DCM33FlowMin = 0.71f;              // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    const float fScaleFactorPnoise = 1e-10f;       // Process noise added to focal length scale factor state variance at each time step
    const uint8_t flowTimeDeltaAvg_ms = 100;       // average interval between optical flow measurements (msec)
    const uint32_t flowIntervalMax_ms = 100;       // maximum allowable time between flow fusion events
    const float gndEffectBaroScaler = 4.0f;        // scaler applied to the barometer observation variance when ground effect mode is active
    const uint8_t gndGradientSigma = 50;           // RMS terrain gradient percentage assumed by the terrain height estimation
    const uint16_t fusionTimeStep_ms = 10;         // The minimum time interval between covariance predictions and measurement fusions in msec
    const uint8_t sensorIntervalMin_ms = 50;       // The minimum allowed time between measurements from any non-IMU sensor (msec)
    const uint8_t flowIntervalMin_ms = 20;         // The minimum allowed time between measurements from optical flow sensors (msec)
    const uint8_t extNavIntervalMin_ms = 20;       // The minimum allowed time between measurements from external navigation sensors (msec)
    const float maxYawEstVelInnov = 2.0f;          // Maximum acceptable length of the velocity innovation returned by the EKF-GSF yaw estimator (m/s)
    const uint16_t deadReckonDeclare_ms = 1000;    // Time without equivalent position or velocity observation to constrain drift before dead reckoning is declared (msec)

    // time at start of current filter update
    uint64_t imuSampleTime_us;

    // time of last lane switch
    uint32_t lastLaneSwitch_ms;

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

#define MAX_EKF_CORES     3 // maximum allowed EKF Cores to be instantiated
#define CORE_ERR_LIM      1 // -LIM to LIM relative error range for a core
#define BETTER_THRESH   0.5 // a lane should have this much relative error difference to be considered for overriding a healthy primary core
    
    bool runCoreSelection;                          // true when the primary core has stabilised and the core selection logic can be started
    bool coreSetupRequired[MAX_EKF_CORES];          // true when this core index needs to be setup
    uint8_t coreImuIndex[MAX_EKF_CORES];            // IMU index used by this core
    float coreRelativeErrors[MAX_EKF_CORES];        // relative errors of cores with respect to primary
    float coreErrorScores[MAX_EKF_CORES];           // the instance error values used to update relative core error
    uint64_t coreLastTimePrimary_us[MAX_EKF_CORES]; // last time we were using this core as primary

    // origin set by one of the cores
    struct Location common_EKF_origin;
    bool common_origin_valid;
    
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

    // Update instance error scores for all available cores 
    float updateCoreErrorScores(void);

    // Update relative error scores for all alternate available cores
    void updateCoreRelativeErrors(void);

    // Reset error scores for all available cores
    void resetCoreErrors(void);

    // return true if a new core has a better score than an existing core, including
    // checks for alignment
    bool coreBetterScore(uint8_t new_core, uint8_t current_core) const;

    // position, velocity and yaw source control
    AP_NavEKF_Source sources;
};
