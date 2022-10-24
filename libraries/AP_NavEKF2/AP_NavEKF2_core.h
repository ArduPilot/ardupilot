/*
  24 state EKF based on the derivation in https://github.com/priseborough/
  InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/
  GenerateNavFilterEquations.m

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

#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>
#include <AP_NavEKF/AP_NavEKF_core_common.h>
#include <AP_NavEKF/EKF_Buffer.h>
#include <AP_DAL/AP_DAL.h>

#include "AP_NavEKF/EKFGSF_yaw.h"

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS      (1<<0)
#define MASK_GPS_HDOP       (1<<1)
#define MASK_GPS_SPD_ERR    (1<<2)
#define MASK_GPS_POS_ERR    (1<<3)
#define MASK_GPS_YAW_ERR    (1<<4)
#define MASK_GPS_POS_DRIFT  (1<<5)
#define MASK_GPS_VERT_SPD   (1<<6)
#define MASK_GPS_HORIZ_SPD  (1<<7)

// active height source
#define HGT_SOURCE_BARO     0
#define HGT_SOURCE_RNG      1
#define HGT_SOURCE_GPS      2
#define HGT_SOURCE_BCN      3
#define HGT_SOURCE_EXTNAV   4

// target EKF update time step
#define EKF_TARGET_DT 0.01f

// mag fusion final reset altitude
#define EKF2_MAG_FINAL_RESET_ALT 2.5f

// maximum number of yaw resets due to detected magnetic anomaly allowed per flight
#define MAG_ANOMALY_RESET_MAX 2

// number of seconds a request to reset the yaw to the GSF estimate is active before it times out
#define YAW_RESET_TO_GSF_TIMEOUT_MS 5000

// limit on horizontal position states
#if HAL_WITH_EKF_DOUBLE
#define EK2_POSXY_STATE_LIMIT 50.0e6
#else
#define EK2_POSXY_STATE_LIMIT 1.0e6
#endif
    
class AP_AHRS;

class NavEKF2_core : public NavEKF_core_common
{
public:
    // Constructor
    NavEKF2_core(class NavEKF2 *_frontend);

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
    ftype errorScore(void) const;

    // Write the last calculated NE position relative to the reference point (m).
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosNE(Vector2f &posNE) const;

    // Write the last calculated D position relative to the reference point (m).
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosD(float &posD) const;

    // return NED velocity in m/s
    void getVelNED(Vector3f &vel) const;

    // return estimate of true airspeed vector in body frame in m/s
    // returns false if estimate is unavailable
    bool getAirSpdVec(Vector3f &vel) const;

    // Return the rate of change of vertical position in the down direction (dPosD/dt) in m/s
    // This can be different to the z component of the EKF velocity state because it will fluctuate with height errors and corrections in the EKF
    // but will always be kinematically consistent with the z component of the EKF position state
    float getPosDownDerivative(void) const;

    // return body axis gyro bias estimates in rad/sec
    void getGyroBias(Vector3f &gyroBias) const;

    // return body axis gyro scale factor error as a percentage
    void getGyroScaleErrorPercentage(Vector3f &gyroScale) const;

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

    // return the Z-accel bias estimate in m/s^2
    void getAccelZBias(float &zbias) const;

    // return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
    void getWind(Vector3f &wind) const;

    // return earth magnetic field estimates in measurement units / 1000
    void getMagNED(Vector3f &magNED) const;

    // return body magnetic field estimates in measurement units / 1000
    void getMagXYZ(Vector3f &magXYZ) const;

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
    // returns false if absolute aiding and GPS is being used or if the origin is already set
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

    // return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
    bool getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

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
    // heightOverride is the fixed height of the sensor above ground in m, when on rover vehicles. 0 if not used
    void  writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride);

    /*
        Returns the following data for debugging range beacon fusion
        ID : beacon identifier
        rng : measured range to beacon (m)
        innov : range innovation (m)
        innovVar : innovation variance (m^2)
        testRatio : innovation consistency test ratio
        beaconPosNED : beacon NED position (m)
    */
    bool getRangeBeaconDebug(uint8_t &ID, float &rng, float &innov, float &innovVar, float &testRatio, Vector3f &beaconPosNED, float &offsetHigh, float &offsetLow);

    // Set to true if the terrain underneath is stable enough to be used as a height reference
    // in combination with a range finder. Set to false if the terrain underneath the vehicle
    // cannot be used as a height reference. Use to prevent range finder operation otherwise
    // enabled by the combination of EK2_RNG_AID_HGT and EK2_RNG_USE_SPD parameters.
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
    void  getFilterFaults(uint16_t &faults) const;

    /*
    return filter gps quality check status
    */
    void  getFilterGpsStatus(nav_gps_status &status) const;

    /*
    Return a filter function status that indicates:
        Which outputs are valid
        If the filter has detected takeoff
        If the filter has activated the mode that mitigates against ground effect static pressure errors
        If GPS data is being used
    */
    void  getFilterStatus(nav_filter_status &status) const;

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

    // return true when external nav data is also being used as a yaw observation
    bool isExtNavUsedForYaw(void) const;

    // Writes the default equivalent airspeed in m/s to be used in forward flight if a measured airspeed is required and not available.
    void writeDefaultAirSpeed(float airspeed);

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

    // get a yaw estimator instance
    const EKFGSF_yaw *get_yawEstimator(void) const { return yawEstimator; }
    
private:
    EKFGSF_yaw *yawEstimator;
    AP_DAL &dal;

    // Reference to the global EKF frontend for parameters
    class NavEKF2 *frontend;
    uint8_t imu_index; // preferred IMU index
    uint8_t gyro_index_active; // active gyro index (in case preferred fails)
    uint8_t accel_index_active; // active accel index (in case preferred fails)
    uint8_t core_index;
    uint8_t imu_buffer_length;

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
    typedef ftype Vector22[22];
    typedef ftype Vector23[23];
    typedef ftype Vector24[24];
    typedef ftype Vector25[25];
    typedef ftype Matrix3[3][3];
    typedef ftype Matrix24[24][24];
    typedef ftype Matrix34_50[34][50];
    typedef uint32_t Vector_u32_50[50];
#endif

    // the states are available in two forms, either as a Vector31, or
    // broken down as individual elements. Both are equivalent (same
    // memory)
    struct state_elements {
        Vector3F    angErr;         // 0..2
        Vector3F    velocity;       // 3..5
        Vector3F    position;       // 6..8
        Vector3F    gyro_bias;      // 9..11
        Vector3F    gyro_scale;     // 12..14
        ftype       accel_zbias;    // 15
        Vector3F    earth_magfield; // 16..18
        Vector3F    body_magfield;  // 19..21
        Vector2F    wind_vel;       // 22..23
        QuaternionF quat;           // 24..27
    };

    union {
        Vector28 statesArray;
        struct state_elements stateStruct;
    };

    struct output_elements {
        QuaternionF quat;           // 0..3
        Vector3F    velocity;       // 4..6
        Vector3F    position;       // 7..9
    };

    struct imu_elements {
        Vector3F    delAng;         // 0..2
        Vector3F    delVel;         // 3..5
        ftype       delAngDT;       // 6
        ftype       delVelDT;       // 7
        uint32_t    time_ms;        // 8
        uint8_t     gyro_index;
        uint8_t     accel_index;
    };

    struct gps_elements : EKF_obs_element_t {
        Vector2F    pos;
        ftype       hgt;
        Vector3F    vel;
        uint8_t     sensor_idx;
    };

    struct mag_elements : EKF_obs_element_t {
        Vector3F    mag;
    };

    struct baro_elements : EKF_obs_element_t {
        ftype       hgt;
    };

    struct range_elements : EKF_obs_element_t {
        ftype       rng;
        uint8_t     sensor_idx;
    };

    struct rng_bcn_elements : EKF_obs_element_t {
        ftype       rng;                // range measurement to each beacon (m)
        Vector3F    beacon_posNED;      // NED position of the beacon (m)
        ftype       rngErr;             // range measurement error 1-std (m)
        uint8_t     beacon_ID;          // beacon identification number
    };

    struct tas_elements : EKF_obs_element_t {
        ftype       tas;
    };

    struct of_elements : EKF_obs_element_t {
        Vector2F    flowRadXY;
        Vector2F    flowRadXYcomp;
        Vector3F    bodyRadXYZ;
        Vector3F    body_offset;
        float       heightOverride;
    };

    struct ext_nav_elements : EKF_obs_element_t {
        Vector3F        pos;        // XYZ position measured in a RH navigation frame (m)
        QuaternionF     quat;       // quaternion describing the rotation from navigation to body frame
        ftype           posErr;     // spherical poition measurement error 1-std (m)
        ftype           angErr;     // spherical angular measurement error 1-std (rad)
        bool            posReset;   // true when the position measurement has been reset
    };

    // bias estimates for the IMUs that are enabled but not being used
    // by this core.
    struct {
        Vector3F gyro_bias;
        Vector3F gyro_scale;
        ftype accel_zbias;
    } inactiveBias[INS_MAX_INSTANCES];

    struct ext_nav_vel_elements : EKF_obs_element_t {
        Vector3F vel;               // velocity in NED (m)
        ftype err;                  // velocity measurement error (m/s)
    };

    // update the navigation filter status
    void  updateFilterStatus(void);

    // update the quaternion, velocity and position states using IMU measurements
    void UpdateStrapdownEquationsNED();

    // calculate the predicted state covariance matrix
    void CovariancePrediction();

    // force symmetry on the state covariance matrix
    void ForceSymmetry();

    // copy covariances across from covariance prediction calculation and fix numerical errors
    void CopyCovariances();

    // constrain variances (diagonal terms) in the state covariance matrix
    void ConstrainVariances();

    // constrain states
    void ConstrainStates();

    // constrain earth field using WMM tables
    void MagTableConstrain(void);

    // fuse selected position, velocity and height measurements
    void FuseVelPosNED();

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

    // determine when to perform fusion of synthetic sideslp measurements
    void SelectBetaFusion();

    // force alignment of the yaw angle using GPS velocity data
    void realignYawGPS();

    // initialise the earth magnetic field states using declination and current attitude and magnetometer measurements
    // and return attitude quaternion
    QuaternionF calcQuatAndFieldStates(ftype roll, ftype pitch);

    // zero stored variables
    void InitialiseVariables();

    void InitialiseVariablesMag();

    // reset the horizontal position states uing the last GPS measurement
    void ResetPosition(void);

    // reset the stateStruct's NE position to the specified position
    void ResetPositionNE(ftype posN, ftype posE);

    // reset velocity states using the last GPS measurement
    void ResetVelocity(void);

    // reset the vertical position state using the last height measurement
    void ResetHeight(void);

    // reset the stateStruct's D position
    void ResetPositionD(ftype posD);

    // return true if we should use the airspeed sensor
    bool useAirspeed(void) const;

    // return true if the vehicle code has requested the filter to be ready for flight
    bool readyToUseGPS(void) const;

    // return true if the filter to be ready to use the beacon range measurements
    bool readyToUseRangeBeacon(void) const;

    // return true if the filter to be ready to use external nav data
    bool readyToUseExtNav(void) const;

    // Check for filter divergence
    void checkDivergence(void);

    // Calculate weighting that is applied to IMU1 accel data to blend data from IMU's 1 and 2
    void calcIMU_Weighting(ftype K1, ftype K2);

    // return true if optical flow data is available
    bool optFlowDataPresent(void) const;

    // return true if we should use the range finder sensor
    bool useRngFinder(void) const;

    // determine when to perform fusion of optical flow measurements
    void SelectFlowFusion();

    // Estimate terrain offset using a single state EKF
    void EstimateTerrainOffset();

    // fuse optical flow measurements into the main filter
    void FuseOptFlow();

    // Control filter mode changes
    void controlFilterModes();

    // Determine if we are flying or on the ground
    void detectFlight();

    // Set inertial navigaton aiding mode
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

    // Assess GPS data quality and set gpsGoodToAlign if good enough to align the EKF
    void calcGpsGoodToAlign(void);

    // return true and set the class variable true if the delta angle bias has been learned
    bool checkGyroCalStatus(void);

    // update inflight calculaton that determines if GPS data is good enough for reliable navigation
    void calcGpsGoodForFlight(void);

    // Read the range finder and take new measurements if available
    // Apply a median filter to range finder data
    void readRangeFinder();

    // check if the vehicle has taken off during optical flow navigation by looking at inertial and range finder data
    void detectOptFlowTakeoff(void);

    // align the NE earth magnetic field states with the published declination
    void alignMagStateDeclination();

    // Fuse compass measurements using a simple declination observation (doesn't require magnetic field states)
    void fuseEulerYaw();

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
    uint8_t effective_magCal(void) const;

    // update timing statistics structure
    void updateTimingStatistics(void);

    // correct gps data for antenna position
    void CorrectGPSForAntennaOffset(gps_elements &gps_data) const;

    // correct external navigation earth-frame position using sensor body-frame offset
    void CorrectExtNavForSensorOffset(Vector3F &ext_position) const;

    // correct external navigation earth-frame velocity using sensor body-frame offset
    void CorrectExtNavVelForSensorOffset(Vector3F &ext_velocity) const;

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
    // isDeltaYaw   : true when the yaw should be added to the existing yaw angle
    void resetQuatStateYawOnly(ftype yaw, ftype yawVariance, bool isDeltaYaw);

    // attempt to reset the yaw to the EKF-GSF value
    // returns false if unsuccessful
    bool EKFGSF_resetMainFilterYaw();

    // Length of FIFO buffers used for non-IMU sensor data.
    // Must be larger than the time period defined by IMU_BUFFER_LENGTH
    static const uint32_t OBS_BUFFER_LENGTH = 5;
    static const uint32_t FLOW_BUFFER_LENGTH = 15;
    static const uint32_t EXTNAV_BUFFER_LENGTH = 15;

    // Variables
    bool statesInitialised;         // boolean true when filter states have been initialised
    bool magHealth;                 // boolean true if magnetometer has passed innovation consistency check
    bool velTimeout;                // boolean true if velocity measurements have failed innovation consistency check and timed out
    bool posTimeout;                // boolean true if position measurements have failed innovation consistency check and timed out
    bool hgtTimeout;                // boolean true if height measurements have failed innovation consistency check and timed out
    bool magTimeout;                // boolean true if magnetometer measurements have failed for too long and have timed out
    bool tasTimeout;                // boolean true if true airspeed measurements have failed for too long and have timed out
    bool badIMUdata;                // boolean true if the bad IMU data is detected

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
    uint32_t airborneDetectTime_ms; // last time flight movement was detected
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
    bool magFusePerformed;          // boolean set to true when magnetometer fusion has been performed in that time step
    uint32_t prevTasStep_ms;        // time stamp of last TAS fusion step
    uint32_t prevBetaStep_ms;       // time stamp of last synthetic sideslip fusion step
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
    uint32_t lastYawTime_ms;        // time stamp when yaw observation was last fused (msec)
    uint32_t ekfStartTime_ms;       // time the EKF was started (msec)
    Vector2F lastKnownPositionNE;   // last known position
    ftype velTestRatio;             // sum of squares of GPS velocity innovation divided by fail threshold
    ftype posTestRatio;             // sum of squares of GPS position innovation divided by fail threshold
    ftype hgtTestRatio;             // sum of squares of baro height innovation divided by fail threshold
    Vector3F magTestRatio;          // sum of squares of magnetometer innovations divided by fail threshold
    ftype tasTestRatio;             // sum of squares of true airspeed innovation divided by fail threshold
    ftype defaultAirSpeed;          // default equivalent airspeed in m/s to be used if the measurement is unavailable. Do not use if not positive.
    bool inhibitWindStates;         // true when wind states and covariances are to remain constant
    bool inhibitMagStates;          // true when magnetic field states and covariances are to remain constant
    bool lastInhibitMagStates;      // previous inhibitMagStates
    bool needMagBodyVarReset;       // we need to reset mag body variances at next CovariancePrediction
    bool gpsNotAvailable;           // bool true when valid GPS data is not available
    uint8_t last_gps_idx;           // sensor ID of the GPS receiver used for the last fusion or reset
    struct Location EKF_origin;     // LLH origin of the NED axis system
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
    Vector3F tiltErrVec;            // Vector of most recent attitude error correction from Vel,Pos fusion
    ftype tiltErrFilt;              // Filtered tilt error metric
    bool tiltAlignComplete;         // true when tilt alignment is complete
    bool yawAlignComplete;          // true when yaw alignment is complete
    bool magStateInitComplete;      // true when the magnetic field sttes have been initialised
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
    mag_elements magDataDelayed;    // Magnetometer data at the fusion time horizon
    gps_elements gpsDataNew;        // GPS data at the current time horizon
    gps_elements gpsDataDelayed;    // GPS data at the fusion time horizon
    output_elements outputDataNew;  // output state data at the current time step
    output_elements outputDataDelayed; // output state data at the current time step
    Vector3F delAngCorrection;      // correction applied to delta angles used by output observer to track the EKF
    Vector3F velErrintegral;        // integral of output predictor NED velocity tracking error (m)
    Vector3F posErrintegral;        // integral of output predictor NED position tracking error (m.sec)
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
    bool startPredictEnabled;       // boolean true when the frontend has given permission to start a new state prediciton cycele
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

    // variables used to calculate a vertical velocity that is kinematically consistent with the verical position
    struct {
        ftype pos;
        ftype vel;
        ftype acc;
    } vertCompFiltState;

    // variables used by the pre-initialisation GPS checks
    struct Location gpsloc_prev;    // LLH location of previous GPS measurement
    uint32_t lastPreAlignGpsCheckTime_ms;   // last time in msec the GPS quality was checked during pre alignment checks
    ftype gpsDriftNE;               // amount of drift detected in the GPS position during pre-flight GPs checks
    ftype gpsVertVelFilt;           // amount of filterred vertical GPS velocity detected durng pre-flight GPS checks
    ftype gpsHorizVelFilt;          // amount of filtered horizontal GPS velocity detected during pre-flight GPS checks

    // variable used by the in-flight GPS quality check
    bool gpsSpdAccPass;             // true when reported GPS speed accuracy passes in-flight checks
    bool ekfInnovationsPass;        // true when GPS innovations pass in-flight checks
    ftype sAccFilterState1;         // state variable for LPF applid to reported GPS speed accuracy
    ftype sAccFilterState2;         // state variable for peak hold filter applied to reported GPS speed
    uint32_t lastGpsCheckTime_ms;   // last time in msec the GPS quality was checked
    uint32_t lastInnovPassTime_ms;  // last time in msec the GPS innovations passed
    uint32_t lastInnovFailTime_ms;  // last time in msec the GPS innovations failed
    bool gpsAccuracyGood;           // true when the GPS accuracy is considered to be good enough for safe flight.

    // variables added for optical flow fusion
    EKF_obs_buffer_t<of_elements> storedOF;    // OF data buffer
    of_elements ofDataNew;          // OF data at the current time horizon
    of_elements ofDataDelayed;      // OF data at the fusion time horizon
    bool flowDataToFuse;            // true when optical flow data is ready for fusion
    bool flowDataValid;             // true while optical flow data is still fresh
    Vector2F auxFlowObsInnov;       // optical flow rate innovation from 1-state terrain offset estimator
    uint32_t flowValidMeaTime_ms;   // time stamp from latest valid flow measurement (msec)
    uint32_t rngValidMeaTime_ms;    // time stamp from latest valid range measurement (msec)
    uint32_t flowMeaTime_ms;        // time stamp from latest flow measurement (msec)
    uint32_t gndHgtValidTime_ms;    // time stamp from last terrain offset state update (msec)
    Matrix3F Tbn_flow;              // transformation matrix from body to nav axes at the middle of the optical flow sample period
    Vector2 varInnovOptFlow;        // optical flow innovations variances (rad/sec)^2
    Vector2 innovOptFlow;           // optical flow LOS innovations (rad/sec)
    ftype Popt;                     // Optical flow terrain height state covariance (m^2)
    ftype terrainState;             // terrain position state (m)
    ftype prevPosN;                 // north position at last measurement
    ftype prevPosE;                 // east position at last measurement
    ftype varInnovRng;              // range finder observation innovation variance (m^2)
    ftype innovRng;                 // range finder observation innovation (m)
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
    Vector3F accelPosOffset;        // position of IMU accelerometer unit in body frame (m)


    // Range finder
    ftype baroHgtOffset;                    // offset applied when when switching to use of Baro height
    ftype rngOnGnd;                         // Expected range finder reading in metres when vehicle is on ground
    ftype storedRngMeas[2][3];              // Ringbuffer of stored range measurements for dual range sensors
    uint32_t storedRngMeasTime_ms[2][3];    // Ringbuffers of stored range measurement times for dual range sensors
    uint32_t lastRngMeasTime_ms;            // Timestamp of last range measurement
    uint8_t rngMeasIndex[2];                // Current range measurement ringbuffer index for dual range sensors
    bool terrainHgtStable;                  // true when the terrain height is stable enough to be used as a height reference

    // Range Beacon Sensor Fusion
    EKF_obs_buffer_t<rng_bcn_elements> storedRangeBeacon; // Beacon range buffer
    rng_bcn_elements rngBcnDataNew;     // Range beacon data at the current time horizon
    rng_bcn_elements rngBcnDataDelayed; // Range beacon data at the fusion time horizon
    uint32_t lastRngBcnPassTime_ms;     // time stamp when the range beacon measurement last passed innvovation consistency checks (msec)
    ftype rngBcnTestRatio;              // Innovation test ratio for range beacon measurements
    bool rngBcnHealth;                  // boolean true if range beacon measurements have passed innovation consistency check
    bool rngBcnTimeout;                 // boolean true if range beacon measurements have faled innovation consistency checks for too long
    ftype varInnovRngBcn;               // range beacon observation innovation variance (m^2)
    ftype innovRngBcn;                  // range beacon observation innovation (m)
    uint32_t lastTimeRngBcn_ms[10];     // last time we received a range beacon measurement (msec)
#if AP_BEACON_ENABLED
    bool rngBcnDataToFuse;              // true when there is new range beacon data to fuse
#else
    const bool rngBcnDataToFuse = false;              // true when there is new range beacon data to fuse
#endif
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
    ftype bcnPosOffset;                 // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)

    ftype bcnPosOffsetMax;             // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
    ftype bcnPosOffsetMaxVar;          // Variance of the bcnPosOffsetHigh state (m)
    ftype OffsetMaxInnovFilt;          // Filtered magnitude of the range innovations using bcnPosOffsetHigh

    ftype bcnPosOffsetMin;              // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
    ftype bcnPosOffsetMinVar;           // Variance of the bcnPosoffset state (m)
    ftype OffsetMinInnovFilt;           // Filtered magnitude of the range innovations using bcnPosOffsetLow

    // Range Beacon Fusion Debug Reporting
    uint8_t rngBcnFuseDataReportIndex;// index of range beacon fusion data last reported
    struct rngBcnFusionReport_t {
        ftype rng;          // measured range to beacon (m)
        ftype innov;        // range innovation (m)
        ftype innovVar;     // innovation variance (m^2)
        ftype testRatio;    // innovation consistency test ratio
        Vector3F beaconPosNED; // beacon NED position
    } rngBcnFusionReport[10];

    // height source selection logic
    uint8_t activeHgtSource;    // integer defining active height source

    // Movement detector
    bool takeOffDetected;           // true when takeoff for optical flow navigation has been detected
    ftype rngAtStartOfFlight;       // range finder measurement at start of flight
    uint32_t timeAtArming_ms;       // time in msec that the vehicle armed

    // baro ground effect
    ftype meaHgtAtTakeOff;            // height measured at commencement of takeoff

    // control of post takeoff magnetic field and heading resets
    bool finalInflightYawInit;      // true when the final post takeoff initialisation of yaw angle has been performed
    bool finalInflightMagInit;      // true when the final post takeoff initialisation of magnetic field states been performed
    bool magStateResetRequest;      // true if magnetic field states need to be reset using the magneteomter measurements
    bool magYawResetRequest;        // true if the vehicle yaw and magnetic field states need to be reset using the magnetometer measurements
    bool gpsYawResetRequest;        // true if the vehicle yaw needs to be reset to the GPS course
    ftype posDownAtLastMagReset;    // vertical position last time the mag states were reset (m)
    ftype yawInnovAtLastMagReset;   // magnetic yaw innovation last time the yaw and mag field states were reset (rad)
    QuaternionF quatAtLastMagReset;  // quaternion states last time the mag states were reset
    uint8_t magYawAnomallyCount;    // Number of times the yaw has been reset due to a magnetic anomaly during initial ascent

    // external navigation fusion
    EKF_obs_buffer_t<ext_nav_elements> storedExtNav; // external navigation data buffer
    ext_nav_elements extNavDataNew;     // External nav data at the current time horizon
    ext_nav_elements extNavDataDelayed; // External nav at the fusion time horizon
    uint32_t extNavMeasTime_ms;         // time external measurements were accepted for input to the data buffer (msec)
    uint32_t extNavLastPosResetTime_ms; // last time the external nav systen performed a position reset (msec)
    uint32_t lastExtNavPassTime_ms;     // time stamp when external nav position measurement last passed innovation consistency check (msec)
    bool extNavDataToFuse;              // true when there is new external nav data to fuse
    bool extNavUsedForYaw;              // true when the external nav data is also being used as a yaw observation
    bool extNavUsedForPos;              // true when the external nav data is being used as a position reference.
    bool extNavYawResetRequest;         // true when a reset of vehicle yaw using the external nav data is requested

    EKF_obs_buffer_t<ext_nav_vel_elements> storedExtNavVel; // external navigation velocity data buffer
    ext_nav_vel_elements extNavVelNew;                       // external navigation velocity data at the current time horizon
    ext_nav_vel_elements extNavVelDelayed;                   // external navigation velocity data at the fusion time horizon
    uint32_t extNavVelMeasTime_ms;                           // time external navigation velocity measurements were accepted for input to the data buffer (msec)
    bool extNavVelToFuse;                                    // true when there is new external navigation velocity to fuse
    bool useExtNavVel;                                       // true external navigation velocity should be used

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
    } faultStatus;

    // flags indicating which GPS quality checks are failing
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
    } gpsCheckStatus;

    // states held by magnetomter fusion across time steps
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
    char prearm_fail_string[41];

    // earth field from WMM tables
    bool have_table_earth_field;   // true when we have initialised table_earth_field_ga
    Vector3F table_earth_field_ga; // earth field from WMM tables
    ftype table_declination;       // declination in radians from the tables

    // timing statistics
    struct ekf_timing timing;

    // when was attitude filter status last non-zero?
    uint32_t last_filter_ok_ms;
    
    // should we assume zero sideslip?
    bool assume_zero_sideslip(void) const;

    // vehicle specific initial gyro bias uncertainty
    ftype InitialGyroBiasUncertainty(void) const;

    // The following declarations are used to control when the main navigation filter resets it's yaw to the estimate provided by the GSF
    uint32_t EKFGSF_yaw_reset_ms;           // timestamp of last emergency yaw reset (uSec)
    uint32_t EKFGSF_yaw_reset_request_ms;   // timestamp of last emergency yaw reset request (uSec)
    uint8_t EKFGSF_yaw_reset_count;         // number of emergency yaw resets performed
    bool EKFGSF_run_filterbank;             // true when the filter bank is active

    // logging timestamps
    uint32_t lastTimingLogTime_ms;

    // logging functions shared by cores:
    void Log_Write_NKF1(uint64_t time_us) const;
    void Log_Write_NKF2(uint64_t time_us) const;
    void Log_Write_NKF3(uint64_t time_us) const;
    void Log_Write_NKF4(uint64_t time_us) const;
    void Log_Write_NKF5(uint64_t time_us) const;
    void Log_Write_Quaternion(uint64_t time_us) const;
    void Log_Write_Beacon(uint64_t time_us);
    void Log_Write_Timing(uint64_t time_us);
    void Log_Write_GSF(uint64_t time_us) const;
};
