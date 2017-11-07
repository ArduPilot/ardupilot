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

#pragma GCC optimize("O3")

#define EK2_DISABLE_INTERRUPTS 0


#include <AP_Math/AP_Math.h>
#include "AP_NavEKF2.h"
#include <stdio.h>
#include <AP_Math/vectorN.h>
#include <AP_NavEKF2/AP_NavEKF2_Buffer.h>

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
#define HGT_SOURCE_BARO 0
#define HGT_SOURCE_RNG  1
#define HGT_SOURCE_GPS  2
#define HGT_SOURCE_BCN  3

// target EKF update time step
#define EKF_TARGET_DT 0.01f

// mag fusion final reset altitude
#define EKF2_MAG_FINAL_RESET_ALT 2.5f

class AP_AHRS;

class NavEKF2_core
{
public:
    // Constructor
    NavEKF2_core(void);

    // setup this core backend
    bool setup_core(NavEKF2 *_frontend, uint8_t _imu_index, uint8_t _core_index);
    
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

    // Write the last calculated D position relative to the reference point (m).
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosD(float &posD) const;

    // return NED velocity in m/s
    void getVelNED(Vector3f &vel) const;

    // Return the rate of change of vertical position in the down diection (dPosD/dt) in m/s
    // This can be different to the z component of the EKF velocity state because it will fluctuate with height errors and corrections in the EKF
    // but will always be kinematically consistent with the z component of the EKF position state
    float getPosDownDerivative(void) const;

    // This returns the specific forces in the NED frame
    void getAccelNED(Vector3f &accelNED) const;

    // return body axis gyro bias estimates in rad/sec
    void getGyroBias(Vector3f &gyroBias) const;

    // return body axis gyro scale factor error as a percentage
    void getGyroScaleErrorPercentage(Vector3f &gyroScale) const;

    // return tilt error convergence metric
    void getTiltError(float &ang) const;

    // reset body axis gyro bias estimates
    void resetGyroBias(void);

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    bool resetHeightDatum(void);

    // Commands the EKF to not use GPS.
    // This command must be sent prior to arming as it will only be actioned when the filter is in static mode
    // This command is forgotten by the EKF each time it goes back into static mode (eg the vehicle disarms)
    // Returns 0 if command rejected
    // Returns 1 if attitude, vertical velocity and vertical position will be provided
    // Returns 2 if attitude, 3D-velocity, vertical position and relative horizontal position will be provided
    uint8_t setInhibitGPS(void);

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

    // return the index for the active magnetometer
    uint8_t getActiveMag() const;

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

    // return the quaternions defining the rotation from NED to XYZ (body) axes
    void getQuaternion(Quaternion &quat) const;

    // return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
    void  getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    // return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
    void  getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

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
    void  writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas, const Vector3f &posOffset);

    // return data for debugging optical flow fusion
    void getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov, float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const;

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

    // called by vehicle code to specify that a takeoff is happening
    // causes the EKF to compensate for expected barometer errors due to ground effect
    void setTakeoffExpected(bool val);

    // called by vehicle code to specify that a touchdown is expected to happen
    // causes the EKF to compensate for expected barometer errors due to ground effect
    void setTouchdownExpected(bool val);

    // Set to true if the terrain underneath is stable enough to be used as a height reference
    // in combination with a range finder. Set to false if the terrain underneath the vehicle
    // cannot be used as a height reference
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
    void  getFilterFaults(uint16_t &faults) const;

    /*
    return filter timeout status as a bitmasked integer
     0 = position measurement timeout
     1 = velocity measurement timeout
     2 = height measurement timeout
     3 = magnetometer measurement timeout
     5 = unassigned
     6 = unassigned
     7 = unassigned
     7 = unassigned
    */
    void  getFilterTimeouts(uint8_t &timeouts) const;

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
    void send_status_report(mavlink_channel_t chan);

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

    // publish output observer angular, velocity and position tracking error
    void getOutputTrackingError(Vector3f &error) const;

    // get the IMU index
    uint8_t getIMUIndex(void) const { return imu_index; }

    // get timing statistics structure
    void getTimingStatistics(struct ekf_timing &timing);
    
private:
    // Reference to the global EKF frontend for parameters
    NavEKF2 *frontend;
    uint8_t imu_index;
    uint8_t core_index;
    uint8_t imu_buffer_length;

    typedef float ftype;
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
    typedef VectorN<ftype,28> Vector28;
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
    typedef ftype Vector28[28];
    typedef ftype Matrix3[3][3];
    typedef ftype Matrix24[24][24];
    typedef ftype Matrix34_50[34][50];
    typedef uint32_t Vector_u32_50[50];
#endif

    const AP_AHRS *_ahrs;

    // the states are available in two forms, either as a Vector31, or
    // broken down as individual elements. Both are equivalent (same
    // memory)
    Vector28 statesArray;
    struct state_elements {
        Vector3f    angErr;         // 0..2
        Vector3f    velocity;       // 3..5
        Vector3f    position;       // 6..8
        Vector3f    gyro_bias;      // 9..11
        Vector3f    gyro_scale;     // 12..14
        float       accel_zbias;    // 15
        Vector3f    earth_magfield; // 16..18
        Vector3f    body_magfield;  // 19..21
        Vector2f    wind_vel;       // 22..23
        Quaternion  quat;           // 24..27
    } &stateStruct;

    struct output_elements {
        Quaternion  quat;           // 0..3
        Vector3f    velocity;       // 4..6
        Vector3f    position;       // 7..9
    };

    struct imu_elements {
        Vector3f    delAng;         // 0..2
        Vector3f    delVel;         // 3..5
        float       delAngDT;       // 6
        float       delVelDT;       // 7
        uint32_t    time_ms;        // 8
    };

    struct gps_elements {
        Vector2f    pos;         // 0..1
        float       hgt;         // 2
        Vector3f    vel;         // 3..5
        uint32_t    time_ms;     // 6
        uint8_t     sensor_idx;  // 7..9
    };

    struct mag_elements {
        Vector3f    mag;         // 0..2
        uint32_t    time_ms;     // 3
    };

    struct baro_elements {
        float       hgt;         // 0
        uint32_t    time_ms;     // 1
    };

    struct range_elements {
        float       rng;         // 0
        uint32_t    time_ms;     // 1
        uint8_t     sensor_idx;  // 2
    };

    struct rng_bcn_elements {
        float       rng;                // range measurement to each beacon (m)
        Vector3f    beacon_posNED;      // NED position of the beacon (m)
        float       rngErr;             // range measurement error 1-std (m)
        uint8_t     beacon_ID;          // beacon identification number
        uint32_t    time_ms;            // measurement timestamp (msec)
    };

    struct tas_elements {
        float       tas;         // 0
        uint32_t    time_ms;     // 1
    };

    struct of_elements {
        Vector2f    flowRadXY;      // 0..1
        Vector2f    flowRadXYcomp;  // 2..3
        uint32_t    time_ms;        // 4
        Vector3f    bodyRadXYZ;     //8..10
        const Vector3f *body_offset;// 5..7
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

    // fuse selected position, velocity and height measurements
    void FuseVelPosNED();

    // fuse range beacon measurements
    void FuseRngBcn();

    // use range beaon measurements to calculate a static position
    void FuseRngBcnStatic();

    // calculate the offset from EKF vetical position datum to the range beacon system datum
    void CalcRangeBeaconPosDownOffset(float obsVar, Vector3f &vehiclePosNED, bool aligning);

    // fuse magnetometer measurements
    void FuseMagnetometer();

    // fuse true airspeed measurements
    void FuseAirspeed();

    // fuse sythetic sideslip measurement of zero
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
    void StoreQuatRotate(Quaternion deltaQuat);

    // store altimeter data
    void StoreBaro();

    // recall altimeter data at the fusion time horizon
    // return true if data found
    bool RecallBaro();

    // store range finder data
    void StoreRange();

    // recall range finder data at the fusion time horizon
    // return true if data found
    bool RecallRange();

    // store magnetometer data
    void StoreMag();

    // recall magetometer data at the fusion time horizon
    // return true if data found
    bool RecallMag();

    // store true airspeed data
    void StoreTAS();

    // recall true airspeed data at the fusion time horizon
    // return true if data found
    bool RecallTAS();

    // store optical flow data
    void StoreOF();

    // recall optical flow data at the fusion time horizon
    // return true if data found
    bool RecallOF();

    // calculate nav to body quaternions from body to nav rotation matrix
    void quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const;

    // calculate the NED earth spin vector in rad/sec
    void calcEarthRateNED(Vector3f &omega, int32_t latitude) const;

    // initialise the covariance matrix
    void CovarianceInit();

    // helper functions for readIMUData
    bool readDeltaVelocity(uint8_t ins_index, Vector3f &dVel, float &dVel_dt);
    bool readDeltaAngle(uint8_t ins_index, Vector3f &dAng);

    // helper functions for correcting IMU data
    void correctDeltaAngle(Vector3f &delAng, float delAngDT);
    void correctDeltaVelocity(Vector3f &delVel, float delVelDT);

    // update IMU delta angle and delta velocity measurements
    void readIMUData();

    // check for new valid GPS data and update stored measurement if available
    void readGpsData();

    // check for new altitude measurement data and update stored measurement if available
    void readBaroData();

    // check for new magnetometer data and update store measurements if available
    void readMagData();

    // check for new airspeed data and update stored measurements if available
    void readAirSpdData();

    // check for new range beacon data and update stored measurements if available
    void readRngBcnData();

    // determine when to perform fusion of GPS position and  velocity measurements
    void SelectVelPosFusion();

    // determine when to perform fusion of range measurements take realtive to a beacon at a known NED position
    void SelectRngBcnFusion();

    // determine when to perform fusion of magnetometer measurements
    void SelectMagFusion();

    // determine when to perform fusion of true airspeed measurements
    void SelectTasFusion();

    // determine when to perform fusion of synthetic sideslp measurements
    void SelectBetaFusion();

    // force alignment of the yaw angle using GPS velocity data
    void realignYawGPS();

    // initialise the earth magnetic field states using declination and current attitude and magnetometer meaasurements
    // and return attitude quaternion
    Quaternion calcQuatAndFieldStates(float roll, float pitch);

    // zero stored variables
    void InitialiseVariables();

    // reset the horizontal position states uing the last GPS measurement
    void ResetPosition(void);

    // reset velocity states using the last GPS measurement
    void ResetVelocity(void);

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
    void calcIMU_Weighting(float K1, float K2);

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

    // Set the NED origin to be used until the next filter reset
    void setOrigin();

    // determine if a takeoff is expected so that we can compensate for expected barometer errors due to ground effect
    bool getTakeoffExpected();

    // determine if a touchdown is expected so that we can compensate for expected barometer errors due to ground effect
    bool getTouchdownExpected();

    // Assess GPS data quality and return true if good enough to align the EKF
    bool calcGpsGoodToAlign(void);

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
    void FuseDeclination(float declErr);

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
    
    // Length of FIFO buffers used for non-IMU sensor data.
    // Must be larger than the time period defined by IMU_BUFFER_LENGTH
    static const uint32_t OBS_BUFFER_LENGTH = 5;

    // Variables
    bool statesInitialised;         // boolean true when filter states have been initialised
    bool velHealth;                 // boolean true if velocity measurements have passed innovation consistency check
    bool posHealth;                 // boolean true if position measurements have passed innovation consistency check
    bool hgtHealth;                 // boolean true if height measurements have passed innovation consistency check
    bool magHealth;                 // boolean true if magnetometer has passed innovation consistency check
    bool tasHealth;                 // boolean true if true airspeed has passed innovation consistency check
    bool velTimeout;                // boolean true if velocity measurements have failed innovation consistency check and timed out
    bool posTimeout;                // boolean true if position measurements have failed innovation consistency check and timed out
    bool hgtTimeout;                // boolean true if height measurements have failed innovation consistency check and timed out
    bool magTimeout;                // boolean true if magnetometer measurements have failed for too long and have timed out
    bool tasTimeout;                // boolean true if true airspeed measurements have failed for too long and have timed out
    bool badMagYaw;                 // boolean true if the magnetometer is declared to be producing bad data
    bool badIMUdata;                // boolean true if the bad IMU data is detected

    float gpsNoiseScaler;           // Used to scale the  GPS measurement noise and consistency gates to compensate for operation with small satellite counts
    Vector28 Kfusion;               // Kalman gain vector
    Matrix24 KH;                    // intermediate result used for covariance updates
    Matrix24 KHP;                   // intermediate result used for covariance updates
    Matrix24 P;                     // covariance matrix
    imu_ring_buffer_t<imu_elements> storedIMU;      // IMU data buffer
    obs_ring_buffer_t<gps_elements> storedGPS;      // GPS data buffer
    obs_ring_buffer_t<mag_elements> storedMag;      // Magnetometer data buffer
    obs_ring_buffer_t<baro_elements> storedBaro;    // Baro data buffer
    obs_ring_buffer_t<tas_elements> storedTAS;      // TAS data buffer
    obs_ring_buffer_t<range_elements> storedRange;  // Range finder data buffer
    imu_ring_buffer_t<output_elements> storedOutput;// output state buffer
    Matrix3f prevTnb;               // previous nav to body transformation used for INS earth rotation compensation
    ftype accNavMag;                // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
    ftype accNavMagHoriz;           // magnitude of navigation accel in horizontal plane (m/s^2)
    Vector3f earthRateNED;          // earths angular rate vector in NED (rad/s)
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
    bool fuseVelData;               // this boolean causes the velNED measurements to be fused
    bool fusePosData;               // this boolean causes the posNE measurements to be fused
    bool fuseHgtData;               // this boolean causes the hgtMea measurements to be fused
    Vector3f innovMag;              // innovation output from fusion of X,Y,Z compass measurements
    Vector3f varInnovMag;           // innovation variance output from fusion of X,Y,Z compass measurements
    ftype innovVtas;                // innovation output from fusion of airspeed measurements
    ftype varInnovVtas;             // innovation variance output from fusion of airspeed measurements
    bool magFusePerformed;          // boolean set to true when magnetometer fusion has been perfomred in that time step
    bool magFuseRequired;           // boolean set to true when magnetometer fusion will be perfomred in the next time step
    uint32_t prevTasStep_ms;        // time stamp of last TAS fusion step
    uint32_t prevBetaStep_ms;       // time stamp of last synthetic sideslip fusion step
    uint32_t lastMagUpdate_us;      // last time compass was updated in usec
    Vector3f velDotNED;             // rate of change of velocity in NED frame
    Vector3f velDotNEDfilt;         // low pass filtered velDotNED
    uint32_t imuSampleTime_ms;      // time that the last IMU value was taken
    bool tasDataToFuse;             // true when new airspeed data is waiting to be fused
    uint32_t lastBaroReceived_ms;   // time last time we received baro height data
    uint16_t hgtRetryTime_ms;       // time allowed without use of height measurements before a height timeout is declared
    uint32_t lastVelPassTime_ms;    // time stamp when GPS velocity measurement last passed innovation consistency check (msec)
    uint32_t lastPosPassTime_ms;    // time stamp when GPS position measurement last passed innovation consistency check (msec)
    uint32_t lastHgtPassTime_ms;    // time stamp when height measurement last passed innovation consistency check (msec)
    uint32_t lastTasPassTime_ms;    // time stamp when airspeed measurement last passed innovation consistency check (msec)
    uint32_t lastTimeGpsReceived_ms;// last time we received GPS data
    uint32_t timeAtLastAuxEKF_ms;   // last time the auxiliary filter was run to fuse range or optical flow measurements
    uint32_t secondLastGpsTime_ms;  // time of second last GPS fix used to determine how long since last update
    uint32_t lastHealthyMagTime_ms; // time the magnetometer was last declared healthy
    bool allMagSensorsFailed;       // true if all magnetometer sensors have timed out on this flight and we are no longer using magnetometer data
    uint32_t lastSynthYawTime_ms;   // time stamp when synthetic yaw measurement was last fused to maintain covariance health (msec)
    uint32_t ekfStartTime_ms;       // time the EKF was started (msec)
    Matrix24 nextP;                 // Predicted covariance matrix before addition of process noise to diagonals
    Vector24 processNoise;          // process noise added to diagonals of predicted covariance matrix
    Vector25 SF;                    // intermediate variables used to calculate predicted covariance matrix
    Vector5 SG;                     // intermediate variables used to calculate predicted covariance matrix
    Vector8 SQ;                     // intermediate variables used to calculate predicted covariance matrix
    Vector23 SPP;                   // intermediate variables used to calculate predicted covariance matrix
    Vector2f lastKnownPositionNE;   // last known position
    uint32_t lastDecayTime_ms;      // time of last decay of GPS position offset
    float velTestRatio;             // sum of squares of GPS velocity innovation divided by fail threshold
    float posTestRatio;             // sum of squares of GPS position innovation divided by fail threshold
    float hgtTestRatio;             // sum of squares of baro height innovation divided by fail threshold
    Vector3f magTestRatio;          // sum of squares of magnetometer innovations divided by fail threshold
    float tasTestRatio;             // sum of squares of true airspeed innovation divided by fail threshold
    bool inhibitWindStates;         // true when wind states and covariances are to remain constant
    bool inhibitMagStates;          // true when magnetic field states and covariances are to remain constant
    bool gpsNotAvailable;           // bool true when valid GPS data is not available
    uint8_t last_gps_idx;           // sensor ID of the GPS receiver used for the last fusion or reset
    struct Location EKF_origin;     // LLH origin of the NED axis system
    bool validOrigin;               // true when the EKF origin is valid
    float gpsSpdAccuracy;           // estimated speed accuracy in m/s returned by the GPS receiver
    float gpsPosAccuracy;           // estimated position accuracy in m returned by the GPS receiver
    float gpsHgtAccuracy;           // estimated height accuracy in m returned by the GPS receiver
    uint32_t lastGpsVelFail_ms;     // time of last GPS vertical velocity consistency check fail
    uint32_t lastGpsAidBadTime_ms;  // time in msec gps aiding was last detected to be bad
    float posDownAtTakeoff;         // flight vehicle vertical position sampled at transition from on-ground to in-air and used as a reference (m)
    bool useGpsVertVel;             // true if GPS vertical velocity should be used
    float yawResetAngle;            // Change in yaw angle due to last in-flight yaw reset in radians. A positive value means the yaw angle has increased.
    uint32_t lastYawReset_ms;       // System time at which the last yaw reset occurred. Returned by getLastYawResetAngle
    Vector3f tiltErrVec;            // Vector of most recent attitude error correction from Vel,Pos fusion
    float tiltErrFilt;              // Filtered tilt error metric
    bool tiltAlignComplete;         // true when tilt alignment is complete
    bool yawAlignComplete;          // true when yaw alignment is complete
    bool magStateInitComplete;      // true when the magnetic field sttes have been initialised
    uint8_t stateIndexLim;          // Max state index used during matrix and array operations
    imu_elements imuDataDelayed;    // IMU data at the fusion time horizon
    imu_elements imuDataNew;        // IMU data at the current time horizon
    imu_elements imuDataDownSampledNew; // IMU data at the current time horizon that has been downsampled to a 100Hz rate
    Quaternion imuQuatDownSampleNew; // Quaternion obtained by rotating through the IMU delta angles since the start of the current down sampled frame
    uint8_t fifoIndexNow;           // Global index for inertial and output solution at current time horizon
    uint8_t fifoIndexDelayed;       // Global index for inertial and output solution at delayed/fusion time horizon
    baro_elements baroDataNew;      // Baro data at the current time horizon
    baro_elements baroDataDelayed;  // Baro data at the fusion time horizon
    uint8_t baroStoreIndex;         // Baro data storage index
    range_elements rangeDataNew;    // Range finder data at the current time horizon
    range_elements rangeDataDelayed;// Range finder data at the fusion time horizon
    uint8_t rangeStoreIndex;        // Range finder data storage index
    tas_elements tasDataNew;        // TAS data at the current time horizon
    tas_elements tasDataDelayed;    // TAS data at the fusion time horizon
    uint8_t tasStoreIndex;          // TAS data storage index
    mag_elements magDataNew;        // Magnetometer data at the current time horizon
    mag_elements magDataDelayed;    // Magnetometer data at the fusion time horizon
    uint8_t magStoreIndex;          // Magnetometer data storage index
    gps_elements gpsDataNew;        // GPS data at the current time horizon
    gps_elements gpsDataDelayed;    // GPS data at the fusion time horizon
    uint8_t gpsStoreIndex;          // GPS data storage index
    output_elements outputDataNew;  // output state data at the current time step
    output_elements outputDataDelayed; // output state data at the current time step
    Vector3f delAngCorrection;      // correction applied to delta angles used by output observer to track the EKF
    Vector3f velErrintegral;        // integral of output predictor NED velocity tracking error (m)
    Vector3f posErrintegral;        // integral of output predictor NED position tracking error (m.sec)
    float innovYaw;                 // compass yaw angle innovation (rad)
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
    Vector3f lastMagOffsets;        // Last magnetometer offsets from COMPASS_ parameters. Used to detect parameter changes.
    bool lastMagOffsetsValid;       // True when lastMagOffsets has been initialized
    Vector2f posResetNE;            // Change in North/East position due to last in-flight reset in metres. Returned by getLastPosNorthEastReset
    uint32_t lastPosReset_ms;       // System time at which the last position reset occurred. Returned by getLastPosNorthEastReset
    Vector2f velResetNE;            // Change in North/East velocity due to last in-flight reset in metres/sec. Returned by getLastVelNorthEastReset
    uint32_t lastVelReset_ms;       // System time at which the last velocity reset occurred. Returned by getLastVelNorthEastReset
    float posResetD;                // Change in Down position due to last in-flight reset in metres. Returned by getLastPosDowntReset
    uint32_t lastPosResetD_ms;      // System time at which the last position reset occurred. Returned by getLastPosDownReset
    float yawTestRatio;             // square of magnetometer yaw angle innovation divided by fail threshold
    Quaternion prevQuatMagReset;    // Quaternion from the last time the magnetic field state reset condition test was performed
    uint8_t fusionHorizonOffset;    // number of IMU samples that the fusion time horizon  has been shifted to prevent multiple EKF instances fusing data at the same time
    float hgtInnovFiltState;        // state used for fitering of the height innovations used for pre-flight checks
    uint8_t magSelectIndex;         // Index of the magnetometer that is being used by the EKF
    bool runUpdates;                // boolean true when the EKF updates can be run
    uint32_t framesSincePredict;    // number of frames lapsed since EKF instance did a state prediction
    bool startPredictEnabled;       // boolean true when the frontend has given permission to start a new state prediciton cycele
    uint8_t localFilterTimeStep_ms; // average number of msec between filter updates
    float posDownObsNoise;          // observation noise variance on the vertical position used by the state and covariance update step (m^2)
    Vector3f delAngCorrected;       // corrected IMU delta angle vector at the EKF time horizon (rad)
    Vector3f delVelCorrected;       // corrected IMU delta velocity vector at the EKF time horizon (m/s)
    bool magFieldLearned;           // true when the magnetic field has been learned
    Vector3f earthMagFieldVar;      // NED earth mag field variances for last learned field (mGauss^2)
    Vector3f bodyMagFieldVar;       // XYZ body mag field variances for last learned field (mGauss^2)
    bool delAngBiasLearned;         // true when the gyro bias has been learned
    nav_filter_status filterStatus; // contains the status of various filter outputs
    float ekfOriginHgtVar;          // Variance of the the EKF WGS-84 origin height estimate (m^2)
    double ekfGpsRefHgt;            // floating point representation of the WGS-84 reference height used to convert GPS height to local height (m)
    uint32_t lastOriginHgtTime_ms;  // last time the ekf's WGS-84 origin height was corrected
    Vector3f outputTrackError;      // attitude (rad), velocity (m/s) and position (m) tracking error magnitudes from the output observer
    Vector3f velOffsetNED;          // This adds to the earth frame velocity estimate at the IMU to give the velocity at the body origin (m/s)
    Vector3f posOffsetNED;          // This adds to the earth frame position estimate at the IMU to give the position at the body origin (m)

    // variables used to calculate a vertical velocity that is kinematically consistent with the verical position
    float posDownDerivative;        // Rate of chage of vertical position (dPosD/dt) in m/s. This is the first time derivative of PosD.
    float posDown;                  // Down position state used in calculation of posDownRate

    // variables used by the pre-initialisation GPS checks
    struct Location gpsloc_prev;    // LLH location of previous GPS measurement
    uint32_t lastPreAlignGpsCheckTime_ms;   // last time in msec the GPS quality was checked during pre alignment checks
    float gpsDriftNE;               // amount of drift detected in the GPS position during pre-flight GPs checks
    float gpsVertVelFilt;           // amount of filterred vertical GPS velocity detected durng pre-flight GPS checks
    float gpsHorizVelFilt;          // amount of filtered horizontal GPS velocity detected during pre-flight GPS checks

    // variable used by the in-flight GPS quality check
    bool gpsSpdAccPass;             // true when reported GPS speed accuracy passes in-flight checks
    bool ekfInnovationsPass;        // true when GPS innovations pass in-flight checks
    float sAccFilterState1;         // state variable for LPF applid to reported GPS speed accuracy
    float sAccFilterState2;         // state variable for peak hold filter applied to reported GPS speed
    uint32_t lastGpsCheckTime_ms;   // last time in msec the GPS quality was checked
    uint32_t lastInnovPassTime_ms;  // last time in msec the GPS innovations passed
    uint32_t lastInnovFailTime_ms;  // last time in msec the GPS innovations failed
    bool gpsAccuracyGood;           // true when the GPS accuracy is considered to be good enough for safe flight.

    // States used for unwrapping of compass yaw error
    float innovationIncrement;
    float lastInnovation;

    // variables added for optical flow fusion
    obs_ring_buffer_t<of_elements> storedOF;    // OF data buffer
    of_elements ofDataNew;          // OF data at the current time horizon
    of_elements ofDataDelayed;      // OF data at the fusion time horizon
    uint8_t ofStoreIndex;           // OF data storage index
    bool flowDataToFuse;            // true when optical flow data has is ready for fusion
    bool flowDataValid;             // true while optical flow data is still fresh
    bool fuseOptFlowData;           // this boolean causes the last optical flow measurement to be fused
    float auxFlowObsInnov;          // optical flow rate innovation from 1-state terrain offset estimator
    float auxFlowObsInnovVar;       // innovation variance for optical flow observations from 1-state terrain offset estimator
    uint32_t flowValidMeaTime_ms;   // time stamp from latest valid flow measurement (msec)
    uint32_t rngValidMeaTime_ms;    // time stamp from latest valid range measurement (msec)
    uint32_t flowMeaTime_ms;        // time stamp from latest flow measurement (msec)
    uint32_t gndHgtValidTime_ms;    // time stamp from last terrain offset state update (msec)
    Matrix3f Tbn_flow;              // transformation matrix from body to nav axes at the middle of the optical flow sample period
    Vector2 varInnovOptFlow;        // optical flow innovations variances (rad/sec)^2
    Vector2 innovOptFlow;           // optical flow LOS innovations (rad/sec)
    float Popt;                     // Optical flow terrain height state covariance (m^2)
    float terrainState;             // terrain position state (m)
    float prevPosN;                 // north position at last measurement
    float prevPosE;                 // east position at last measurement
    float varInnovRng;              // range finder observation innovation variance (m^2)
    float innovRng;                 // range finder observation innovation (m)
    float hgtMea;                   // height measurement derived from either baro, gps or range finder data (m)
    bool inhibitGndState;           // true when the terrain position state is to remain constant
    uint32_t prevFlowFuseTime_ms;   // time both flow measurement components passed their innovation consistency checks
    Vector2 flowTestRatio;          // square of optical flow innovations divided by fail threshold used by main filter where >1.0 is a fail
    float auxFlowTestRatio;         // sum of squares of optical flow innovation divided by fail threshold used by 1-state terrain offset estimator
    float R_LOS;                    // variance of optical flow rate measurements (rad/sec)^2
    float auxRngTestRatio;          // square of range finder innovations divided by fail threshold used by main filter where >1.0 is a fail
    Vector2f flowGyroBias;          // bias error of optical flow sensor gyro output
    bool rangeDataToFuse;           // true when valid range finder height data has arrived at the fusion time horizon.
    bool baroDataToFuse;            // true when valid baro height finder data has arrived at the fusion time horizon.
    bool gpsDataToFuse;             // true when valid GPS data has arrived at the fusion time horizon.
    bool magDataToFuse;             // true when valid magnetometer data has arrived at the fusion time horizon
    Vector2f heldVelNE;             // velocity held when no aiding is available
    enum AidingMode {AID_ABSOLUTE=0,    // GPS or some other form of absolute position reference aiding is being used (optical flow may also be used in parallel) so position estimates are absolute.
                     AID_NONE=1,       // no aiding is being used so only attitude and height estimates are available. Either constVelMode or constPosMode must be used to constrain tilt drift.
                     AID_RELATIVE=2    // only optical flow aiding is being used so position estimates will be relative
                    };
    AidingMode PV_AidingMode;       // Defines the preferred mode for aiding of velocity and position estimates from the INS
    AidingMode PV_AidingModePrev;   // Value of PV_AidingMode from the previous frame - used to detect transitions
    bool gpsInhibit;                // externally set flag informing the EKF not to use the GPS
    bool gndOffsetValid;            // true when the ground offset state can still be considered valid
    Vector3f delAngBodyOF;          // bias corrected delta angle of the vehicle IMU measured summed across the time since the last OF measurement
    float delTimeOF;                // time that delAngBodyOF is summed across
    Vector3f accelPosOffset;        // position of IMU accelerometer unit in body frame (m)


    // Range finder
    float baroHgtOffset;                    // offset applied when when switching to use of Baro height
    float rngOnGnd;                         // Expected range finder reading in metres when vehicle is on ground
    float storedRngMeas[2][3];              // Ringbuffer of stored range measurements for dual range sensors
    uint32_t storedRngMeasTime_ms[2][3];    // Ringbuffers of stored range measurement times for dual range sensors
    uint32_t lastRngMeasTime_ms;            // Timestamp of last range measurement
    uint8_t rngMeasIndex[2];                // Current range measurement ringbuffer index for dual range sensors
    bool terrainHgtStable;                  // true when the terrain height is stable enough to be used as a height reference
    uint32_t terrainHgtStableSet_ms;        // system time at which terrainHgtStable was set

    // Range Beacon Sensor Fusion
    obs_ring_buffer_t<rng_bcn_elements> storedRangeBeacon; // Beacon range buffer
    rng_bcn_elements rngBcnDataNew;     // Range beacon data at the current time horizon
    rng_bcn_elements rngBcnDataDelayed; // Range beacon data at the fusion time horizon
    uint8_t rngBcnStoreIndex;           // Range beacon data storage index
    uint32_t lastRngBcnPassTime_ms;     // time stamp when the range beacon measurement last passed innvovation consistency checks (msec)
    float rngBcnTestRatio;              // Innovation test ratio for range beacon measurements
    bool rngBcnHealth;                  // boolean true if range beacon measurements have passed innovation consistency check
    bool rngBcnTimeout;                 // boolean true if range beacon measurements have faled innovation consistency checks for too long
    float varInnovRngBcn;               // range beacon observation innovation variance (m^2)
    float innovRngBcn;                  // range beacon observation innovation (m)
    uint32_t lastTimeRngBcn_ms[10];     // last time we received a range beacon measurement (msec)
    bool rngBcnDataToFuse;              // true when there is new range beacon data to fuse
    Vector3f beaconVehiclePosNED;       // NED position estimate from the beacon system (NED)
    float beaconVehiclePosErr;          // estimated position error from the beacon system (m)
    uint32_t rngBcnLast3DmeasTime_ms;   // last time the beacon system returned a 3D fix (msec)
    bool rngBcnGoodToAlign;             // true when the range beacon systems 3D fix can be used to align the filter
    uint8_t lastRngBcnChecked;          // index of the last range beacon checked for data
    Vector3f receiverPos;               // receiver NED position (m) - alignment 3 state filter
    float receiverPosCov[3][3];         // Receiver position covariance (m^2) - alignment 3 state filter (
    bool rngBcnAlignmentStarted;        // True when the initial position alignment using range measurements has started
    bool rngBcnAlignmentCompleted;      // True when the initial position alignment using range measurements has finished
    uint8_t lastBeaconIndex;            // Range beacon index last read -  used during initialisation of the 3-state filter
    Vector3f rngBcnPosSum;              // Sum of range beacon NED position (m) - used during initialisation of the 3-state filter
    uint8_t numBcnMeas;                 // Number of beacon measurements - used during initialisation of the 3-state filter
    float rngSum;                       // Sum of range measurements (m) - used during initialisation of the 3-state filter
    uint8_t N_beacons;                  // Number of range beacons in use
    float maxBcnPosD;                   // maximum position of all beacons in the down direction (m)
    float minBcnPosD;                   // minimum position of all beacons in the down direction (m)
    float bcnPosOffset;                 // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)

    float bcnPosOffsetMax;             // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
    float bcnPosOffsetMaxVar;          // Variance of the bcnPosOffsetHigh state (m)
    float OffsetMaxInnovFilt;          // Filtered magnitude of the range innovations using bcnPosOffsetHigh

    float bcnPosOffsetMin;              // Vertical position offset of the beacon constellation origin relative to the EKF origin (m)
    float bcnPosOffsetMinVar;           // Variance of the bcnPosoffset state (m)
    float OffsetMinInnovFilt;           // Filtered magnitude of the range innovations using bcnPosOffsetLow

    // Range Beacon Fusion Debug Reporting
    uint8_t rngBcnFuseDataReportIndex;// index of range beacon fusion data last reported
    struct {
        float rng;          // measured range to beacon (m)
        float innov;        // range innovation (m)
        float innovVar;     // innovation variance (m^2)
        float testRatio;    // innovation consistency test ratio
        Vector3f beaconPosNED; // beacon NED position
    } rngBcnFusionReport[10];

    // height source selection logic
    uint8_t activeHgtSource;    // integer defining active height source

    // Movement detector
    bool takeOffDetected;           // true when takeoff for optical flow navigation has been detected
    float rngAtStartOfFlight;       // range finder measurement at start of flight
    uint32_t timeAtArming_ms;       // time in msec that the vehicle armed

    // baro ground effect
    bool expectGndEffectTakeoff;      // external state from ArduCopter - takeoff expected
    uint32_t takeoffExpectedSet_ms;   // system time at which expectGndEffectTakeoff was set
    bool expectGndEffectTouchdown;    // external state from ArduCopter - touchdown expected
    uint32_t touchdownExpectedSet_ms; // system time at which expectGndEffectTouchdown was set
    float meaHgtAtTakeOff;            // height measured at commencement of takeoff

    // control of post takeoff magentic field and heading resets
    bool finalInflightYawInit;      // true when the final post takeoff initialisation of yaw angle has been performed
    bool finalInflightMagInit;      // true when the final post takeoff initialisation of magnetic field states been performed
    bool magStateResetRequest;      // true if magnetic field states need to be reset using the magneteomter measurements
    bool magYawResetRequest;        // true if the vehicle yaw and magnetic field states need to be reset using the magnetometer measurements
    bool gpsYawResetRequest;        // true if the vehicle yaw needs to be reset to the GPS course
    float posDownAtLastMagReset;    // vertical position last time the mag states were reset (m)
    float yawInnovAtLastMagReset;   // magnetic yaw innovation last time the yaw and mag field states were reset (rad)
    Quaternion quatAtLastMagReset;  // quaternion states last time the mag states were reset

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
        uint8_t obsIndex;
        Matrix3f DCM;
        Vector3f MagPred;
        ftype R_MAG;
        Vector9 SH_MAG;
    } mag_state;

    // string representing last reason for prearm failure
    char prearm_fail_string[40];

    // performance counters
    AP_HAL::Util::perf_counter_t  _perf_UpdateFilter;
    AP_HAL::Util::perf_counter_t  _perf_CovariancePrediction;
    AP_HAL::Util::perf_counter_t  _perf_FuseVelPosNED;
    AP_HAL::Util::perf_counter_t  _perf_FuseMagnetometer;
    AP_HAL::Util::perf_counter_t  _perf_FuseAirspeed;
    AP_HAL::Util::perf_counter_t  _perf_FuseSideslip;
    AP_HAL::Util::perf_counter_t  _perf_TerrainOffset;
    AP_HAL::Util::perf_counter_t  _perf_FuseOptFlow;
    AP_HAL::Util::perf_counter_t  _perf_test[10];

    // timing statistics
    struct ekf_timing timing;
    
    // should we assume zero sideslip?
    bool assume_zero_sideslip(void) const;

    // vehicle specific initial gyro bias uncertainty
    float InitialGyroBiasUncertainty(void) const;
};
