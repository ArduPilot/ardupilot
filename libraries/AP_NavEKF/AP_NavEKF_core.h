/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  22 state EKF based on https://github.com/priseborough/InertialNav

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

#include <AP_HAL/AP_HAL.h>
#include <AP_NavEKF/AP_NavEKF.h>

#pragma GCC optimize("O3")

/*
  avoid a hang in the optimiser in clang++
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#define OPT_MATHS __attribute__ ((optnone))
#else
#define OPT_MATHS
#endif

#define EKF_DISABLE_INTERRUPTS 0

#include <AP_Math/vectorN.h>

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS      (1<<0)
#define MASK_GPS_HDOP       (1<<1)
#define MASK_GPS_SPD_ERR    (1<<2)
#define MASK_GPS_POS_ERR    (1<<3)
#define MASK_GPS_YAW_ERR 	(1<<4)
#define MASK_GPS_POS_DRIFT  (1<<5)
#define MASK_GPS_VERT_SPD   (1<<6)
#define MASK_GPS_HORIZ_SPD  (1<<7)

class AP_AHRS;

class NavEKF_core
{
public:
    friend class NavEKF;
    
    typedef float ftype;
#if MATH_CHECK_INDEXES
    typedef VectorN<ftype,2> Vector2;
    typedef VectorN<ftype,3> Vector3;
    typedef VectorN<ftype,4> Vector4;
    typedef VectorN<ftype,5> Vector5;
    typedef VectorN<ftype,6> Vector6;
    typedef VectorN<ftype,8> Vector8;
    typedef VectorN<ftype,9> Vector9;
    typedef VectorN<ftype,10> Vector10;
    typedef VectorN<ftype,11> Vector11;
    typedef VectorN<ftype,13> Vector13;
    typedef VectorN<ftype,14> Vector14;
    typedef VectorN<ftype,15> Vector15;
    typedef VectorN<ftype,22> Vector22;
    typedef VectorN<ftype,31> Vector31;
    typedef VectorN<ftype,34> Vector34;
    typedef VectorN<VectorN<ftype,3>,3> Matrix3;
    typedef VectorN<VectorN<ftype,22>,22> Matrix22;
    typedef VectorN<VectorN<ftype,34>,22> Matrix34_50;
    typedef VectorN<uint32_t,50> Vector_u32_50;
#else
    typedef ftype Vector2[2];
    typedef ftype Vector3[3];
    typedef ftype Vector4[4];
    typedef ftype Vector5[5];
    typedef ftype Vector6[6];
    typedef ftype Vector8[8];
    typedef ftype Vector9[9];
    typedef ftype Vector10[10];
    typedef ftype Vector11[11];
    typedef ftype Vector13[13];
    typedef ftype Vector14[14];
    typedef ftype Vector15[15];
    typedef ftype Vector22[22];
    typedef ftype Vector31[31];
    typedef ftype Vector34[34];
    typedef ftype Matrix3[3][3];
    typedef ftype Matrix22[22][22];
    typedef ftype Matrix34_50[34][50];
    typedef uint32_t Vector_u32_50[50];
#endif

    // Constructor
    NavEKF_core(NavEKF &frontend, const AP_AHRS *ahrs, AP_Baro &baro, const RangeFinder &rng);

    // This function is used to initialise the filter whilst moving, using the AHRS DCM solution
    // It should NOT be used to re-initialise after a timeout as DCM will also be corrupted
    bool InitialiseFilterDynamic(void);

    // Initialise the states from accelerometer and magnetometer data (if present)
    // This method can only be used when the vehicle is static
    bool InitialiseFilterBootstrap(void);

    // Update Filter States - this should be called whenever new IMU data is available
    void UpdateFilter(void);

    // Check basic filter health metrics and return a consolidated health status
    bool healthy(void) const;

    // Return the last calculated NED position relative to the reference point (m).
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosNED(Vector3f &pos) const;

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

    // return weighting of first IMU in blending function
    void getIMU1Weighting(float &ret) const;

    // return the individual Z-accel bias estimates in m/s^2
    void getAccelZBias(float &zbias1, float &zbias2) const;

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
    // All NED positions calcualted by the filter will be relative to this location
    // The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
    // Returns false if the filter has rejected the attempt to set the origin
    bool setOriginLLH(struct Location &loc);

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
    void  getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov) const;

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
    void  writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas);

    // return data for debugging optical flow fusion
    void getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov, float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const;

    // called by vehicle code to specify that a takeoff is happening
    // causes the EKF to compensate for expected barometer errors due to ground effect
    void setTakeoffExpected(bool val);

    // called by vehicle code to specify that a touchdown is expected to happen
    // causes the EKF to compensate for expected barometer errors due to ground effect
    void setTouchdownExpected(bool val);

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
    return filter timeout status as a bitmasked integer
     0 = position measurement timeout
     1 = velocity measurement timeout
     2 = height measurement timeout
     3 = magnetometer measurement timeout
     4 = true airspeed measurement timeout
     5 = unassigned
     6 = unassigned
     7 = unassigned
    */
    void  getFilterTimeouts(uint8_t &timeouts) const;

    /*
    return filter status flags
    */
    void  getFilterStatus(nav_filter_status &status) const;

    /*
    return filter gps quality check status
    */
    void  getFilterGpsStatus(nav_gps_status &status) const;

    // send an EKF_STATUS_REPORT message to GCS
    void send_status_report(mavlink_channel_t chan);

    // provides the height limit to be observed by the control loops
    // returns false if no height limiting is required
    // this is needed to ensure the vehicle does not fly too high when using optical flow navigation
    bool getHeightControlLimit(float &height) const;

    // returns true of the EKF thinks the GPS is glitching
    bool getGpsGlitchStatus(void) const;

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    uint32_t getLastYawResetAngle(float &yawAng) const;

    // return the amount of NE position change due to the last position reset in metres
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &pos) const;

    // return the amount of NE velocity change due to the last velocity reset in metres/sec
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    // report any reason for why the backend is refusing to initialise
    const char *prearm_failure_reason(void) const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    NavEKF &frontend;
    const AP_AHRS *_ahrs;
    AP_Baro &_baro;
    const RangeFinder &_rng;

    // the states are available in two forms, either as a Vector34, or
    // broken down as individual elements. Both are equivalent (same
    // memory)
    Vector34 states;
    struct state_elements {
        Quaternion  quat;           // 0..3
        Vector3f    velocity;       // 4..6
        Vector3f    position;       // 7..9
        Vector3f    gyro_bias;      // 10..12
        float       accel_zbias1;   // 13
        Vector2f    wind_vel;       // 14..15
        Vector3f    earth_magfield; // 16..18
        Vector3f    body_magfield;  // 19..21
        float       accel_zbias2;   // 22
        Vector3f    vel1;           // 23 .. 25
        float       posD1;          // 26
        Vector3f    vel2;           // 27 .. 29
        float       posD2;          // 30
        Vector3f    omega;          // 31 .. 33
    } &state;

    // update the quaternion, velocity and position states using IMU measurements
    void UpdateStrapdownEquationsNED();

    // calculate the predicted state covariance matrix
    void CovariancePrediction();

    // force symmetry on the state covariance matrix
    void ForceSymmetry();

    // copy covariances across from covariance prediction calculation and fix numerical errors
    void CopyAndFixCovariances();

    // constrain variances (diagonal terms) in the state covariance matrix
    void ConstrainVariances();

    // constrain states
    void ConstrainStates();

    // fuse selected position, velocity and height measurements
    void FuseVelPosNED();

    // fuse magnetometer measurements
    void FuseMagnetometer();

    // fuse true airspeed measurements
    void FuseAirspeed();

    // fuse sythetic sideslip measurement of zero
    void FuseSideslip();

    // zero specified range of rows in the state covariance matrix
    void zeroRows(Matrix22 &covMat, uint8_t first, uint8_t last);

    // zero specified range of columns in the state covariance matrix
    void zeroCols(Matrix22 &covMat, uint8_t first, uint8_t last);

    // store states along with system time stamp in msces
    void StoreStates(void);

    // Reset the stored state history and store the current state
    void StoreStatesReset(void);

    // recall state vector stored at closest time to the one specified by msec
    void RecallStates(state_elements &statesForFusion, uint32_t msec);

    // calculate nav to body quaternions from body to nav rotation matrix
    void quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const;

    // calculate the NED earth spin vector in rad/sec
    void calcEarthRateNED(Vector3f &omega, int32_t latitude) const;

    // calculate whether the flight vehicle is on the ground or flying from height, airspeed and GPS speed
    void SetFlightAndFusionModes();

    // initialise the covariance matrix
    void CovarianceInit();

    // helper functions for readIMUData
    bool readDeltaVelocity(uint8_t ins_index, Vector3f &dVel, float &dVel_dt);
    bool readDeltaAngle(uint8_t ins_index, Vector3f &dAng);

    // update IMU delta angle and delta velocity measurements
    void readIMUData();

    // check for new valid GPS data and update stored measurement if available
    void readGpsData();

    // check for new altitude measurement data and update stored measurement if available
    void readHgtData();

    // check for new magnetometer data and update store measurements if available
    void readMagData();

    // check for new airspeed data and update stored measurements if available
    void readAirSpdData();

    // determine when to perform fusion of GPS position and  velocity measurements
    void SelectVelPosFusion();

    // determine when to perform fusion of true airspeed measurements
    void SelectTasFusion();

    // determine when to perform fusion of synthetic sideslp measurements
    void SelectBetaFusion();

    // determine when to perform fusion of magnetometer measurements
    void SelectMagFusion();

    // force alignment of the yaw angle using GPS velocity data
    void alignYawGPS();

    // Forced alignment of the wind velocity states so that they are set to the reciprocal of
    // the ground speed and scaled to 6 m/s. This is used when launching a fly-forward vehicle without an airspeed sensor
    // on the assumption that launch will be into wind and 6 is representative global average at height
    // http://maps.google.com/gallery/details?id=zJuaSgXp_WLc.kTBytKPmNODY&hl=en
    void setWindVelStates();

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
    bool getVehicleArmStatus(void) const;

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

    // recall omega (angular rate vector) average from time specified by msec to current time
    // this is useful for motion compensation of optical flow measurements
    void RecallOmega(Vector3f &omegaAvg, uint32_t msecStart, uint32_t msecEnd);

    // Estimate terrain offset using a single state EKF
    void EstimateTerrainOffset();

    // fuse optical flow measurements into the main filter
    void FuseOptFlow();

    // Check arm status and perform required checks and mode changes
    void performArmingChecks();

    // Set the NED origin to be used until the next filter reset
    void setOrigin();

    // determine if a takeoff is expected so that we can compensate for expected barometer errors due to ground effect
    bool getTakeoffExpected();

    // determine if a touchdown is expected so that we can compensate for expected barometer errors due to ground effect
    bool getTouchdownExpected();

    // Assess GPS data quality and return true if good enough to align the EKF
    bool calcGpsGoodToAlign(void);

    // Read the range finder and take new measurements if available
    // Apply a median filter to range finder data
    void readRangeFinder();

    // check if the vehicle has taken off during optical flow navigation by looking at inertial and range finder data
    void detectOptFlowTakeoff(void);

    // align the NE earth magnetic field states with the published declination
    void alignMagStateDeclination();

    // Check for signs of bad gyro health before flight
    bool checkGyroHealthPreFlight(void) const;

    // update inflight calculaton that determines if GPS data is good enough for reliable navigation
    void calcGpsGoodForFlight(void);

    // calculate the derivative of the down position using a complementary filter applied to vertical acceleration and EKF height
    void calcPosDownDerivative();

    // Tuning parameters
    const float gpsNEVelVarAccScale;    // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    const float gpsDVelVarAccScale;     // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    const float gpsPosVarAccScale;      // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    const float msecHgtDelay;           // Height measurement delay (msec)
    const uint16_t msecMagDelay;        // Magnetometer measurement delay (msec)
    const uint16_t msecTasDelay;        // Airspeed measurement delay (msec)
    const uint16_t gpsRetryTimeUseTAS;  // GPS retry time with airspeed measurements (msec)
    const uint16_t gpsRetryTimeNoTAS;   // GPS retry time without airspeed measurements (msec)
    const uint16_t gpsFailTimeWithFlow; // If we have no GPs for longer than this and we have optical flow, then we will switch across to using optical flow (msec)
    const uint16_t hgtRetryTimeMode0;   // Height retry time with vertical velocity measurement (msec)
    const uint16_t hgtRetryTimeMode12;  // Height retry time without vertical velocity measurement (msec)
    const uint16_t tasRetryTime;        // True airspeed timeout and retry interval (msec)
    const uint32_t magFailTimeLimit_ms; // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    const float magVarRateScale;        // scale factor applied to magnetometer variance due to angular rate
    const float gyroBiasNoiseScaler;    // scale factor applied to gyro bias state process noise when on ground
    const float accelBiasNoiseScaler;   // scale factor applied to accel bias state process noise when on ground
    const uint16_t msecGpsAvg;          // average number of msec between GPS measurements
    const uint16_t msecHgtAvg;          // average number of msec between height measurements
    const uint16_t msecMagAvg;          // average number of msec between magnetometer measurements
    const uint16_t msecBetaAvg;         // average number of msec between synthetic sideslip measurements
    const uint16_t msecBetaMax;         // maximum number of msec between synthetic sideslip measurements
    const uint16_t msecFlowAvg;         // average number of msec between optical flow measurements
    const float dtVelPos;               // number of seconds between position and velocity corrections. This should be a multiple of the imu update interval.
    const float covTimeStepMax;         // maximum time (sec) between covariance prediction updates
    const float covDelAngMax;           // maximum delta angle between covariance prediction updates
    const uint32_t TASmsecMax;          // maximum allowed interval between airspeed measurement updates
    const float DCM33FlowMin;           // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    const float fScaleFactorPnoise;     // Process noise added to focal length scale factor state variance at each time step
    const uint8_t flowTimeDeltaAvg_ms;  // average interval between optical flow measurements (msec)
    const uint32_t flowIntervalMax_ms;  // maximum allowable time between flow fusion events


    // ground effect tuning parameters
    const uint16_t gndEffectTimeout_ms;      // time in msec that ground effect mode is active after being activated
    const float gndEffectBaroScaler;    // scaler applied to the barometer observation variance when ground effect mode is active


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
    bool badMag;                    // boolean true if the magnetometer is declared to be producing bad data
    bool badIMUdata;                // boolean true if the bad IMU data is detected

    float gpsNoiseScaler;           // Used to scale the  GPS measurement noise and consistency gates to compensate for operation with small satellite counts
    Vector31 Kfusion;               // Kalman gain vector
    Matrix22 KH;                    // intermediate result used for covariance updates
    Matrix22 KHP;                   // intermediate result used for covariance updates
    Matrix22 P;                     // covariance matrix
    VectorN<state_elements,50> storedStates;       // state vectors stored for the last 50 time steps
    Vector_u32_50 statetimeStamp;    // time stamp for each state vector stored
    Vector3f correctedDelAng;       // delta angles about the xyz body axes corrected for errors (rad)
    Quaternion correctedDelAngQuat; // quaternion representation of correctedDelAng
    Vector3f correctedDelVel12;     // delta velocities along the XYZ body axes for weighted average of IMU1 and IMU2 corrected for errors (m/s)
    Vector3f correctedDelVel1;      // delta velocities along the XYZ body axes for IMU1 corrected for errors (m/s)
    Vector3f correctedDelVel2;      // delta velocities along the XYZ body axes for IMU2 corrected for errors (m/s)
    Vector3f summedDelAng;          // corrected & summed delta angles about the xyz body axes (rad)
    Vector3f summedDelVel;          // corrected & summed delta velocities along the XYZ body axes (m/s)
    Vector3f lastGyroBias;          // previous gyro bias vector used by filter divergence check
    Matrix3f prevTnb;               // previous nav to body transformation used for INS earth rotation compensation
    ftype accNavMag;                // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
    ftype accNavMagHoriz;           // magnitude of navigation accel in horizontal plane (m/s^2)
    Vector3f earthRateNED;          // earths angular rate vector in NED (rad/s)
    Vector3f dVelIMU1;              // delta velocity vector in XYZ body axes measured by IMU1 (m/s)
    Vector3f dVelIMU2;              // delta velocity vector in XYZ body axes measured by IMU2 (m/s)
    Vector3f dAngIMU;               // delta angle vector in XYZ body axes measured by the IMU (rad)
    ftype dtIMUavg;                 // expected time between IMU measurements (sec)
    ftype dtDelAng;              // time lapsed since the last IMU measurement (sec)
    ftype dt;                       // time lapsed since the last covariance prediction (sec)
    ftype hgtRate;                  // state for rate of change of height filter
    bool onGround;                  // boolean true when the flight vehicle is on the ground (not flying)
    bool prevOnGround;              // value of onGround from previous update
    bool manoeuvring;               // boolean true when the flight vehicle is performing horizontal changes in velocity
    uint32_t airborneDetectTime_ms; // last time flight movement was detected
    Vector6 innovVelPos;            // innovation output for a group of measurements
    Vector6 varInnovVelPos;         // innovation variance output for a group of measurements
    bool fuseVelData;               // this boolean causes the velNED measurements to be fused
    bool fusePosData;               // this boolean causes the posNE measurements to be fused
    bool fuseHgtData;               // this boolean causes the hgtMea measurements to be fused
    Vector3f velNED;                // North, East, Down velocity measurements (m/s)
    Vector2f gpsPosNE;              // North, East position measurements (m)
    ftype hgtMea;                   //  height measurement relative to reference point  (m)
    state_elements statesAtVelTime; // States at the effective time of velNED measurements
    state_elements statesAtPosTime; // States at the effective time of posNE measurements
    state_elements statesAtHgtTime; // States at the effective time of hgtMea measurement
    Vector3f innovMag;              // innovation output from fusion of X,Y,Z compass measurements
    Vector3f varInnovMag;           // innovation variance output from fusion of X,Y,Z compass measurements
    Vector3f magData;               // magnetometer flux readings in X,Y,Z body axes
    state_elements statesAtMagMeasTime;   // filter states at the effective time of compass measurements
    ftype innovVtas;                // innovation output from fusion of airspeed measurements
    ftype varInnovVtas;             // innovation variance output from fusion of airspeed measurements
    bool fuseVtasData;              // boolean true when airspeed data is to be fused
    float VtasMeas;                 // true airspeed measurement (m/s)
    state_elements statesAtVtasMeasTime;  // filter states at the effective measurement time
    bool covPredStep;               // boolean set to true when a covariance prediction step has been performed
    bool magFusePerformed;          // boolean set to true when magnetometer fusion has been perfomred in that time step
    bool magFuseRequired;           // boolean set to true when magnetometer fusion will be perfomred in the next time step
    bool posVelFuseStep;            // boolean set to true when position and velocity fusion is being performed
    bool tasFuseStep;               // boolean set to true when airspeed fusion is being performed
    uint32_t TASmsecPrev;           // time stamp of last TAS fusion step
    uint32_t BETAmsecPrev;          // time stamp of last synthetic sideslip fusion step
    uint32_t MAGmsecPrev;           // time stamp of last compass fusion step
    uint32_t HGTmsecPrev;           // time stamp of last height measurement fusion step
    bool constPosMode;              // true when fusing a constant position to maintain attitude reference for planned operation without GPS or optical flow data
    uint32_t lastMagUpdate;         // last time compass was updated
    Vector3f velDotNED;             // rate of change of velocity in NED frame
    Vector3f velDotNEDfilt;         // low pass filtered velDotNED
    uint32_t lastAirspeedUpdate;    // last time airspeed was updated
    uint32_t imuSampleTime_ms;      // time that the last IMU value was taken
    bool newDataGps;                // true when new GPS data has arrived
    bool newDataMag;                // true when new magnetometer data has arrived
    bool newDataTas;                // true when new airspeed data has arrived
    bool tasDataWaiting;            // true when new airspeed data is waiting to be fused
    bool newDataHgt;                // true when new height data has arrived
    uint32_t lastHgtMeasTime;       // time of last height measurement used to determine if new data has arrived
    uint16_t hgtRetryTime;          // time allowed without use of height measurements before a height timeout is declared
    uint32_t lastVelPassTime;       // time stamp when GPS velocity measurement last passed innovation consistency check (msec)
    uint32_t lastPosPassTime;       // time stamp when GPS position measurement last passed innovation consistency check (msec)
    uint32_t lastPosFailTime;       // time stamp when GPS position measurement last failed innovation consistency check (msec)
    uint32_t lastHgtPassTime_ms;       // time stamp when height measurement last passed innovation consistency check (msec)
    uint32_t lastTasPassTime;       // time stamp when airspeed measurement last passed innovation consistency check (msec)
    uint8_t storeIndex;             // State vector storage index
    uint32_t lastStateStoreTime_ms; // time of last state vector storage
    uint32_t lastFixTime_ms;        // time of last GPS fix used to determine if new data has arrived
    uint32_t timeAtLastAuxEKF_ms;   // last time the auxiliary filter was run to fuse range or optical flow measurements
    uint32_t secondLastFixTime_ms;  // time of second last GPS fix used to determine how long since last update
    uint32_t lastHealthyMagTime_ms; // time the magnetometer was last declared healthy
    uint32_t ekfStartTime_ms;       // time the EKF was started (msec)
    Vector3f lastAngRate;           // angular rate from previous IMU sample used for trapezoidal integrator
    Vector3f lastAccel1;            // acceleration from previous IMU1 sample used for trapezoidal integrator
    Vector3f lastAccel2;            // acceleration from previous IMU2 sample used for trapezoidal integrator
    Matrix22 nextP;                 // Predicted covariance matrix before addition of process noise to diagonals
    Vector22 processNoise;          // process noise added to diagonals of predicted covariance matrix
    Vector15 SF;                    // intermediate variables used to calculate predicted covariance matrix
    Vector8 SG;                     // intermediate variables used to calculate predicted covariance matrix
    Vector11 SQ;                    // intermediate variables used to calculate predicted covariance matrix
    Vector8 SPP;                    // intermediate variables used to calculate predicted covariance matrix
    float IMU1_weighting;           // Weighting applied to use of IMU1. Varies between 0 and 1.
    bool yawAligned;                // true when the yaw angle has been aligned
    Vector2f lastKnownPositionNE;   // last known position
    uint32_t lastDecayTime_ms;      // time of last decay of GPS position offset
    float velTestRatio;             // sum of squares of GPS velocity innovation divided by fail threshold
    float posTestRatio;             // sum of squares of GPS position innovation divided by fail threshold
    float hgtTestRatio;             // sum of squares of baro height innovation divided by fail threshold
    Vector3f magTestRatio;          // sum of squares of magnetometer innovations divided by fail threshold
    float tasTestRatio;             // sum of squares of true airspeed innovation divided by fail threshold
    bool inhibitWindStates;         // true when wind states and covariances are to remain constant
    bool inhibitMagStates;          // true when magnetic field states and covariances are to remain constant
    bool firstArmComplete;          // true when first transition out of static mode has been performed after start up
    bool firstMagYawInit;           // true when the first post takeoff initialisation of earth field and yaw angle has been performed
    bool secondMagYawInit;          // true when the second post takeoff initialisation of earth field and yaw angle has been performed
    bool flowTimeout;               // true when optical flow measurements have time out
    bool gpsNotAvailable;           // bool true when valid GPS data is not available
    bool vehicleArmed;              // true when the vehicle is disarmed
    bool prevVehicleArmed;          // vehicleArmed from previous frame
    struct Location EKF_origin;     // LLH origin of the NED axis system - do not change unless filter is reset
    bool validOrigin;               // true when the EKF origin is valid
    float gpsSpdAccuracy;           // estimated speed accuracy in m/s returned by the UBlox GPS receiver
    uint32_t lastGpsVelFail_ms;     // time of last GPS vertical velocity consistency check fail
    Vector3f lastMagOffsets;        // magnetometer offsets returned by compass object from previous update
    bool gpsAidingBad;              // true when GPS position measurements have been consistently rejected by the filter
    uint32_t lastGpsAidBadTime_ms;  // time in msec gps aiding was last detected to be bad
    float posDownAtArming;          // flight vehicle vertical position at arming used as a reference point
    bool highYawRate;               // true when the vehicle is doing rapid yaw rotation where gyro scel factor errors could cause loss of heading reference
    float yawRateFilt;              // filtered yaw rate used to determine when the vehicle is doing rapid yaw rotation where gyro scel factor errors could cause loss of heading reference
    bool useGpsVertVel;             // true if GPS vertical velocity should be used
    float yawResetAngle;            // Change in yaw angle due to last in-flight yaw reset in radians. A positive value means the yaw angle has increased.
    uint32_t lastYawReset_ms;       // System time at which the last yaw reset occurred. Returned by getLastYawResetAngle
    uint32_t magYawResetTimer_ms;   // timer in msec used to track how long good magnetometer data is failing innovation consistency checks
    bool consistentMagData;         // true when the magnetometers are passing consistency checks
    bool gpsAccuracyGood;           // true when the GPS accuracy is considered to be good enough for safe flight.
    uint32_t timeAtDisarm_ms;       // time of last disarm event in msec
    float gpsDriftNE;               // amount of drift detected in the GPS position during pre-flight GPs checks
    float gpsVertVelFilt;           // amount of filterred vertical GPS velocity detected durng pre-flight GPS checks
    float gpsHorizVelFilt;          // amount of filtered horizontal GPS velocity detected during pre-flight GPS checks
    bool gpsGoodToAlign;            // true when GPS quality is good enough to set an EKF origin and commence GPS navigation
    uint32_t lastConstPosFuseTime_ms;   // last time in msec the constant position constraint was applied
    float posDownDerivative;        // Rate of chage of vertical position (dPosD/dt) in m/s. This is the first time derivative of PosD.
    float posDown;                  // Down position state used in calculation of posDownRate
    Vector2f posResetNE;            // Change in North/East position due to last in-flight reset in metres. Returned by getLastPosNorthEastReset
    uint32_t lastPosReset_ms;       // System time at which the last position reset occurred. Returned by getLastPosNorthEastReset
    Vector2f velResetNE;            // Change in North/East velocity due to last in-flight reset in metres/sec. Returned by getLastVelNorthEastReset
    uint32_t lastVelReset_ms;       // System time at which the last velocity reset occurred. Returned by getLastVelNorthEastReset
    Vector3f delAngBiasAtArming;    // value of the gyro delta angle bias at arming
    float hgtInnovFiltState;        // state used for fitering of the height innovations used for pre-flight checks
    Quaternion prevQuatMagReset;    // Quaternion from the previous frame that the magnetic field state reset condition test was performed

    // Used by smoothing of state corrections
    Vector10 gpsIncrStateDelta;    // vector of corrections to attitude, velocity and position to be applied over the period between the current and next GPS measurement
    Vector10 hgtIncrStateDelta;    // vector of corrections to attitude, velocity and position to be applied over the period between the current and next height measurement
    Vector10 magIncrStateDelta;    // vector of corrections to attitude, velocity and position to be applied over the period between the current and next magnetometer measurement
    uint8_t gpsUpdateCount;         // count of the number of minor state corrections using GPS data
    uint8_t gpsUpdateCountMax;      // limit on the number of minor state corrections using GPS data
    float gpsUpdateCountMaxInv;     // floating point inverse of gpsFilterCountMax
    uint8_t hgtUpdateCount;         // count of the number of minor state corrections using Baro data
    uint8_t hgtUpdateCountMax;      // limit on the number of minor state corrections using Baro data
    float hgtUpdateCountMaxInv;     // floating point inverse of hgtFilterCountMax
    uint8_t magUpdateCount;         // count of the number of minor state corrections using Magnetometer data
    uint8_t magUpdateCountMax;      // limit on the number of minor state corrections using Magnetometer data
    float magUpdateCountMaxInv;     // floating point inverse of magFilterCountMax

    // variables added for optical flow fusion
    bool newDataFlow;               // true when new optical flow data has arrived
    bool flowFusePerformed;         // true when optical flow fusion has been performed in that time step
    bool flowDataValid;             // true while optical flow data is still fresh
    state_elements statesAtFlowTime;// States at the middle of the optical flow sample period
    bool fuseOptFlowData;           // this boolean causes the last optical flow measurement to be fused
    float auxFlowObsInnov;          // optical flow rate innovation from 1-state terrain offset estimator
    float auxFlowObsInnovVar;       // innovation variance for optical flow observations from 1-state terrain offset estimator
    Vector2 flowRadXYcomp;         // motion compensated optical flow angular rates(rad/sec)
    Vector2 flowRadXY;             // raw (non motion compensated) optical flow angular rates (rad/sec)
    uint32_t flowValidMeaTime_ms;   // time stamp from latest valid flow measurement (msec)
    uint32_t rngValidMeaTime_ms;    // time stamp from latest valid range measurement (msec)
    uint32_t flowMeaTime_ms;        // time stamp from latest flow measurement (msec)
    uint8_t flowQuality;            // unsigned integer representing quality of optical flow data. 255 is maximum quality.
    uint32_t gndHgtValidTime_ms;    // time stamp from last terrain offset state update (msec)
    Vector3f omegaAcrossFlowTime;   // body angular rates averaged across the optical flow sample period
    Matrix3f Tnb_flow;              // transformation matrix from nav to body axes at the middle of the optical flow sample period
    Matrix3f Tbn_flow;              // transformation matrix from body to nav axes at the middle of the optical flow sample period
    Vector2 varInnovOptFlow;       // optical flow innovations variances (rad/sec)^2
    Vector2 innovOptFlow;          // optical flow LOS innovations (rad/sec)
    float Popt;                     // Optical flow terrain height state covariance (m^2)
    float terrainState;             // terrain position state (m)
    float prevPosN;                 // north position at last measurement
    float prevPosE;                 // east position at last measurement
    state_elements statesAtRngTime; // States at the range finder measurement time
    bool fuseRngData;               // true when fusion of range data is demanded
    float varInnovRng;              // range finder observation innovation variance (m^2)
    float innovRng;                 // range finder observation innovation (m)
    float rngMea;                   // range finder measurement (m)
    bool inhibitGndState;           // true when the terrain position state is to remain constant
    uint32_t prevFlowFuseTime_ms;   // time both flow measurement components passed their innovation consistency checks
    Vector2 flowTestRatio;         // square of optical flow innovations divided by fail threshold used by main filter where >1.0 is a fail
    float auxFlowTestRatio;         // sum of squares of optical flow innovation divided by fail threshold used by 1-state terrain offset estimator
    float R_LOS;                    // variance of optical flow rate measurements (rad/sec)^2
    float auxRngTestRatio;          // square of range finder innovations divided by fail threshold used by main filter where >1.0 is a fail
    Vector2f flowGyroBias;          // bias error of optical flow sensor gyro output
    uint8_t flowUpdateCount;        // count of the number of minor state corrections using optical flow data
    uint8_t flowUpdateCountMax;     // limit on the number of minor state corrections using optical flow data
    float flowUpdateCountMaxInv;    // floating point inverse of flowUpdateCountMax
    Vector10 flowIncrStateDelta;   // vector of corrections to attitude, velocity and position to be applied over the period between the current and next magnetometer measurement
    bool newDataRng;                // true when new valid range finder data has arrived.
    Vector2f heldVelNE;             // velocity held when no aiding is available
    enum AidingMode {AID_ABSOLUTE=0,    // GPS aiding is being used (optical flow may also be used) so position estimates are absolute.
                      AID_NONE=1,       // no aiding is being used so only attitude and height estimates are available. Either constVelMode or constPosMode must be used to constrain tilt drift.
                      AID_RELATIVE=2    // only optical flow aiding is being used so position estimates will be relative
                     };
    AidingMode PV_AidingMode;           // Defines the preferred mode for aiding of velocity and position estimates from the INS
    bool gndOffsetValid;            // true when the ground offset state can still be considered valid
    bool flowXfailed;               // true when the X optical flow measurement has failed the innovation consistency check

    // Range finder
    float baroHgtOffset;            // offset applied when baro height used as a backup height reference if range-finder fails
    float rngOnGnd;                 // Expected range finder reading in metres when vehicle is on ground

    // Movement detector
    bool takeOffDetected;           // true when takeoff for optical flow navigation has been detected
    float rangeAtArming;            // range finder measurement when armed
    uint32_t timeAtArming_ms;       // time in msec that the vehicle armed

    // IMU processing
    float dtDelVel1;
    float dtDelVel2;

    // baro ground effect
    bool expectGndEffectTakeoff;      // external state from ArduCopter - takeoff expected
    uint32_t takeoffExpectedSet_ms;   // system time at which expectGndEffectTakeoff was set
    bool expectGndEffectTouchdown;    // external state from ArduCopter - touchdown expected
    uint32_t touchdownExpectedSet_ms; // system time at which expectGndEffectTouchdown was set
    float meaHgtAtTakeOff;            // height measured at commencement of takeoff

    // monitoring IMU quality
    float imuNoiseFiltState1;       // peak hold noise estimate for IMU 1
    float imuNoiseFiltState2;       // peak hold noise estimate for IMU 2
    Vector3f accelDiffFilt;         // filtered difference between IMU 1 and 2
    enum ImuSwitchState {
        IMUSWITCH_MIXED=0,          // IMU 0 & 1 are mixed
        IMUSWITCH_IMU0,             // only IMU 0 is used
        IMUSWITCH_IMU1              // only IMU 1 is used
    };
    ImuSwitchState lastImuSwitchState;  // last switch state (see imuSwitchState enum)

    // states held by optical flow fusion across time steps
    // optical flow X,Y motion compensated rate measurements are fused across two time steps
    // to level computational load as this can be an expensive operation
    struct {
        uint8_t obsIndex;
        Vector4 SH_LOS;
        Vector10 SK_LOS;
        ftype q0;
        ftype q1;
        ftype q2;
        ftype q3;
        ftype vn;
        ftype ve;
        ftype vd;
        ftype pd;
        Vector2 losPred;
    } flow_state;

    struct {
        bool bad_xmag:1;
        bool bad_ymag:1;
        bool bad_zmag:1;
        bool bad_airspeed:1;
        bool bad_sideslip:1;
    } faultStatus;

    // flags indicating which GPS quality checks are failing
    struct {
        bool bad_sAcc:1;
        bool bad_hAcc:1;
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
    AP_HAL::Util::perf_counter_t  _perf_OpticalFlowEKF;
    AP_HAL::Util::perf_counter_t  _perf_FuseOptFlow;
    
    // should we assume zero sideslip?
    bool assume_zero_sideslip(void) const;

    // vehicle specific initial gyro bias uncertainty
    float InitialGyroBiasUncertainty(void) const;
};
