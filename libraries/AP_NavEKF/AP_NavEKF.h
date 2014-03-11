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

#ifndef AP_NavEKF
#define AP_NavEKF

#include <AP_Math.h>
#include <AP_InertialSensor.h>
#include <AP_Baro.h>
#include <AP_Airspeed.h>
#include <AP_Compass.h>
#include <AP_Param.h>

// #define MATH_CHECK_INDEXES 1

#include <vectorN.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <systemlib/perf_counter.h>
#endif


class AP_AHRS;

class NavEKF
{
public:
    typedef float ftype;
#if MATH_CHECK_INDEXES
    typedef VectorN<ftype,2> Vector2;
    typedef VectorN<ftype,3> Vector3;
    typedef VectorN<ftype,6> Vector6;
    typedef VectorN<ftype,8> Vector8;
    typedef VectorN<ftype,11> Vector11;
    typedef VectorN<ftype,13> Vector13;
    typedef VectorN<ftype,14> Vector14;
    typedef VectorN<ftype,15> Vector15;
    typedef VectorN<ftype,22> Vector22;
    typedef VectorN<VectorN<ftype,3>,3> Matrix3;
    typedef VectorN<VectorN<ftype,22>,22> Matrix22;
    typedef VectorN<VectorN<ftype,50>,22> Matrix22_50;
#else
    typedef ftype Vector2[2];
    typedef ftype Vector3[3];
    typedef ftype Vector6[6];
    typedef ftype Vector8[8];
    typedef ftype Vector11[11];
    typedef ftype Vector13[13];
    typedef ftype Vector14[14];
    typedef ftype Vector15[15];
    typedef ftype Vector22[22];
    typedef ftype Vector31[31];
    typedef ftype Matrix3[3][3];
    typedef ftype Matrix22[22][22];
    typedef ftype Matrix31_50[31][50];
#endif

    // Constructor
    NavEKF(const AP_AHRS *ahrs, AP_Baro &baro);

    // This function is used to initialise the filter whilst moving, using the AHRS DCM solution
    // It should NOT be used to re-initialise after a timeout as DCM will also be corrupted
    void InitialiseFilterDynamic(void);

    // Initialise the states from accelerometer and magnetometer data (if present)
    // This method can only be used when the vehicle is static
    void InitialiseFilterBootstrap(void);

    // Update Filter States - this should be called whenever new IMU data is available
    void UpdateFilter(void);

    // Check basic filter health metrics and return a consolidated health status
    bool healthy(void) const;

    // return true if filter is dead-reckoning height
    bool HeightDrifting(void) const;

    // return true if filter is dead-reckoning position
    bool PositionDrifting(void) const;

    // return the last calculated NED position relative to the reference point (m).
    // return false if no position is available
    bool getPosNED(Vector3f &pos) const;

    // return NED velocity in m/s
    void getVelNED(Vector3f &vel) const;

    // return body axis gyro bias estimates in rad/sec
    void getGyroBias(Vector3f &gyroBias) const;

    // return weighting of first IMU in blending function and the individual Z-accel bias estimates in m/s^2
    void getAccelBias(Vector3f &accelBias) const;

    // return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
    void getWind(Vector3f &wind) const;

    // return earth magnetic field estimates in measurement units / 1000
    void getMagNED(Vector3f &magNED) const;

    // return body magnetic field estimates in measurement units / 1000
    void getMagXYZ(Vector3f &magXYZ) const;

    // return the last calculated latitude, longitude and height
    bool getLLH(struct Location &loc) const;

    // return the Euler roll, pitch and yaw angle in radians
    void getEulerAngles(Vector3f &eulers) const;

    // return the transformation matrix from XYZ (body) to NED axes
    void getRotationBodyToNED(Matrix3f &mat) const;

    // return the quaternions defining the rotation from NED to XYZ (body) axes
    void getQuaternion(Quaternion &quat) const;

    // return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
    void  getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov) const;

    // return the innovation variances for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
    void  getVariances(Vector3f &velVar, Vector3f &posVar, Vector3f &magVar, float &tasVar) const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_AHRS *_ahrs;
    AP_Baro &_baro;

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
    void RecallStates(Vector31 &statesForFusion, uint32_t msec);

    // calculate nav to body quaternions from body to nav rotation matrix
    void quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const;

    // calculate the NED earth spin vector in rad/sec
    void calcEarthRateNED(Vector3f &omega, int32_t latitude) const;

    // calculate whether the flight vehicle is on the ground or flying from height, airspeed and GPS speed
    void OnGroundCheck();

    // initialise the covariance matrix
    void CovarianceInit(float roll, float pitch, float yaw);

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
    void ForceYawAlignment();

    // zero stored variables
    void ZeroVariables();

    // reset the horizontal position states uing the last GPS measurement
    void ResetPosition(void);

    // reset velocity states using the last GPS measurement
    void ResetVelocity(void);

    // reset the vertical position state using the last height measurement
    void ResetHeight(void);

    // return true if we should use the airspeed sensor
    bool useAirspeed(void) const;

    // return true if the vehicle code has requested use of static mode
    // in static mode, position and height are constrained to zero, allowing an attitude
    // reference to be initialised and maintained when on the ground and without GPS lock
    bool static_mode_demanded(void) const;

private:

    // the states are available in two forms, either as a Vector27, or
    // broken down as individual elements. Both are equivalent (same
    // memory)
    Vector31 states;
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
    } &state;

    // EKF Mavlink Tuneable Parameters
    AP_Float _gpsHorizVelNoise;     // GPS horizontal velocity measurement noise : m/s
    AP_Float _gpsVertVelNoise;      // GPS vertical velocity measurement noise : m/s
    AP_Float _gpsHorizPosNoise;     // GPS horizontal position measurement noise m
    AP_Float _baroAltNoise;         // Baro height measurement noise : m^2
    AP_Float _magNoise;             // magnetometer measurement noise : gauss
    AP_Float _easNoise;             // equivalent airspeed measurement noise : m/s
    AP_Float _windVelProcessNoise;  // wind velocity state process noise : m/s^2
    AP_Float _wndVarHgtRateScale;   // scale factor applied to wind process noise due to height rate
    AP_Float _magEarthProcessNoise; // earth magnetic field process noise : gauss/sec
    AP_Float _magBodyProcessNoise;  // earth magnetic field process noise : gauss/sec
    AP_Float _gyrNoise;             // gyro process noise : rad/s
    AP_Float _accNoise;             // accelerometer process noise : m/s^2
    AP_Float _gyroBiasProcessNoise; // gyro bias state process noise : rad/s
    AP_Float _accelBiasProcessNoise;// accel bias state process noise : m/s^2
    AP_Int16 _msecVelDelay;         // effective average delay of GPS velocity measurements rel to IMU (msec)
    AP_Int16 _msecPosDelay;         // effective average delay of GPS position measurements rel to (msec)
    AP_Int8  _fusionModeGPS;        // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity
    AP_Int8  _gpsVelInnovGate;      // Number of standard deviations applied to GPS velocity innovation consistency check
    AP_Int8  _gpsPosInnovGate;      // Number of standard deviations applied to GPS position innovation consistency check
    AP_Int8  _hgtInnovGate;         // Number of standard deviations applied to height innovation consistency check
    AP_Int8  _magInnovGate;         // Number of standard deviations applied to magnetometer innovation consistency check
    AP_Int8  _tasInnovGate;         // Number of standard deviations applied to true airspeed innovation consistency check
    AP_Int8  _magCal;               // Forces magentic field states to be always active to aid magnetometer calibration

    // Tuning parameters
    AP_Float _gpsNEVelVarAccScale;  // scale factor applied to NE velocity measurement variance due to Vdot
    AP_Float _gpsDVelVarAccScale;   // scale factor applied to D velocity measurement variance due to Vdot
    AP_Float _gpsPosVarAccScale;    // scale factor applied to position measurement variance due to Vdot
    AP_Int16 _msecHgtDelay;         // effective average delay of height measurements rel to (msec)
    AP_Int16 _msecMagDelay;         // effective average delay of magnetometer measurements rel to IMU (msec)
    AP_Int16 _msecTasDelay;         // effective average delay of airspeed measurements rel to IMU (msec)
    AP_Int16 _gpsRetryTimeUseTAS;   // GPS retry time following innovation consistency fail if TAS measurements are used (msec)
    AP_Int16 _gpsRetryTimeNoTAS;    // GPS retry time following innovation consistency fail if no TAS measurements are used (msec)
    AP_Int16 _hgtRetryTimeMode0;    // height measurement retry time following innovation consistency fail if GPS fusion mode is = 0 (msec)
    AP_Int16 _hgtRetryTimeMode12;   // height measurement retry time following innovation consistency fail if GPS fusion mode is > 0 (msec)
    float _gyroBiasNoiseScaler;     // scale factor applied to gyro bias state process variance when on ground
    float _magVarRateScale;         // scale factor applied to magnetometer variance due to angular rate
    uint16_t _msecGpsAvg;           // average number of msec between GPS measurements
    uint16_t _msecHgtAvg;           // average number of msec between height measurements
    uint16_t _msecBetaAvg;          // maximum number of msec between synthetic sideslip measurements
    float dtVelPos;                 // average of msec between position and velocity corrections

    // Variables
    uint8_t skipCounter;            // counter used to skip position and height corrections to achieve _skipRatio
    bool statesInitialised;         // boolean true when filter states have been initialised
    bool velHealth;                 // boolean true if velocity measurements have failed innovation consistency check
    bool posHealth;                 // boolean true if position measurements have failed innovation consistency check
    bool hgtHealth;                 // boolean true if height measurements have failed innovation consistency check
    bool velTimeout;                // boolean true if velocity measurements have failed innovation consistency check and timed out
    bool posTimeout;                // boolean true if position measurements have failed innovation consistency check and timed out
    bool hgtTimeout;                // boolean true if height measurements have failed innovation consistency check and timed out

    Vector31 Kfusion;               // Kalman gain vector
    Matrix22 KH;                    // intermediate result used for covariance updates
    Matrix22 KHP;                   // intermediate result used for covariance updates
    Matrix22 P;                     // covariance matrix
    Matrix31_50 storedStates;       // state vectors stored for the last 50 time steps
    uint32_t statetimeStamp[50];    // time stamp for each state vector stored
    Vector3f correctedDelAng;       // delta angles about the xyz body axes corrected for errors (rad)
    Vector3f correctedDelVel12;     // delta velocities along the XYZ body axes for weighted average of IMU1 and IMU2 corrected for errors (m/s)
    Vector3f correctedDelVel1;      // delta velocities along the XYZ body axes for IMU1 corrected for errors (m/s)
    Vector3f correctedDelVel2;      // delta velocities along the XYZ body axes for IMU2 corrected for errors (m/s)
    Vector3f summedDelAng;          // corrected & summed delta angles about the xyz body axes (rad)
    Vector3f summedDelVel;          // corrected & summed delta velocities along the XYZ body axes (m/s)
	Vector3f prevDelAng;            // previous delta angle use for INS coning error compensation
    Matrix3f prevTnb;               // previous nav to body transformation used for INS earth rotation compensation
    ftype accNavMag;                // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
    ftype accNavMagHoriz;           // magnitude of navigation accel in horizontal plane (m/s^2)
    Vector3f earthRateNED;          // earths angular rate vector in NED (rad/s)
    Vector3f dVelIMU1;              // delta velocity vector in XYZ body axes measured by IMU1 (m/s)
    Vector3f dVelIMU2;              // delta velocity vector in XYZ body axes measured by IMU2 (m/s)
    Vector3f dAngIMU;               // delta angle vector in XYZ body axes measured by the IMU (rad)
    ftype dtIMU;                    // time lapsed since the last IMU measurement (sec)
    ftype dt;                       // time lapsed since the last covariance prediction (sec)
    ftype hgtRate;                  // state for rate of change of height filter
    bool onGround;                  // boolean true when the flight vehicle is on the ground (not flying)
    bool prevOnGround;              // value of onGround from previous update
    Vector6 innovVelPos;            // innovation output for a group of measurements
    Vector6 varInnovVelPos;         // innovation variance output for a group of measurements
    bool fuseVelData;               // this boolean causes the velNED measurements to be fused
    bool fusePosData;               // this boolean causes the posNE measurements to be fused
    bool fuseHgtData;               // this boolean causes the hgtMea measurements to be fused
    Vector3f velNED;                // North, East, Down velocity measurements (m/s)
    Vector2 posNE;                  // North, East position measurements (m)
    ftype hgtMea;                   //  height measurement relative to reference point  (m)
    Vector31 statesAtVelTime;       // States at the effective time of velNED measurements
    Vector31 statesAtPosTime;       // States at the effective time of posNE measurements
    Vector31 statesAtHgtTime;       // States at the effective time of hgtMea measurement
    Vector3f innovMag;              // innovation output from fusion of X,Y,Z compass measurements
    Vector3f varInnovMag;           // innovation variance output from fusion of X,Y,Z compass measurements
    bool fuseMagData;               // boolean true when magnetometer data is to be fused
    Vector3f magData;               // magnetometer flux readings in X,Y,Z body axes
    Vector31 statesAtMagMeasTime;   // filter states at the effective time of compass measurements
    ftype innovVtas;                // innovation output from fusion of airspeed measurements
    ftype varInnovVtas;             // innovation variance output from fusion of airspeed measurements
    bool fuseVtasData;              // boolean true when airspeed data is to be fused
    float VtasMeas;                 // true airspeed measurement (m/s)
    Vector31 statesAtVtasMeasTime;  // filter states at the effective measurement time
    Vector3f magBias;               // magnetometer bias vector in XYZ body axes
    const ftype covTimeStepMax;     // maximum time allowed between covariance predictions
    const ftype covDelAngMax;       // maximum delta angle between covariance predictions
    bool covPredStep;               // boolean set to true when a covariance prediction step has been performed
    bool magFusePerformed;          // boolean set to true when magnetometer fusion has been perfomred in that time step
    bool magFuseRequired;           // boolean set to true when magnetometer fusion will be perfomred in the next time step
    bool posVelFuseStep;            // boolean set to true when position and velocity fusion is being performed
    bool tasFuseStep;               // boolean set to true when airspeed fusion is being performed
    uint32_t TASmsecPrev;           // time stamp of last TAS fusion step
    uint32_t BETAmsecPrev;          // time stamp of last synthetic sideslip fusion step
    const uint32_t TASmsecMax;      // maximum allowed interval between TAS fusion steps
    uint32_t MAGmsecPrev;           // time stamp of last compass fusion step
    uint32_t HGTmsecPrev;           // time stamp of last height measurement fusion step
    const bool fuseMeNow;           // boolean to force fusion whenever data arrives
    bool staticMode;                // boolean to force position and velocity measurements to zero for pre-arm or bench testing
    bool prevStaticMode;            // value of static mode from last update
    uint32_t lastMagUpdate;         // last time compass was updated
    Vector3f velDotNED;             // rate of change of velocity in NED frame
    Vector3f velDotNEDfilt;         // low pass filtered velDotNED
    uint32_t lastAirspeedUpdate;    // last time airspeed was updated
    uint32_t IMUmsec;               // time that the last IMU value was taken
    ftype gpsCourse;                // GPS ground course angle(rad) 
    ftype gpsGndSpd;                // GPS ground speed (m/s)
    bool newDataGps;                // true when new GPS data has arrived
    bool newDataMag;                // true when new magnetometer data has arrived
    float gpsVarScaler;             // scaler applied to gps measurement variance to allow for oversampling
    bool newDataTas;                // true when new airspeed data has arrived
    bool tasDataWaiting;            // true when new airspeed data is waiting to be fused
    bool newDataHgt;                // true when new height data has arrived
    uint32_t lastHgtMeasTime;       // time of last height measurement used to determine if new data has arrived
    uint32_t lastHgtTime_ms;        // time of last height update (msec) used to calculate timeout
    float hgtVarScaler;             // scaler applied to height measurement variance to allow for oversampling
    uint32_t velFailTime;           // time stamp when GPS velocity measurement last failed covaraiance consistency check (msec)
    uint32_t posFailTime;           // time stamp when GPS position measurement last failed covaraiance consistency check (msec)
    uint32_t hgtFailTime;           // time stamp when height measurement last failed covaraiance consistency check (msec)
    uint8_t storeIndex;             // State vector storage index
    uint32_t lastFixTime_ms;        // time of last GPS fix used to determine if new data has arrived
    uint32_t secondLastFixTime_ms;  // time of second last GPS fix used to determine how long since last update
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
        ftype SH_MAG[9];
	} mag_state;


#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // performance counters
    perf_counter_t  _perf_UpdateFilter;
    perf_counter_t  _perf_CovariancePrediction;
    perf_counter_t  _perf_FuseVelPosNED;
    perf_counter_t  _perf_FuseMagnetometer;
    perf_counter_t  _perf_FuseAirspeed;
    perf_counter_t  _perf_FuseSideslip;
#endif
    
    // should we use the compass?
    bool use_compass(void) const;
};

#if CONFIG_HAL_BOARD != HAL_BOARD_PX4
#define perf_begin(x)
#define perf_end(x)
#endif

#endif // AP_NavEKF

