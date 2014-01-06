/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  21 state EKF based on https://github.com/priseborough/InertialNav

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
    typedef VectorN<ftype,21> Vector21;
    typedef VectorN<VectorN<ftype,3>,3> Matrix3;
    typedef VectorN<VectorN<ftype,21>,21> Matrix21;
    typedef VectorN<VectorN<ftype,50>,21> Matrix21_50;
#else
    typedef ftype Vector2[2];
    typedef ftype Vector3[3];
    typedef ftype Vector6[6];
    typedef ftype Vector8[8];
    typedef ftype Vector11[11];
    typedef ftype Vector13[13];
    typedef ftype Vector14[14];
    typedef ftype Vector21[21];
    typedef ftype Matrix3[3][3];
    typedef ftype Matrix21[21][21];
    typedef ftype Matrix21_50[21][50];
#endif

    // Constructor
    NavEKF(const AP_AHRS *ahrs, AP_Baro &baro);

    // Initialise the filter states from the AHRS and magnetometer data (if present)
    void InitialiseFilter(void);

    // Reset the position and height states
    void ResetPosition(void);

    // inhibits position and velocity aittude corrections when set to false
    void SetStaticMode(bool setting);

    // Update Filter States - this should be called whenever new IMU data is available
    void UpdateFilter(void);

    // return true if the filter is healthy
    bool healthy(void) const;

    // fill in latitude, longitude and height of the reference point
    void getRefLLH(struct Location &loc) const;

    // set latitude, longitude and height of the reference point
    void setRefLLH(int32_t lat, int32_t lng, int32_t alt_cm);

    // return the last calculated NED position relative to the
    // reference point (m). Return false if no position is available
    bool getPosNED(Vector3f &pos) const;

    // return NED velocity in m/s
    void getVelNED(Vector3f &vel) const;

    // return bodyaxis gyro bias estimates in deg/hr
    void getGyroBias(Vector3f &gyroBias) const;

    // return body axis accelerometer bias estimates in m/s^2
    void getAccelBias(Vector3f &accelBias) const;

    // return the NED wind speed estimates in m/s
    void getWind(Vector3f &wind) const;

    // return earth magnetic field estimates in measurement units
    void getMagNED(Vector3f &magNED) const;

    // return body magnetic field estimates in measurement units
    void getMagXYZ(Vector3f &magXYZ) const;

    // return the last calculated latitude, longitude and height
    bool getLLH(struct Location &loc) const;

    // return the Euler roll, pitch and yaw angle in radians
    void getEulerAngles(Vector3f &eulers) const;

    // get the transformation matrix from NED to XYD (body) axes
    void getRotationNEDToBody(Matrix3f &mat) const;

    // get the transformation matrix from XYZ (body) to NED axes
    void getRotationBodyToNED(Matrix3f &mat) const;

    // get the quaternions defining the rotation from NED to XYZ (body) axes
    void getQuaternion(Quaternion &quat) const;

private:
    const AP_AHRS *_ahrs;
    AP_Baro &_baro;

    void UpdateStrapdownEquationsNED();

    void CovariancePrediction();

    void ForceSymmetry();

    void ConstrainVariances();

    void FuseVelPosNED();

    void FuseMagnetometer();

    void FuseAirspeed();

    void zeroRows(Matrix21 &covMat, uint8_t first, uint8_t last);

    void zeroCols(Matrix21 &covMat, uint8_t first, uint8_t last);

    void quatNorm(Quaternion &quatOut, const Quaternion &quatIn) const;

    // store states along with system time stamp in msces
    void StoreStates(void);

    // recall state vector stored at closest time to the one specified by msec
    void RecallStates(Vector21 &statesForFusion, uint32_t msec);

    void quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const;

    void calcEarthRateNED(Vector3f &omega, int32_t latitude) const;

    void calcvelNED(Vector3f &velNED, float gpsCourse, float gpsGndSpd, float gpsVelD) const;

    void calcllh(float &lat, float &lon, float &hgt) const;

    void OnGroundCheck();

    void CovarianceInit();

    void readIMUData();

    void readGpsData();

    void readHgtData();

    void readMagData();

    void readAirSpdData();

    void SelectVelPosFusion();

    void SelectHgtFusion();

    void SelectTasFusion();

    void SelectMagFusion();

    bool statesInitialised;

public:
    // Tuning Parameters
    ftype _gpsHorizVelNoise;    // GPS horizontal velocity noise : m/s
    ftype _gpsVertVelNoise;    // GPS vertical velocity noise : m/s
    ftype _gpsHorizPosNoise;    // GPS horizontal position noise m
    ftype _gpsVertPosNoise;    // GPS or Baro vertical position variance : m^2
    ftype _gpsVelVarAccScale; // scale factor applied to velocity variance due to Vdot
    ftype _gpsPosVarAccScale; // scale factor applied to position variance due to Vdot
    ftype _magNoise; // magnetometer measurement noise : gauss
    ftype _magVarRateScale; // scale factor applied to magnetometer variance due to angular rate
    ftype _easNoise; // equivalent airspeed noise : m/s
    ftype _windStateNoise; // rate of change of wind : m/s^2
    ftype _wndVarHgtRateScale; // scale factor applied to wind process noise from height rate
    ftype _gyrNoise; // gyro process noise : rad/s
    ftype _accNoise; // accelerometer process noise : m/s^2
    ftype _dAngBiasNoise; // gyro bias state noise : rad/s^2
    ftype _magEarthNoise; // earth magnetic field process noise : gauss/sec
    ftype _magBodyNoise; // earth magnetic field process noise : gauss/sec
private:

    Vector21 states; // state matrix - 4 x quaternions, 3 x Vel, 3 x Pos, 3 x gyro bias, 3 x accel bias, 2 x wind vel, 3 x earth mag field, 3 x body mag field

    Matrix21 KH; //  intermediate result used for covariance updates
    Matrix21 KHP; // intermediate result used for covariance updates
    Matrix21 P; // covariance matrix
    Matrix21_50 storedStates; // state vectors stored for the last 50 time steps
    uint32_t statetimeStamp[50]; // time stamp for each state vector stored
    Vector3f correctedDelAng; // delta angles about the xyz body axes corrected for errors (rad)
    Vector3f correctedDelVel; // delta velocities along the XYZ body axes corrected for errors (m/s)
    Vector3f summedDelAng; // corrected & summed delta angles about the xyz body axes (rad)
    Vector3f summedDelVel; // corrected & summed delta velocities along the XYZ body axes (m/s)
	Vector3f prevDelAng; // previous delta angle use for INS coning error compensation
    Matrix3f prevTnb; // previous nav to body transformation used for INS earth rotation compensation
    ftype accNavMag; // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
    Vector3f earthRateNED; // earths angular rate vector in NED (rad/s)
    Vector3f dVelIMU; // delta velocity vector in XYZ body axes measured by the IMU (m/s)
    Vector3f dAngIMU; // delta angle vector in XYZ body axes measured by the IMU (rad)
    ftype dtIMU; // time lapsed since the last IMU measurement (sec)
    ftype dt; // time lapsed since the last covariance prediction (sec)
    ftype hgtRate; // state for rate of change of height filter
    bool onGround; // boolean true when the flight vehicle is on the ground (not flying)
    const bool useAirspeed; // boolean true if airspeed data is being used
    const bool useCompass; // boolean true if magnetometer data is being used
    const uint8_t fusionModeGPS; // 0 = GPS outputs 3D velocity, 1 = GPS outputs 2D velocity, 2 = GPS outputs no velocity
    Vector6 innovVelPos; // innovation output for a group of measurements
    Vector6 varInnovVelPos; // innovation variance output for a group of measurements
    bool fuseVelData; // this boolean causes the velNED measurements to be fused
    bool fusePosData; // this boolean causes the posNE measurements to be fused
    bool fuseHgtData; // this boolean causes the hgtMea measurements to be fused
    Vector3f velNED; // North, East, Down velocity measurements (m/s)
    Vector2 posNE; // North, East position measurements (m)
    ftype hgtMea; //  height measurement relative to reference point  (m)
    Vector21 statesAtVelTime; // States at the effective time of velNED measurements
    Vector21 statesAtPosTime; // States at the effective time of posNE measurements
    Vector21 statesAtHgtTime; // States at the effective time of hgtMea measurement
    Vector3f innovMag; // innovation output from fusion of X,Y,Z compass measurements
    Vector3f varInnovMag; // innovation variance output from fusion of X,Y,Z compass measurements
    bool fuseMagData; // boolean true when magnetometer data is to be fused
    Vector3f magData; // magnetometer flux readings in X,Y,Z body axes
    Vector21 statesAtMagMeasTime; // filter states at the effective time of compass measurements
    ftype innovVtas; // innovation output from fusion of airspeed measurements
    ftype varInnovVtas; // innovation variance output from fusion of airspeed measurements
    bool fuseVtasData; // boolean true when airspeed data is to be fused
    float VtasMeas; // true airspeed measurement (m/s)
    Vector21 statesAtVtasMeasTime; // filter states at the effective measurement time
    Vector3f magBias; // magnetometer bias vector in XYZ body axes
    const ftype covTimeStepMax; // maximum time allowed between covariance predictions
    const ftype covDelAngMax; // maximum delta angle between covariance predictions
    bool covPredStep; // boolean set to true when a covariance prediction step has been performed
    const ftype yawVarScale; // scale factor applied to yaw gyro errors when on ground
    bool magFusePerformed; // boolean set to true when magnetometer fusion has been perfomred in that time step
    bool magFuseRequired; // boolean set to true when magnetometer fusion will be perfomred in the next time step
    bool posVelFuseStep; // boolean set to true when position and velocity fusion is being performed
    bool tasFuseStep; // boolean set to true when airspeed fusion is being performed
    uint32_t TASmsecPrev; // time stamp of last TAS fusion step
    const uint32_t TASmsecMax; // maximum allowed interval between TAS fusion steps
    uint32_t MAGmsecPrev; // time stamp of last compass fusion step
    const uint32_t MAGmsecMax; // maximum allowed interval between compass fusion steps
    uint32_t HGTmsecPrev; // time stamp of last height measurement fusion step
    const uint32_t HGTmsecMax; // maximum allowed interval between height measurement fusion steps
    const bool fuseMeNow; // boolean to force fusion whenever data arrives
    bool staticMode; // boolean to force position and velocity measurements to zero for pre-arm or bench testing

    // last time compass was updated
    uint32_t lastMagUpdate;

    // last time airspeed was updated
    uint32_t lastAirspeedUpdate;

    // Estimated time delays (msec) for different measurements relative to IMU
    const uint32_t msecVelDelay;
    const uint32_t msecPosDelay;
    const uint32_t msecHgtDelay;
    const uint32_t msecMagDelay;
    const uint32_t msecTasDelay;

    // IMU input data variables
    uint32_t IMUmsec;

    // GPS input data variables
    ftype gpsCourse;
    ftype gpsGndSpd;
    bool newDataGps;

    // Magnetometer input data variables
    ftype magIn;
    bool newDataMag;

    // TAS input variables
    bool newDataTas;

    // HGT input variables
    bool newDataHgt;
    uint32_t lastHgtUpdate;

    // Time stamp when vel, pos or height measurements last failed checks
	uint32_t velFailTime;
	uint32_t posFailTime;
	uint32_t hgtFailTime;

    // states held by magnetomter fusion across time steps
    // magnetometer X,Y,Z measurements are fused across three time steps
    // to
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

    // State vector storage index
	uint8_t storeIndex;

    // time of last GPS fix used to determine if new data has arrived
	uint32_t lastFixTime_ms;

    Vector3f lastAngRate;
    Vector3f lastAccel;

    // CovariancePrediction variables
    Matrix21 nextP;
    Vector21 processNoise;
    Vector14 SF;
    Vector8 SG;
    Vector11 SQ;
    Vector8 SPP;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // performance counters
    perf_counter_t  _perf_UpdateFilter;
    perf_counter_t  _perf_CovariancePrediction;
    perf_counter_t  _perf_FuseVelPosNED;
    perf_counter_t  _perf_FuseMagnetometer;
    perf_counter_t  _perf_FuseAirspeed;
#endif
};

#if CONFIG_HAL_BOARD != HAL_BOARD_PX4
#define perf_begin(x)
#define perf_end(x)
#endif

#endif // AP_NavEKF

