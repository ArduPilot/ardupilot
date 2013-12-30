/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  24 state EKF based on https://github.com/priseborough/InertialNav

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
#include <AP_AHRS.h>
#include <AP_InertialSensor.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Compass.h>

// #define MATH_CHECK_INDEXES 1

#include <vectorN.h>


class NavEKF
{
public:
#if MATH_CHECK_INDEXES
    typedef VectorN<float,2> Vector2;
    typedef VectorN<float,3> Vector3;
    typedef VectorN<float,6> Vector6;
    typedef VectorN<float,8> Vector8;
    typedef VectorN<float,11> Vector11;
    typedef VectorN<float,13> Vector13;
    typedef VectorN<float,21> Vector21;
    typedef VectorN<float,24> Vector24;
    typedef VectorN<VectorN<float,3>,3> Matrix3;
    typedef VectorN<VectorN<float,24>,24> Matrix24;
    typedef VectorN<VectorN<float,50>,24> Matrix24_50;
#else
    typedef float Vector2[2];
    typedef float Vector3[3];
    typedef float Vector6[6];
    typedef float Vector8[8];
    typedef float Vector11[11];
    typedef float Vector13[13];
    typedef float Vector21[21];
    typedef float Vector24[24];
    typedef float Matrix3[3][3];
    typedef float Matrix24[24][24];
    typedef float Matrix24_50[24][50];
#endif

    // Constructor 
    NavEKF(const AP_AHRS &ahrs, AP_Baro &baro);
 
    // Initialise the filter states from the AHRS and magnetometer data (if present)
    void InitialiseFilter(void);

    // Update Filter States - this should be called whenever new IMU data is available
    void UpdateFilter(void);

    // fill in latitude, longitude and height of the reference point
    void getRefLLH(struct Location &loc);

    // return the last calculated NED position relative to the
    // reference point (m). Return false if no position is available
    bool getPosNED(Vector3f &pos);

    // return the last calculated NED velocity (m/s)
    void getVelNED(Vector3f &vel);

    // return the last calculated latitude, longitude and height
    bool getLLH(struct Location &loc);

    // return the Euler roll, pitch and yaw angle in radians
    void getEulerAngles(Vector3f &eulers);

    // get the transformation matrix from NED to XYD (body) axes
    void getRotationNEDToBody(Matrix3f &mat);

    // get the transformation matrix from XYZ (body) to NED axes
    void getRotationBodyToNED(Matrix3f &mat);

    // get the quaternions defining the rotation from NED to XYZ (body) axes
    void getQuaternion(Quaternion &quat);

private:
    const AP_AHRS &_ahrs;
    AP_Baro &_baro;

    void UpdateStrapdownEquationsNED();

    void CovariancePrediction();
    
    void FuseVelPosNED();
    
    void FuseMagnetometer();
    
    void FuseAirspeed();

    void zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last);

    void zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last);

    void quatNorm(Quaternion &quatOut, const Quaternion &quatIn);

    // store states along with system time stamp in msces
    void StoreStates(void);

    // recall state vector stored at closest time to the one specified by msec
    void RecallStates(Vector24 &statesForFusion, uint32_t msec);

    void quat2Tnb(Matrix3f &Tnb, const Quaternion &quat);

    void quat2Tbn(Matrix3f &Tbn, const Quaternion &quat);

    void calcEarthRateNED(Vector3f &omega, float latitude);

    void eul2quat(Quaternion &quat, const Vector3f &eul);

    void quat2eul(Vector3f &eul, const Quaternion &quat);

    void calcvelNED(Vector3f &velNED, float gpsCourse, float gpsGndSpd, float gpsVelD);

    void calcposNE(float lat, float lon);

    void calcllh(float &lat, float &lon, float &hgt);

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

    Vector24 states; // state matrix - 4 x quaternions, 3 x Vel, 3 x Pos, 3 x gyro bias, 3 x accel bias, 2 x wind vel, 3 x earth mag field, 3 x body mag field

    Matrix24 KH; //  intermediate result used for covariance updates
    Matrix24 KHP; // intermediate result used for covariance updates
    Matrix24 P; // covariance matrix
    Matrix24_50 storedStates; // state vectors stored for the last 50 time steps
    uint32_t statetimeStamp[50]; // time stamp for each state vector stored
    Vector3f correctedDelAng; // delta angles about the xyz body axes corrected for errors (rad)
    Vector3f correctedDelVel; // delta velocities along the XYZ body axes corrected for errors (m/s)
    Vector3f summedDelAng; // corrected & summed delta angles about the xyz body axes (rad)
    Vector3f summedDelVel; // corrected & summed delta velocities along the XYZ body axes (m/s)
	Vector3f prevDelAng; // previous delta angle use for INS coning error compensation
    Matrix3f prevTnb; // previous nav to body transformation used for INS earth rotation compensation
    float accNavMag; // magnitude of navigation accel - used to adjust GPS obs variance (m/s^2)
    Vector3f earthRateNED; // earths angular rate vector in NED (rad/s)
    Vector3f dVelIMU; // delta velocity vector in XYZ body axes measured by the IMU (m/s)
    Vector3f dAngIMU; // delta angle vector in XYZ body axes measured by the IMU (rad)
    float dtIMU; // time lapsed since the last IMU measurement (sec)
    float dt; // time lapsed since the last covariance prediction (sec)
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
    float hgtMea; //  height measurement relative to reference point  (m)
    Vector24 statesAtVelTime; // States at the effective time of velNED measurements
    Vector24 statesAtPosTime; // States at the effective time of posNE measurements
    Vector24 statesAtHgtTime; // States at the effective time of hgtMea measurement
    Vector3f innovMag; // innovation output from fusion of X,Y,Z compass measurements
    Vector3f varInnovMag; // innovation variance output from fusion of X,Y,Z compass measurements
    bool fuseMagData; // boolean true when magnetometer data is to be fused
    Vector3f magData; // magnetometer flux readings in X,Y,Z body axes
    Vector24 statesAtMagMeasTime; // filter states at the effective time of compass measurements
    float innovVtas; // innovation output from fusion of airspeed measurements
    float varInnovVtas; // innovation variance output from fusion of airspeed measurements
    bool fuseVtasData; // boolean true when airspeed data is to be fused
    float VtasMeas; // true airspeed measurement (m/s)
    Vector24 statesAtVtasMeasTime; // filter states at the effective measurement time
    float latRef; // WGS-84 latitude of reference point (rad)
    float lonRef; // WGS-84 longitude of reference point (rad)
    float hgtRef; // WGS-84 height of reference point (m)
    Vector3f magBias; // magnetometer bias vector in XYZ body axes
    Vector3f eulerEst; // Euler angles calculated from filter states
    Vector3f eulerDif; // difference between Euler angle estimated by EKF and the AHRS solution
    const float covTimeStepMax; // maximum time allowed between covariance predictions
    const float covDelAngMax; // maximum delta angle between covariance predictions
    bool covPredStep; // boolean set to true when a covariance prediction step has been performed
    bool magFusePerformed; // boolean set to true when magnetometer fusion has been perfomred in that time step
    bool magFuseRequired; // boolean set to true when magnetometer fusion will be perfomred in the next time step
    bool posVelFuseStep; // boolean set to true when position and velocity fusion is being performed
    bool tasFuseStep; // boolean set to true when airspeed fusion is being performed
    uint32_t TASmsecPrev; // time stamp of last TAS fusion step
    const uint32_t TASmsecTgt; // target interval between TAS fusion steps
    const uint32_t TASmsecMax; // maximum allowed interval between TAS fusion steps
    uint32_t MAGmsecPrev; // time stamp of last compass fusion step
    const uint32_t MAGmsecTgt; // target interval between compass fusion steps
    const uint32_t MAGmsecMax; // maximum allowed interval between compass fusion steps
    uint32_t HGTmsecPrev; // time stamp of last height measurement fusion step
    const uint32_t HGTmsecMax; // maximum allowed interval between height measurement fusion steps

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
    float imuIn;
    Vector8 tempImu;
    uint32_t IMUmsec;

    // GPS input data variables
    float gpsCourse;
    float gpsGndSpd;
    float gpsLat;
    float gpsLon;
    float gpsHgt;
    bool newDataGps;

    // Magnetometer input data variables
    float magIn;
    Vector8 tempMag;
    Vector8 tempMagPrev;
    uint32_t MAGframe;
    uint32_t MAGtime;
    uint32_t lastMAGtime;
    bool newDataMag;

    // TAS input variables
    bool newDataTas;

    // AHRS input data variables
    Vector3f ahrsEul;

    // Time stamp when vel, pos or height measurements last failed checks
	uint32_t velFailTime;
	uint32_t posFailTime;
	uint32_t hgtFailTime;

    // states held by magnetomter fusion across time steps
    // magnetometer X,Y,Z measurements are fused across three time steps
    // to 
    struct {
    	float q0;
        float q1;
        float q2;
        float q3;
        float magN;
        float magE;
        float magD;
        float magXbias;
        float magYbias;
        float magZbias;
        uint8_t obsIndex;
        Matrix3f DCM;
        Vector3f MagPred;
        float R_MAG;
        float SH_MAG[9];
	} mag_state;

    // State vector storage index
	uint8_t storeIndex;

    // high precision time stamp for previous IMU data processing
	uint32_t lastIMUusec;

    // time of alst GPS fix used to determine if new data has arrived
	uint32_t lastFixTime;

    Vector3f lastAngRate;
    Vector3f lastAccel;

    // CovariancePrediction variables
    Matrix24 nextP;
    Vector24 processNoise;
    Vector21 SF;
    Vector8 SG;
    Vector11 SQ;
    Vector13 SPP;

};
#endif // AP_NavEKF

