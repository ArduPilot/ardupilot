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

class NavEKF
{
public:

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

    void zeroRows(float covMat[24][24], uint8_t first, uint8_t last);

    void zeroCols(float covMat[24][24], uint8_t first, uint8_t last);

    void quatNorm(float quatOut[4], float quatIn[4]);

    // store states along with system time stamp in msces
    void StoreStates(void);

    // recall state vector stored at closest time to the one specified by msec
    void RecallStates(float statesForFusion[24], uint32_t msec);

    void quat2Tnb(Matrix3f &Tnb, float quat[4]);

    void quat2Tbn(Matrix3f &Tbn, float quat[4]);

    void calcEarthRateNED(Vector3f &omega, float latitude);

    void eul2quat(float quat[4], float eul[3]);

    void quat2eul(Vector3f &eul, float quat[4]);

    void calcvelNED(float velNED[3], float gpsCourse, float gpsGndSpd, float gpsVelD);

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

    float KH[24][24]; //  intermediate result used for covariance updates
    float KHP[24][24]; // intermediate result used for covariance updates
    float P[24][24]; // covariance matrix
    float states[24]; // state matrix - 4 x quaternions, 3 x Vel, 3 x Pos, 3 x gyro bias, 3 x accel bias, 2 x wind vel, 3 x earth mag field, 3 x body mag field
    float storedStates[24][50]; // state vectors stored for the last 50 time steps
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
    float innovVelPos[6]; // innovation output for a group of measurements
    float varInnovVelPos[6]; // innovation variance output for a group of measurements
    bool fuseVelData; // this boolean causes the velNED measurements to be fused
    bool fusePosData; // this boolean causes the posNE measurements to be fused
    bool fuseHgtData; // this boolean causes the hgtMea measurements to be fused
    float velNED[3]; // North, East, Down velocity measurements (m/s)
    float posNE[2]; // North, East position measurements (m)
    float hgtMea; //  height measurement relative to reference point  (m)
    float posNED[3]; // North, East Down position relative to reference point (m)
    float statesAtVelTime[24]; // States at the effective time of velNED measurements
    float statesAtPosTime[24]; // States at the effective time of posNE measurements
    float statesAtHgtTime[24]; // States at the effective time of hgtMea measurement
    float innovMag[3]; // innovation output from fusion of X,Y,Z compass measurements
    float varInnovMag[3]; // innovation variance output from fusion of X,Y,Z compass measurements
    bool fuseMagData; // boolean true when magnetometer data is to be fused
    Vector3f magData; // magnetometer flux readings in X,Y,Z body axes
    float statesAtMagMeasTime[24]; // filter states at the effective time of compass measurements
    float innovVtas; // innovation output from fusion of airspeed measurements
    float varInnovVtas; // innovation variance output from fusion of airspeed measurements
    bool fuseVtasData; // boolean true when airspeed data is to be fused
    float VtasMeas; // true airspeed measurement (m/s)
    float statesAtVtasMeasTime[24]; // filter states at the effective measurement time
    float latRef; // WGS-84 latitude of reference point (rad)
    float lonRef; // WGS-84 longitude of reference point (rad)
    float hgtRef; // WGS-84 height of reference point (m)
    Vector3f magBias; // magnetometer bias vector in XYZ body axes
    float eulerEst[3]; // Euler angles calculated from filter states
    float eulerDif[3]; // difference between Euler angle estimated by EKF and the AHRS solution
    const float covTimeStepMax; // maximum time allowed between covariance predictions
    const float covDelAngMax; // maximum delta angle between covariance predictions
    bool covPredStep; // boolean set to true when a covariance prediction step has been performed
    bool magFuseStep; // boolean set to true when magnetometer fusion is being performed
    bool posVelFuseStep; // boolean set to true when position and velocity fusion is being performed
    bool tasFuseStep; // boolean set to true when airspeed fusion is being performed
    uint32_t TASmsecPrev; // time stamp of last TAS fusion step
    const uint32_t TASmsecTgt; // target interval between TAS fusion steps
    uint32_t MAGmsecPrev; // time stamp of last compass fusion step
    const uint32_t MAGmsecTgt; // target interval between compass fusion steps
    uint32_t HGTmsecPrev; // time stamp of last height measurement fusion step
    const uint32_t HGTmsecTgt; // target interval between height measurement fusion steps
    
    // Estimated time delays (msec) for different measurements relative to IMU
    const uint32_t msecVelDelay;
    const uint32_t msecPosDelay;
    const uint32_t msecHgtDelay;
    const uint32_t msecMagDelay;
    const uint32_t msecTasDelay;

    // IMU input data variables
    float imuIn;
    float tempImu[8];
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
    float tempMag[8];
    float tempMagPrev[8];
    uint32_t MAGframe;
    uint32_t MAGtime;
    uint32_t lastMAGtime;
    bool newDataMag;
    Vector3f magDataPrev;

    // TAS input variables
    bool newDataTas;
    float VtasMeasPrev;

    // AHRS input data variables
    float ahrsEul[3];

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

};
#endif // AP_NavEKF

