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
#endif




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
    // Don't know how to do this !!
    NavEKF(AP_AHRS* ahrs, AP_Baro* baro, GPS* gps) :
        _ahrs(ahrs),
        _baro(baro),
        _gps(gps)
 
    // Initialise the filter states from the AHRS and magnetometer data (if present)
    void InitialiseFilter();
    // Update Filter States - this should be called whenever new IMU data is available
    void UpdateFilter();
    // return the Lat (rad), long(rad) and height (m) of the reference point
    void getRefLLH();
    // return the last calculated NED position relative to the reference point (m)
    void getPosNED();
    // return the last calculated NED velocity (m/s)
    void getVelNED();
    // return the last calculated Lat (rad), long(rad) and height (m)
    void getLLH();
    // return the Euler roll, pitch and yaw angle in radians
    void getEulAng();
    // get the transformation matrix from NED to XYD (body) axes
    void getTnb();
    // get the transformation matrix from XYZ (body) to NED axes
    void getTbn();
    // get the quaternions defining the rotation from NED to XYZ (body) axes
    void getQuat();

private:

void UpdateStrapdownEquationsNED();

void CovariancePrediction();

void FuseVelPosNED();

void FuseMagnetometer();

void FuseAirspeed();

void zeroRows(float covMat[24][24], uint8_t first, uint8_t last);

void zeroCols(float covMat[24][24], uint8_t first, uint8_t last);

void quatNorm(float quatOut[4], float quatIn[4]);

// store states along with system time stamp in msces
void StoreStates(uint32_t msec);

// recall state vector stored at closest time to the one specified by msec
void RecallStates(float statesForFusion[24], uint32_t msec);

void quat2Tnb(Matrix3f &Tnb, float quat[4]);

void quat2Tbn(Matrix3f &Tbn, float quat[4]);

void calcEarthRateNED(Vector3f &omega, float latitude);

void eul2quat(float quat[4], float eul[3]);

void quat2eul(float eul[3],float quat[4]);

void calcvelNED(float velNED[3], float gpsCourse, float gpsGndSpd, float gpsVelD);

void calcposNE(float posNE[2], float lat, float lon, float latRef, float lonRef);

void calcllh(float posNED[3], float lat, float lon, float hgt, float latRef, float lonRef, float hgtRef);

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
	
#define deg2rad 0.017453292
#define rad2deg 57.295780
#define pi 3.141592657
#define earthRate 0.000072921
#define earthRadius 6378145.0
static float KH[24][24]; //  intermediate result used for covariance updates
static float KHP[24][24]; // intermediate result used for covariance updates
static float P[24][24]; // covariance matrix
static float Kfusion[24]; // Kalman gains
static float states[24]; // state matrix
static float storedStates[24][50]; // state vectors stored for the last 50 time steps
static uint32_t statetimeStamp[50]; // time stamp for each state vector stored
static Vector3f correctedDelAng; // delta angles about the xyz body axes corrected for errors (rad)
static Vector3f correctedDelVel; // delta velocities along the XYZ body axes corrected for errors (m/s)
static Vector3f summedDelAng; // summed delta angles about the xyz body axes corrected for errors (rad)
static Vector3f summedDelVel; // summed delta velocities along the XYZ body axes corrected for errors (m/s)
static float accNavMag; // magnitude of navigation accel (- used to adjust GPS obs variance (m/s^2)
static Vector3f earthRateNED; // earths angular rate vector in NED (rad/s)
static Vector3f dVelIMU; // delta velocity vector in XYZ body axes measured by the IMU (m/s)
static Vector3f dAngIMU; // delta angle vector in XYZ body axes measured by the IMU (rad)
static float dtIMU; // time lapsed since the last IMU measurement or covariance update (sec)
static float dt; // time lapsed since last covariance prediction
static bool onGround; // boolean true when the flight vehicle is on the ground (not flying)
const bool useAirspeed = true; // boolean true if airspeed data is being used
const bool useCompass = true; // boolean true if magnetometer data is being used
const uint8_t fusionModeGPS = 0; // 0 = GPS outputs 3D velocity, 1 = GPS outputs 2D velocity, 2 = GPS outputs no velocity
static float innovVelPos[6]; // innovation output
static float varInnovVelPos[6]; // innovation variance output
static bool fuseVelData; // this boolean causes the posNE and velNED obs to be fused
static bool fusePosData; // this boolean causes the posNE and velNED obs to be fused
static bool fuseHgtData; // this boolean causes the hgtMea obs to be fused
static float velNED[3]; // North, East, Down velocity obs (m/s)
static float posNE[2]; // North, East position obs (m)
static float hgtMea; //  measured height (m)
static float posNED[3]; // North, East Down position (m)
static float statesAtVelTime[24]; // States at the effective measurement time for posNE and velNED measurements
static float statesAtPosTime[24]; // States at the effective measurement time for posNE and velNED measurements
static float statesAtHgtTime[24]; // States at the effective measurement time for the hgtMea measurement
static float innovMag[3]; // innovation output
static float varInnovMag[3]; // innovation variance output
static bool fuseMagData; // boolean true when magnetometer data is to be fused
static Vector3f magData; // magnetometer flux radings in X,Y,Z body axes
static float statesAtMagMeasTime[24]; // filter satates at the effective measurement time
static float innovVtas; // innovation output
static float varInnovVtas; // innovation variance output
static bool fuseVtasData; // boolean true when airspeed data is to be fused
static float VtasMeas; // true airspeed measurement (m/s)
static float statesAtVtasMeasTime[24]; // filter states at the effective measurement time
static float latRef; // WGS-84 latitude of reference point (rad)
static float lonRef; // WGS-84 longitude of reference point (rad)
static float hgtRef; // WGS-84 height of reference point (m)
static Vector3f magBias; // states representing magnetometer bias vector in XYZ body axes
static float eulerEst[3]; // Euler angles calculated from filter states
static float eulerDif[3]; // difference between Euler angle estimated by EKF and the AHRS solution
const float covTimeStepMax = 0.07; // maximum time allowed between covariance predictions
const float covDelAngMax = 0.05; // maximum delta angle between covariance predictions
static bool covPredStep; // boolean set to true when a covariance prediction step has been performed
static bool magFuseStep; // boolean set to true when magnetometer fusion steps are being performed
static bool posVelFuseStep; // boolean set to true when position and velocity fusion is being performed
static bool tasFuseStep; // boolean set to true when airspeed fusion is being performed
static uint32_t TASmsecPrev; // time stamp of last TAS fusion step
const uint32_t TASmsecTgt = 250; // target interval between TAS fusion steps
static uint32_t MAGmsecPrev; // time stamp of last compass fusion step
const uint32_t MAGmsecTgt = 200; // target interval between compass fusion steps
static uint32_t HGTmsecPrev; // time stamp of last height measurement fusion step
const uint32_t HGTmsecTgt = 200; // target interval between height measurement fusion steps

// Estimated time delays (msec)
const uint32_t msecVelDelay = 200;
const uint32_t msecPosDelay = 200;
const uint32_t msecHgtDelay = 350;
const uint32_t msecMagDelay = 30;
const uint32_t msecTasDelay = 200;

// IMU input data variables
static float imuIn;
static float tempImu[8];
static uint32_t IMUmsec;

// GPS input data variables
static float gpsCourse;
static float gpsGndSpd;
static float gpsVelD;
static float gpsLat;
static float gpsLon;
static float gpsHgt;
static bool newDataGps;
static uint8_t GPSstatus;

// Magnetometer input data variables
static float magIn;
static float tempMag[8];
static float tempMagPrev[8];
static uint32_t MAGframe;
static uint32_t MAGtime;
static uint32_t lastMAGtime;
static bool newDataMag;

// AHRS input data variables
static float ahrsEul[3];

};
