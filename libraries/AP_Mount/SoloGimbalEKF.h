/*
  smaller EKF for simpler estimation applications

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

#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Param/AP_Param.h>
#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_AHRS/AP_AHRS.h>
//#include <AP_NavEKF2/AP_NavEKF2.h>

#include <AP_Math/vectorN.h>

class SoloGimbalEKF
{
public:
    typedef float ftype;
#if MATH_CHECK_INDEXES
    typedef VectorN<ftype,2> Vector2;
    typedef VectorN<ftype,3> Vector3;
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
    SoloGimbalEKF();

    // Run the EKF main loop once every time we receive sensor data
    void RunEKF(float delta_time, const Vector3f &delta_angles, const Vector3f &delta_velocity, const Vector3f &joint_angles);

    void reset();

    // get gyro bias data
    void getGyroBias(Vector3f &gyroBias) const;

    // set gyro bias
    void setGyroBias(const Vector3f &gyroBias);

    // get quaternion data
    void getQuat(Quaternion &quat) const;

    // get filter alignment status - true is aligned
    bool getStatus(void) const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    // the states are available in two forms, either as a Vector13 or
    // broken down as individual elements. Both are equivalent (same
    // memory)
    Vector13 states;
    struct state_elements {
        Vector3f    angErr;         // 0..2 rotation vector representing the growth in angle error since the last state correction (rad)
        Vector3f    velocity;       // 3..5 NED velocity (m/s)
        Vector3f    delAngBias;     // 6..8 estimated bias errors in the IMU delta angles
        Quaternion  quat;           // 9..12 these states are used by the INS prediction only and are not used by the EKF state equations.
    } &state;

    // data from sensors
    struct {
        Vector3f delAng;
        Vector3f delVel;
        float gPhi;
        float gPsi;
        float gTheta;
    } gSense;

    float Cov[9][9];                // covariance matrix
    Matrix3f Tsn;                   // Sensor to NED rotation matrix
    float TiltCorrectionSquared;    // Angle correction applied to tilt from last velocity fusion (rad)
    bool newDataMag;                // true when new magnetometer data is waiting to be used
    uint32_t StartTime_ms;          // time the EKF was started (msec)
    bool FiltInit;                  // true when EKF is initialised
    bool YawAligned;          // true when EKF heading is initialised
    float cosPhi;// = cosf(gSense.gPhi);
    float cosTheta;// = cosf(gSense.gTheta);
    float sinPhi;// = sinf(gSense.gPhi);
    float sinTheta;// = sinf(gSense.gTheta);
    float sinPsi;// = sinf(gSense.gPsi);
    float cosPsi;// = cosf(gSense.gPsi);
    uint32_t lastMagUpdate;
    Vector3f magData;

    uint32_t imuSampleTime_ms;
    float dtIMU;

    // States used for unwrapping of compass yaw error
    float innovationIncrement;
    float lastInnovation;

    // state prediction
    void predictStates();

    // EKF covariance prediction
    void predictCovariance();

    // EKF velocity fusion
    void fuseVelocity();
    
    // read magnetometer data
    void readMagData();

    // EKF compass fusion
    void fuseCompass();

    // Perform an initial heading alignment using the magnetic field and assumed declination
    void alignHeading();

    // Calculate magnetic heading innovation
    float calcMagHeadingInnov();

    // Force symmmetry and non-negative diagonals on state covarinace matrix
    void fixCovariance();
};
