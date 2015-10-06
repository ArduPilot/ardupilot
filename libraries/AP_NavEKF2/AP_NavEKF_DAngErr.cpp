/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

/*
  optionally turn down optimisation for debugging
 */
// #pragma GCC optimize("O0")

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;


// return the Euler roll, pitch and yaw angle in radians
void NavEKF2_core::getEulerAngles(Vector3f &euler) const
{
    outputDataNew.quat.to_euler(euler.x, euler.y, euler.z);
    euler = euler - _ahrs->get_trim();
}

// return body axis gyro bias estimates in rad/sec
void NavEKF2_core::getGyroBias(Vector3f &gyroBias) const
{
    if (dtIMUavg < 1e-6f) {
        gyroBias.zero();
        return;
    }
    gyroBias = stateStruct.gyro_bias / dtIMUavg;
}

// return body axis gyro scale factor error as a percentage
void NavEKF2_core::getGyroScaleErrorPercentage(Vector3f &gyroScale) const
{
    if (!statesInitialised) {
        gyroScale.x = gyroScale.y = gyroScale.z = 0;
        return;
    }
    gyroScale.x = 100.0f/stateStruct.gyro_scale.x - 100.0f;
    gyroScale.y = 100.0f/stateStruct.gyro_scale.y - 100.0f;
    gyroScale.z = 100.0f/stateStruct.gyro_scale.z - 100.0f;
}

// return tilt error convergence metric
void NavEKF2_core::getTiltError(float &ang) const
{
    ang = tiltErrFilt;
}

// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
void NavEKF2_core::resetGyroBias(void)
{
    stateStruct.gyro_bias.zero();
    zeroRows(P,9,11);
    zeroCols(P,9,11);
    P[9][9] = sq(radians(InitialGyroBiasUncertainty() * dtIMUavg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
}

/*
   vehicle specific initial gyro bias uncertainty in deg/sec
 */
float NavEKF2_core::InitialGyroBiasUncertainty(void) const
{
    return 5.0f;
}

bool NavEKF2_core::readDeltaAngle(uint8_t ins_index, Vector3f &dAng) {
    const AP_InertialSensor &ins = _ahrs->get_ins();

    if (ins_index < ins.get_gyro_count()) {
        ins.get_delta_angle(ins_index,dAng);
        return true;
    }
    return false;
}


#endif // HAL_CPU_CLASS