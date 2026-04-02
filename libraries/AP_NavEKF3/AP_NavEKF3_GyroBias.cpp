#include "AP_NavEKF3_core.h"
#include <AP_DAL/AP_DAL.h>

// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
// Assume that the calibration is performed to an accuracy of 0.5 deg/sec which will require averaging under static conditions
// WARNING - a non-blocking calibration method must be used
void NavEKF3_core::resetGyroBias(void)
{
    stateStruct.gyro_bias.zero();
    zeroRows(P,10,12);
    zeroCols(P,10,12);

    P[10][10] = sq(radians(0.5f * dtIMUavg));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
}

/*
   vehicle specific initial gyro bias uncertainty in deg/sec
 */
ftype NavEKF3_core::InitialGyroBiasUncertainty(void) const
{
    if (dal.ins().is_low_drift(imu_index)) {
        return 1.0f;
    }
    return 2.5f;
}

/*
   get the gyro bias limit for this core's IMU
 */
ftype NavEKF3_core::getGyroBiasLimit(void) const
{
    if (dal.ins().is_low_drift(imu_index)) {
        return GYRO_BIAS_LIMIT_LOW_DRIFT;
    }
    return GYRO_BIAS_LIMIT;
}

