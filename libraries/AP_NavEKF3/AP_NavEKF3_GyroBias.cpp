#include "AP_NavEKF3_core.h"
#include <AP_DAL/AP_DAL.h>

#define P (const_cast<const Matrix24 &>(Pmut))

// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
// Assume that the calibration is performed to an accuracy of 0.5 deg/sec which will require averaging under static conditions
// WARNING - a non-blocking calibration method must be used
void NavEKF3_core::resetGyroBias(void)
{
    stateStruct.gyro_bias.zero();
    zeroStatesVarCov(10, 12);

    Pmut[10][10] = sq(radians(0.5f * dtIMUavg));
    Pmut[11][11] = P[10][10];
    Pmut[12][12] = P[10][10];
}

// zero the gyro bias states of the active and inactive IMUs. A bootstrap keeps the Z gyro
// biases it finds, so a reset the EKF makes itself starts from the Z bias already learned;
// a commanded reset calls this first, so it starts them all from zero as a full reset should
void NavEKF3_core::zeroGyroBiases(void)
{
    stateStruct.gyro_bias.zero();
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        inactiveBias[i].gyro_bias.zero();
    }
}

/*
   sensor specific initial gyro bias 1-sigma uncertainty in deg/sec
 */
ftype NavEKF3_core::InitialGyroBiasUncertainty(void) const
{
    return dal.ins().get_gyro_bias_init_dps(imu_index);
}

/*
   get the gyro bias state clamp (rad/s) for this core's IMU
 */
ftype NavEKF3_core::getGyroBiasLimit(void) const
{
    return dal.ins().get_gyro_bias_limit(imu_index);
}

