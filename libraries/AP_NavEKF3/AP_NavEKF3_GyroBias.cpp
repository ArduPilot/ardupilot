#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

extern const AP_HAL::HAL& hal;

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
float NavEKF3_core::InitialGyroBiasUncertainty(void) const
{
    return 2.5f;
}

