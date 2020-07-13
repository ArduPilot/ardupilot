#include "AP_Nav_Common.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

/*
  write an EKF timing message
 */

// @LoggerMessage: NKT,XKT
// @Description: EKF timing information
// @Field: TimeUS: Time since system startup
// @Field: C: EKF core this message instance applies to
// @Field: Cnt: count of samples used to create this message
// @Field: IMUMin: smallest IMU sample interval
// @Field: IMUMax: largest IMU sample interval
// @Field: EKFMin: low-passed achieved average time step rate for the EKF (minimum)
// @Field: EKFMax: low-passed achieved average time step rate for the EKF (maximum)
// @Field: AngMin: accumulated measurement time interval for the delta angle (minimum)
// @Field: AngMax: accumulated measurement time interval for the delta angle (maximum)
// @Field: VMin: accumulated measurement time interval for the delta velocity (minimum)
// @Field: VMax: accumulated measurement time interval for the delta velocity (maximum)
void Log_EKF_Timing(const char * name, const uint8_t core, uint64_t time_us, const struct ekf_timing &timing)
{
    AP::logger().Write(
        name,
        "TimeUS,C,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax",
        "s#sssssssss", // Units
        "F-000000000", // Mults
        "QBIffffffff", // Format
        time_us,
        core,
        timing.count,
        (double)timing.dtIMUavg_min,
        (double)timing.dtIMUavg_max,
        (double)timing.dtEKFavg_min,
        (double)timing.dtEKFavg_max,
        (double)timing.delAngDT_min,
        (double)timing.delAngDT_max,
        (double)timing.delVelDT_min,
        (double)timing.delVelDT_max);
}
