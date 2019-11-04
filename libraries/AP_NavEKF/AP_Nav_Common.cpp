#include "AP_Nav_Common.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

/*
  write an EKF timing message
 */
void Log_EKF_Timing(const char *name, uint64_t time_us, const struct ekf_timing &timing)
{
    AP::logger().Write(
        name,
        "TimeUS,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax",
        "QIffffffff",
        time_us,
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

const char* EKF_Init_Failure::getFailureReason(const char* prefix) const {
    hal.util->snprintf(const_cast<char*>(prearm_fail_string), sizeof(prearm_fail_string), "%s: %s", prefix, initFailureReason[int(failure_reason)]);
    return prearm_fail_string;
};
