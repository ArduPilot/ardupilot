#include "AP_AHRS_config.h"

#if AP_AHRS_NAVEKF3_ENABLED

#include "AP_AHRS_NavEKF3.h"

#include <AP_HAL/HAL.h>

extern const AP_HAL::HAL& hal;

bool AP_AHRS_NavEKF3::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (!_ekf3_started) {
        hal.util->snprintf(failure_msg, failure_msg_len, "EKF3 not started");
        return false;
    }
    return EKF3.pre_arm_check(requires_position, failure_msg, failure_msg_len);
}

#endif  // AP_AHRS_NAVEKF3_ENABLED
