#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SOARING_ENABLED
#define HAL_SOARING_ENABLED 1
#endif

// Whether to publish named-value-float of the kalman filter thermal estimator.
// This is used with the mavproxy_soar plugin.
#ifndef HAL_SOARING_NVF_EKF_ENABLED
#define HAL_SOARING_NVF_EKF_ENABLED 0
#endif
