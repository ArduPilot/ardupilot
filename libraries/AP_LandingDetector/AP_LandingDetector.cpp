#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

#include "AP_LandingDetector.h"

#ifndef LDZI_ENABLE_DEFAULT
#define LDZI_ENABLE_DEFAULT 0
#endif

const AP_Param::GroupInfo AP_LandingDetector::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable accz i landing detector
    // @Description: Controls if accz I landing detector is enabled
    // @Values: 0:None,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_LandingDetector, _enable, LDZI_ENABLE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_LandingDetector *AP_LandingDetector::_singleton;
