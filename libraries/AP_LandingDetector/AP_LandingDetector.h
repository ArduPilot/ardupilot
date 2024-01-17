#pragma once

#include <AP_Param/AP_Param.h>

/*
  A landing detector based on the z acceleration controller integrator value
 */

class AP_LandingDetector
{
public:

    AP_LandingDetector() {
        AP_Param::setup_object_defaults(this, var_info);

        if (_singleton != nullptr) {
            AP_HAL::panic("AP_LandingDetector must be singleton");
        }
        _singleton = this;
    }

    CLASS_NO_COPY(AP_LandingDetector);

    static AP_LandingDetector *get_singleton(void) {
        return _singleton;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int8 _enable;

    static AP_LandingDetector *_singleton;
};
