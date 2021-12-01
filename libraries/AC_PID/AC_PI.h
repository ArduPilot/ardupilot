#pragma once

/*
 Generic PI for systems like heater control, no filtering
*/

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AC_PI {
public:
    // Constructor
    AC_PI(float initial_p, float initial_i, float initial_imax);

    CLASS_NO_COPY(AC_PI);

    // update controller
    float update(float measurement, float target, float dt);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    float get_P() const {
        return output_P;
    }
    float get_I() const {
        return integrator;
    }

protected:
    AP_Float        kP;
    AP_Float        kI;
    AP_Float        imax;
    float           integrator;
    float           output_P;
};
