#pragma once

/// @file	AC_PD.h
/// @brief	Generic P controller with EEPROM-backed storage of constants.
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include "AC_P_Basic.h"

/// @class	AC_P
/// @brief	Object managing one P controller
class AC_P : public AC_P_Basic {
public:

    AC_P(const float &initial_p, const AP_Float& accel_max_cds, const float sqrt_accel_min, const float sqrt_accel_max, const float dt);

    CLASS_NO_COPY(AC_P);

    float update(float error) const;

    void set_disable_sqrt_controller(bool b) { _disable_sqrt_control = b; }

private:

    float get_accel_max_radss() const { return radians(_accel_max_cds * 0.01); }

    float _dt;
    const AP_Float& _accel_max_cds;

    const float _sqrt_accel_min;
    const float _sqrt_accel_max;

    // Specifies whether the attitude controller should use the square root controller in the attitude correction.
    // This is used during Autotune to ensure the P term is tuned without being influenced by the acceleration limit of the square root controller.
    bool _disable_sqrt_control;

};
