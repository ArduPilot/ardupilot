#pragma once

#include "AC_PosControl.h"

#define POSCONTROL_JERK_RATIO                   1.0f    // Defines the time it takes to reach the requested acceleration

class AC_PosControl_Sub : public AC_PosControl {
public:
    AC_PosControl_Sub(AP_AHRS_View & ahrs, const AP_InertialNav& inav,
                      const AP_Motors& motors, AC_AttitudeControl& attitude_control, float dt);

    /// set_alt_max - sets maximum altitude above the ekf origin in cm
    ///   only enforced when set_pos_target_z_from_climb_rate_cm is used
    ///   set to zero to disable limit
    void set_alt_max(float alt) { _alt_max = alt; }

    /// set_alt_min - sets the minimum altitude (maximum depth) in cm
    ///   only enforced when set_pos_target_z_from_climb_rate_cm is used
    ///   set to zero to disable limit
    void set_alt_min(float alt) { _alt_min = alt; }

private:
    float       _alt_max; // max altitude - should be updated from the main code with altitude limit from fence
    float       _alt_min; // min altitude - should be updated from the main code with altitude limit from fence
};
