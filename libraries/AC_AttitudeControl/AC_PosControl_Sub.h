#pragma once

#include "AC_PosControl.h"

class AC_PosControl_Sub : public AC_PosControl {
public:
    AC_PosControl_Sub(const AP_AHRS_View & ahrs, const AP_InertialNav& inav,
                      const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                      AC_P& p_pos_z, AC_P& p_vel_z, AC_PID& pid_accel_z,
                      AC_P& p_pos_xy, AC_PI_2D& pi_vel_xy);

    /// set_alt_max - sets maximum altitude above the ekf origin in cm
    ///   only enforced when set_alt_target_from_climb_rate is used
    ///   set to zero to disable limit
    void set_alt_max(float alt) { _alt_max = alt; }

    /// set_alt_min - sets the minimum altitude (maximum depth) in cm
    ///   only enforced when set_alt_target_from_climb_rate is used
    ///   set to zero to disable limit
    void set_alt_min(float alt) { _alt_min = alt; }

    /// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    ///     set force_descend to true during landing to allow target to move low enough to slow the motors
    void set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend) override;

    /// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    ///     set force_descend to true during landing to allow target to move low enough to slow the motors
    void set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend) override;

private:
    float       _alt_max; // max altitude - should be updated from the main code with altitude limit from fence
    float       _alt_min; // min altitude - should be updated from the main code with altitude limit from fence
};
