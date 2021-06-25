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

    /// input_vel_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by :
    ///         maximum velocity - vel_max,
    ///         maximum acceleration - accel_max,
    ///         time constant - tc.
    ///     The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
    ///     The time constant also defines the time taken to achieve the maximum acceleration.
    ///     The time constant must be positive.
    ///     The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
    void input_vel_accel_z(float &vel, float accel, bool force_descend) override;

private:
    float       _alt_max; // max altitude - should be updated from the main code with altitude limit from fence
    float       _alt_min; // min altitude - should be updated from the main code with altitude limit from fence
};
