#include <AP_HAL/AP_HAL.h>
#include "AC_PosControl_TS.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;
#include "AC_PosControl_TS.h"

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl_TS::AC_PosControl_TS(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                             const AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    AC_PosControl(ahrs, inav, motors, attitude_control) 
{
}

/// init_xy_controller - initialise the xy controller
///     this should be called after setting the position target and the desired velocity and acceleration
///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
///     should be called once whenever significant changes to the position target are made
///     this does not update the xy target.
///     If init_I_terms is false, do not assume that current lean angles represent the desired accelerations.
void AC_PosControl_TS::init_xy_controller(bool init_I_terms)
{
    // set roll, pitch lean angle targets to current attitude
    // todo: this should probably be based on the desired attitude not the current attitude
    _roll_target = _ahrs.roll_sensor;
    _pitch_target = _ahrs.pitch_sensor;

    // initialise I terms from lean angles
    _pid_vel_xy.reset_filter();
    if (init_I_terms) {
        lean_angles_to_accel(_accel_target.x, _accel_target.y);
        _pid_vel_xy.set_integrator(_accel_target - _accel_desired);
    }

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_accel_to_lean_xy = true;

    // initialise ekf xy reset handler
    init_ekf_xy_reset();
}
