#include "AC_CustomControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#if CUSTOMCONTROL_ENABLED

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl::var_info[] = {
    // parameters from parent vehicle

    // @Param: CUST_CNT_ENB
    // @DisplayName: Custom Controller enabled
    // @Description: Custom Controller enabled parameter
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("CUST_CNT_ENB", 1, AC_CustomControl, _custom_controller_enabled, 0,  AP_PARAM_FLAG_ENABLE),

    // @Param: CUST_CNT_MSK
    // @DisplayName: Custom Controller bitmask
    // @Description: Custom Controller bitmask to chose which axis to run
    // @Bitmask: 0:Roll, 1:Pitch, 2:Yaw
    // @User: Advanced
    AP_GROUPINFO("CUST_CNT_MSK", 2, AC_CustomControl, _custom_controller_mask, 7),

    // custom controller related parameters
    AP_SUBGROUPINFO(_pid_atti_rate_roll, "RA2_RLL_", 3, AC_CustomControl, AC_PID),
    AP_SUBGROUPINFO(_pid_atti_rate_pitch, "RA2_PIT_", 4, AC_CustomControl, AC_PID),
    AP_SUBGROUPINFO(_pid_atti_rate_yaw, "RA2_YAW_", 5, AC_CustomControl, AC_PID),

    AP_GROUPEND
};

// define variable initial values at contstruction
AC_CustomControl::AC_CustomControl(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt):
AC_AttitudeControl_Multi(ahrs,aparm,motors,dt),
    _pid_atti_rate_roll(1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, dt),
    _pid_atti_rate_pitch(1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, dt),
    _pid_atti_rate_yaw(1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_CustomControl::rate_controller_run()
{
    if (_custom_controller_active == true && _custom_controller_enabled) {
        _last_active_us = AP_HAL::micros64();

        // display target and current attitude variable for easy access        
        // attitude target in quaternion
        // _attitude_target;

        // current attitude in quaternion
        // Quaternion attitude_body;
        // _ahrs.get_quat_body_to_ned(attitude_body);

        // run custom controller here
        ////////// start

        // example: single PID controller for attitude control
        switch (_motors.get_spool_state()) {
            case AP_Motors::SpoolState::SHUT_DOWN:
            case AP_Motors::SpoolState::GROUND_IDLE:
                // We are still at the ground. Reset custom controller to avoid
                // build up, ex: integrator
                reset_custom_controller();
                break;

            case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
            case AP_Motors::SpoolState::SPOOLING_UP:
            case AP_Motors::SpoolState::SPOOLING_DOWN:
                // we are off the ground
                break;
        }

        Vector3f rpy_out = run_custom_controller();

        motor_set(Vector3f(rpy_out[0], rpy_out[1], rpy_out[2]));
        ////////// end

        // reset main controller incase we switch out of custom one
        reset_main_controller();
    } else {
        // custom controller not enabled and switched on, run primary controller
        AC_AttitudeControl_Multi::rate_controller_run();

        // reset custom controller filter, integrator etc.
        reset_custom_controller();
    }
    
}

Vector3f AC_CustomControl::run_custom_controller(void) {
    float target_roll, target_pitch, target_yaw;
    _attitude_target.to_euler(target_roll, target_pitch, target_yaw);

    float roll_out, pitch_out, yaw_out;
    roll_out = _pid_atti_rate_roll.update_all(target_roll, _ahrs.roll, 1);
    pitch_out = _pid_atti_rate_pitch.update_all(target_pitch, _ahrs.pitch, 1);
    yaw_out = _pid_atti_rate_yaw.update_all(target_yaw, _ahrs.yaw, 1);

    return Vector3f(roll_out, pitch_out, yaw_out);
}


// choose which axis to apply custom controller output
void AC_CustomControl::motor_set(Vector3f rpy) {
    // run primary controller if all axises are not operational 
    if ( _custom_controller_mask != ((uint8_t)CustomControllerOption::ROLL 
        | (uint8_t)CustomControllerOption::PITCH | (uint8_t)CustomControllerOption::YAW) ) {
        AC_AttitudeControl_Multi::rate_controller_run();
    }

    if (_custom_controller_mask & (uint8_t)CustomControllerOption::ROLL) {
        _motors.set_roll(rpy[0]);
    }
    if (_custom_controller_mask & (uint8_t)CustomControllerOption::PITCH) {
        _motors.set_pitch(rpy[1]);
    }
    if (_custom_controller_mask & (uint8_t)CustomControllerOption::YAW) {
        _motors.set_yaw(rpy[2]);
    }
}


void AC_CustomControl::reset_custom_controller(void) {
    _pid_atti_rate_roll.reset_I();
    _pid_atti_rate_pitch.reset_I();
    _pid_atti_rate_yaw.reset_I();
    _pid_atti_rate_roll.reset_filter();
    _pid_atti_rate_pitch.reset_filter();
    _pid_atti_rate_yaw.reset_filter();
}

// reset unused main controller axis
void AC_CustomControl::reset_main_controller(void) 
{
    if (_custom_controller_mask & (uint8_t)CustomControllerOption::ROLL) {
        get_rate_roll_pid().reset_filter();
        get_rate_roll_pid().reset_I();
    }
    if (_custom_controller_mask & (uint8_t)CustomControllerOption::PITCH) {
        get_rate_pitch_pid().reset_filter();
        get_rate_pitch_pid().reset_I();
    }
    if (_custom_controller_mask & (uint8_t)CustomControllerOption::YAW) {
        get_rate_yaw_pid().reset_filter();
        get_rate_yaw_pid().reset_I();
    }
}



void AC_CustomControl::set_custom_controller(bool enabled)
{
    _custom_controller_active = enabled;
}

// called from fourhundred_hz_logging function in Copter
// left empty for simplicity
void AC_CustomControl::log_write(void) {

}
#endif