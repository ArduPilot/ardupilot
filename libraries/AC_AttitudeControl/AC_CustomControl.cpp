#include "AC_CustomControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#if CUSTOMCONTROL_ENABLED

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl::var_info[] = {
    // parameters from parent vehicle
    // AP_NESTEDGROUPINFO(AC_AttitudeControl_Multi, 0),

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
    AP_SUBGROUPINFO(_p_angle_roll2, "ANG2_RLL_", 3, AC_CustomControl, AC_P),
    AP_SUBGROUPINFO(_p_angle_pitch2, "ANG2_PIT_", 4, AC_CustomControl, AC_P),
    AP_SUBGROUPINFO(_p_angle_yaw2, "ANG2_YAW_", 5, AC_CustomControl, AC_P),

    AP_SUBGROUPINFO(_pid_atti_rate_roll, "RA2_RLL_", 6, AC_CustomControl, AC_PID),
    AP_SUBGROUPINFO(_pid_atti_rate_pitch, "RA2_PIT_", 7, AC_CustomControl, AC_PID),
    AP_SUBGROUPINFO(_pid_atti_rate_yaw, "RA2_YAW_", 8, AC_CustomControl, AC_PID),

    AP_GROUPEND
};

// define variable initial values at contstruction
AC_CustomControl::AC_CustomControl(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt):
AC_AttitudeControl_Multi(ahrs,aparm,motors,dt),
    _p_angle_roll2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f),
    _p_angle_pitch2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f),
    _p_angle_yaw2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f),
    _pid_atti_rate_roll(AC_ATC_MULTI_RATE_RP_P * 0.90f, AC_ATC_MULTI_RATE_RP_I * 0.90f, AC_ATC_MULTI_RATE_RP_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.90f, AC_ATC_MULTI_RATE_RP_FILT_HZ * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ * 0.90f, dt),
    _pid_atti_rate_pitch(AC_ATC_MULTI_RATE_RP_P * 0.90f, AC_ATC_MULTI_RATE_RP_I * 0.90f, AC_ATC_MULTI_RATE_RP_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.90f, AC_ATC_MULTI_RATE_RP_FILT_HZ * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ * 0.90f, dt),
    _pid_atti_rate_yaw(AC_ATC_MULTI_RATE_YAW_P * 0.90f, AC_ATC_MULTI_RATE_YAW_I * 0.90f, AC_ATC_MULTI_RATE_YAW_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX * 0.90f, AC_ATC_MULTI_RATE_RP_FILT_HZ * 0.90f, AC_ATC_MULTI_RATE_YAW_FILT_HZ * 0.90f, 0.0f, dt)
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
    } else {
        // custom controller not enabled and switched on, run primary controller
        AC_AttitudeControl_Multi::rate_controller_run();
    }
    
}

Vector3f AC_CustomControl::run_custom_controller(void) {
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    thrust_heading_rotation_angles(_attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);

    // recalculate ang vel feedforward from attitude target model
    // rotation from the target frame to the body frame
    Quaternion rotation_target_to_body = attitude_body.inverse() * _attitude_target;
    // target angle velocity vector in the body frame
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _ang_vel_target;

    Vector3f target_rate;
    target_rate[0] = _p_angle_roll2.kP() * attitude_error.x + ang_vel_body_feedforward[0];
    target_rate[1] = _p_angle_pitch2.kP() * attitude_error.y + ang_vel_body_feedforward[1];
    target_rate[2] = _p_angle_yaw2.kP() * attitude_error.z + ang_vel_body_feedforward[2];

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    Vector3f motor_out;
    motor_out.x = _pid_atti_rate_roll.update_all(target_rate[0], gyro_latest[0], 1);
    motor_out.y = _pid_atti_rate_pitch.update_all(target_rate[1], gyro_latest[1], 1);
    motor_out.z = _pid_atti_rate_yaw.update_all(target_rate[2], gyro_latest[2], 1);

    return motor_out;
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

// reset I terms and all of the filter to allow smooth transion 
// to the primary controller
void AC_CustomControl::reset_custom_controller(void) {
    _pid_atti_rate_roll.reset_I();
    _pid_atti_rate_pitch.reset_I();
    _pid_atti_rate_yaw.reset_I();
    _pid_atti_rate_roll.reset_filter();
    _pid_atti_rate_pitch.reset_filter();
    _pid_atti_rate_yaw.reset_filter();
}


// reset rate controller filters and move integrator to motor output
// to allow smooth transion to the primary controller per axis
void AC_CustomControl::reset_main_controller(void) 
{
    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    if (_custom_controller_mask & (uint8_t)CustomControllerOption::ROLL) {
        get_rate_roll_pid().reset_filter();
        get_rate_roll_pid().set_integrator(_ang_vel_body.x - gyro_latest.x, _motors.get_roll());
    }
    if (_custom_controller_mask & (uint8_t)CustomControllerOption::PITCH) {
        get_rate_pitch_pid().reset_filter();
        get_rate_pitch_pid().set_integrator(_ang_vel_body.y - gyro_latest.y, _motors.get_pitch());
    }
    if (_custom_controller_mask & (uint8_t)CustomControllerOption::YAW) {
        get_rate_yaw_pid().reset_filter();
        get_rate_yaw_pid().set_integrator(_ang_vel_body.z - gyro_latest.z, _motors.get_yaw());
    }
}

void AC_CustomControl::set_custom_controller(bool enabled)
{
    if (enabled && _custom_controller_enabled) {
        // reset custom controller filter, integrator etc.
        reset_custom_controller();
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller is ON");
    } else {
        // reset main controller         
        reset_main_controller();
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller is OFF");
    }
    _custom_controller_active = enabled;
}

// called from fourhundred_hz_logging function in Copter
// left empty for simplicity
void AC_CustomControl::log_write(void) {

}
#endif