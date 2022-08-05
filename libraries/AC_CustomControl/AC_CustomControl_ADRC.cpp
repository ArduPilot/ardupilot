#include "AC_CustomControl_ADRC.h"

#if CUSTOMCONTROL_ADRC_ENABLED

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ADRC::var_info[] = {
    // @Param: RAT_RLL_WC
    // @DisplayName: ADRC roll axis control bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_RLL_WO
    // @DisplayName: ADRC roll axis ESO bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_RLL_B0
    // @DisplayName: ADRC roll axis control input gain
    // @User: Advanced

    // @Param: RAT_RLL_DELT
    // @DisplayName: ADRC roll axis control linear zone length
    // @User: Advanced

    // @Param: RAT_RLL_ORDR
    // @DisplayName: ADRC roll axis control model order
    // @User: Advanced

    // @Param: RAT_RLL_LM
    // @DisplayName: ADRC roll axis control output limit
    // @User: Advanced
    AP_SUBGROUPINFO(_rate_roll_cont, "RAT_RLL_", 1, AC_CustomControl_ADRC, AC_ADRC),

    // @Param: RAT_PIT_WC
    // @DisplayName: ADRC pitch axis control bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_PIT_WO
    // @DisplayName: ADRC pitch axis ESO bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_PIT_B0
    // @DisplayName: ADRC pitch axis control input gain
    // @User: Advanced

    // @Param: RAT_PIT_DELT
    // @DisplayName: ADRC pitch axis control linear zone length
    // @User: Advanced

    // @Param: RAT_PIT_ORDR
    // @DisplayName: ADRC pitch axis control model order
    // @User: Advanced

    // @Param: RAT_PIT_LM
    // @DisplayName: ADRC pitch axis control output limit
    // @User: Advanced
    AP_SUBGROUPINFO(_rate_pitch_cont, "RAT_PIT_", 2, AC_CustomControl_ADRC, AC_ADRC),

    // @Param: RAT_YAW_WC
    // @DisplayName: ADRC yaw axis control bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_YAW_WO
    // @DisplayName: ADRC yaw axis ESO bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_YAW_B0
    // @DisplayName: ADRC yaw axis control input gain
    // @User: Advanced

    // @Param: RAT_YAW_DELT
    // @DisplayName: ADRC yaw axis control linear zone length
    // @User: Advanced

    // @Param: RAT_YAW_ORDR
    // @DisplayName: ADRC yaw axis control model order
    // @User: Advanced

    // @Param: RAT_YAW_LM
    // @DisplayName: ADRC yaw axis control output limit
    // @User: Advanced
    AP_SUBGROUPINFO(_rate_yaw_cont, "RAT_YAW_", 3, AC_CustomControl_ADRC, AC_ADRC),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_ADRC::AC_CustomControl_ADRC(AC_CustomControl &frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _rate_roll_cont(100.0, dt),
    _rate_pitch_cont(100.0, dt),
    _rate_yaw_cont(10.0, dt)
{    
    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_ADRC::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }

    // Active disturbance rejection controller
    // only rate control is implemented
    Vector3f rate_target = _att_control->rate_bf_targets();
    Vector3f gyro_latest = _ahrs->get_gyro_latest();

    Vector3f motor_out;

    motor_out.x = _rate_roll_cont.update_all(rate_target.x, gyro_latest.x, _motors->limit.roll);
    motor_out.y = _rate_pitch_cont.update_all(rate_target.y, gyro_latest.y, _motors->limit.pitch);
    motor_out.z = _rate_yaw_cont.update_all(rate_target.z, gyro_latest.z, _motors->limit.yaw);

    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
// TODO: this doesn't result in bumpless transition. 
void AC_CustomControl_ADRC::reset(void)
{
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    _rate_roll_cont.reset_eso(gyro_latest.x);
    _rate_pitch_cont.reset_eso(gyro_latest.y);
    _rate_yaw_cont.reset_eso(gyro_latest.z);
}

#endif
