#include "AC_CustomControl_LQR.h"

#if CUSTOMCONTROL_LQR_ENABLED

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_LQR::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: LQR param1
    // @Description: Dumy parameter for LQR custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AC_CustomControl_LQR, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: LQR param2
    // @Description: Dumy parameter for LQR custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AC_CustomControl_LQR, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: LQR param3
    // @Description: Dumy parameter for LQR custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AC_CustomControl_LQR, param3, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_LQR::AC_CustomControl_LQR(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_LQR::update(void)
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

    // arducopter main attitude controller already runned
    // we don't need to do anything else

    //gcs().send_text(MAV_SEVERITY_INFO, "LQR custom controller working");

    Quaternion attitude_body, attitude_target;
   _ahrs->get_quat_body_to_ned(attitude_body);


    attitude_target = _att_control->get_attitude_target_quat();


    Quaternion attitude_error_quat = attitude_target.inverse() * attitude_body;


// axis-angle representation of quaternion is used. The resulting //state-space for attitude control is [v1 v2 v3 p q r]
    Vector3f attitude_error_angle;
    attitude_error_quat.to_axis_angle(attitude_error_angle);


    Vector3f gyro_latest = _ahrs->get_gyro_latest();


// k_gain LQR matrix is defined as [kij], where i:[T1 T2 T3](torques in  // three axis) and j:[v1 v2 v3 p q r](roll pitch yaw roll-rate //pitch-rate yaw-rate)
// k_gain = [k11 k12 k13 k14 k15 k16;
//       k21 k22 k23 k24 k25 k26;
//       k31 k32 k33 k34 k35 k36;]
// [T1 T2 T3] = -k_gain*[v1 v2 v3 p q r]
    _integralX += 0.03*attitude_error_angle.x*_dt;
    _integralY += 0.08*attitude_error_angle.y*_dt;
    _integralZ += 0.005*attitude_error_angle.z*_dt;
    constrain_float(_integralX, -_kimax_LQR, _kimax_LQR);
    constrain_float(_integralY, -_kimax_LQR, _kimax_LQR);
    constrain_float(_integralZ, -_kimax_LQR, _kimax_LQR);
    
    Vector3f motor_out;

    motor_out.x = -_integralX - 0.375*attitude_error_angle.x - 0.09*gyro_latest.x;

    motor_out.y = -_integralY - 0.375*attitude_error_angle.y - 0.125*gyro_latest.y;
   
    motor_out.z = -_integralZ - 0.40*attitude_error_angle.z - 0.09*gyro_latest.z;

   return motor_out;

}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_LQR::reset(void)
{
    _integralX = 0;
    _integralY = 0;
    _integralZ = 0;
}

#endif
