#pragma once
#if AP_SCRIPTING_ENABLED

#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_Multi_6DoF : public AC_AttitudeControl_Multi {
public:
    AC_AttitudeControl_Multi_6DoF(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt):
        AC_AttitudeControl_Multi(ahrs,aparm,motors,dt) {

        if (_singleton != nullptr) {
            AP_HAL::panic("Can only be one AC_AttitudeControl_Multi_6DoF");
        }
        _singleton = this;
    }

    static AC_AttitudeControl_Multi_6DoF *get_singleton() {
        return _singleton;
    }

    // Command a Quaternion attitude with feedforward and smoothing
    // attitude_desired_quat: is updated on each time_step (_dt) by the integral of the angular velocity
    // not used anywhere in current code, panic so this implementaiton is not overlooked
    void input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_target) override;
    /*
        override input functions to attitude controller and convert desired angles into thrust angles and substitute for osset angles
    */

    // Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
    void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)  override;

    // Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
    void input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw) override;

    // Command a thrust vector in the earth frame and a heading angle and/or rate
    void input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw = true) override;
    void input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds) override;

    /*
        all other input functions should zero thrust vectoring and behave as a normal copter
    */

    // Command euler yaw rate and pitch angle with roll angle specified in body frame
    // (used only by tailsitter quadplanes)
    void input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) override;

    // Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
    void input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds) override;

     // Command an angular velocity with angular velocity feedforward and smoothing
    void input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

    // Command an angular velocity with angular velocity feedforward and smoothing
    void input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

    // Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
    void input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

    // Command an angular step (i.e change) in body frame angle
    void input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd) override;

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // limiting lean angle based on throttle makes no sense for 6DoF, always allow 90 deg, return in centi-degrees
    float get_althold_lean_angle_max_cd() const override { return 9000.0f; }

    // set the attitude that will be used in 6DoF flight
    void set_offset_roll_pitch(float roll_deg, float pitch_deg) {
        roll_offset_deg = roll_deg;
        pitch_offset_deg = pitch_deg;
    }

    // these flags enable or disable lateral or forward thrust, with both disabled the vehicle acts like a traditional copter
    // it will roll and pitch to move, its also possible to enable only forward or lateral to suit the vehicle configuration.
    // by default the vehicle is full 6DoF, these can be set in flight via scripting
    void set_forward_enable(bool b) {
        forward_enable = b;
    }
    void set_lateral_enable(bool b) {
        lateral_enable = b;
    }

private:

    void set_forward_lateral(float &euler_pitch_angle_cd, float &euler_roll_angle_cd);

    float roll_offset_deg;
    float pitch_offset_deg;

    bool forward_enable = true;
    bool lateral_enable = true;

    static AC_AttitudeControl_Multi_6DoF *_singleton;

};

#endif // AP_SCRIPTING_ENABLED
