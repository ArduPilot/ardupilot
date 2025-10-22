#pragma once
#if AP_SCRIPTING_ENABLED

#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_Multi_6DoF : public AC_AttitudeControl_Multi {
public:
    AC_AttitudeControl_Multi_6DoF(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors):
        AC_AttitudeControl_Multi(ahrs,aparm,motors) {

        if (_singleton != nullptr) {
            AP_HAL::panic("Can only be one AC_AttitudeControl_Multi_6DoF");
        }
        _singleton = this;
    }

    static AC_AttitudeControl_Multi_6DoF *get_singleton() {
        return _singleton;
    }

    // Sets a desired attitude using a quaternion and body-frame angular velocity (rad/s).
    // The desired quaternion is incrementally updated each timestep. Angular velocity is shaped by acceleration limits and feedforward.
    void input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_body) override;
    /*
        override input functions to attitude controller and convert desired angles into thrust angles and substitute for offset angles
    */

    // Sets desired roll and pitch angles (in radians) and yaw rate (in radians/s).
    // Used when roll/pitch stabilization is needed with manual or autonomous yaw rate control.
    // Applies acceleration-limited input shaping for smooth transitions and computes body-frame angular velocity targets.
    void input_euler_angle_roll_pitch_euler_rate_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads)  override;


    // Sets desired roll, pitch, and yaw angles (in radians).
    // Used to follow an absolute attitude setpoint. Input shaping and yaw slew limits are applied.
    // Outputs are passed to the rate controller via shaped angular velocity targets.
    void input_euler_angle_roll_pitch_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_angle_rad, bool slew_yaw) override;

    // Sets desired thrust vector and heading rate (in radians/s).
    // Used for tilt-based navigation with independent yaw control.
    // The thrust vector defines the desired orientation (e.g., pointing direction for vertical thrust),
    // while the heading rate adjusts yaw. The input is shaped by acceleration and slew limits.
    void input_thrust_vector_rate_heading_rads(const Vector3f& thrust_vector, float heading_rate_rads, bool slew_yaw = true) override;
    
    // Sets desired thrust vector and heading (in radians) with heading rate (in radians/s).
    // Used for advanced attitude control where thrust direction is separated from yaw orientation.
    // Heading slew is constrained based on configured limits.
    void input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_angle_rad, float heading_rate_rads) override;

    /*
        all other input functions should zero thrust vectoring and behave as a normal copter
    */

    // Command euler yaw rate and pitch angle with roll angle specified in body frame
    // (used only by tailsitter quadplanes)
    void input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(bool plane_controls, float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads) override;

    // Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
    void input_euler_rate_roll_pitch_yaw_rads(float euler_roll_rate_rads, float euler_pitch_rate_rads, float euler_yaw_rate_rads) override;


    // Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
    // This command is used by fully stabilized acro modes.
    // It applies angular velocity targets in the body frame,
    // shaped using acceleration limits and passed to the rate controller.
    void input_rate_bf_roll_pitch_yaw_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) override;

    // Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
    // Used by Copter's rate-only acro mode.
    // Applies raw angular velocity targets directly to the rate controller with smoothing
    // and no attitude feedback or stabilization.
    void input_rate_bf_roll_pitch_yaw_2_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) override;


    // Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
    // Used by Plane's acro mode with rate error integration.
    // Integrates attitude error over time to generate target angular rates.
    void input_rate_bf_roll_pitch_yaw_3_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) override;

    // Applies a one-time angular offset in body-frame roll/pitch/yaw angles (in radians).
    // Used for initiating step responses during autotuning or manual test inputs.
    void input_angle_step_bf_roll_pitch_yaw_rad(float roll_angle_step_bf_rad, float pitch_angle_step_bf_rad, float yaw_angle_step_bf_rad) override;

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // limiting lean angle based on throttle makes no sense for 6DoF, always allow 90 deg, return in centi-degrees
    float get_althold_lean_angle_max_rad() const override { return radians(90.0f); }

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

    void set_forward_lateral_rad(float &euler_pitch_angle_rad, float &euler_roll_angle_rad);

    float roll_offset_deg;
    float pitch_offset_deg;

    bool forward_enable = true;
    bool lateral_enable = true;

    static AC_AttitudeControl_Multi_6DoF *_singleton;

};

#endif // AP_SCRIPTING_ENABLED
