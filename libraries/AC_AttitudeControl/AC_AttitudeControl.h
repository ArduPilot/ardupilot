#pragma once

/// @file    AC_AttitudeControl.h
/// @brief   ArduCopter attitude control library

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>
#include <AP_Vehicle/AP_MultiCopter.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#define AC_ATTITUDE_CONTROL_ANGLE_P                     4.5f             // default angle P gain for roll, pitch and yaw

#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS       radians(40.0f)   // minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS       radians(720.0f)  // maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS        radians(10.0f)   // minimum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS        radians(120.0f)  // maximum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS        6000      // constraint on yaw angle error in degrees.  This should lead to maximum turn rate of 10deg/sec * Stab Rate P so by default will be 45deg/sec.
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS   110000.0f // default maximum acceleration for roll/pitch axis in centidegrees/sec/sec
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS    27000.0f  // default maximum acceleration for yaw axis in centidegrees/sec/sec

#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          1.0f    // body-frame rate controller maximum output (for roll-pitch axis)
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         1.0f    // body-frame rate controller maximum output (for yaw axis)
#define AC_ATTITUDE_RATE_RELAX_TC                       0.16f   // This is used to decay the rate I term to 5% in half a second.

#define AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD              radians(30.0f) // Thrust angle error above which yaw corrections are limited
#define AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD             radians(45.0f) // Thrust angle error above which yaw corrections are limited

#define AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT          1       // body-frame rate feedforward enabled by default

#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT      1.0f    // Time constant used to limit lean angle so that vehicle does not lose altitude
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX    0.8f    // Max throttle used to limit lean angle so that vehicle does not lose altitude

#define AC_ATTITUDE_CONTROL_MIN_DEFAULT                 0.1f    // minimum throttle mix default
#define AC_ATTITUDE_CONTROL_MAN_DEFAULT                 0.1f    // manual throttle mix default
#define AC_ATTITUDE_CONTROL_MAX_DEFAULT                 0.5f    // maximum throttle mix default
#define AC_ATTITUDE_CONTROL_MIN_LIMIT                   0.5f    // min throttle mix upper limit
#define AC_ATTITUDE_CONTROL_MAN_LIMIT                   4.0f    // man throttle mix upper limit
#define AC_ATTITUDE_CONTROL_MAX                         5.0f    // maximum throttle mix default

#define AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT             0.5f  // ratio controlling the max throttle output during competing requests of low throttle from the pilot (or autopilot) and higher throttle for attitude control.  Higher favours Attitude over pilot input
#define AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH          1.0f  // default angle-p/pd throttle boost threshold

class AC_AttitudeControl {
public:
    AC_AttitudeControl( AP_AHRS_View &ahrs,
                        const AP_MultiCopter &aparm,
                        AP_Motors& motors) :
        _p_angle_roll(AC_ATTITUDE_CONTROL_ANGLE_P),
        _p_angle_pitch(AC_ATTITUDE_CONTROL_ANGLE_P),
        _p_angle_yaw(AC_ATTITUDE_CONTROL_ANGLE_P),
        _angle_boost(0),
        _use_sqrt_controller(true),
        _throttle_rpy_mix_desired(AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT),
        _throttle_rpy_mix(AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT),
        _ahrs(ahrs),
        _aparm(aparm),
        _motors(motors)
        {
            _singleton = this;
            AP_Param::setup_object_defaults(this, var_info);
        }

    static AC_AttitudeControl *get_singleton(void) {
        return _singleton;
    }

    // Empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl() {}

    // Sets the internal controller timestep (in seconds).
    // This is the elapsed time since the last controller update and is used in rate and smoothing calculations.
    void set_dt_s(float dt_s) { _dt_s = dt_s; }

    // Returns the internal controller timestep (in seconds).
    float get_dt_s() const { return _dt_s; }

    // pid accessors
    AC_P& get_angle_roll_p() { return _p_angle_roll; }
    AC_P& get_angle_pitch_p() { return _p_angle_pitch; }
    AC_P& get_angle_yaw_p() { return _p_angle_yaw; }
    virtual AC_PID& get_rate_roll_pid() = 0;
    virtual AC_PID& get_rate_pitch_pid() = 0;
    virtual AC_PID& get_rate_yaw_pid() = 0;
    virtual const AC_PID& get_rate_roll_pid() const = 0;
    virtual const AC_PID& get_rate_pitch_pid() const = 0;
    virtual const AC_PID& get_rate_yaw_pid() const = 0;

    // get the roll acceleration limit in radians/s/s
    float get_accel_roll_max_radss() const { return cd_to_rad(_accel_roll_max_cdss); }

    // Sets the roll acceleration limit in radians/s/s
    void set_accel_roll_max_radss(float accel_roll_max_radss) { _accel_roll_max_cdss.set(rad_to_cd(accel_roll_max_radss)); }

    // Sets and saves the roll acceleration limit in radians/s/s
    void save_accel_roll_max_radss(float accel_roll_max_radss) { _accel_roll_max_cdss.set_and_save(rad_to_cd(accel_roll_max_radss)); }

    // get the pitch acceleration limit in radians/s/s
    float get_accel_pitch_max_radss() const { return cd_to_rad(_accel_pitch_max_cdss); }

    // Sets the pitch acceleration limit in radians/s/s
    void set_accel_pitch_max_radss(float accel_pitch_max_radss) { _accel_pitch_max_cdss.set(rad_to_cd(accel_pitch_max_radss)); }

    // Sets and saves the pitch acceleration limit in radians/s/s
    void save_accel_pitch_max_radss(float accel_pitch_max_radss) { _accel_pitch_max_cdss.set_and_save(rad_to_cd(accel_pitch_max_radss)); }

    // get the yaw acceleration limit in radians/s/s
    float get_accel_yaw_max_radss() const { return cd_to_rad(_accel_yaw_max_cdss); }

    // Sets the yaw acceleration limit in radians/s/s
    void set_accel_yaw_max_radss(float accel_yaw_max_radss) { _accel_yaw_max_cdss.set(rad_to_cd(accel_yaw_max_radss)); }

    // Sets and saves the yaw acceleration limit in radians/s/s
    void save_accel_yaw_max_radss(float accel_yaw_max_radss) { _accel_yaw_max_cdss.set_and_save(rad_to_cd(accel_yaw_max_radss)); }

    // get the roll angular velocity limit in radians/s
    float get_ang_vel_roll_max_rads() const { return radians(_ang_vel_roll_max_degs); }

    // set the roll angular velocity limit in radians/s
    void set_ang_vel_roll_max_rads(float vel_roll_max_rads) { _ang_vel_roll_max_degs.set(degrees(vel_roll_max_rads)); }

    // get the pitch angular velocity limit in radians/s
    float get_ang_vel_pitch_max_rads() const { return radians(_ang_vel_pitch_max_degs); }

    // set the pitch angular velocity limit in radians/s
    void set_ang_vel_pitch_max_rads(float vel_pitch_max_rads) { _ang_vel_pitch_max_degs.set(degrees(vel_pitch_max_rads)); }

    // get the yaw angular velocity limit in radians/s
    float get_ang_vel_yaw_max_rads() const { return radians(_ang_vel_yaw_max_degs); }

    // set the yaw angular velocity limit in radians/s
    void set_ang_vel_yaw_max_rads(float vel_yaw_max_rads) { _ang_vel_yaw_max_degs.set(degrees(vel_yaw_max_rads)); }

    // get the slew yaw rate limit in radians/s
    float get_slew_yaw_max_rads() const;

    // get the rate control input smoothing time constant
    float get_input_tc() const { return _input_tc; }

    // set the rate control input smoothing time constant
    void set_input_tc(float input_tc) { _input_tc.set(constrain_float(input_tc, 0.0f, 1.0f)); }

    // rate loop visible functions
    // Ensure attitude controller have zero errors to relax rate controller output
    void relax_attitude_controllers();

    // Used by child class AC_AttitudeControl_TS to change behaviour for tailsitter quadplanes
    virtual void relax_attitude_controllers(bool exclude_pitch) { relax_attitude_controllers(); }

    // reset rate controller I terms
    void reset_rate_controller_I_terms();

    // Smoothly decays I-term to zero over ~0.5s. Useful for transitions to prevent overshoot.
    void reset_rate_controller_I_terms_smoothly();

    // Reduce attitude control gains while landed to stop ground resonance
    void landed_gain_reduction(bool landed);

    // Sets the attitude and angular rate targets to the current vehicle state.
    // If reset_rate is false, only the attitude target is reset and current rate targets are preserved.
    void reset_target_and_rate(bool reset_rate = true);

    // Sets the yaw angle target to the vehicle's current heading. Optionally resets yaw rate to zero.
    void reset_yaw_target_and_rate(bool reset_rate = true);

    // handle reset of attitude from EKF since the last iteration
    void inertial_frame_reset();

    // Commands a body-frame roll angle (in centidegrees), an euler pitch angle (in centidegrees), and a yaw rate (in centidegrees/s).
    // See input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad() for full details.
    // (implemented only in AC_AttitudeControl_TS for tailsitter quadplanes)
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll_cd(bool plane_controls, float euler_roll_angle_cd, 
        float euler_pitch_angle_cd, float euler_yaw_rate_cds) {}

    // Commands a body-frame roll angle (in radians), an euler pitch angle (in radians), and a yaw rate (in radians/s).
    // Used by tailsitter quadplanes. Optionally swaps roll and yaw effects as pitch nears 90° if plane_controls is true.
    // (implemented only in AC_AttitudeControl_TS for tailsitter quadplanes)
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(bool plane_controls, float euler_roll_angle_rad, 
        float euler_pitch_angle_rad, float euler_yaw_rate_rads) {}

    ////// begin rate update functions //////
    // These functions all update _ang_vel_body_rads which is used as the rate target by the rate controller.
    // Since _ang_vel_body_rads can be seen by the rate controller thread all these functions only set it
    // at the end once all of the calculations have been performed. This avoids intermediate results being
    // used by the rate controller when running concurrently. _ang_vel_body_rads is accessed so commonly that
    // locking proves to be moderately expensive, however since this is changing incrementally values combining 
    // previous and current elements are safe and do not have an impact on control.
    // Any additional functions that are added to manipulate _ang_vel_body_rads should follow this pattern.

    // Calculates the body frame angular velocities to follow the target attitude
    // This is used by most of the subsequent functions
    void attitude_controller_run_quat();

    // Sets a desired attitude using a quaternion and body-frame angular velocity (rad/s).
    // The desired quaternion is incrementally updated each timestep. Angular velocity is shaped by acceleration limits and feedforward.
    virtual void input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_body_rads);

    // Sets desired roll and pitch angles (in centidegrees) and yaw rate (in centidegrees/s).
    // See input_euler_angle_roll_pitch_euler_rate_yaw_rad() for full details.
    void input_euler_angle_roll_pitch_euler_rate_yaw_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds);

    // Sets desired roll and pitch angles (in radians) and yaw rate (in radians/s).
    // Used when roll/pitch stabilization is needed with manual or autonomous yaw rate control.
    // Applies acceleration-limited input shaping for smooth transitions and computes body-frame angular velocity targets.
    virtual void input_euler_angle_roll_pitch_euler_rate_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads);

    // Sets desired roll, pitch, and yaw angles (in centidegrees).
    // See input_euler_angle_roll_pitch_yaw_rad() for full details.
    void input_euler_angle_roll_pitch_yaw_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw);

    // Sets desired roll, pitch, and yaw angles (in radians).
    // Used to follow an absolute attitude setpoint. Input shaping and yaw slew limits are applied.
    // Outputs are passed to the rate controller via shaped angular velocity targets.
    virtual void input_euler_angle_roll_pitch_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_angle_rad, bool slew_yaw);

    // Sets desired roll, pitch, and yaw angular rates (in radians/s).
    // This command is used to apply angular rate targets in the earth frame.
    // The inputs are shaped using acceleration limits and time constants.
    // Resulting targets are converted into body-frame angular velocities
    // and passed to the rate controller.
    virtual void input_euler_rate_roll_pitch_yaw_rads(float euler_roll_rate_rads, float euler_pitch_rate_rads, float euler_yaw_rate_rads);

    // Sets desired roll, pitch, and yaw angular rates (in centidegrees/s).
    // See input_rate_bf_roll_pitch_yaw_rads() for full details.
    void input_rate_bf_roll_pitch_yaw_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
    // This command is used by fully stabilized acro modes.
    // It applies angular velocity targets in the body frame,
    // shaped using acceleration limits and passed to the rate controller.
    virtual void input_rate_bf_roll_pitch_yaw_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads);

    // Sets desired roll, pitch, and yaw angular rates in body-frame (in centidegrees/s).
    // See input_rate_bf_roll_pitch_yaw_2_rads() for full details.
    void input_rate_bf_roll_pitch_yaw_2_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
    // Used by Copter's rate-only acro mode.
    // Applies raw angular velocity targets directly to the rate controller with smoothing
    // and no attitude feedback or stabilization.
    virtual void input_rate_bf_roll_pitch_yaw_2_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads);

    // Sets desired roll, pitch, and yaw angular rates in body-frame (in centidegrees/s).
    // See input_rate_bf_roll_pitch_yaw_3_rads() for full details.
    void input_rate_bf_roll_pitch_yaw_3_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // Sets desired roll, pitch, and yaw angular rates in body-frame (in radians/s).
    // Used by Plane's acro mode with rate error integration.
    // Integrates attitude error over time to generate target angular rates.
    virtual void input_rate_bf_roll_pitch_yaw_3_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads);

    // Directly sets the body-frame angular rates without smoothing (in centidegrees/s).
    // See input_rate_bf_roll_pitch_yaw_no_shaping_rads() for full details.
    void input_rate_bf_roll_pitch_yaw_no_shaping_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // Directly sets the body-frame angular rates without smoothing (in radians/s).
    // This command is used when external control logic (e.g. fixed-wing controller)
    // dictates VTOL rates. No smoothing or shaping is applied.
    void input_rate_bf_roll_pitch_yaw_no_shaping_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads);

    // Applies a one-time angular offset in body-frame roll/pitch/yaw angles (in radians).
    // Used for initiating step responses during autotuning or manual test inputs.
    virtual void input_angle_step_bf_roll_pitch_yaw_rad(float roll_angle_step_bf_rad, float pitch_angle_step_bf_rad, float yaw_angle_step_bf_rad);
    
    // Applies a one-time angular velocity offset in body-frame roll/pitch/yaw (in radians/s).
    // Used to apply discrete disturbances or step inputs for system identification.
    virtual void input_rate_step_bf_roll_pitch_yaw_rads(float roll_rate_step_bf_rads, float pitch_rate_step_bf_rads, float yaw_rate_step_bf_rads);

    // Sets desired thrust vector and heading rate (in radians/s).
    // Used for tilt-based navigation with independent yaw control.
    // The thrust vector defines the desired orientation (e.g., pointing direction for vertical thrust),
    // while the heading rate adjusts yaw. The input is shaped by acceleration and slew limits.
    virtual void input_thrust_vector_rate_heading_rads(const Vector3f& thrust_vector, float heading_rate_rads, bool slew_yaw = true);

    // Sets desired thrust vector and heading (in radians) with heading rate (in radians/s).
    // Used for advanced attitude control where thrust direction is separated from yaw orientation.
    // Heading slew is constrained based on configured limits.
    virtual void input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_angle_rad, float heading_rate_rads);

    // Sets desired thrust vector and heading (in centidegrees), with zero heading rate.
    // See input_thrust_vector_heading_rad() for full details.
    void input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_rad) {input_thrust_vector_heading_rad(thrust_vector, heading_rad, 0.0f);}

    ////// end rate update functions //////

    // Converts a thrust vector and heading angle to a body-to-NED quaternion attitude
    Quaternion attitude_from_thrust_vector(Vector3f thrust_vector, float heading_angle_rad) const;

    // Run angular velocity controller and send outputs to the motors
    virtual void rate_controller_run() = 0;

    // Resets any internal state maintained by the rate controller (e.g., smoothing filters or integrators).
    // This base function is a no-op and may be overridden by child classes.
    virtual void rate_controller_target_reset() {}

    // Run the angular velocity controller with a specified timestep. Must be implemented by derived class.
    virtual void rate_controller_run_dt(const Vector3f& gyro_rads, float dt) { AP_BoardConfig::config_error("rate_controller_run_dt() must be defined"); };

    // Convert a 321-intrinsic euler angle derivative to an angular velocity vector
    void euler_rate_to_ang_vel(const Quaternion& att, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads);

    // Convert an angular velocity vector to a 321-intrinsic euler angle derivative
    // Returns false if the vehicle is pitched 90 degrees up or down
    bool ang_vel_to_euler_rate(const Quaternion& att, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads);

    // Specifies whether the attitude controller should use the square root controller in the attitude correction.
    // This is used during Autotune to ensure the P term is tuned without being influenced by the acceleration limit of the square root controller.
    void use_sqrt_controller(bool use_sqrt_cont) { _use_sqrt_controller = use_sqrt_cont; }

    // Return 321-intrinsic euler angles in centidegrees representing the rotation from NED earth frame to the
    // attitude controller's target attitude.
    // **NOTE** Using vector3f*deg(100) is more efficient than deg(vector3f)*100 or deg(vector3d*100) because it gives the
    // same result with the fewest multiplications. Even though it may look like a bug, it is intentional. See issue 4895.
    Vector3f get_att_target_euler_cd() const { return _euler_angle_target_rad * degrees(100.0f); }
    const Vector3f & get_att_target_euler_rad() const { return _euler_angle_target_rad; }

    // Returns the target attitude quaternion used by the attitude controller.
    // This represents the desired orientation in NED earth frame.
    Quaternion get_attitude_target_quat() const { return _attitude_target; }

    // Return the angular velocity of the target (setpoint) [rad/s] in the target attitude frame
    const Vector3f& get_attitude_target_ang_vel() const { return _ang_vel_target_rads;}

    // Return the angle between the target thrust vector and the current thrust vector.
    float get_att_error_angle_deg() const { return degrees(_thrust_error_angle_rad); }

    // Returns the current attitude target as 321 Euler angles in centidegrees.
    // Note: Centidegrees are used for legacy compatibility with older messaging formats.
    void rate_bf_yaw_target(float rate_cds) { _ang_vel_body_rads.z = cd_to_rad(rate_cds); }

    // Set x-axis system identification angular velocity in radians/s
    void rate_bf_roll_sysid_rads(float rate_rads) { _sysid_ang_vel_body_rads.x = rate_rads; }

    // Set y-axis system identification angular velocity in radians/s
    void rate_bf_pitch_sysid_rads(float rate_rads) { _sysid_ang_vel_body_rads.y = rate_rads; }

    // Set z-axis system identification angular velocity in radians/s
    void rate_bf_yaw_sysid_rads(float rate_rads) { _sysid_ang_vel_body_rads.z = rate_rads; }

    // Set x-axis system identification actuator
    void actuator_roll_sysid(float command) { _actuator_sysid.x = command; }

    // Set y-axis system identification actuator
    void actuator_pitch_sysid(float command) { _actuator_sysid.y = command; }

    // Set z-axis system identification actuator
    void actuator_yaw_sysid(float command) { _actuator_sysid.z = command; }

    // Return roll rate step size in radians/s that results in maximum output after 4 time steps
    float max_rate_step_bf_roll();

    // Return pitch rate step size in radians/s that results in maximum output after 4 time steps
    float max_rate_step_bf_pitch();

    // Return yaw rate step size in radians/s that results in maximum output after 4 time steps
    float max_rate_step_bf_yaw();

    // Return roll step size in radians that results in maximum output after 4 time steps
    float max_angle_step_bf_roll() { return max_rate_step_bf_roll() / _p_angle_roll.kP(); }

    // Return pitch step size in radians that results in maximum output after 4 time steps
    float max_angle_step_bf_pitch() { return max_rate_step_bf_pitch() / _p_angle_pitch.kP(); }

    // Return yaw step size in radians that results in maximum output after 4 time steps
    float max_angle_step_bf_yaw() { return max_rate_step_bf_yaw() / _p_angle_yaw.kP(); }

    // Return the body-frame angular velocity (in rad/s) used by the angular velocity controller.
    Vector3f rate_bf_targets() const { return _ang_vel_body_rads + _sysid_ang_vel_body_rads; }

    // return the angular velocity of the target (setpoint) attitude rad/s
    const Vector3f& get_rate_ef_targets() const { return _euler_rate_target_rads; }

    // Enable or disable body-frame feed forward
    void bf_feedforward(bool enable_or_disable) { _rate_bf_ff_enabled.set(enable_or_disable); }

    // Enable or disable body-frame feed forward and save
    void bf_feedforward_save(bool enable_or_disable) { _rate_bf_ff_enabled.set_and_save(enable_or_disable); }

    // Return body-frame feed forward setting
    bool get_bf_feedforward() { return _rate_bf_ff_enabled; }

    // Enable or disable body-frame feed forward
    void accel_limiting(bool enable_or_disable);

    // Update Alt_Hold angle maximum
    virtual void update_althold_lean_angle_max(float throttle_in) = 0;

    // Set output throttle
    virtual void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) = 0;

    // get throttle passed into attitude controller (i.e. throttle_in provided to set_throttle_out)
    float get_throttle_in() const { return _throttle_in; }

    // Return throttle increase applied for tilt compensation
    float angle_boost() const { return _angle_boost; }

    // Returns maximum allowable tilt angle (in radians) for pilot input when in altitude hold mode.
    // Used to limit lean angle based on available thrust margin, prioritising altitude stability.
    virtual float get_althold_lean_angle_max_rad() const;

    // Returns maximum allowable tilt angle (in centidegrees) for pilot input when in altitude hold mode.
    // See get_althold_lean_angle_max_rad() for full details.
    float get_althold_lean_angle_max_cd() const;

    // Return configured tilt angle limit in centidegrees
    float lean_angle_max_cd() const { return _aparm.angle_max; }

    // Return configured tilt angle limit in radians
    float lean_angle_max_rad() const { return cd_to_rad(_aparm.angle_max); }

    // Return tilt angle in degrees
    float lean_angle_deg() const { return degrees(_thrust_angle_rad); }
    float lean_angle_rad() const { return _thrust_angle_rad; }

    // Calculates the velocity correction from an angle error, applying acceleration/deceleration limits and a simple jerk-limiting mechanism via the smoothing gain.
    static float input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float desired_ang_vel, float max_ang_vel, float dt);
    static float input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float dt){ return input_shaping_angle(error_angle,  input_tc,  accel_max,  target_ang_vel,  0.0f,  0.0f,  dt); }

    // Shapes the velocity request based on a rate time constant. The angular acceleration and deceleration is limited.
    static float input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max, float dt, float input_tc);

    // calculates the expected angular velocity correction from an angle error based on the AC_AttitudeControl settings.
    // This function can be used to predict the delay associated with angle requests.
    void input_shaping_rate_predictor(const Vector2f &error_angle_rad, Vector2f& target_ang_vel_rads, float dt) const;

    // translates body frame acceleration limits to the euler axis
    void ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max_rads, float ang_vel_pitch_max_rads, float ang_vel_yaw_max_rads) const;

    // translates body frame acceleration limits to the euler axis
    Vector3f euler_accel_limit(const Quaternion &att, const Vector3f &euler_accel);

    // Calculates the body frame angular velocities to follow the target attitude
    void update_attitude_target();

    // thrust_heading_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
    // The maximum error in the yaw axis is limited based on the angle yaw P value and acceleration.
    void thrust_heading_rotation_angles(Quaternion& attitude_target, const Quaternion& attitude_body, Vector3f& attitude_error_rad, float& thrust_angle_rad, float& thrust_error_angle_rad) const;

    // thrust_vector_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
    // The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
    void thrust_vector_rotation_angles(const Quaternion& attitude_target, const Quaternion& attitude_body, Quaternion& thrust_vector_correction, Vector3f& attitude_error_rad, float& thrust_angle_rad, float& thrust_error_angle_rad) const;

    // sanity check parameters.  should be called once before take-off
    virtual void parameter_sanity_check() {}

    // Optional override: sets the notch filter sample rate for rate PID controllers (if supported).
    // Default implementation is empty.
    virtual void set_notch_sample_rate(float sample_rate) {}

    // Returns true if throttle RPY mix is at its minimum value.
    // Used to determine if attitude control should be reduced in favor of vertical thrust.
    virtual bool is_throttle_mix_min() const { return true; }

    // control rpy throttle mix
    virtual void set_throttle_mix_min() {}
    virtual void set_throttle_mix_man() {}
    virtual void set_throttle_mix_max(float ratio) {}
    virtual void set_throttle_mix_value(float value) {}
    virtual float get_throttle_mix(void) const { return 0; }

    // enable use of flybar passthrough on heli
    virtual void use_flybar_passthrough(bool passthrough, bool tail_passthrough) {}

	// use_leaky_i - controls whether we use leaky i term for body-frame to motor output stage on heli
	virtual void use_leaky_i(bool leaky_i) {}

    // set_hover_roll_scalar - scales Hover Roll Trim parameter. To be used by vehicle code according to vehicle condition.
    virtual void set_hover_roll_trim_scalar(float scalar) {}

    // Return angle in centidegrees to be added to roll angle for hover collective learn. Used by heli to counteract
    // tail rotor thrust in hover. Overloaded by AC_Attitude_Heli to return angle.
    virtual float get_roll_trim_cd() { return 0;}

    // Return angle in radians to be added to roll angle. Used by heli to counteract
    // tail rotor thrust in hover. Overloaded by AC_Attitude_Heli to return angle.
    float get_roll_trim_rad() { return cd_to_rad(get_roll_trim_cd()); }

    // passthrough_bf_roll_pitch_rate_yaw_norm - roll, pitch and yaw passed through directly to the motor mixers
    virtual void passthrough_bf_roll_pitch_rate_yaw_norm(float roll_passthrough_norm, float pitch_passthrough_norm, float yaw_passthrough_norm) {};

    // provide feedback on whether arming would be a good idea right now:
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    // enable inverted flight on backends that support it
    virtual void set_inverted_flight(bool inverted) {}

    // enable accessor for inverted flight flag on backends that support it
    virtual bool get_inverted_flight() { return false;}

    // get the slew rate value for roll, pitch and yaw, for oscillation detection in lua scripts
    void get_rpy_srate(float &roll_srate, float &pitch_srate, float &yaw_srate);
    
    // Sets the roll and pitch rate shaping time constant
    void set_roll_pitch_rate_tc(float input_tc) { _rate_rp_tc = input_tc; }

    // Sets the yaw rate shaping time constant
    void set_yaw_rate_tc(float input_tc) { _rate_y_tc = input_tc; }

    // setup a one loop angle P scale multiplier. This replaces any previous scale applied
    // so should only be used when only one source of scaling is needed
    void set_angle_P_scale(const Vector3f &angle_P_scale) { _angle_P_scale = angle_P_scale; }

    // setup a one loop angle P scale multiplier, multiplying by any
    // previously applied scale from this loop. This allows for more
    // than one type of scale factor to be applied for different
    // purposes
    void set_angle_P_scale_mult(const Vector3f &angle_P_scale) { _angle_P_scale *= angle_P_scale; }

    // get the value of the angle P scale that was used in the last loop
    const Vector3f &get_last_angle_P_scale(void) const { return _angle_P_scale_used; }
    
    // setup a one loop PD scale multiplier, multiplying by any
    // previously applied scale from this loop. This allows for more
    // than one type of scale factor to be applied for different
    // purposes
    void set_PD_scale_mult(const Vector3f &pd_scale) { _pd_scale *= pd_scale; }

    // setup a one loop I scale multiplier, multiplying by any
    // previously applied scale from this loop. This allows for more
    // than one type of scale factor to be applied for different
    // purposes
    void set_I_scale_mult(const Vector3f &i_scale) { _i_scale *= i_scale; }

    // scale I to represent the control given by angle P
    void scale_I_to_angle_P();

    // write RATE message
    void Write_Rate(const AC_PosControl &pos_control) const;

    // write ANG message
    void Write_ANG() const;

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    static constexpr Vector3f VECTORF_111{1.0f,1.0f,1.0f};

protected:

    // Update rate_target_ang_vel using attitude_error_rot_vec_rad
    Vector3f update_ang_vel_target_from_att_error(const Vector3f &attitude_error_rot_vec_rad);

    // Returns the most recent angular velocity reading (rad/s) for the rate controller.
    // Ensures minimum latency when rate control is run before or after attitude control.
    const Vector3f get_latest_gyro() const;

    // Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    AP_Float            _slew_yaw_cds;

    // Maximum angular velocity (in degrees/second) for earth-frame roll, pitch and yaw axis
    AP_Float            _ang_vel_roll_max_degs;
    AP_Float            _ang_vel_pitch_max_degs;
    AP_Float            _ang_vel_yaw_max_degs;

    // Maximum rotation acceleration for earth-frame roll axis
    AP_Float            _accel_roll_max_cdss;

    // Maximum rotation acceleration for earth-frame pitch axis
    AP_Float            _accel_pitch_max_cdss;

    // Maximum rotation acceleration for earth-frame yaw axis
    AP_Float            _accel_yaw_max_cdss;

    // Enable/Disable body frame rate feed forward
    AP_Int8             _rate_bf_ff_enabled;

    // Enable/Disable angle boost
    AP_Int8             _angle_boost_enabled;

    // angle controller P objects
    AC_P                _p_angle_roll;
    AC_P                _p_angle_pitch;
    AC_P                _p_angle_yaw;

    // Angle limit time constant (to maintain altitude)
    AP_Float            _angle_limit_tc;

    // Time constant [s] used to smooth user/control input changes before being passed to the rate controller. 
    // Affects angular response stiffness.
    AP_Float            _input_tc;

    // Controller gain multiplyer to be used when landed
    AP_Float            _land_roll_mult;
    AP_Float            _land_pitch_mult;
    AP_Float            _land_yaw_mult;

    // Latest body-frame gyro measurement (rad/s) used by rate controller
    Vector3f            _rate_gyro_rads;
    // timestamp of the latest gyro measurement (in microseconds) value used by the rate controller
    uint64_t            _rate_gyro_time_us;

    // Intersampling period in seconds
    float               _dt_s;

    // This represents a 321-intrinsic rotation in NED frame to the target (setpoint)
    // attitude used in the attitude controller, in radians.
    Vector3f            _euler_angle_target_rad;

    // This represents the angular velocity of the target (setpoint) attitude used in
    // the attitude controller as 321-intrinsic euler angle derivatives, in radians per
    // second.
    Vector3f            _euler_rate_target_rads;

    // This represents a quaternion rotation in NED frame to the target (setpoint)
    // attitude used in the attitude controller.
    Quaternion          _attitude_target;

    // This represents the angular velocity of the target (setpoint) attitude used in
    // the attitude controller as an angular velocity vector, in radians per second in
    // the target attitude frame.
    Vector3f            _ang_vel_target_rads;

    // This represents the angular velocity in radians per second in the body frame, used in the angular
    // velocity controller and most importantly the rate controller.
    Vector3f            _ang_vel_body_rads;

    // This is the angular velocity in radians per second in the body frame, added to the output angular
    // attitude controller by the System Identification Mode.
    // It is reset to zero immediately after it is used.
    Vector3f            _sysid_ang_vel_body_rads;

    // System Identification actuator override applied to motor commands. 
    // This is a unitless signal added post-PID for excitation purposes and is reset every loop.
    // It is reset to zero immediately after it is used.
    Vector3f            _actuator_sysid;

    // This represents a quaternion attitude error in the body frame, used for inertial frame reset handling.
    Quaternion          _attitude_ang_error;

    // The angle between the target thrust vector and the current thrust vector.
    float               _thrust_angle_rad;

    // The angle between the target thrust vector and the current thrust vector.
    float               _thrust_error_angle_rad;

    // throttle provided as input to attitude controller.  This does not include angle boost.
    float               _throttle_in = 0.0f;

    // Throttle increase applied to maintain vertical thrust during attitude tilt. 
    // Used logging for logging and diagnostics.
    float               _angle_boost;

    // Specifies whether the attitude controller should use the square root controller in the attitude correction.
    // This is used during Autotune to ensure the P term is tuned without being influenced by the acceleration limit of the square root controller.
    bool                _use_sqrt_controller;

    // Filtered Alt_Hold lean angle max - used to limit lean angle when throttle is saturated using Alt_Hold
    float               _althold_lean_angle_max_rad = 0.0f;

    // desired throttle_low_comp value, actual throttle_low_comp is slewed towards this value over 1~2 seconds
    float               _throttle_rpy_mix_desired;

    // mix between throttle and hover throttle for 0 to 1 and ratio above hover throttle for >1
    float               _throttle_rpy_mix;

    // Yaw feed forward percent to allow zero yaw actuator output during extreme roll and pitch corrections
    float               _feedforward_scalar = 1.0f;

    // Rate controller input smoothing time constants
    // Time constant for shaping roll/pitch rate input [s]
    float               _rate_rp_tc;
    // Time constant for shaping yaw rate input [s]
    float               _rate_y_tc;

    // Active scaling applied to Angle P gains for roll, pitch, yaw
    Vector3f            _angle_P_scale{1,1,1};

    // Active scaling applied to Angle P gains this loop (for logging/debugging)
    Vector3f            _angle_P_scale_used;

    // Proportional-Derivative gains applied dynamically per axis
    Vector3f            _pd_scale{1,1,1};

    // Proportional-Derivative gains this loop (for logging/debugging)
    Vector3f            _pd_scale_used;

    // Integrator gains applied dynamically per axis
    Vector3f            _i_scale{1,1,1};

    // Integrator gains this loop (for logging/debugging)
    Vector3f            _i_scale_used;

    // Ratio of normal to reduced rate controller gain when landed to suppress ground resonance
    float               _landed_gain_ratio;

    // References to external libraries
    const AP_AHRS_View&  _ahrs;
    const AP_MultiCopter &_aparm;
    AP_Motors&          _motors;

    static AC_AttitudeControl *_singleton;

public:
    // structure for angle and/or rate target
    enum class HeadingMode {
        Angle_Only,
        Angle_And_Rate,
        Rate_Only
    };
    struct HeadingCommand {
        float yaw_angle_rad;
        float yaw_rate_rads;
        HeadingMode heading_mode;
    };
    void input_thrust_vector_heading(const Vector3f& thrust_vector, HeadingCommand heading);
};
