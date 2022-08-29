#pragma once

/// @file    AC_AttitudeControl.h
/// @brief   ArduCopter attitude control library

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>
#include <AP_ADRC/AP_ADRC.h>

#define AC_ATTITUDE_CONTROL_ANGLE_P                     4.5f             // default angle P gain for roll, pitch and yaw

#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS       radians(40.0f)   // minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS       radians(720.0f)  // maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS        radians(10.0f)   // minimum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS        radians(120.0f)  // maximum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS        6000      // constraint on yaw angle error in degrees.  This should lead to maximum turn rate of 10deg/sec * Stab Rate P so by default will be 45deg/sec.
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS   110000.0f // default maximum acceleration for roll/pitch axis in centidegrees/sec/sec
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS    27000.0f  // default maximum acceleration for yaw axis in centidegrees/sec/sec

#define AC_ATTITUDE_RATE_CONTROLLER_TIMEOUT             1.0f    // body-frame rate controller timeout in seconds
#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          1.0f    // body-frame rate controller maximum output (for roll-pitch axis)
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         1.0f    // body-frame rate controller maximum output (for yaw axis)
#define AC_ATTITUDE_RATE_RELAX_TC                       0.16f   // This is used to decay the rate I term to 5% in half a second.

#define AC_ATTITUDE_THRUST_ERROR_ANGLE                  radians(30.0f) // Thrust angle error above which yaw corrections are limited

#define AC_ATTITUDE_400HZ_DT                            0.0025f // delta time in seconds for 400hz update rate

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

class AC_AttitudeControl {
public:
    AC_AttitudeControl( AP_AHRS_View &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_Motors& motors,
                        float dt) :
        _p_angle_roll(AC_ATTITUDE_CONTROL_ANGLE_P),
        _p_angle_pitch(AC_ATTITUDE_CONTROL_ANGLE_P),
        _p_angle_yaw(AC_ATTITUDE_CONTROL_ANGLE_P),
        _dt(dt),
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

    // pid accessors
    AC_P& get_angle_roll_p() { return _p_angle_roll; }
    AC_P& get_angle_pitch_p() { return _p_angle_pitch; }
    AC_P& get_angle_yaw_p() { return _p_angle_yaw; }
    virtual AC_PID& get_rate_roll_pid() = 0;
    virtual AC_PID& get_rate_pitch_pid() = 0;
    virtual AC_PID& get_rate_yaw_pid() = 0;

    // get the roll acceleration limit in centidegrees/s/s or radians/s/s
    float get_accel_roll_max_cdss() const { return _accel_roll_max; }
    float get_accel_roll_max_radss() const { return radians(_accel_roll_max * 0.01f); }

    // Sets the roll acceleration limit in centidegrees/s/s
    void set_accel_roll_max_cdss(float accel_roll_max) { _accel_roll_max.set(accel_roll_max); }

    // Sets and saves the roll acceleration limit in centidegrees/s/s
    void save_accel_roll_max_cdss(float accel_roll_max) { _accel_roll_max.set_and_save(accel_roll_max); }

    // get the pitch acceleration limit in centidegrees/s/s or radians/s/s
    float get_accel_pitch_max_cdss() const { return _accel_pitch_max; }
    float get_accel_pitch_max_radss() const { return radians(_accel_pitch_max * 0.01f); }

    // Sets the pitch acceleration limit in centidegrees/s/s
    void set_accel_pitch_max_cdss(float accel_pitch_max) { _accel_pitch_max.set(accel_pitch_max); }

    // Sets and saves the pitch acceleration limit in centidegrees/s/s
    void save_accel_pitch_max_cdss(float accel_pitch_max) { _accel_pitch_max.set_and_save(accel_pitch_max); }

    // get the yaw acceleration limit in centidegrees/s/s or radians/s/s
    float get_accel_yaw_max_cdss() const { return _accel_yaw_max; }
    float get_accel_yaw_max_radss() const { return radians(_accel_yaw_max * 0.01f); }

    // Sets the yaw acceleration limit in centidegrees/s/s
    void set_accel_yaw_max_cdss(float accel_yaw_max) { _accel_yaw_max.set(accel_yaw_max); }

    // Sets and saves the yaw acceleration limit in centidegrees/s/s
    void save_accel_yaw_max_cdss(float accel_yaw_max) { _accel_yaw_max.set_and_save(accel_yaw_max); }

    // get the roll angular velocity limit in radians/s
    float get_ang_vel_roll_max_rads() const { return radians(_ang_vel_roll_max); }

    // get the pitch angular velocity limit in radians/s
    float get_ang_vel_pitch_max_rads() const { return radians(_ang_vel_pitch_max); }

    // get the yaw angular velocity limit in radians/s
    float get_ang_vel_yaw_max_rads() const { return radians(_ang_vel_yaw_max); }

    // get the slew yaw rate limit in deg/s
    float get_slew_yaw_max_degs() const;

    // get the rate control input smoothing time constant
    float get_input_tc() const { return _input_tc; }

    // set the rate control input smoothing time constant
    void set_input_tc(float input_tc) { _input_tc.set(constrain_float(input_tc, 0.0f, 1.0f)); }

    // Ensure attitude controller have zero errors to relax rate controller output
    void relax_attitude_controllers();

    // Used by child class AC_AttitudeControl_TS to change behaviour for tailsitter quadplanes
    virtual void relax_attitude_controllers(bool exclude_pitch) { relax_attitude_controllers(); }

    // reset rate controller I terms
    void reset_rate_controller_I_terms();

    // reset rate controller I terms smoothly to zero in 0.5 seconds
    void reset_rate_controller_I_terms_smoothly();

    // Sets attitude target to vehicle attitude and sets all rates to zero
    // If reset_rate is false rates are not reset to allow the rate controllers to run
    void reset_target_and_rate(bool reset_rate = true);

    // Sets yaw target to vehicle heading and sets yaw rate to zero
    // If reset_rate is false rates are not reset to allow the rate controllers to run
    void reset_yaw_target_and_rate(bool reset_rate = true);

    // handle reset of attitude from EKF since the last iteration
    void inertial_frame_reset();

    // Command a Quaternion attitude with feedforward and smoothing
    // attitude_desired_quat: is updated on each time_step (_dt) by the integral of the angular velocity
    virtual void input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_target);

    // Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
    virtual void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds);
    // Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
    virtual void input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw);

    // Command euler yaw rate and pitch angle with roll angle specified in body frame
    // (implemented only in AC_AttitudeControl_TS for tailsitter quadplanes)
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float euler_roll_angle_cd,
        float euler_pitch_angle_cd, float euler_yaw_rate_cds) {}

    // Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
    virtual void input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds);

    // Command an angular velocity with angular velocity feedforward and smoothing
    virtual void input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // Command an angular velocity with angular velocity feedforward and smoothing
    virtual void input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
    virtual void input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // Command an angular step (i.e change) in body frame angle
    virtual void input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd);

    // Command a thrust vector in the earth frame and a heading angle and/or rate
    virtual void input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw = true);
    virtual void input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds);
    void input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_cd) {input_thrust_vector_heading(thrust_vector, heading_cd, 0.0f);}

    // Converts thrust vector and heading angle to quaternion rotation in the earth frame
    Quaternion attitude_from_thrust_vector(Vector3f thrust_vector, float heading_angle) const;

    // Run angular velocity controller and send outputs to the motors
    virtual void rate_controller_run() = 0;

    // Convert a 321-intrinsic euler angle derivative to an angular velocity vector
    void euler_rate_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads);

    // Convert an angular velocity vector to a 321-intrinsic euler angle derivative
    // Returns false if the vehicle is pitched 90 degrees up or down
    bool ang_vel_to_euler_rate(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads);

    // Specifies whether the attitude controller should use the square root controller in the attitude correction.
    // This is used during Autotune to ensure the P term is tuned without being influenced by the acceleration limit of the square root controller.
    void use_sqrt_controller(bool use_sqrt_cont) { _use_sqrt_controller = use_sqrt_cont; }

    // Return 321-intrinsic euler angles in centidegrees representing the rotation from NED earth frame to the
    // attitude controller's target attitude.
    // **NOTE** Using vector3f*deg(100) is more efficient than deg(vector3f)*100 or deg(vector3d*100) because it gives the
    // same result with the fewest multiplications. Even though it may look like a bug, it is intentional. See issue 4895.
    Vector3f get_att_target_euler_cd() const { return _euler_angle_target * degrees(100.0f); }
    const Vector3f & get_att_target_euler_rad() const { return _euler_angle_target; }

    // Return the body-to-NED target attitude used by the quadplane-specific attitude control input methods
    Quaternion get_attitude_target_quat() const { return _attitude_target; }

    // Return the angular velocity of the target (setpoint) [rad/s] in the target attitude frame
    const Vector3f& get_attitude_target_ang_vel() const { return _ang_vel_target;}

    // Return the angle between the target thrust vector and the current thrust vector.
    float get_att_error_angle_deg() const { return degrees(_thrust_error_angle); }

    // Set x-axis angular velocity in centidegrees/s
    void rate_bf_roll_target(float rate_cds) { _ang_vel_body.x = radians(rate_cds * 0.01f); }

    // Set y-axis angular velocity in centidegrees/s
    void rate_bf_pitch_target(float rate_cds) { _ang_vel_body.y = radians(rate_cds * 0.01f); }

    // Set z-axis angular velocity in centidegrees/s
    void rate_bf_yaw_target(float rate_cds) { _ang_vel_body.z = radians(rate_cds * 0.01f); }

    // Set x-axis system identification angular velocity in degrees/s
    void rate_bf_roll_sysid(float rate) { _sysid_ang_vel_body.x = rate; }

    // Set y-axis system identification angular velocity in degrees/s
    void rate_bf_pitch_sysid(float rate) { _sysid_ang_vel_body.y = rate; }

    // Set z-axis system identification angular velocity in degrees/s
    void rate_bf_yaw_sysid(float rate) { _sysid_ang_vel_body.z = rate; }

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

    // Return angular velocity in radians used in the angular velocity controller
    Vector3f rate_bf_targets() const { return _ang_vel_body + _sysid_ang_vel_body; }

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

    // Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
    virtual float get_althold_lean_angle_max_cd() const;

    // Return configured tilt angle limit in centidegrees
    float lean_angle_max_cd() const { return _aparm.angle_max; }

    // Return tilt angle in degrees
    float lean_angle_deg() const { return degrees(_thrust_angle); }

    // calculates the velocity correction from an angle error. The angular velocity has acceleration and
    // deceleration limits including basic jerk limiting using smoothing_gain
    static float input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float desired_ang_vel, float max_ang_vel, float dt);
    static float input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float dt){ return input_shaping_angle(error_angle,  input_tc,  accel_max,  target_ang_vel,  0.0f,  0.0f,  dt); }

    // Shapes the velocity request based on a rate time constant. The angular acceleration and deceleration is limited.
    static float input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max, float dt, float input_tc);

    // calculates the expected angular velocity correction from an angle error based on the AC_AttitudeControl settings.
    // This function can be used to predict the delay associated with angle requests.
    void input_shaping_rate_predictor(const Vector2f &error_angle, Vector2f& target_ang_vel, float dt) const;

    // translates body frame acceleration limits to the euler axis
    void ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max, float ang_vel_pitch_max, float ang_vel_yaw_max) const;

    // translates body frame acceleration limits to the euler axis
    Vector3f euler_accel_limit(const Vector3f &euler_rad, const Vector3f &euler_accel);

    // Calculates the body frame angular velocities to follow the target attitude
    void attitude_controller_run_quat();

    // thrust_heading_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
    // The maximum error in the yaw axis is limited based on the angle yaw P value and acceleration.
    void thrust_heading_rotation_angles(Quaternion& attitude_target, const Quaternion& attitude_body, Vector3f& attitude_error, float& thrust_angle, float& thrust_error_angle) const;

    // thrust_vector_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
    // The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
    void thrust_vector_rotation_angles(const Quaternion& attitude_target, const Quaternion& attitude_body, Quaternion& thrust_vector_correction, Vector3f& attitude_error, float& thrust_angle, float& thrust_error_angle) const;

    // sanity check parameters.  should be called once before take-off
    virtual void parameter_sanity_check() {}

    // return true if the rpy mix is at lowest value
    virtual bool is_throttle_mix_min() const { return true; }

    // control rpy throttle mix
    virtual void set_throttle_mix_min() {}
    virtual void set_throttle_mix_man() {}
    virtual void set_throttle_mix_max(float ratio) {}
    virtual void set_throttle_mix_value(float value) {}
    virtual float get_throttle_mix(void) const { return 0; }

    // enable use of flybass passthrough on heli
    virtual void use_flybar_passthrough(bool passthrough, bool tail_passthrough) {}

	// use_leaky_i - controls whether we use leaky i term for body-frame to motor output stage on heli
	virtual void use_leaky_i(bool leaky_i) {}

    // set_hover_roll_scalar - scales Hover Roll Trim parameter. To be used by vehicle code according to vehicle condition.
    virtual void set_hover_roll_trim_scalar(float scalar) {}

    // Return angle in centidegrees to be added to roll angle for hover collective learn. Used by heli to counteract
    // tail rotor thrust in hover. Overloaded by AC_Attitude_Heli to return angle.
    virtual float get_roll_trim_cd() { return 0;}

    // passthrough_bf_roll_pitch_rate_yaw - roll and pitch are passed through directly, body-frame rate target for yaw
    virtual void passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf_cds) {};

    // provide feedback on whether arming would be a good idea right now:
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    // enable inverted flight on backends that support it
    virtual void set_inverted_flight(bool inverted) {}

    // get the slew rate value for roll, pitch and yaw, for oscillation detection in lua scripts
    void get_rpy_srate(float &roll_srate, float &pitch_srate, float &yaw_srate);

    // Sets the roll and pitch rate shaping time constant
    void set_roll_pitch_rate_tc(float input_tc) { _rate_rp_tc = input_tc; }

    // Sets the yaw rate shaping time constant
    void set_yaw_rate_tc(float input_tc) { _rate_y_tc = input_tc; }
    // enable/disable alternative rate control
    virtual void set_alt_rate_control(bool enable) {}

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Update rate_target_ang_vel using attitude_error_rot_vec_rad
    Vector3f update_ang_vel_target_from_att_error(const Vector3f &attitude_error_rot_vec_rad);

    // Return angle in radians to be added to roll angle. Used by heli to counteract
    // tail rotor thrust in hover. Overloaded by AC_Attitude_Heli to return angle.
    virtual float get_roll_trim_rad() { return 0;}

    // Return the yaw slew rate limit in radians/s
    float get_slew_yaw_max_rads() const { return radians(get_slew_yaw_max_degs()); }

    // Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    AP_Float            _slew_yaw;

    // Maximum angular velocity (in degrees/second) for earth-frame roll, pitch and yaw axis
    AP_Float            _ang_vel_roll_max;
    AP_Float            _ang_vel_pitch_max;
    AP_Float            _ang_vel_yaw_max;

    // Maximum rotation acceleration for earth-frame roll axis
    AP_Float            _accel_roll_max;

    // Maximum rotation acceleration for earth-frame pitch axis
    AP_Float            _accel_pitch_max;

    // Maximum rotation acceleration for earth-frame yaw axis
    AP_Float            _accel_yaw_max;

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

    // rate controller input smoothing time constant
    AP_Float            _input_tc;

    // Intersampling period in seconds
    float               _dt;

    // This represents a 321-intrinsic rotation in NED frame to the target (setpoint)
    // attitude used in the attitude controller, in radians.
    Vector3f            _euler_angle_target;

    // This represents the angular velocity of the target (setpoint) attitude used in
    // the attitude controller as 321-intrinsic euler angle derivatives, in radians per
    // second.
    Vector3f            _euler_rate_target;

    // This represents a quaternion rotation in NED frame to the target (setpoint)
    // attitude used in the attitude controller.
    Quaternion          _attitude_target;

    // This represents the angular velocity of the target (setpoint) attitude used in
    // the attitude controller as an angular velocity vector, in radians per second in
    // the target attitude frame.
    Vector3f            _ang_vel_target;

    // This represents the angular velocity in radians per second in the body frame, used in the angular
    // velocity controller.
    Vector3f            _ang_vel_body;

    // This is the angular velocity in radians per second in the body frame, added to the output angular
    // attitude controller by the System Identification Mode.
    // It is reset to zero immediately after it is used.
    Vector3f            _sysid_ang_vel_body;

    // This is the unitless value added to the output of the PID by the System Identification Mode.
    // It is reset to zero immediately after it is used.
    Vector3f            _actuator_sysid;

    // This represents a quaternion attitude error in the body frame, used for inertial frame reset handling.
    Quaternion          _attitude_ang_error;

    // The angle between the target thrust vector and the current thrust vector.
    float               _thrust_angle;

    // The angle between the target thrust vector and the current thrust vector.
    float               _thrust_error_angle;

    // throttle provided as input to attitude controller.  This does not include angle boost.
    float               _throttle_in = 0.0f;

    // This represents the throttle increase applied for tilt compensation.
    // Used only for logging.
    float               _angle_boost;

    // Specifies whether the attitude controller should use the square root controller in the attitude correction.
    // This is used during Autotune to ensure the P term is tuned without being influenced by the acceleration limit of the square root controller.
    bool                _use_sqrt_controller;

    // Filtered Alt_Hold lean angle max - used to limit lean angle when throttle is saturated using Alt_Hold
    float               _althold_lean_angle_max = 0.0f;

    // desired throttle_low_comp value, actual throttle_low_comp is slewed towards this value over 1~2 seconds
    float               _throttle_rpy_mix_desired;

    // mix between throttle and hover throttle for 0 to 1 and ratio above hover throttle for >1
    float               _throttle_rpy_mix;

    // Yaw feed forward percent to allow zero yaw actuator output during extreme roll and pitch corrections
    float               _feedforward_scalar = 1.0f;

    // rate controller input smoothing time constant
    float               _rate_rp_tc;
    float               _rate_y_tc;

    // References to external libraries
    const AP_AHRS_View&  _ahrs;
    const AP_Vehicle::MultiCopter &_aparm;
    AP_Motors&          _motors;

    static AC_AttitudeControl *_singleton;

protected:
    /*
      state of control monitoring
    */
    struct {
        float rms_roll_P;
        float rms_roll_D;
        float rms_pitch_P;
        float rms_pitch_D;
        float rms_yaw;
    } _control_monitor;

    // update state in ControlMonitor
    void control_monitor_filter_pid(float value, float &rms_P);
    void control_monitor_update(void);

    // true in inverted flight mode
    bool _inverted_flight;

public:
    // log a CTRL message
    void control_monitor_log(void) const;

    // return current RMS controller filter for each axis
    float control_monitor_rms_output_roll(void) const;
    float control_monitor_rms_output_roll_P(void) const;
    float control_monitor_rms_output_roll_D(void) const;
    float control_monitor_rms_output_pitch_P(void) const;
    float control_monitor_rms_output_pitch_D(void) const;
    float control_monitor_rms_output_pitch(void) const;
    float control_monitor_rms_output_yaw(void) const;
};
