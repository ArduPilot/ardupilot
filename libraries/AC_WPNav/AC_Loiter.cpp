#include <AP_HAL/AP_HAL.h>
#include "AC_Loiter.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AC_Avoidance/AC_Avoid.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_Heli)
 #define LOITER_SPEED_DEFAULT_MS             30.0    // Default horizontal loiter speed in m/s.
 #define LOITER_BRAKE_ACCEL_DEFAULT_MSS      1.25    // Default maximum braking acceleration when sticks are released (m/s²).
 #define LOITER_BRAKE_JERK_DEFAULT_MSSS      2.5    // Default maximum jerk applied during braking transitions (m/s³).
#else
 #define LOITER_SPEED_DEFAULT_MS             12.5    // Default horizontal loiter speed in m/s.
 #define LOITER_BRAKE_ACCEL_DEFAULT_MSS      2.5     // Default maximum braking acceleration when sticks are released (m/s²).
 #define LOITER_BRAKE_JERK_DEFAULT_MSSS      5.0     // Default maximum jerk applied during braking transitions (m/s³).
#endif

// Common defaults between trad heli and copter
#define LOITER_SPEED_MIN_MS                 0.2     // Minimum allowed horizontal loiter speed in m/s.
#define LOITER_ACCEL_MAX_DEFAULT_MSS        5.0     // Default maximum horizontal acceleration in loiter mode (m/s²).
#define LOITER_BRAKE_START_DELAY_DEFAULT_S  1.0     // Delay (in seconds) before braking begins after sticks are released.
#define LOITER_VEL_CORRECTION_MAX_MS        2.0     // Maximum speed (in m/s) used for correcting position errors in loiter.
#define LOITER_POS_CORRECTION_MAX_M         2.0     // Maximum horizontal position error allowed before correction (m).
#define LOITER_ACTIVE_TIMEOUT_MS            200     // Loiter is considered active if updated within the past 200 ms.
#define LOITER_DEFAULT_OPTIONS              0       // Coordinated Turns disabled by default.

const AP_Param::GroupInfo AC_Loiter::var_info[] = {

    // @Param: ANG_MAX
    // @DisplayName: Loiter pilot angle max
    // @Description{Copter, Sub}: Loiter maximum pilot requested lean angle. Set to zero for 2/3 of PSC_ANGLE_MAX/ATC_ANGLE_MAX. The maximum vehicle lean angle is still limited by PSC_ANGLE_MAX/ATC_ANGLE_MAX
    // @Description: Loiter maximum pilot requested lean angle. Set to zero for 2/3 of Q_P_ANGLE_MAX/Q_A_ANGLE_MAX. The maximum vehicle lean angle is still limited by Q_P_ANGLE_MAX/Q_A_ANGLE_MAX
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ANG_MAX",  1, AC_Loiter, _angle_max_deg, 0.0f),

    // 2 was SPEED
    // 3 was ACC_MAX
    // 4 was BRK_ACCEL
    // 5 was BRK_JERK

    // @Param: BRK_DELAY
    // @DisplayName: Loiter brake start delay (in seconds)
    // @Description: Loiter brake start delay (in seconds)
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BRK_DELAY",  6, AC_Loiter, _brake_delay_s, LOITER_BRAKE_START_DELAY_DEFAULT_S),

    // @Param: OPTIONS
    // @DisplayName: Loiter mode options
    // @Description: Enables optional Loiter mode behaviors
    // @Bitmask: 0: Enable Coordinated turns
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 7, AC_Loiter, _options, LOITER_DEFAULT_OPTIONS),

    // @Param: SPEED_MS
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in m/s which the aircraft will travel horizontally while in loiter mode
    // @Units: m/s
    // @Range: 0.20 35
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("SPEED_MS", 8, AC_Loiter, _speed_max_ne_ms, LOITER_SPEED_DEFAULT_MS),

    // @Param: ACC_MAX_M
    // @DisplayName: Loiter maximum acceleration
    // @Description: Loiter maximum acceleration in m/s/s. Limits the acceleration the pilot can command: full stick requests this or the LOIT_ANG_MAX lean-angle acceleration, whichever is lower, and combined roll and pitch input is capped at this value. Also limits position-error correction. Higher values cause the copter to accelerate and correct position errors more aggressively.
    // @Units: m/s/s
    // @Range: 1 9.81
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ACC_MAX_M", 9, AC_Loiter, _accel_max_ne_mss, LOITER_ACCEL_MAX_DEFAULT_MSS),

    // @Param: BRK_ACC_M
    // @DisplayName: Loiter braking acceleration
    // @Description: Loiter braking acceleration in m/s/s. Higher values stop the copter more quickly when the stick is centered. Also caps the deceleration the pilot can command by moving the stick against the direction of travel.
    // @Units: m/s/s
    // @Range: 0.25 2.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("BRK_ACC_M", 10, AC_Loiter, _brake_accel_max_mss, LOITER_BRAKE_ACCEL_DEFAULT_MSS),

    // @Param: BRK_JRK_M
    // @DisplayName: Loiter braking jerk
    // @Description: Loiter braking jerk in m/s/s/s. Higher values will remove braking faster if the pilot moves the sticks during a braking maneuver. Also limits how quickly pilot-commanded deceleration builds when the stick is moved against the direction of travel.
    // @Units: m/s/s/s
    // @Range: 5 50
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("BRK_JRK_M", 11, AC_Loiter, _brake_jerk_max_msss, LOITER_BRAKE_JERK_DEFAULT_MSSS),

    // @Param: JRK_M
    // @DisplayName: Loiter pilot jerk
    // @Description: Loiter maximum jerk of pilot-commanded acceleration in m/s/s/s, for every change that is not braking against the direction of travel (that uses LOIT_BRK_JRK_M). Lower values make the response to stick input smoother. Set to zero to disable.
    // @Units: m/s/s/s
    // @Range: 0 50
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("JRK_M", 12, AC_Loiter, _jerk_max_ne_msss, 0.0f),

    // @Param: STOP_JRK_M
    // @DisplayName: Loiter stop jerk
    // @Description: Loiter jerk in m/s/s/s with which braking finishes when the vehicle comes to rest after the stick is centered. Braking still builds at LOIT_BRK_JRK_M; it is then held, up to LOIT_BRK_ACC_M, until the speed is low enough for a ramp at this jerk to reach zero braking at zero speed. Higher values end the stop more crisply. Values no higher than PSC_NE_JERK are recommended, as the position controller shapes at that jerk. Set to zero to keep the default stop, whose last part decays exponentially at a rate set by PSC_NE_VEL_P.
    // @Units: m/s/s/s
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("STOP_JRK_M", 13, AC_Loiter, _stop_jerk_max_msss, 0.0f),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
AC_Loiter::AC_Loiter(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Sets the initial loiter target position in meters from the EKF origin.
// - position_ne_m: horizontal position in the NE frame, in meters.
// - Initializes internal control state including acceleration targets and feed-forward planning.
void AC_Loiter::init_target_m(const Vector2p& position_ne_m)
{
    sanity_check_params();

    // Configure speed/accel limits in meters using internal parameter (_accel_max_ne_mss)
    _pos_control.NE_set_correction_speed_accel_m(LOITER_VEL_CORRECTION_MAX_MS, _accel_max_ne_mss);
    _pos_control.NE_set_pos_error_max_m(LOITER_POS_CORRECTION_MAX_M);

    // Reset controller state for stationary loiter
    _pos_control.NE_init_controller_stopping_point();

    // Zero out desired and predicted accelerations and angles
    _predicted_accel_ne_mss.zero();
    _desired_accel_ne_mss.zero();
    _predicted_euler_angle_rad.zero();
    _brake_accel_mss = 0.0f;
    _pilot_accel_ne_mss.zero();

    // Set position target for stationary loiter
    _pos_control.set_pos_desired_NE_m(position_ne_m);
}

// Initializes the loiter controller using the current position and velocity.
// Updates feed-forward velocity, predicted acceleration, and resets control state.
void AC_Loiter::init_target()
{
    sanity_check_params();

    // Configure correction speed and acceleration limits (in m/s and m/s²)
    _pos_control.NE_set_correction_speed_accel_m(LOITER_VEL_CORRECTION_MAX_MS, _accel_max_ne_mss);
    _pos_control.NE_set_pos_error_max_m(LOITER_POS_CORRECTION_MAX_M);

    // Apply velocity smoothing: softly transitions target acceleration to zero
    _pos_control.NE_relax_velocity_controller();

    // Initialize prediction state using current acceleration and lean angles
    _predicted_accel_ne_mss = _pos_control.get_accel_target_NED_mss().xy();
    _predicted_euler_angle_rad.x = _pos_control.get_roll_rad();
    _predicted_euler_angle_rad.y = _pos_control.get_pitch_rad();
    _predicted_euler_rate.zero();
    _predicted_euler_accel.zero();
    _brake_accel_mss = 0.0f;
    _pilot_accel_ne_mss.zero();
}

// Reduces loiter responsiveness for smoother descent during landing.
// Internally softens horizontal control gains.
void AC_Loiter::soften_for_landing()
{
    _pos_control.NE_soften_for_landing();
}

// Sets pilot desired acceleration using Euler angles in centidegrees.
// See set_pilot_desired_acceleration_rad() for full details.
void AC_Loiter::set_pilot_desired_acceleration_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd)
{
    set_pilot_desired_acceleration_rad(cd_to_rad(euler_roll_angle_cd), cd_to_rad(euler_pitch_angle_cd));
}

// Sets pilot desired acceleration using Euler angles in radians.
// - Internally computes a smoothed acceleration vector based on predictive rate shaping.
// - Inputs: `euler_roll_angle_rad`, `euler_pitch_angle_rad` in radians.
// - Applies internal shaping using the current attitude controller dt.
void AC_Loiter::set_pilot_desired_acceleration_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad)
{
    const float dt_s = _attitude_control.get_dt_s();

    // convert our desired attitude to an acceleration vector assuming we are not accelerating vertically
    const Vector3f desired_euler_rad {euler_roll_angle_rad, euler_pitch_angle_rad, _ahrs.yaw};
    const Vector2f request_ne_mss = _pos_control.lean_angles_rad_to_accel_NED_mss(desired_euler_rad).xy();

    // Reset brake timer if there is pilot input. This tests the stick request rather than the
    // limited acceleration, so a jerk-limited ramp-down does not delay the start of braking.
    if (!request_ne_mss.is_zero()) {
        _brake_timer_ms = AP_HAL::millis();
    }

    // Apply the pilot acceleration, braking and jerk limits
    _desired_accel_ne_mss = limit_pilot_accel_NE_mss(request_ne_mss, dt_s);

    // Lean angles that produce the limited acceleration. When no limit acted these are the stick
    // angles themselves, so the command model sees exactly the input it always did.
    float target_roll_rad = euler_roll_angle_rad;
    float target_pitch_rad = euler_pitch_angle_rad;
    if (!(_desired_accel_ne_mss == request_ne_mss)) {
        // inverse of lean_angles_rad_to_accel_NED_mss(): rotate into the body forward-right frame
        const float accel_forward_mss = _desired_accel_ne_mss.x * _ahrs.cos_yaw() + _desired_accel_ne_mss.y * _ahrs.sin_yaw();
        const float accel_right_mss = -_desired_accel_ne_mss.x * _ahrs.sin_yaw() + _desired_accel_ne_mss.y * _ahrs.cos_yaw();
        target_pitch_rad = accel_mss_to_angle_rad(-accel_forward_mss);
        target_roll_rad = accel_mss_to_angle_rad(accel_right_mss * cosf(target_pitch_rad));
    }

    // Compute attitude error between desired and predicted lean angles
    Vector2f angle_error_euler_rad(wrap_PI(target_roll_rad - _predicted_euler_angle_rad.x), wrap_PI(target_pitch_rad - _predicted_euler_angle_rad.y));

    // Predict roll/pitch rate required to achieve target attitude
    _attitude_control.command_model_rate_predictor(angle_error_euler_rad, _predicted_euler_rate, _predicted_euler_accel, dt_s);

    // Update internal attitude estimate for next iteration
    _predicted_euler_angle_rad += _predicted_euler_rate * dt_s;

    // Convert predicted angles into an acceleration vector for braking/shaping
    const Vector3f predicted_euler_rad {_predicted_euler_angle_rad.x, _predicted_euler_angle_rad.y, _ahrs.yaw};
    const Vector3f predicted_accel_ned_mss = _pos_control.lean_angles_rad_to_accel_NED_mss(predicted_euler_rad);

    _predicted_accel_ne_mss = predicted_accel_ned_mss.xy();

    if (loiter_option_is_set(LoiterOption::COORDINATED_TURN_ENABLED)) {
        Vector3f target_ang_vel_rads = _attitude_control.get_attitude_target_ang_vel();
        Vector3f desired_velocity_ms = _pos_control.get_vel_desired_NED_ms();
        Vector2f turn_accel_ne_mss = Vector2f(-desired_velocity_ms.y * target_ang_vel_rads.z, desired_velocity_ms.x * target_ang_vel_rads.z);
        _desired_accel_ne_mss += turn_accel_ne_mss;
        _predicted_accel_ne_mss += turn_accel_ne_mss;
    }
}

// Calculates the expected stopping point based on current velocity and position in the NE frame.
// Result is returned in meters.
// Uses the position controller’s deceleration model.
void AC_Loiter::get_stopping_point_NE_m(Vector2f& stopping_point_ne_m) const
{
    Vector2p stop_ne_m;
    // Query stopping point from position controller in postype (float or double)
    _pos_control.get_stopping_point_NE_m(stop_ne_m);

    // Convert from postype to float (Vector2f)
    stopping_point_ne_m = stop_ne_m.tofloat();
}

// Returns the maximum pilot-commanded lean angle in centidegrees.
// See get_angle_max_rad() for full details.
float AC_Loiter::get_angle_max_cd() const
{
    // Convert radians to centidegrees
    return rad_to_cd(get_angle_max_rad());
}

// Returns the maximum pilot-commanded lean angle in radians.
// - If `_angle_max_deg` is zero, this returns 2/3 of the limiting PSC angle.
// - Otherwise, returns the minimum of `_angle_max_deg` and PSC’s configured angle limit.
float AC_Loiter::get_angle_max_rad() const
{
    if (!is_positive(_angle_max_deg)) {
        // Use 2/3 of the smallest system-wide max lean angle
        return MIN(_attitude_control.lean_angle_max_rad(), _pos_control.get_lean_angle_max_rad()) * (2.0f / 3.0f);
    }
    // Use configured parameter (in degrees), constrained to PSC limit
    return MIN(radians(_angle_max_deg), _pos_control.get_lean_angle_max_rad());
}

// Runs the loiter control loop, computing desired acceleration and updating position control.
// If `avoidance_on` is true, velocity is adjusted using avoidance logic before being applied.
void AC_Loiter::update(bool avoidance_on)
{
    // Calculate desired velocity using pilot inputs, braking, and drag
    calc_desired_velocity(avoidance_on);

    // Run position controller to compute desired attitude and thrust
    _pos_control.NE_update_controller();
}

// Sets the maximum allowed horizontal loiter speed in m/s.
void AC_Loiter::set_speed_max_NE_ms(float speed_max_ne_ms)
{
    _speed_max_ne_ms.set(MAX(speed_max_ne_ms, LOITER_SPEED_MIN_MS));
}

// perform any required parameter conversions
void AC_Loiter::convert_parameters()
{
    // PARAMETER_CONVERSION - Added: Jan-2026 for ArduPilot-4.7

    // return immediately if no conversion is needed
    if (_speed_max_ne_ms.configured() || _accel_max_ne_mss.configured() || _brake_accel_max_mss.configured() || _brake_jerk_max_msss.configured()) {
        return;
    }

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // convert QuadPlane parameters
    static const AP_Param::ConversionInfo conversion_info[] = {
        { 205, 8379, AP_PARAM_FLOAT, "Q_LOIT_SPEED_MS" },   // Q_LOIT_SPEED moved to Q_LOIT_SPEED_MS
        { 205, 12475, AP_PARAM_FLOAT, "Q_LOIT_ACC_MAX_M" }, // Q_LOIT_ACC_MAX moved to Q_LOIT_ACC_MAX_M
        { 205, 16571, AP_PARAM_FLOAT, "Q_LOIT_BRK_ACC_M" }, // Q_LOIT_BRK_ACCEL moved to Q_LOIT_BRK_ACC_M
        { 205, 20667, AP_PARAM_FLOAT, "Q_LOIT_BRK_JRK_M" }, // Q_LOIT_BRK_JERK moved to Q_LOIT_BRK_JRK_M
    };
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
    // convert Sub parameters
    static const AP_Param::ConversionInfo conversion_info[] = {
        { 63, 2, AP_PARAM_FLOAT, "LOIT_SPEED_MS" },     // LOIT_SPEED moved to LOIT_SPEED_MS
        { 63, 3, AP_PARAM_FLOAT, "LOIT_ACC_MAX_M" },    // LOIT_ACC_MAX moved to LOIT_ACC_MAX_M
        { 63, 4, AP_PARAM_FLOAT, "LOIT_BRK_ACC_M" },    // LOIT_BRK_ACCEL moved to LOIT_BRK_ACC_M
        { 63, 5, AP_PARAM_FLOAT, "LOIT_BRK_JRK_M" },    // LOIT_BRK_JERK moved to LOIT_BRK_JRK_M
    };
#else
    // convert Copter parameters
    static const AP_Param::ConversionInfo conversion_info[] = {
        { 105, 2, AP_PARAM_FLOAT, "LOIT_SPEED_MS" },    // LOIT_SPEED moved to LOIT_SPEED_MS
        { 105, 3, AP_PARAM_FLOAT, "LOIT_ACC_MAX_M" },   // LOIT_ACC_MAX moved to LOIT_ACC_MAX_M
        { 105, 4, AP_PARAM_FLOAT, "LOIT_BRK_ACC_M" },   // LOIT_BRK_ACCEL moved to LOIT_BRK_ACC_M
        { 105, 5, AP_PARAM_FLOAT, "LOIT_BRK_JRK_M" },   // LOIT_BRK_JERK moved to LOIT_BRK_JRK_M
    };
#endif

    AP_Param::convert_old_parameters_scaled(conversion_info, ARRAY_SIZE(conversion_info), 0.01, 0);
}

// Ensures internal parameters are within valid safety limits.
// Applies min/max constraints on speed and acceleration settings.
void AC_Loiter::sanity_check_params()
{
    // Enforce minimum loiter speed
    _speed_max_ne_ms.set(MAX(_speed_max_ne_ms, LOITER_SPEED_MIN_MS));

    // Clamp horizontal accel to lean-angle-limited max
    _accel_max_ne_mss.set(MIN(_accel_max_ne_mss, GRAVITY_MSS * tanf(_attitude_control.lean_angle_max_rad())));
}

bool AC_Loiter::loiter_option_is_set(LoiterOption option) const {
    return (_options & int8_t(option)) != 0;
}

// Returns the largest horizontal acceleration the pilot can command, in m/s².
// The lower of the LOIT_ANG_MAX lean-angle acceleration and LOIT_ACC_MAX_M.
float AC_Loiter::get_pilot_accel_max_mss() const
{
    const float lean_accel_max_mss = angle_rad_to_accel_mss(get_angle_max_rad());
    if (!is_positive(_accel_max_ne_mss)) {
        return lean_accel_max_mss;
    }
    return MIN(lean_accel_max_mss, _accel_max_ne_mss.get());
}

// Limits the pilot's requested NE acceleration in m/s² and returns the limited request.
// - Full stick is scaled to get_pilot_accel_max_mss() and the magnitude is capped at LOIT_ACC_MAX_M.
// - A component opposing the desired velocity is braking: it is capped at LOIT_BRK_ACC_M and
//   slewed at LOIT_BRK_JRK_M. Every other change is slewed at LOIT_JRK_M (0 disables).
// Without this, moving the stick against the direction of travel decelerates at the full
// LOIT_ANG_MAX acceleration plus drag, bypassing the braking limits a centered stick obeys.
Vector2f AC_Loiter::limit_pilot_accel_NE_mss(const Vector2f& request_ne_mss, float dt_s)
{
    // Restart from zero if the limiter has not run recently, e.g. on entering a mode that uses loiter
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _pilot_accel_update_ms > LOITER_ACTIVE_TIMEOUT_MS) {
        _pilot_accel_ne_mss.zero();
    }
    _pilot_accel_update_ms = now_ms;

    Vector2f target_ne_mss = request_ne_mss;

    // Scale so that full stick on one axis requests get_pilot_accel_max_mss() rather than the full
    // lean-angle acceleration, then cap the magnitude so combined roll and pitch cannot exceed it.
    const float lean_accel_max_mss = angle_rad_to_accel_mss(get_angle_max_rad());
    const float accel_max_mss = get_pilot_accel_max_mss();
    if (is_positive(lean_accel_max_mss) && accel_max_mss < lean_accel_max_mss) {
        target_ne_mss *= accel_max_mss / lean_accel_max_mss;
    }
    if (is_positive(_accel_max_ne_mss)) {
        target_ne_mss.limit_length(_accel_max_ne_mss);
    }

    // A component opposing the desired velocity is braking: cap it at LOIT_BRK_ACC_M
    bool braking = false;
    const Vector2f vel_desired_ne_ms = _pos_control.get_vel_desired_NED_ms().xy();
    const float speed_ms = vel_desired_ne_ms.length();
    if (speed_ms > LOITER_SPEED_MIN_MS) {
        const Vector2f vel_unit_ne = vel_desired_ne_ms / speed_ms;
        float accel_along_mss = target_ne_mss * vel_unit_ne;
        if (is_negative(accel_along_mss)) {
            braking = true;
            if (is_positive(_brake_accel_max_mss)) {
                const Vector2f accel_cross_ne_mss = target_ne_mss - vel_unit_ne * accel_along_mss;
                accel_along_mss = MAX(accel_along_mss, -_brake_accel_max_mss.get());
                target_ne_mss = accel_cross_ne_mss + vel_unit_ne * accel_along_mss;
            }
        }
    }

    // Slew toward the target: braking uses LOIT_BRK_JRK_M, everything else LOIT_JRK_M
    const float jerk_max_msss = braking ? _brake_jerk_max_msss.get() : _jerk_max_ne_msss.get();
    Vector2f delta_ne_mss = target_ne_mss - _pilot_accel_ne_mss;
    if (is_positive(jerk_max_msss) && is_positive(dt_s) && delta_ne_mss.limit_length(jerk_max_msss * dt_s)) {
        _pilot_accel_ne_mss += delta_ne_mss;
    } else {
        _pilot_accel_ne_mss = target_ne_mss;
    }

    return _pilot_accel_ne_mss;
}

// Updates feed-forward velocity using pilot-requested acceleration and braking logic.
// - Applies drag and braking forces when sticks are released.
// - Velocity is adjusted for fence/avoidance if enabled.
// - Resulting velocity and acceleration are sent to the position controller.
void AC_Loiter::calc_desired_velocity(bool avoidance_on)
{
    // the estimator might impose limits on maximum velocity (e.g. due
    // to the sensors being used to supply its estimated velocity):
    const float ekfGndSpdLimit_ms = AP::ahrs().get_control_ground_speed_limit_ms();

    const float dt_s = _pos_control.get_dt_s();

    // Apply speed limit from LOITER_SPEED and EKF constraint
    float gnd_speed_limit_ms = MIN(_speed_max_ne_ms, ekfGndSpdLimit_ms);
    gnd_speed_limit_ms = MAX(gnd_speed_limit_ms, LOITER_SPEED_MIN_MS);

    // Determine the pilot acceleration limit (lean angle and LOIT_ACC_MAX_M). The drag below uses the
    // same value so that full stick still settles at the speed limit.
    float pilot_acceleration_max_mss = (gnd_speed_limit_ms / _speed_max_ne_ms) * get_pilot_accel_max_mss();

    // Check for invalid dt
    if (is_negative(dt_s)) {
        return;
    }

    // Integrate predicted acceleration
    Vector2f desired_vel_ne_ms = _pos_control.get_vel_desired_NED_ms().xy();

    // update the desired velocity using our predicted acceleration
    desired_vel_ne_ms += _predicted_accel_ne_mss * dt_s;

    Vector2f loiter_accel_brake_mss;
    float desired_speed_ms = desired_vel_ne_ms.length();
    if (!is_zero(desired_speed_ms)) {
        Vector2f desired_vel_norm = desired_vel_ne_ms / desired_speed_ms;

        // Apply drag: deceleration proportional to current velocity
        float drag_decel_mss = pilot_acceleration_max_mss * desired_speed_ms / gnd_speed_limit_ms;

        // Determine braking acceleration based on stick release and delay timer, stick must have been released for at least a full loop
        float loiter_brake_accel_mss = 0.0f;
        const bool jerk_shaped_stop = is_positive(_stop_jerk_max_msss);
        bool stopping = false;
        if ((AP_HAL::millis() - _brake_timer_ms) > MAX(_brake_delay_s, dt_s) * 1000.0) {
            if (jerk_shaped_stop) {
                // hold braking until a ramp at LOIT_STOP_JRK_M reaches zero braking at zero speed
                loiter_brake_accel_mss = MIN(safe_sqrt(2.0f * _stop_jerk_max_msss * desired_speed_ms), _brake_accel_max_mss.get());
                if (is_positive(dt_s)) {
                    loiter_brake_accel_mss = MIN(loiter_brake_accel_mss, desired_speed_ms / dt_s);
                }
                stopping = true;
            } else {
                const float brake_gain = _pos_control.NE_get_vel_pid().kP() * 0.5f;
                loiter_brake_accel_mss = constrain_float(sqrt_controller(desired_speed_ms, brake_gain, _brake_jerk_max_msss, dt_s), 0.0f, _brake_accel_max_mss);
            }
        }

        // Integrate jerk-limited brake acceleration. Braking builds at LOIT_BRK_JRK_M; as the vehicle comes to
        // rest it is released at LOIT_STOP_JRK_M, while pilot input still removes it at LOIT_BRK_JRK_M.
        const float release_jerk_msss = stopping ? _stop_jerk_max_msss.get() : _brake_jerk_max_msss.get();
        _brake_accel_mss += constrain_float(loiter_brake_accel_mss - _brake_accel_mss, -release_jerk_msss * dt_s, _brake_jerk_max_msss * dt_s);
        loiter_accel_brake_mss = desired_vel_norm * _brake_accel_mss;

        // Update desired speed based on braking and drag
        desired_speed_ms = MAX(desired_speed_ms - (drag_decel_mss + _brake_accel_mss) * dt_s, 0.0f);

        // a jerk-shaped stop has finished: clear any braking left over, so it cannot act against the next stick input
        if (stopping && is_zero(desired_speed_ms)) {
            _brake_accel_mss = 0.0f;
        }
        desired_vel_ne_ms = desired_vel_norm * desired_speed_ms;
    }

    // Apply braking acceleration to overall feed-forward acceleration
    _desired_accel_ne_mss -= loiter_accel_brake_mss;

    // Apply final velocity magnitude constraint
    float desired_vel_ms = desired_vel_ne_ms.length();
    if (desired_vel_ms > gnd_speed_limit_ms) {
        desired_vel_ne_ms = desired_vel_ne_ms * gnd_speed_limit_ms / desired_vel_ms;
    }

#if AP_AVOIDANCE_ENABLED && !APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    if (avoidance_on) {
        // Apply fence/obstacle avoidance adjustments (velocity only)
        // TODO: We need to also limit the _desired_accel_ne_mss
        AC_Avoid *_avoid = AP::ac_avoid();
        if (_avoid != nullptr) {
            Vector3f avoidance_vel_ned_cms{desired_vel_ne_ms.x * 100.0, desired_vel_ne_ms.y * 100.0, 0.0f};
            _avoid->adjust_velocity(avoidance_vel_ned_cms, _pos_control.NE_get_pos_p().kP(), _accel_max_ne_mss * 100.0, _pos_control.D_get_pos_p().kP(), _pos_control.D_get_max_accel_mss() * 100.0, dt_s);
            desired_vel_ne_ms = avoidance_vel_ned_cms.xy() * 0.01;
        }
    }
#endif // !APM_BUILD_ArduPlane

    // Retrieve current desired position
    Vector2p desired_pos_ned_m = _pos_control.get_pos_desired_NED_m().xy();

    // Integrate velocity to update desired position
    desired_pos_ned_m += (desired_vel_ne_ms * dt_s).topostype();

    // Send updated position, velocity, and acceleration to the position controller
    _pos_control.set_pos_vel_accel_NE_m(desired_pos_ned_m, desired_vel_ne_ms, _desired_accel_ne_mss);
}
