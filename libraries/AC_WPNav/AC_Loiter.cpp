#include <AP_HAL/AP_HAL.h>
#include "AC_Loiter.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AC_Avoidance/AC_Avoid.h>

extern const AP_HAL::HAL& hal;

#define LOITER_SPEED_DEFAULT_CM             1250.0  // Default horizontal loiter speed in cm/s.
#define LOITER_SPEED_MIN_CMS                20.0    // Minimum allowed horizontal loiter speed in cm/s.
#define LOITER_ACCEL_MAX_DEFAULT_CMSS       500.0   // Default maximum horizontal acceleration in loiter mode (cm/s²).
#define LOITER_BRAKE_ACCEL_DEFAULT_CMSS     250.0   // Default maximum braking acceleration when sticks are released (cm/s²).
#define LOITER_BRAKE_JERK_DEFAULT_CMSSS     500.0   // Default maximum jerk applied during braking transitions (cm/s³).
#define LOITER_BRAKE_START_DELAY_DEFAULT_S  1.0     // Delay (in seconds) before braking begins after sticks are released.
#define LOITER_VEL_CORRECTION_MAX_MS        2.0     // Maximum speed (in m/s) used for correcting position errors in loiter.
#define LOITER_POS_CORRECTION_MAX_M         2.0     // Maximum horizontal position error allowed before correction (m).
#define LOITER_ACTIVE_TIMEOUT_MS            200     // Loiter is considered active if updated within the past 200 ms.
#define LOITER_DEFAULT_OPTIONS              1       // Enable Coordinated Turn by default.

const AP_Param::GroupInfo AC_Loiter::var_info[] = {

    // @Param: ANG_MAX
    // @DisplayName: Loiter pilot angle max
    // @Description{Copter, Sub}: Loiter maximum pilot requested lean angle. Set to zero for 2/3 of PSC_ANGLE_MAX/ANGLE_MAX. The maximum vehicle lean angle is still limited by PSC_ANGLE_MAX/ANGLE_MAX
    // @Description: Loiter maximum pilot requested lean angle. Set to zero for 2/3 of Q_P_ANGLE_MAX/Q_ANGLE_MAX. The maximum vehicle lean angle is still limited by Q_P_ANGLE_MAX/Q_ANGLE_MAX
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ANG_MAX",  1, AC_Loiter, _angle_max_deg, 0.0f),

    // @Param: SPEED
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: cm/s
    // @Range: 20 3500
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED", 2, AC_Loiter, _speed_max_ne_cms, LOITER_SPEED_DEFAULT_CM),

    // @Param: ACC_MAX
    // @DisplayName: Loiter maximum correction acceleration
    // @Description: Loiter maximum correction acceleration in cm/s/s.  Higher values cause the copter to correct position errors more aggressively.
    // @Units: cm/s/s
    // @Range: 100 981
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ACC_MAX", 3, AC_Loiter, _accel_max_ne_cmss, LOITER_ACCEL_MAX_DEFAULT_CMSS),

    // @Param: BRK_ACCEL
    // @DisplayName: Loiter braking acceleration
    // @Description: Loiter braking acceleration in cm/s/s. Higher values stop the copter more quickly when the stick is centered.
    // @Units: cm/s/s
    // @Range: 25 250
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("BRK_ACCEL", 4, AC_Loiter, _brake_accel_max_cmss, LOITER_BRAKE_ACCEL_DEFAULT_CMSS),

    // @Param: BRK_JERK
    // @DisplayName: Loiter braking jerk
    // @Description: Loiter braking jerk in cm/s/s/s. Higher values will remove braking faster if the pilot moves the sticks during a braking maneuver.
    // @Units: cm/s/s/s
    // @Range: 500 5000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("BRK_JERK", 5, AC_Loiter, _brake_jerk_max_cmsss, LOITER_BRAKE_JERK_DEFAULT_CMSSS),

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

    // Configure speed/accel limits in meters using internal parameter (_accel_max_ne_cmss)
    _pos_control.NE_set_correction_speed_accel_m(LOITER_VEL_CORRECTION_MAX_MS, _accel_max_ne_cmss * 0.01);
    _pos_control.NE_set_pos_error_max_m(LOITER_POS_CORRECTION_MAX_M);

    // Reset controller state for stationary loiter
    _pos_control.NE_init_controller_stopping_point();

    // Zero out desired and predicted accelerations and angles
    _predicted_accel_ne_mss.zero();
    _desired_accel_ne_mss.zero();
    _predicted_euler_angle_rad.zero();
    _brake_accel_mss = 0.0f;

    // Set position target for stationary loiter
    _pos_control.set_pos_desired_NE_m(position_ne_m);
}

// Initializes the loiter controller using the current position and velocity.
// Updates feed-forward velocity, predicted acceleration, and resets control state.
void AC_Loiter::init_target()
{
    sanity_check_params();

    // Configure correction speed and acceleration limits (in m/s and m/s²)
    _pos_control.NE_set_correction_speed_accel_m(LOITER_VEL_CORRECTION_MAX_MS, _accel_max_ne_cmss * 0.01);
    _pos_control.NE_set_pos_error_max_m(LOITER_POS_CORRECTION_MAX_M);

    // Apply velocity smoothing: softly transitions target acceleration to zero
    _pos_control.NE_relax_velocity_controller();

    // Initialize prediction state using current acceleration and lean angles
    _predicted_accel_ne_mss = _pos_control.get_accel_target_NED_mss().xy();
    _predicted_euler_angle_rad.x = _pos_control.get_roll_rad();
    _predicted_euler_angle_rad.y = _pos_control.get_pitch_rad();
    _brake_accel_mss = 0.0f;
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
    const Vector3f desired_accel_ned_mss = _pos_control.lean_angles_rad_to_accel_NED_mss(desired_euler_rad);

    _desired_accel_ne_mss.x = desired_accel_ned_mss.x;
    _desired_accel_ne_mss.y = desired_accel_ned_mss.y;

    // Compute attitude error between desired and predicted lean angles
    Vector2f angle_error_euler_rad(wrap_PI(euler_roll_angle_rad - _predicted_euler_angle_rad.x), wrap_PI(euler_pitch_angle_rad - _predicted_euler_angle_rad.y));

    // Predict roll/pitch rate required to achieve target attitude
    _attitude_control.input_shaping_rate_predictor(angle_error_euler_rad, _predicted_euler_rate, dt_s);

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
// Internally converts to cm/s and clamps to a minimum of LOITER_SPEED_MIN_CMS.
void AC_Loiter::set_speed_max_NE_ms(float speed_max_ne_ms)
{
    // Convert to cm/s and apply minimum clamp
    _speed_max_ne_cms.set(MAX(speed_max_ne_ms * 100.0, LOITER_SPEED_MIN_CMS));
}

// Ensures internal parameters are within valid safety limits.
// Applies min/max constraints on speed and acceleration settings.
void AC_Loiter::sanity_check_params()
{
    // Enforce minimum loiter speed
    _speed_max_ne_cms.set(MAX(_speed_max_ne_cms, LOITER_SPEED_MIN_CMS));

    // Clamp horizontal accel to lean-angle-limited max (converted to cm/s²)
    _accel_max_ne_cmss.set(MIN(_accel_max_ne_cmss, GRAVITY_MSS * 100.0f * tanf(_attitude_control.lean_angle_max_rad())));
}

bool AC_Loiter::loiter_option_is_set(LoiterOption option) const {
    return (_options & int8_t(option)) != 0;
}

// Updates feed-forward velocity using pilot-requested acceleration and braking logic.
// - Applies drag and braking forces when sticks are released.
// - Velocity is adjusted for fence/avoidance if enabled.
// - Resulting velocity and acceleration are sent to the position controller.
void AC_Loiter::calc_desired_velocity(bool avoidance_on)
{
    float ekfGndSpdLimit_ms, ahrsControlScaleXY;
    // Query EKF-imposed horizontal ground speed limit (e.g. for optical flow)
    AP::ahrs().getControlLimits(ekfGndSpdLimit_ms, ahrsControlScaleXY);

    const float dt_s = _pos_control.get_dt_s();

    // Apply speed limit from LOITER_SPEED and EKF constraint
    float gnd_speed_limit_ms = MIN(_speed_max_ne_cms * 0.01, ekfGndSpdLimit_ms);
    gnd_speed_limit_ms = MAX(gnd_speed_limit_ms, LOITER_SPEED_MIN_CMS * 0.01);

    // Determine acceleration limit based on maximum allowed lean angle
    float pilot_acceleration_max_mss = angle_rad_to_accel_mss(get_angle_max_rad());

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

        // Determine braking acceleration based on stick release and delay timer
        float loiter_brake_accel_mss = 0.0f;
        if (_desired_accel_ne_mss.is_zero()) {
            if ((AP_HAL::millis() - _brake_timer_ms) > _brake_delay_s * 1000.0) {
                float brake_gain = _pos_control.NE_get_vel_pid().kP() * 0.5f;
                loiter_brake_accel_mss = constrain_float(sqrt_controller(desired_speed_ms, brake_gain, _brake_jerk_max_cmsss * 0.01, dt_s), 0.0f, _brake_accel_max_cmss * 0.01);
            }
        } else {
            loiter_brake_accel_mss = 0.0f;
            _brake_timer_ms = AP_HAL::millis();
        }

        // Integrate jerk-limited brake acceleration
        _brake_accel_mss += constrain_float(loiter_brake_accel_mss - _brake_accel_mss, -_brake_jerk_max_cmsss * 0.01 * dt_s, _brake_jerk_max_cmsss * 0.01 * dt_s);
        loiter_accel_brake_mss = desired_vel_norm * _brake_accel_mss;

        // Update desired speed based on braking and drag
        desired_speed_ms = MAX(desired_speed_ms - (drag_decel_mss + _brake_accel_mss) * dt_s, 0.0f);
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
            _avoid->adjust_velocity(avoidance_vel_ned_cms, _pos_control.NE_get_pos_p().kP(), _accel_max_ne_cmss, _pos_control.D_get_pos_p().kP(), _pos_control.D_get_max_accel_mss() * 100.0, dt_s);
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
