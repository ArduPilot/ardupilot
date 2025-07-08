#include "AC_Autorotation.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// Autorotation controller defaults
#define HEAD_SPEED_TARGET_RATIO 1.0 // Normalised target main rotor head speed

const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_Autorotation, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_P
    // @DisplayName: P gain for head speed controller
    // @Description: Increase value to increase sensitivity of head speed controller during autonomous autorotation.
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_hs, "HS_", 2, AC_Autorotation, AC_P),

    // @Param: HS_SET_PT
    // @DisplayName: Target Head Speed
    // @Description: The target head speed in RPM during autorotation. Start by setting to desired hover speed and tune from there as necessary.
    // @Units: RPM
    // @Range: 1000 2800
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HS_SET_PT", 3, AC_Autorotation, _param_head_speed_set_point, 1500),

    // @Param: FWD_SP_TARG
    // @DisplayName: Target Glide Body Frame Forward Speed
    // @Description: Target ground speed in cm/s for the autorotation controller to try and achieve/ maintain during the glide phase.
    // @Units: m/s
    // @Range: 8 20
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("FWD_SP_TARG", 4, AC_Autorotation, _param_target_speed_ms, 11),

    // @Param: COL_FILT_E
    // @DisplayName: Entry Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter. For the entry phase. Acts as a following trim.  Must be higher than AROT_COL_FILT_G.
    // @Units: Hz
    // @Range: 0.2 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("COL_FILT_E", 5, AC_Autorotation, _param_col_entry_cutoff_freq, 0.7),

    // @Param: COL_FILT_G
    // @DisplayName: Glide Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter. For the glide phase. Acts as a following trim.  Must be lower than AROT_COL_FILT_E.
    // @Units: Hz
    // @Range: 0.03 0.15
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("COL_FILT_G", 6, AC_Autorotation, _param_col_glide_cutoff_freq, 0.1),

    // @Param: XY_ACC_MAX
    // @DisplayName: Body Frame XY Acceleration Limit
    // @Description: Maximum body frame acceleration allowed in the in speed controller. This limit defines a circular constraint in accel. Minimum used is 0.5 m/s/s.
    // @Units: m/s/s
    // @Range: 0.5 8.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("XY_ACC_MAX", 7, AC_Autorotation, _param_accel_max_mss, 2.0),

    // @Param: HS_SENSOR
    // @DisplayName: Main Rotor RPM Sensor 
    // @Description: Allocate the RPM sensor instance to use for measuring head speed. RPM1 = 0.  RPM2 = 1.
    // @Units: s
    // @Range: 0.5 3
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("HS_SENSOR", 8, AC_Autorotation, _param_rpm_instance, 0),

    // @Param: FWD_P
    // @DisplayName: Forward Speed Controller P Gain
    // @Description: Converts the difference between desired forward speed and actual speed into an acceleration target that is passed to the pitch angle controller.
    // @Range: 1.000 8.000
    // @User: Standard

    // @Param: FWD_I
    // @DisplayName: Forward Speed Controller I Gain
    // @Description: Corrects long-term difference in desired velocity to a target acceleration.
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: FWD_IMAX
    // @DisplayName: Forward Speed Controller I Gain Maximum
    // @Description: Constrains the target acceleration that the I gain will output.
    // @Range: 1.000 8.000
    // @User: Standard

    // @Param: FWD_D
    // @DisplayName: Forward Speed Controller D Gain
    // @Description: Provides damping to velocity controller.
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: FWD_FF
    // @DisplayName: Forward Speed Controller Feed Forward Gain
    // @Description: Produces an output that is proportional to the magnitude of the target.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: FWD_FLTE
    // @DisplayName: Forward Speed Controller Error Filter
    // @Description: This filter low pass filter is applied to the input for P and I terms.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: FWD_FLTD
    // @DisplayName: Forward Speed Controller input filter for D term
    // @Description: This filter low pass filter is applied to the input for D terms.
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(_fwd_speed_pid, "FWD_", 9, AC_Autorotation, AC_PID_Basic),

    AP_GROUPEND
};

// Constructor
AC_Autorotation::AC_Autorotation(AP_MotorsHeli*& motors, AC_AttitudeControl*& att_crtl) :
    _motors_heli(motors),
    _attitude_control(att_crtl)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

void AC_Autorotation::init(void)
{
    // Initialisation of head speed controller
    // Set initial collective position to be the current collective position for smooth init
    const float collective_out = _motors_heli->get_throttle_out();

    // Reset feed forward filter
    col_trim_lpf.reset(collective_out);

    // Protect against divide by zero TODO: move this to an accessor function
    _param_head_speed_set_point.set(MAX(_param_head_speed_set_point, 500.0));

    // Reset the landed reason
    _landed_reason.min_speed = false;
    _landed_reason.land_col = false;
    _landed_reason.is_still = false;
}

// Functions and config that are only to be done once at the beginning of the entry
void AC_Autorotation::init_entry(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AROT: Entry Phase");

    // Target head speed is set to rpm at initiation to prevent steps in controller
    if (!get_norm_head_speed(_target_head_speed)) {
        // Cannot get a valid RPM sensor reading so we default to not slewing the head speed target
        _target_head_speed = HEAD_SPEED_TARGET_RATIO;
    }

    // The rate to move the head speed from the current measurement to the target
    _hs_accel = (HEAD_SPEED_TARGET_RATIO - _target_head_speed) / (float(entry_time_ms)*1e-3);

    // Set collective following trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_entry_cutoff_freq.get());

    // Set collective low-pass cut off filter at 2 Hz
    _motors_heli->set_throttle_filter_cutoff(2.0);

    // Set speed target to maintain the current speed whilst we enter the autorotation
    _desired_vel_ms = _param_target_speed_ms.get();
    _target_vel_ms = get_speed_forward_ms();

    // Reset I term of velocity PID
    _fwd_speed_pid.reset_filter();
    _fwd_speed_pid.set_integrator(0.0);
}

// The entry controller just a special case of the glide controller with head speed target slewing
void AC_Autorotation::run_entry(float pilot_accel_norm)
{
    // Slowly change the target head speed until the target head speed matches the parameter defined value
    float head_speed_norm;
    if (!get_norm_head_speed(head_speed_norm)) {
        // RPM sensor is bad, so we don't attempt to slew the head speed target as we do not know what head speed actually is
        // The collective output handling of the rpm sensor failure is handled later in the head speed controller 
         head_speed_norm = HEAD_SPEED_TARGET_RATIO;
    }

    // Slew the head speed target from the initial condition to the target head speed ratio for the glide
    const float max_change = _hs_accel * _dt;
    _target_head_speed = constrain_float(HEAD_SPEED_TARGET_RATIO, _target_head_speed - max_change, _target_head_speed + max_change);

    run_glide(pilot_accel_norm);
}

// Functions and config that are only to be done once at the beginning of the glide
void AC_Autorotation::init_glide(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AROT: Glide Phase");

    // Set collective following trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_glide_cutoff_freq.get());

    // Ensure target head speed is set to setpoint, in case it didn't reach the target during entry
    _target_head_speed = HEAD_SPEED_TARGET_RATIO;

    // Ensure desired forward speed target is set to param value
    _desired_vel_ms = _param_target_speed_ms.get();
}

// Maintain head speed and forward speed as we glide to the ground
void AC_Autorotation::run_glide(float pilot_accel_norm)
{
    update_headspeed_controller();

    update_forward_speed_controller(pilot_accel_norm);
}

void AC_Autorotation::update_headspeed_controller(void)
{
    // Get current rpm and update healthy signal counters
    float head_speed_norm;
    if (!get_norm_head_speed(head_speed_norm)) {
        // RPM sensor is bad, set collective to angle of -2 deg and hope for the best
         _motors_heli->set_coll_from_ang(-2.0);
         return;
    }

    // Calculate the head speed error.
    _head_speed_error = head_speed_norm - _target_head_speed;

    _p_term_hs = _p_hs.get_p(_head_speed_error);

    // Adjusting collective trim using feed forward (not yet been updated, so this value is the previous time steps collective position)
    _ff_term_hs = col_trim_lpf.apply(_motors_heli->get_throttle(), _dt);

    // Calculate collective position to be set
    const float collective_out = constrain_value((_p_term_hs + _ff_term_hs), 0.0f, 1.0f);

    // Send collective to setting to motors output library
    _motors_heli->set_throttle(collective_out);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: ARHS
    // @Vehicles: Copter
    // @Description: helicopter AutoRotation Head Speed (ARHS) controller information
    // @Field: TimeUS: Time since system startup
    // @Field: Tar: Normalised target head speed
    // @Field: Act: Normalised measured head speed
    // @Field: Err: Head speed controller error
    // @Field: P: P-term for head speed controller response
    // @Field: FF: FF-term for head speed controller response

    // Write to data flash log
    AP::logger().WriteStreaming("ARHS",
                                "TimeUS,Tar,Act,Err,P,FF",
                                "s-----",
                                "F00000",
                                "Qfffff",
                                AP_HAL::micros64(),
                                _target_head_speed,
                                head_speed_norm,
                                _head_speed_error,
                                _p_term_hs,
                                _ff_term_hs);
#endif
}

// Get measured head speed and normalise by head speed set point. Returns false if a valid rpm measurement cannot be obtained
bool AC_Autorotation::get_norm_head_speed(float& norm_rpm) const
{
    // Assuming zero rpm is safer as it will drive collective in the direction of increasing head speed
    float current_rpm = 0.0;

#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // Checking to ensure no nullptr, we do have a pre-arm check for this so it will be very bad if RPM has gone away
    if (rpm == nullptr) {
        return false;
    }

    // Check RPM sensor is returning a healthy status
    if (!rpm->get_rpm(_param_rpm_instance.get(), current_rpm)) {
        return false;
    }
#endif

    // Protect against div by zeros later in the code
    float head_speed_set_point = MAX(1.0, _param_head_speed_set_point.get());

    // Normalize the RPM by the setpoint
    norm_rpm = current_rpm/head_speed_set_point;
    return true;
}

// Update speed controller
void AC_Autorotation::update_forward_speed_controller(float pilot_accel_norm)
{
    // Limiting the desired velocity based on the max acceleration limit to get an update target
    const float min_vel_ms = _target_vel_ms - get_accel_max_mss() * _dt;
    const float max_vel_ms = _target_vel_ms + get_accel_max_mss() * _dt;
    _target_vel_ms = constrain_float(_desired_vel_ms, min_vel_ms, max_vel_ms); // (m/s)

    // Calculate acceleration target
    const float fwd_accel_target_mss  = _fwd_speed_pid.update_all(_target_vel_ms, get_speed_forward_ms(), _dt, _limit_accel); // (m/s/s)

    // Build the body frame XY accel vector.
    // Pilot can request as much as 1/2 of the max accel laterally to perform a turn.
    // We only allow up to half as we need to prioritize building/maintaining airspeed.
    Vector2f bf_accel_target_mss = {fwd_accel_target_mss, pilot_accel_norm * get_accel_max_mss() * 0.5};

    // Ensure we do not exceed the accel limit
    _limit_accel = bf_accel_target_mss.limit_length(get_accel_max_mss());

    // Calculate roll and pitch targets from angles, negative accel for negative pitch (pitch forward)
    Vector2f angle_target_rad = { accel_mss_to_angle_rad(-bf_accel_target_mss.x), // Pitch
                                  accel_mss_to_angle_rad(bf_accel_target_mss.y)}; // Roll

    // Ensure that the requested angles do not exceed angle max
    _limit_accel |= angle_target_rad.limit_length(_attitude_control->lean_angle_max_rad());

    // we may have scaled the lateral accel in the angle limit scaling, so we need to
    // back calculate the resulting accel from this constrained angle for the yaw rate calc
    const float bf_lat_accel_target_mss = angle_rad_to_accel_mss(angle_target_rad.y);

    // Calc yaw rate from desired body-frame accels
    // this seems suspiciously simple, but it is correct
    // accel = r * w^2, r = radius and w = angular rate
    // radius can be calculated as the distance traveled in the time it takes to do 360 deg
    // One rotation takes: (2*pi)/w seconds
    // Distance traveled in that time: (vel*2*pi)/w
    // radius for that distance: ((vel*2*pi)/w) / (2*pi)
    // r = vel / w
    // accel = (vel / w) * w^2
    // accel = vel * w
    // w = accel / vel
    float yaw_rate_rads = 0.0;
    if (!is_zero(_target_vel_ms)) {
        yaw_rate_rads = bf_lat_accel_target_mss / _target_vel_ms;
    }

    // Output to attitude controller
    _attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(angle_target_rad.y, angle_target_rad.x, yaw_rate_rads);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: ARSC
    // @Vehicles: Copter
    // @Description: Helicopter AutoRotation Speed Controller (ARSC) information 
    // @Field: TimeUS: Time since system startup
    // @Field: Des: Desired forward velocity
    // @Field: Tar: Target forward velocity
    // @Field: Act: Measured forward velocity
    // @Field: P: Velocity to acceleration P-term component
    // @Field: I: Velocity to acceleration I-term component
    // @Field: D: Velocity to acceleration D-term component
    // @Field: FF: Velocity to acceleration feed forward component
    // @Field: Lim: Accel limit flag
    // @Field: FA: Forward acceleration target
    // @Field: LA: Lateral acceleration target

    const AP_PIDInfo& pid_info = _fwd_speed_pid.get_pid_info();
    AP::logger().WriteStreaming("ARSC",
                                "TimeUS,Des,Tar,Act,P,I,D,FF,Lim,FA,LA",
                                "snnn-----oo",
                                "F0000000-00",
                                "QfffffffBff",
                                AP_HAL::micros64(),
                                _desired_vel_ms,
                                pid_info.target,
                                pid_info.actual,
                                pid_info.P,
                                pid_info.I,
                                pid_info.D,
                                pid_info.FF,
                                uint8_t(_limit_accel),
                                bf_accel_target_mss.x,
                                bf_accel_target_mss.y);
#endif
}

// smoothly zero velocity and accel
void AC_Autorotation::run_landed(void)
{
    _desired_vel_ms *= 0.95;
    update_forward_speed_controller(0.0);
}

// Determine the body frame forward speed
float AC_Autorotation::get_speed_forward_ms(void) const
{
    Vector3f vel_NED = {0,0,0};
    const AP_AHRS &ahrs = AP::ahrs();
    if (ahrs.get_velocity_NED(vel_NED)) {
        vel_NED = ahrs.earth_to_body(vel_NED);
    }
    // TODO: need to improve the handling of the velocity NED not ok case
    return vel_NED.x;
}

#if HAL_LOGGING_ENABLED
// Logging of lower rate autorotation specific variables. This is meant for stuff that
// doesn't need a high rate, e.g. controller variables that are need for tuning.
void AC_Autorotation::log_write_autorotation(void) const
{
    // enum class for bitmask documentation in logging
    enum class AC_Autorotation_Landed_Reason : uint8_t {
        LOW_SPEED = 1<<0, // true if below 1 m/s
        LAND_COL  = 1<<1, // true if collective below land col min
        IS_STILL  = 1<<2, // passes inertial nav is_still() check
    };

    uint8_t reason = 0;
    if (_landed_reason.min_speed) {
        reason |= uint8_t(AC_Autorotation_Landed_Reason::LOW_SPEED);
    }
    if (_landed_reason.land_col) {
        reason |= uint8_t(AC_Autorotation_Landed_Reason::LAND_COL);
    }
    if (_landed_reason.is_still) {
        reason |= uint8_t(AC_Autorotation_Landed_Reason::IS_STILL);
    }

    // @LoggerMessage: AROT
    // @Vehicles: Copter
    // @Description: Helicopter AutoROTation (AROT) information
    // @Field: TimeUS: Time since system startup
    // @Field: LR: Landed Reason state flags
    // @FieldBitmaskEnum: LR: AC_Autorotation_Landed_Reason

    // Write to data flash log
    AP::logger().WriteStreaming("AROT",
                                "TimeUS,LR",
                                "s-",
                                "F-",
                                "QB",
                                AP_HAL::micros64(),
                                reason);
}
#endif  // HAL_LOGGING_ENABLED

// Arming checks for autorotation, mostly checking for miss-configurations
bool AC_Autorotation::arming_checks(size_t buflen, char *buffer) const
{
    if (!enabled()) {
        // Don't run arming checks if not enabled
        return true;
    }

    // Check for correct RPM sensor config
#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // Get current rpm, checking to ensure no nullptr
    if (rpm == nullptr) {
        hal.util->snprintf(buffer, buflen, "Can't access RPM");
        return false;
    }

    // Sanity check that the designated rpm sensor instance is there
    if (_param_rpm_instance.get() < 0) {
        hal.util->snprintf(buffer, buflen, "RPM instance <0");
        return false;
    }

    if (!rpm->enabled(_param_rpm_instance.get())) {
        hal.util->snprintf(buffer, buflen, "RPM%i not enabled", _param_rpm_instance.get()+1);
        return false;
    }
#endif

    // Check that heli motors is configured for autorotation
    if (!_motors_heli->rsc_autorotation_enabled()) {
        hal.util->snprintf(buffer, buflen, "H_RSC_AROT_* not configured");
        return false;
    }

    return true;
}

// Check if we believe we have landed. We need the landed state to zero all
// controls and make sure that the copter landing detector will trip
bool AC_Autorotation::check_landed(void)
{
    // minimum speed (m/s) used for "is moving" check
    const float min_moving_speed = 1.0;

    Vector3f velocity;
    const AP_AHRS &ahrs = AP::ahrs();
    _landed_reason.min_speed = ahrs.get_velocity_NED(velocity) && (velocity.length() < min_moving_speed);
    _landed_reason.land_col = _motors_heli->get_below_land_min_coll();
    _landed_reason.is_still = AP::ins().is_still();

    return _landed_reason.min_speed && _landed_reason.land_col && _landed_reason.is_still;
}

// Dynamically update time step used in autorotation controllers
void AC_Autorotation::set_dt(float delta_sec)
{
    if (is_positive(delta_sec)) {
        _dt = delta_sec;
        return;
    }
    _dt = 2.5e-3; // Assume 400 Hz
}
