#include <AP_HAL/AP_HAL.h>
#include "AC_PosControl.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Motors/AP_Motors.h>    // motors library
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Scheduler/AP_Scheduler.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // default gains for Plane
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_VEL_Z_IMAX                 1000.0f // vertical velocity controller IMAX gain default
 # define POSCONTROL_VEL_Z_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_VEL_Z_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_ACC_Z_P                    0.3f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              10.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.02f   // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   0.5f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   0.7f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.35f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.17f   // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
 // default gains for Sub
 # define POSCONTROL_POS_Z_P                    3.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
 # define POSCONTROL_VEL_Z_IMAX                 1000.0f // vertical velocity controller IMAX gain default
 # define POSCONTROL_VEL_Z_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_VEL_Z_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   1.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.5f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.0f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#else
 // default gains for Copter / TradHeli
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_VEL_Z_IMAX                 1000.0f // vertical velocity controller IMAX gain default
 # define POSCONTROL_VEL_Z_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_VEL_Z_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   2.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   1.0f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.5f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#endif

// vibration compensation gains
#define POSCONTROL_VIBE_COMP_P_GAIN 0.250f
#define POSCONTROL_VIBE_COMP_I_GAIN 0.125f

// velocity offset targets timeout if not updated within 3 seconds
#define POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS 3000

AC_PosControl *AC_PosControl::_singleton;

const AP_Param::GroupInfo AC_PosControl::var_info[] = {
    // 0 was used for HOVER

    // @Param: _ACC_XY_FILT
    // @DisplayName: XY Acceleration filter cutoff frequency
    // @Description: Lower values will slow the response of the navigation controller and reduce twitchiness
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_u, "_POSZ_", 2, AC_PosControl, AC_P_1D),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard

    // @Param: _VELZ_I
    // @DisplayName: Velocity (vertical) controller I gain
    // @Description: Velocity (vertical) controller I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VELZ_IMAX
    // @DisplayName: Velocity (vertical) controller I gain maximum
    // @Description: Velocity (vertical) controller I gain maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 1.000 8.000
    // @User: Standard

    // @Param: _VELZ_D
    // @DisplayName: Velocity (vertical) controller D gain
    // @Description: Velocity (vertical) controller D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _VELZ_FF
    // @DisplayName: Velocity (vertical) controller Feed Forward gain
    // @Description: Velocity (vertical) controller Feed Forward gain.  Produces an output that is proportional to the magnitude of the target
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VELZ_FLTE
    // @DisplayName: Velocity (vertical) error filter
    // @Description: Velocity (vertical) error filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VELZ_FLTD
    // @DisplayName: Velocity (vertical) input filter for D term
    // @Description: Velocity (vertical) input filter for D term.  This filter (in Hz) is applied to the input for D terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_u, "_VELZ_", 3, AC_PosControl, AC_PID_Basic),

    // @Param: _ACCZ_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.200 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCZ_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCZ_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCZ_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCZ_FF
    // @DisplayName: Acceleration (vertical) controller feed forward
    // @Description: Acceleration (vertical) controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: _ACCZ_FLTT
    // @DisplayName: Acceleration (vertical) controller target frequency in Hz
    // @Description: Acceleration (vertical) controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_FLTE
    // @DisplayName: Acceleration (vertical) controller error frequency in Hz
    // @Description: Acceleration (vertical) controller error frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_FLTD
    // @DisplayName: Acceleration (vertical) controller derivative frequency in Hz
    // @Description: Acceleration (vertical) controller derivative frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_SMAX
    // @DisplayName: Accel (vertical) slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: _ACCZ_PDMX
    // @DisplayName: Acceleration (vertical) controller PD sum maximum
    // @Description: Acceleration (vertical) controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1000
    // @Units: d%

    // @Param: _ACCZ_D_FF
    // @DisplayName: Accel (vertical) Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: _ACCZ_NTF
    // @DisplayName: Accel (vertical) Target notch filter index
    // @Description: Accel (vertical) Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: _ACCZ_NEF
    // @DisplayName: Accel (vertical) Error notch filter index
    // @Description: Accel (vertical) Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_accel_u, "_ACCZ_", 4, AC_PosControl, AC_PID),

    // @Param: _POSXY_P
    // @DisplayName: Position (horizontal) controller P gain
    // @Description: Position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_ne, "_POSXY_", 5, AC_PosControl, AC_P_2D),

    // @Param: _VELXY_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired and actual velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _VELXY_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VELXY_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _VELXY_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: _VELXY_FLTE
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VELXY_FLTD
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for D term
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VELXY_FF
    // @DisplayName: Velocity (horizontal) feed forward gain
    // @Description: Velocity (horizontal) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_ne, "_VELXY_", 6, AC_PosControl, AC_PID_2D),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_ANGLE_MAX", 7, AC_PosControl, _lean_angle_max_deg, 0.0f),

    // IDs 8,9 used for _TC_XY and _TC_Z in beta release candidate

    // @Param: _JERK_XY
    // @DisplayName: Jerk limit for the horizontal kinematic input shaping
    // @Description: Jerk limit of the horizontal kinematic path generation used to determine how quickly the aircraft varies the acceleration target
    // @Units: m/s/s/s
    // @Range: 1 20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_JERK_XY", 10, AC_PosControl, _shaping_jerk_ne_msss, POSCONTROL_JERK_NE),

    // @Param: _JERK_Z
    // @DisplayName: Jerk limit for the vertical kinematic input shaping
    // @Description: Jerk limit of the vertical kinematic path generation used to determine how quickly the aircraft varies the acceleration target
    // @Units: m/s/s/s
    // @Range: 5 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_JERK_Z", 11, AC_PosControl, _shaping_jerk_u_msss, POSCONTROL_JERK_U),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl::AC_PosControl(AP_AHRS_View& ahrs, const AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _motors(motors),
    _attitude_control(attitude_control),
    _p_pos_ne(POSCONTROL_POS_XY_P),
    _p_pos_u(POSCONTROL_POS_Z_P),
    _pid_vel_ne(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, 0.0f, POSCONTROL_VEL_XY_IMAX, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ),
    _pid_vel_u(POSCONTROL_VEL_Z_P, 0.0f, 0.0f, 0.0f, POSCONTROL_VEL_Z_IMAX, POSCONTROL_VEL_Z_FILT_HZ, POSCONTROL_VEL_Z_FILT_D_HZ),
    _pid_accel_u(POSCONTROL_ACC_Z_P, POSCONTROL_ACC_Z_I, POSCONTROL_ACC_Z_D, 0.0f, POSCONTROL_ACC_Z_IMAX, 0.0f, POSCONTROL_ACC_Z_FILT_HZ, 0.0f),
    _vel_max_ne_cms(POSCONTROL_SPEED),
    _vel_max_up_cms(POSCONTROL_SPEED_UP),
    _vel_max_down_cms(POSCONTROL_SPEED_DOWN),
    _accel_max_ne_cmss(POSCONTROL_ACCEL_NE),
    _accel_max_u_cmss(POSCONTROL_ACCEL_U),
    _jerk_max_ne_cmsss(POSCONTROL_JERK_NE * 100.0),
    _jerk_max_u_cmsss(POSCONTROL_JERK_U * 100.0)
{
    AP_Param::setup_object_defaults(this, var_info);

    _singleton = this;
}


///
/// 3D position shaper
///

/// input_pos_NEU_cm - computes a jerk-limited trajectory from the current NEU position, velocity, and acceleration to a new position input (in cm).
/// This function updates the desired acceleration using a smooth kinematic path constrained by acceleration and jerk limits.
///     The jerk limit defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
///     The jerk limit also defines the time taken to achieve the maximum acceleration.
///     The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
void AC_PosControl::input_pos_NEU_cm(const Vector3p& pos_neu_cm, float pos_terrain_target_u_cm, float terrain_buffer_cm)
{
    // Terrain following velocity scalar must be calculated before we remove the position offset
    const float offset_u_scalar = pos_terrain_U_scaler(pos_terrain_target_u_cm, terrain_buffer_cm);
    set_pos_terrain_target_U_cm(pos_terrain_target_u_cm);

    // calculated increased maximum acceleration and jerk if over speed
    const float overspeed_gain = calculate_overspeed_gain();
    const float accel_max_u_cmss = _accel_max_u_cmss * overspeed_gain;
    const float jerk_max_u_cmsss = _jerk_max_u_cmsss * overspeed_gain;

    update_pos_vel_accel_xy(_pos_desired_neu_cm.xy(), _vel_desired_neu_cms.xy(), _accel_desired_neu_cmss.xy(), _dt, _limit_vector.xy(), _p_pos_ne.get_error(), _pid_vel_ne.get_error());

    // adjust desired altitude if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_neu_cm.z, _vel_desired_neu_cms.z, _accel_desired_neu_cmss.z, _dt, _limit_vector.z, _p_pos_u.get_error(), _pid_vel_u.get_error());

    // calculate the horizontal and vertical velocity limits to travel directly to the destination defined by pos_ne_cm
    float vel_max_ne_cms = 0.0f;
    float vel_max_u_cms = 0.0f;
    Vector3f travel_dir_unit = (pos_neu_cm - _pos_desired_neu_cm).tofloat();
    if (is_positive(travel_dir_unit.length_squared()) ) {
        travel_dir_unit.normalize();
        float travel_dir_unit_ne_length = travel_dir_unit.xy().length();

        float vel_max_cms = kinematic_limit(travel_dir_unit, _vel_max_ne_cms, _vel_max_up_cms, _vel_max_down_cms);
        vel_max_ne_cms = vel_max_cms * travel_dir_unit_ne_length;
        vel_max_u_cms = fabsf(vel_max_cms * travel_dir_unit.z);
    }

    // reduce speed if we are reaching the edge of our vertical buffer
    vel_max_ne_cms *= offset_u_scalar;

    Vector2f vel_ne_cms;
    Vector2f accel_ne_cmss;
    shape_pos_vel_accel_xy(pos_neu_cm.xy(), vel_ne_cms, accel_ne_cmss, _pos_desired_neu_cm.xy(), _vel_desired_neu_cms.xy(), _accel_desired_neu_cmss.xy(),
                           vel_max_ne_cms, _accel_max_ne_cmss, _jerk_max_ne_cmsss, _dt, false);

    float pos_u_cm = pos_neu_cm.z;
    shape_pos_vel_accel(pos_u_cm, 0, 0,
                        _pos_desired_neu_cm.z, _vel_desired_neu_cms.z, _accel_desired_neu_cmss.z,
                        -vel_max_u_cms, vel_max_u_cms,
                        -constrain_float(accel_max_u_cmss, 0.0f, 750.0f), accel_max_u_cmss,
                        jerk_max_u_cmsss, _dt, false);
}


/// pos_terrain_U_scaler - computes a scaling factor applied to horizontal velocity limits to ensure the vertical position controller remains within its terrain buffer.
float AC_PosControl::pos_terrain_U_scaler(float pos_terrain_u_cm, float pos_terrain_u_buffer_cm) const
{
    if (is_zero(pos_terrain_u_buffer_cm)) {
        return 1.0;
    }
    float pos_offset_error_u_cm = _pos_estimate_neu_cm.z - (_pos_target_neu_cm.z + (pos_terrain_u_cm - _pos_terrain_u_cm));
    return constrain_float((1.0 - (fabsf(pos_offset_error_u_cm) - 0.5 * pos_terrain_u_buffer_cm) / (0.5 * pos_terrain_u_buffer_cm)), 0.01, 1.0);
}

///
/// Lateral position controller
///

/// set_max_speed_accel_NE_cm - set the maximum horizontal speed in cm/s and acceleration in cm/s/s
///     This function only needs to be called if using the kinematic shaping.
///     This can be done at any time as changes in these parameters are handled smoothly
///     by the kinematic shaping.
void AC_PosControl::set_max_speed_accel_NE_cm(float speed_ne_cms, float accel_ne_cmss)
{
    _vel_max_ne_cms = speed_ne_cms;
    _accel_max_ne_cmss = accel_ne_cmss;

    // ensure the horizontal jerk is less than the vehicle is capable of
    const float jerk_max_cmsss = MIN(_attitude_control.get_ang_vel_roll_max_rads(), _attitude_control.get_ang_vel_pitch_max_rads()) * GRAVITY_MSS * 100.0;
    const float snap_max_cmssss = MIN(_attitude_control.get_accel_roll_max_radss(), _attitude_control.get_accel_pitch_max_radss()) * GRAVITY_MSS * 100.0;

    // get specified jerk limit
    _jerk_max_ne_cmsss = _shaping_jerk_ne_msss * 100.0;

    // limit maximum jerk based on maximum angular rate
    if (is_positive(jerk_max_cmsss) && _attitude_control.get_bf_feedforward()) {
        _jerk_max_ne_cmsss = MIN(_jerk_max_ne_cmsss, jerk_max_cmsss);
    }

    // limit maximum jerk to maximum possible average jerk based on angular acceleration
    if (is_positive(snap_max_cmssss) && _attitude_control.get_bf_feedforward()) {
        _jerk_max_ne_cmsss = MIN(0.5 * safe_sqrt(_accel_max_ne_cmss * snap_max_cmssss), _jerk_max_ne_cmsss);
    }
}

/// set_correction_speed_accel_xy_cm - set the position controller correction velocity and acceleration limit
///     This should be done only during initialisation to avoid discontinuities
void AC_PosControl::set_correction_speed_accel_NE_cm(float speed_ne_cms, float accel_ne_cmss)
{
    _p_pos_ne.set_limits(speed_ne_cms, accel_ne_cmss, 0.0f);
}

/// init_NE_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
///     This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
///     The starting position can be retrieved by getting the position target using get_pos_desired_NEU_cm() after calling this function.
void AC_PosControl::init_NE_controller_stopping_point()
{
    init_NE_controller();

    get_stopping_point_NE_cm(_pos_desired_neu_cm.xy());
    _pos_target_neu_cm.xy() = _pos_desired_neu_cm.xy() + _pos_offset_neu_cm.xy();
    _vel_desired_neu_cms.xy().zero();
    _accel_desired_neu_cmss.xy().zero();
}

// relax_velocity_controller_NE - initialise the position controller to the current position and velocity with decaying acceleration.
///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
void AC_PosControl::relax_velocity_controller_NE()
{
    // decay acceleration and therefore current attitude target to zero
    // this will be reset by init_NE_controller() if !is_active_NE()
    if (is_positive(_dt)) {
        float decay = 1.0 - _dt / (_dt + POSCONTROL_RELAX_TC);
        _accel_target_neu_cmss.xy() *= decay;
    }

    init_NE_controller();
}

/// Reduces controller response for landing by decaying position error and preventing I-term windup.
void AC_PosControl::soften_for_landing_NE()
{
    // decay position error to zero
    if (is_positive(_dt)) {
        _pos_target_neu_cm.xy() += (_pos_estimate_neu_cm.xy() - _pos_target_neu_cm.xy()) * (_dt / (_dt + POSCONTROL_RELAX_TC));
        _pos_desired_neu_cm.xy() = _pos_target_neu_cm.xy() - _pos_offset_neu_cm.xy();
    }

    // Prevent I term build up in xy velocity controller.
    // Note that this flag is reset on each loop in update_NE_controller()
    set_externally_limited_NE();
}

/// init_NE_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
void AC_PosControl::init_NE_controller()
{
    // initialise offsets to target offsets and ensure offset targets are zero if they have not been updated.
    init_offsets_NE();
    
    // set roll, pitch lean angle targets to current attitude
    const Vector3f &att_target_euler_rad = _attitude_control.get_att_target_euler_rad();
    _roll_target_rad = att_target_euler_rad.x;
    _pitch_target_rad = att_target_euler_rad.y;
    _yaw_target_rad = att_target_euler_rad.z; // todo: this should be thrust vector heading, not yaw.
    _yaw_rate_target_rads = 0.0f;
    _angle_max_override_rad = 0.0;

    _pos_target_neu_cm.xy() = _pos_estimate_neu_cm.xy();
    _pos_desired_neu_cm.xy() = _pos_target_neu_cm.xy() - _pos_offset_neu_cm.xy();

    _vel_target_neu_cms.xy() = _vel_estimate_neu_cms.xy();
    _vel_desired_neu_cms.xy() = _vel_target_neu_cms.xy() - _vel_offset_neu_cms.xy();

    // Set desired acceleration to zero because raw acceleration is prone to noise
    _accel_desired_neu_cmss.xy().zero();

    if (!is_active_NE()) {
        lean_angles_to_accel_NE_cmss(_accel_target_neu_cmss.x, _accel_target_neu_cmss.y);
    }

    // limit acceleration using maximum lean angles
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_cmss = angle_rad_to_accel_mss(angle_max_rad) * 100.0;
    _accel_target_neu_cmss.xy().limit_length(accel_max_cmss);

    // initialise I terms from lean angles
    _pid_vel_ne.reset_filter();
    // initialise the I term to (_accel_target_neu_cmss - _accel_desired_neu_cmss)
    // _accel_desired_neu_cmss is zero and can be removed from the equation
    _pid_vel_ne.set_integrator(_accel_target_neu_cmss.xy() - _vel_target_neu_cms.xy() * _pid_vel_ne.ff());

    // initialise ekf xy reset handler
    init_ekf_NE_reset();

    // initialise z_controller time out
    _last_update_ne_ticks = AP::scheduler().ticks32();
}

/// input_accel_NE_cm - calculate a jerk limited path from the current position, velocity and acceleration to an input acceleration.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_ne.
///     The jerk limit defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
///     The jerk limit also defines the time taken to achieve the maximum acceleration.
void AC_PosControl::input_accel_NE_cm(const Vector3f& accel_neu_cmss)
{
    update_pos_vel_accel_xy(_pos_desired_neu_cm.xy(), _vel_desired_neu_cms.xy(), _accel_desired_neu_cmss.xy(), _dt, _limit_vector.xy(), _p_pos_ne.get_error(), _pid_vel_ne.get_error());
    shape_accel_xy(accel_neu_cmss.xy(), _accel_desired_neu_cmss.xy(), _jerk_max_ne_cmsss, _dt);
}

/// input_vel_accel_NE_cm - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity and acceleration.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_ne.
///     The function modifies vel_ne_cms to follow the kinematic trajectory toward accel_cmss.
///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
void AC_PosControl::input_vel_accel_NE_cm(Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output)
{
    update_pos_vel_accel_xy(_pos_desired_neu_cm.xy(), _vel_desired_neu_cms.xy(), _accel_desired_neu_cmss.xy(), _dt, _limit_vector.xy(), _p_pos_ne.get_error(), _pid_vel_ne.get_error());

    shape_vel_accel_xy(vel_ne_cms, accel_ne_cmss, _vel_desired_neu_cms.xy(), _accel_desired_neu_cmss.xy(),
        _accel_max_ne_cmss, _jerk_max_ne_cmsss, _dt, limit_output);

    update_vel_accel_xy(vel_ne_cms, accel_ne_cmss, _dt, Vector2f(), Vector2f());
}

/// input_pos_vel_accel_NE_cm - calculate a jerk limited path from the current position, velocity and acceleration to an input position velocity and acceleration.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_ne.
///     The function modifies pos_ne_cm and vel_ne_cms to follow the jerk-limited trajectory defined by accel_ne_cmss.
///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
void AC_PosControl::input_pos_vel_accel_NE_cm(Vector2p& pos_ne_cm, Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output)
{
    update_pos_vel_accel_xy(_pos_desired_neu_cm.xy(), _vel_desired_neu_cms.xy(), _accel_desired_neu_cmss.xy(), _dt, _limit_vector.xy(), _p_pos_ne.get_error(), _pid_vel_ne.get_error());

    shape_pos_vel_accel_xy(pos_ne_cm, vel_ne_cms, accel_ne_cmss, _pos_desired_neu_cm.xy(), _vel_desired_neu_cms.xy(), _accel_desired_neu_cmss.xy(),
                           _vel_max_ne_cms, _accel_max_ne_cmss, _jerk_max_ne_cmsss, _dt, limit_output);

    update_pos_vel_accel_xy(pos_ne_cm, vel_ne_cms, accel_ne_cmss, _dt, Vector2f(), Vector2f(), Vector2f());
}

/// update the horizontal position and velocity offsets
/// this moves the offsets (e.g _pos_offset_neu_cm, _vel_offset_neu_cms, _accel_offset_neu_cmss) towards the targets (e.g. _pos_offset_target_neu_cm, _vel_offset_target_neu_cms, _accel_offset_target_neu_cmss)
void AC_PosControl::update_offsets_NE()
{
    // check for offset target timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_ne_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        _pos_offset_target_neu_cm.xy().zero();
        _vel_offset_target_neu_cms.xy().zero();
        _accel_offset_target_neu_cmss.xy().zero();
    }

    // update position, velocity, acceleration offsets for this iteration
    update_pos_vel_accel_xy(_pos_offset_target_neu_cm.xy(), _vel_offset_target_neu_cms.xy(), _accel_offset_target_neu_cmss.xy(), _dt, Vector2f(), Vector2f(), Vector2f());
    update_pos_vel_accel_xy(_pos_offset_neu_cm.xy(), _vel_offset_neu_cms.xy(), _accel_offset_neu_cmss.xy(), _dt, _limit_vector.xy(), _p_pos_ne.get_error(), _pid_vel_ne.get_error());

    // input shape horizontal position, velocity and acceleration offsets
    shape_pos_vel_accel_xy(_pos_offset_target_neu_cm.xy(), _vel_offset_target_neu_cms.xy(), _accel_offset_target_neu_cmss.xy(),
                            _pos_offset_neu_cm.xy(), _vel_offset_neu_cms.xy(), _accel_offset_neu_cmss.xy(),
                            _vel_max_ne_cms, _accel_max_ne_cmss, _jerk_max_ne_cmsss, _dt, false);
}

/// stop_pos_NE_stabilisation - sets the target to the current position to remove any position corrections from the system
void AC_PosControl::stop_pos_NE_stabilisation()
{
    _pos_target_neu_cm.xy() = _pos_estimate_neu_cm.xy();
    _pos_desired_neu_cm.xy() = _pos_target_neu_cm.xy() - _pos_offset_neu_cm.xy();
}

/// stop_vel_NE_stabilisation - sets the target to the current position and velocity to the current velocity to remove any position and velocity corrections from the system
void AC_PosControl::stop_vel_NE_stabilisation()
{
    _pos_target_neu_cm.xy() =  _pos_estimate_neu_cm.xy();
    _pos_desired_neu_cm.xy() = _pos_target_neu_cm.xy() - _pos_offset_neu_cm.xy();
    
    _vel_target_neu_cms.xy() = _vel_estimate_neu_cms.xy();
    _vel_desired_neu_cms.xy() = _vel_target_neu_cms.xy() - _vel_offset_neu_cms.xy();

    // initialise I terms from lean angles
    _pid_vel_ne.reset_filter();
    _pid_vel_ne.reset_I();
}

// is_active_NE - returns true if the xy position controller has been run in the previous loop
bool AC_PosControl::is_active_NE() const
{
    const uint32_t dt_ticks = AP::scheduler().ticks32() - _last_update_ne_ticks;
    return dt_ticks <= 1;
}

/// update_NE_controller - runs the horizontal position controller correcting position, velocity and acceleration errors.
///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
///     Desired velocity and accelerations are added to these corrections as they are calculated
///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
void AC_PosControl::update_NE_controller()
{
    // check for ekf xy position reset
    handle_ekf_NE_reset();

    // Check for position control time out
    if (!is_active_NE()) {
        init_NE_controller();
        if (has_good_timing()) {
            // call internal error because initialisation has not been done
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }
    _last_update_ne_ticks = AP::scheduler().ticks32();

    float ahrsGndSpdLimit, ahrsControlScaleXY;
    AP::ahrs().getControlLimits(ahrsGndSpdLimit, ahrsControlScaleXY);

    // update the position, velocity and acceleration offsets
    update_offsets_NE();

    // Position Controller

    _pos_target_neu_cm.xy() = _pos_desired_neu_cm.xy() + _pos_offset_neu_cm.xy();

    // determine the combined position of the actual position and the disturbance from system ID mode
    // calculate the target velocity correction
    Vector2p comb_pos = _pos_estimate_neu_cm.xy();
    comb_pos += _disturb_pos_ne_cm.topostype();

    Vector2f vel_target = _p_pos_ne.update_all(_pos_target_neu_cm.xy(), comb_pos);
    _pos_desired_neu_cm.xy() = _pos_target_neu_cm.xy() - _pos_offset_neu_cm.xy();

    // Velocity Controller

    // add velocity feed-forward scaled to compensate for optical flow measurement induced EKF noise
    vel_target *= ahrsControlScaleXY;
    vel_target *= _ne_control_scale_factor;

    _vel_target_neu_cms.xy() = vel_target;
    _vel_target_neu_cms.xy() += _vel_desired_neu_cms.xy() + _vel_offset_neu_cms.xy();

    // determine the combined velocity of the actual velocity and the disturbance from system ID mode
    // Velocity Controller
    Vector2f comb_vel = _vel_estimate_neu_cms.xy();
    comb_vel += _disturb_vel_ne_cms;

    Vector2f accel_target_ne_cmss = _pid_vel_ne.update_all(_vel_target_neu_cms.xy(), comb_vel, _dt, _limit_vector.xy());

    // Acceleration Controller
    
    // acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    accel_target_ne_cmss *= ahrsControlScaleXY;
    accel_target_ne_cmss *= _ne_control_scale_factor;

    _ne_control_scale_factor = 1.0;

    // pass the correction acceleration to the target acceleration output
    _accel_target_neu_cmss.xy() = accel_target_ne_cmss;
    _accel_target_neu_cmss.xy() += _accel_desired_neu_cmss.xy() + _accel_offset_neu_cmss.xy();

    // limit acceleration using maximum lean angles
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_cmss = angle_rad_to_accel_mss(angle_max_rad) * 100.0;
    // Define the limit vector before we constrain _accel_target_neu_cmss 
    _limit_vector.xy() = _accel_target_neu_cmss.xy();
    if (!limit_accel_xy(_vel_desired_neu_cms.xy(), _accel_target_neu_cmss.xy(), accel_max_cmss)) {
        // _accel_target_neu_cmss was not limited so we can zero the xy limit vector
        _limit_vector.xy().zero();
    }

    // update angle targets that will be passed to stabilize controller
    accel_NE_cmss_to_lean_angles_rad(_accel_target_neu_cmss.x, _accel_target_neu_cmss.y, _roll_target_rad, _pitch_target_rad);
    calculate_yaw_and_rate_yaw();

    // reset the disturbance from system ID mode to zero
    _disturb_pos_ne_cm.zero();
    _disturb_vel_ne_cms.zero();
}


///
/// Vertical position controller
///

/// set_max_speed_accel_U_cm - set the maximum vertical speed in cm/s and acceleration in cm/s/s
///     speed_down_cms can be positive or negative but will always be interpreted as a descent speed.
///     This function only needs to be called if using the kinematic shaping.
///     This can be done at any time as changes in these parameters are handled smoothly
///     by the kinematic shaping.
void AC_PosControl::set_max_speed_accel_U_cm(float speed_down_cms, float speed_up_cms, float accel_cmss)
{
    // ensure speed_down_cms is always negative
    speed_down_cms = -fabsf(speed_down_cms);

    // sanity check and update
    if (is_negative(speed_down_cms)) {
        _vel_max_down_cms = speed_down_cms;
    }
    if (is_positive(speed_up_cms)) {
        _vel_max_up_cms = speed_up_cms;
    }
    if (is_positive(accel_cmss)) {
        _accel_max_u_cmss = accel_cmss;
    }

    // ensure the vertical Jerk is not limited by the filters in the Z acceleration PID object
    _jerk_max_u_cmsss = _shaping_jerk_u_msss * 100.0;
    if (is_positive(_pid_accel_u.filt_T_hz())) {
        _jerk_max_u_cmsss = MIN(_jerk_max_u_cmsss, MIN(GRAVITY_MSS * 100.0, _accel_max_u_cmss) * (M_2PI * _pid_accel_u.filt_T_hz()) / 5.0);
    }
    if (is_positive(_pid_accel_u.filt_E_hz())) {
        _jerk_max_u_cmsss = MIN(_jerk_max_u_cmsss, MIN(GRAVITY_MSS * 100.0, _accel_max_u_cmss) * (M_2PI * _pid_accel_u.filt_E_hz()) / 5.0);
    }
}

/// set_correction_speed_accel_U_cmss - set the position controller correction velocity and acceleration limit
///     speed_down_cms can be positive or negative but will always be interpreted as a descent speed.
///     This should be done only during initialisation to avoid discontinuities
void AC_PosControl::set_correction_speed_accel_U_cmss(float speed_down_cms, float speed_up_cms, float accel_cmss)
{
    // define maximum position error and maximum first and second differential limits
    _p_pos_u.set_limits(-fabsf(speed_down_cms), speed_up_cms, accel_cmss, 0.0f);
}

/// init_U_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
///     This function does not allow any negative velocity or acceleration
void AC_PosControl::init_U_controller_no_descent()
{
    // Initialise the position controller to the current throttle, position, velocity and acceleration.
    init_U_controller();

    // remove all descent if present
    _vel_target_neu_cms.z = MAX(0.0, _vel_target_neu_cms.z);
    _vel_desired_neu_cms.z = MAX(0.0, _vel_desired_neu_cms.z);
    _vel_terrain_u_cms = MAX(0.0, _vel_terrain_u_cms);
    _vel_offset_neu_cms.z = MAX(0.0, _vel_offset_neu_cms.z);
    _accel_target_neu_cmss.z = MAX(0.0, _accel_target_neu_cmss.z);
    _accel_desired_neu_cmss.z = MAX(0.0, _accel_desired_neu_cmss.z);
    _accel_terrain_u_cmss = MAX(0.0, _accel_terrain_u_cmss);
    _accel_offset_neu_cmss.z = MAX(0.0, _accel_offset_neu_cmss.z);
}

/// init_U_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
///     This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
///     The starting position can be retrieved by getting the position target using get_pos_target_NEU_cm() after calling this function.
void AC_PosControl::init_U_controller_stopping_point()
{
    // Initialise the position controller to the current throttle, position, velocity and acceleration.
    init_U_controller();

    get_stopping_point_U_cm(_pos_desired_neu_cm.z);
    _pos_target_neu_cm.z = _pos_desired_neu_cm.z + _pos_offset_neu_cm.z;
    _vel_desired_neu_cms.z = 0.0f;
    _accel_desired_neu_cmss.z = 0.0f;
}

// relax_U_controller - initialise the position controller to the current position and velocity with decaying acceleration.
///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
void AC_PosControl::relax_U_controller(float throttle_setting)
{
    // Initialise the position controller to the current position, velocity and acceleration.
    init_U_controller();

    // init_U_controller has set the acceleration PID I term to generate the current throttle set point
    // Use relax_integrator to decay the throttle set point to throttle_setting
    _pid_accel_u.relax_integrator((throttle_setting - _motors.get_throttle_hover()) * 1000.0f, _dt, POSCONTROL_RELAX_TC);
}

/// init_U_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
///     This function is private and contains all the shared z axis initialisation functions
void AC_PosControl::init_U_controller()
{
    // initialise terrain targets and offsets to zero
    init_terrain();

    // initialise offsets to target offsets and ensure offset targets are zero if they have not been updated.
    init_offsets_U();

    _pos_target_neu_cm.z = _pos_estimate_neu_cm.z;
    _pos_desired_neu_cm.z = _pos_target_neu_cm.z - _pos_offset_neu_cm.z;

    _vel_target_neu_cms.z = _vel_estimate_neu_cms.z;
    _vel_desired_neu_cms.z = _vel_target_neu_cms.z - _vel_offset_neu_cms.z;

    // Reset I term of velocity PID
    _pid_vel_u.reset_filter();
    _pid_vel_u.set_integrator(0.0f);

    _accel_target_neu_cmss.z = constrain_float(get_measured_accel_U_cmss(), -_accel_max_u_cmss, _accel_max_u_cmss);
    _accel_desired_neu_cmss.z = _accel_target_neu_cmss.z - (_accel_offset_neu_cmss.z + _accel_terrain_u_cmss);
    _pid_accel_u.reset_filter();

    // Set acceleration PID I term based on the current throttle
    // Remove the expected P term due to _accel_desired_neu_cmss.z being constrained to _accel_max_u_cmss
    // Remove the expected FF term due to non-zero _accel_target_neu_cmss.z
    _pid_accel_u.set_integrator((_attitude_control.get_throttle_in() - _motors.get_throttle_hover()) * 1000.0f
        - _pid_accel_u.kP() * (_accel_target_neu_cmss.z - get_measured_accel_U_cmss())
        - _pid_accel_u.ff() * _accel_target_neu_cmss.z);

    // initialise ekf z reset handler
    init_ekf_U_reset();

    // initialise z_controller time out
    _last_update_u_ticks = AP::scheduler().ticks32();
}

/// input_accel_U_cm - calculate a jerk limited path from the current position, velocity and acceleration to an input acceleration.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_U_cmss.
void AC_PosControl::input_accel_U_cm(float accel_cmss)
{
    // calculated increased maximum jerk if over speed
    float jerk_max_u_cmsss = _jerk_max_u_cmsss * calculate_overspeed_gain();

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_neu_cm.z, _vel_desired_neu_cms.z, _accel_desired_neu_cmss.z, _dt, _limit_vector.z, _p_pos_u.get_error(), _pid_vel_u.get_error());

    shape_accel(accel_cmss, _accel_desired_neu_cmss.z, jerk_max_u_cmsss, _dt);
}

/// input_vel_accel_U_cm - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity and acceleration.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_U_cmss.
///     The function modifies vel_u_cms to follow the jerk-limited trajectory defined by accel_u_cmss.
///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
void AC_PosControl::input_vel_accel_U_cm(float &vel_u_cms, float accel_cmss, bool limit_output)
{
    // calculated increased maximum acceleration and jerk if over speed
    const float overspeed_gain = calculate_overspeed_gain();
    const float accel_max_u_cmss = _accel_max_u_cmss * overspeed_gain;
    const float jerk_max_u_cmsss = _jerk_max_u_cmsss * overspeed_gain;

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_neu_cm.z, _vel_desired_neu_cms.z, _accel_desired_neu_cmss.z, _dt, _limit_vector.z, _p_pos_u.get_error(), _pid_vel_u.get_error());

    shape_vel_accel(vel_u_cms, accel_cmss,
                    _vel_desired_neu_cms.z, _accel_desired_neu_cmss.z,
                    -constrain_float(accel_max_u_cmss, 0.0f, 750.0f), accel_max_u_cmss,
                    jerk_max_u_cmsss, _dt, limit_output);

    update_vel_accel(vel_u_cms, accel_cmss, _dt, 0.0, 0.0);
}

/// set_pos_target_U_from_climb_rate_cm - adjusts target up or down using a commanded climb rate in cm/s
///     using the default position control kinematic path.
///     The zero target altitude is varied to follow pos_offset_z
void AC_PosControl::set_pos_target_U_from_climb_rate_cm(float vel_u_cms)
{
    input_vel_accel_U_cm(vel_u_cms, 0.0);
}

/// land_at_climb_rate_cm - adjusts target up or down using a commanded climb rate in cm/s
///     using the default position control kinematic path.
///     ignore_descent_limit turns off output saturation handling to aid in landing detection. ignore_descent_limit should be false unless landing.
void AC_PosControl::land_at_climb_rate_cm(float vel_u_cms, bool ignore_descent_limit)
{
    if (ignore_descent_limit) {
        // turn off limits in the negative z direction
        _limit_vector.z = MAX(_limit_vector.z, 0.0f);
    }

    input_vel_accel_U_cm(vel_u_cms, 0.0);
}

/// input_pos_vel_accel_U_cm - calculate a jerk limited path from the current position, velocity and acceleration to an input position velocity and acceleration.
///     The pos_u_cm and vel_u_cms are projected forwards in time based on a time step of dt and acceleration accel_cmss.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The function alters the pos_u_cm and vel_u_cms to be the kinematic path based on accel_cmss
///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
void AC_PosControl::input_pos_vel_accel_U_cm(float &pos_u_cm, float &vel_u_cms, float accel_cmss, bool limit_output)
{
    // calculated increased maximum acceleration and jerk if over speed
    const float overspeed_gain = calculate_overspeed_gain();
    const float accel_max_u_cmss = _accel_max_u_cmss * overspeed_gain;
    const float jerk_max_u_cmsss = _jerk_max_u_cmsss * overspeed_gain;

    // adjust desired altitude if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_neu_cm.z, _vel_desired_neu_cms.z, _accel_desired_neu_cmss.z, _dt, _limit_vector.z, _p_pos_u.get_error(), _pid_vel_u.get_error());

    shape_pos_vel_accel(pos_u_cm, vel_u_cms, accel_cmss,
                        _pos_desired_neu_cm.z, _vel_desired_neu_cms.z, _accel_desired_neu_cmss.z,
                        _vel_max_down_cms, _vel_max_up_cms,
                        -constrain_float(accel_max_u_cmss, 0.0f, 750.0f), accel_max_u_cmss,
                        jerk_max_u_cmsss, _dt, limit_output);

    postype_t posp = pos_u_cm;
    update_pos_vel_accel(posp, vel_u_cms, accel_cmss, _dt, 0.0, 0.0, 0.0);
    pos_u_cm = posp;
}

/// set_alt_target_with_slew_cm - adjusts target up or down using a commanded altitude in cm
///     using the default position control kinematic path.
void AC_PosControl::set_alt_target_with_slew_cm(float pos_u_cm)
{
    float zero = 0;
    input_pos_vel_accel_U_cm(pos_u_cm, zero, 0);
}

/// update_offsets_U - updates the vertical offsets used by terrain following
void AC_PosControl::update_offsets_U()
{
    // check for offset target timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_u_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        _pos_offset_target_neu_cm.z = 0.0;
        _vel_offset_target_neu_cms.z = 0.0;
        _accel_offset_target_neu_cmss.z = 0.0;
    }

    // update position, velocity, acceleration offsets for this iteration
    postype_t p_offset_u_cm = _pos_offset_neu_cm.z;
    update_pos_vel_accel(p_offset_u_cm, _vel_offset_neu_cms.z, _accel_offset_neu_cmss.z, _dt, MIN(_limit_vector.z, 0.0f), _p_pos_u.get_error(), _pid_vel_u.get_error());
    _pos_offset_neu_cm.z = p_offset_u_cm;

    // input shape vertical position, velocity and acceleration offsets
    shape_pos_vel_accel(_pos_offset_target_neu_cm.z, _vel_offset_target_neu_cms.z, _accel_offset_target_neu_cmss.z,
        _pos_offset_neu_cm.z, _vel_offset_neu_cms.z, _accel_offset_neu_cmss.z,
        get_max_speed_down_cms(), get_max_speed_up_cms(),
        -get_max_accel_U_cmss(), get_max_accel_U_cmss(),
        _jerk_max_u_cmsss, _dt, false);

    p_offset_u_cm = _pos_offset_target_neu_cm.z;
    update_pos_vel_accel(p_offset_u_cm, _vel_offset_target_neu_cms.z, _accel_offset_target_neu_cmss.z, _dt, 0.0, 0.0, 0.0);
    _pos_offset_target_neu_cm.z = p_offset_u_cm;
}

// is_active_U - returns true if the z position controller has been run in the previous loop
bool AC_PosControl::is_active_U() const
{
    const uint32_t dt_ticks = AP::scheduler().ticks32() - _last_update_u_ticks;
    return dt_ticks <= 1;
}

/// update_U_controller - runs the vertical position controller correcting position, velocity and acceleration errors.
///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
///     Desired velocity and accelerations are added to these corrections as they are calculated
///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
void AC_PosControl::update_U_controller()
{
    // check for ekf z-axis position reset
    handle_ekf_U_reset();

    // Check for z_controller time out
    if (!is_active_U()) {
        init_U_controller();
        if (has_good_timing()) {
            // call internal error because initialisation has not been done
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }
    _last_update_u_ticks = AP::scheduler().ticks32();

    // update the position, velocity and acceleration offsets
    update_offsets_U();
    update_terrain();
    _pos_target_neu_cm.z = _pos_desired_neu_cm.z + _pos_offset_neu_cm.z + _pos_terrain_u_cm;

    // calculate the target velocity correction
    float pos_target_zf = _pos_target_neu_cm.z;

    _vel_target_neu_cms.z = _p_pos_u.update_all(pos_target_zf, _pos_estimate_neu_cm.z);
    _vel_target_neu_cms.z *= AP::ahrs().getControlScaleZ();

    _pos_target_neu_cm.z = pos_target_zf;
    _pos_desired_neu_cm.z = _pos_target_neu_cm.z - (_pos_offset_neu_cm.z + _pos_terrain_u_cm);

    // add feed forward component
    _vel_target_neu_cms.z += _vel_desired_neu_cms.z + _vel_offset_neu_cms.z + _vel_terrain_u_cms;

    // Velocity Controller

    _accel_target_neu_cmss.z = _pid_vel_u.update_all(_vel_target_neu_cms.z, _vel_estimate_neu_cms.z, _dt, _motors.limit.throttle_lower, _motors.limit.throttle_upper);
    _accel_target_neu_cmss.z *= AP::ahrs().getControlScaleZ();

    // add feed forward component
    _accel_target_neu_cmss.z += _accel_desired_neu_cmss.z + _accel_offset_neu_cmss.z + _accel_terrain_u_cmss;

    // Acceleration Controller

    // Calculate vertical acceleration
    const float measured_accel_u_cmss = get_measured_accel_U_cmss();

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_u.imax()) {
        _pid_accel_u.set_imax(_motors.get_throttle_hover() * 1000.0f);
    }
    float thr_out;
    if (_vibe_comp_enabled) {
        thr_out = get_throttle_with_vibration_override();
    } else {
        thr_out = _pid_accel_u.update_all(_accel_target_neu_cmss.z, measured_accel_u_cmss, _dt, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001f;
        thr_out += _pid_accel_u.get_ff() * 0.001f;
    }
    thr_out += _motors.get_throttle_hover();

    // Actuator commands

    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);

    // Check for vertical controller health

    // _speed_down_cms is checked to be non-zero when set
    float error_ratio = _pid_vel_u.get_error() / _vel_max_down_cms;
    _vel_u_control_ratio += _dt * 0.1f * (0.5 - error_ratio);
    _vel_u_control_ratio = constrain_float(_vel_u_control_ratio, 0.0f, 2.0f);

    // set vertical component of the limit vector
    if (_motors.limit.throttle_upper) {
        _limit_vector.z = 1.0f;
    } else if (_motors.limit.throttle_lower) {
        _limit_vector.z = -1.0f;
    } else {
        _limit_vector.z = 0.0f;
    }
}


///
/// Accessors
///

/// get_lean_angle_max_rad - returns the maximum lean angle the autopilot may request
float AC_PosControl::get_lean_angle_max_rad() const
{
    if (is_positive(_angle_max_override_rad)) {
        return _angle_max_override_rad;
    }
    if (!is_positive(_lean_angle_max_deg)) {
        return _attitude_control.lean_angle_max_rad();
    }
    return radians(_lean_angle_max_deg);
}

/// set the desired position, velocity and acceleration targets
void AC_PosControl::set_pos_vel_accel_NEU_cm(const Vector3p& pos_neu_cm, const Vector3f& vel_neu_cms, const Vector3f& accel_neu_cmss)
{
    _pos_desired_neu_cm = pos_neu_cm;
    _vel_desired_neu_cms = vel_neu_cms;
    _accel_desired_neu_cmss = accel_neu_cmss;
}

/// set the desired position, velocity and acceleration targets
void AC_PosControl::set_pos_vel_accel_NE_cm(const Vector2p& pos_ne_cm, const Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss)
{
    _pos_desired_neu_cm.xy() = pos_ne_cm;
    _vel_desired_neu_cms.xy() = vel_ne_cms;
    _accel_desired_neu_cmss.xy() = accel_ne_cmss;
}

// get_lean_angles_to_accel - convert roll, pitch lean target angles to lat/lon frame accelerations in cm/s/s
Vector3f AC_PosControl::lean_angles_to_accel_NEU_cmss(const Vector3f& att_target_euler_rad) const
{
    // rotate our roll, pitch angles into lat/lon frame
    const float sin_roll = sinf(att_target_euler_rad.x);
    const float cos_roll = cosf(att_target_euler_rad.x);
    const float sin_pitch = sinf(att_target_euler_rad.y);
    const float cos_pitch = cosf(att_target_euler_rad.y);
    const float sin_yaw = sinf(att_target_euler_rad.z);
    const float cos_yaw = cosf(att_target_euler_rad.z);

    return Vector3f{
        (GRAVITY_MSS * 100.0f) * (-cos_yaw * sin_pitch * cos_roll - sin_yaw * sin_roll) / MAX(cos_roll * cos_pitch, 0.1f),
        (GRAVITY_MSS * 100.0f) * (-sin_yaw * sin_pitch * cos_roll + cos_yaw * sin_roll) / MAX(cos_roll * cos_pitch, 0.1f),
        (GRAVITY_MSS * 100.0f)
    };
}

/// Terrain

/// set the terrain position, velocity and acceleration in cm, cms and cm/s/s from EKF origin in NE frame
/// this is used to initiate the offsets when initialise the position controller or do an offset reset
void AC_PosControl::init_terrain()
{
    // set terrain position and target to zero
    _pos_terrain_target_u_cm = 0.0;
    _pos_terrain_u_cm = 0.0;

    // set velocity offset to zero
    _vel_terrain_u_cms = 0.0;

    // set acceleration offset to zero
    _accel_terrain_u_cmss = 0.0;
}

// init_pos_terrain_U_cm - initialises the current terrain altitude and target altitude to pos_offset_terrain_cm
void AC_PosControl::init_pos_terrain_U_cm(float pos_terrain_u_cm)
{
    _pos_desired_neu_cm.z -= (pos_terrain_u_cm - _pos_terrain_u_cm);
    _pos_terrain_target_u_cm = pos_terrain_u_cm;
    _pos_terrain_u_cm = pos_terrain_u_cm;
}


/// Offsets

/// set the horizontal position, velocity and acceleration offsets in cm, cms and cm/s/s from EKF origin in NE frame
/// this is used to initiate the offsets when initialise the position controller or do an offset reset
void AC_PosControl::init_offsets_NE()
{
    // check for offset target timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_ne_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        _pos_offset_target_neu_cm.xy().zero();
        _vel_offset_target_neu_cms.xy().zero();
        _accel_offset_target_neu_cmss.xy().zero();
    }

    // set position offset to target
    _pos_offset_neu_cm.xy() = _pos_offset_target_neu_cm.xy();

    // set velocity offset to target
    _vel_offset_neu_cms.xy() = _vel_offset_target_neu_cms.xy();

    // set acceleration offset to target
    _accel_offset_neu_cmss.xy() = _accel_offset_target_neu_cmss.xy();
}

/// set the horizontal position, velocity and acceleration offsets in cm, cms and cm/s/s from EKF origin in NE frame
/// this is used to initiate the offsets when initialise the position controller or do an offset reset
void AC_PosControl::init_offsets_U()
{
    // check for offset target timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_u_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        _pos_offset_target_neu_cm.z = 0.0;
        _vel_offset_target_neu_cms.z = 0.0;
        _accel_offset_target_neu_cmss.z = 0.0;
    }
    // set position offset to target
    _pos_offset_neu_cm.z = _pos_offset_target_neu_cm.z;

    // set velocity offset to target
    _vel_offset_neu_cms.z = _vel_offset_target_neu_cms.z;

    // set acceleration offset to target
    _accel_offset_neu_cmss.z = _accel_offset_target_neu_cmss.z;
}

#if AP_SCRIPTING_ENABLED
// add an additional offset to vehicle's target position, velocity and acceleration
// units are m, m/s and m/s/s in NED frame
// Z-axis is not currently supported and is ignored
bool AC_PosControl::set_posvelaccel_offset(const Vector3f &pos_offset_NED, const Vector3f &vel_offset_NED, const Vector3f &accel_offset_NED)
{
    set_posvelaccel_offset_target_NE_cm(pos_offset_NED.topostype().xy() * 100.0, vel_offset_NED.xy() * 100.0, accel_offset_NED.xy() * 100.0);
    set_posvelaccel_offset_target_U_cm(-pos_offset_NED.topostype().z * 100.0, -vel_offset_NED.z * 100, -accel_offset_NED.z * 100.0);
    return true;
}

// get position and velocity offset to vehicle's target velocity and acceleration
// units are m and m/s in NED frame
bool AC_PosControl::get_posvelaccel_offset(Vector3f &pos_offset_NED, Vector3f &vel_offset_NED, Vector3f &accel_offset_NED)
{
    pos_offset_NED.xy() = _pos_offset_target_neu_cm.xy().tofloat() * 0.01;
    pos_offset_NED.z = -_pos_offset_target_neu_cm.z * 0.01;
    vel_offset_NED.xy() = _vel_offset_target_neu_cms.xy() * 0.01;
    vel_offset_NED.z = -_vel_offset_target_neu_cms.z * 0.01;
    accel_offset_NED.xy() = _accel_offset_target_neu_cmss.xy() * 0.01;
    accel_offset_NED.z = -_accel_offset_target_neu_cmss.z * 0.01;
    return true;
}

// get target velocity in m/s in NED frame
bool AC_PosControl::get_vel_target(Vector3f &vel_target_NED)
{
    if (!is_active_NE() || !is_active_U()) {
        return false;
    }
    vel_target_NED.xy() = _vel_target_neu_cms.xy() * 0.01;
    vel_target_NED.z = -_vel_target_neu_cms.z * 0.01;
    return true;
}

// get target acceleration in m/s/s in NED frame
bool AC_PosControl::get_accel_target(Vector3f &accel_target_NED)
{
    if (!is_active_NE() || !is_active_U()) {
        return false;
    }
    accel_target_NED.xy() = _accel_target_neu_cmss.xy() * 0.01;
    accel_target_NED.z = -_accel_target_neu_cmss.z * 0.01;
    return true;
}
#endif

/// set the horizontal position, velocity and acceleration offset targets in cm, cms and cm/s/s from EKF origin in NE frame
/// these must be set every 3 seconds (or less) or they will timeout and return to zero
void AC_PosControl::set_posvelaccel_offset_target_NE_cm(const Vector2p& pos_offset_target_ne_cm, const Vector2f& vel_offset_target_ne_cms, const Vector2f& accel_offset_target_ne_cmss)
{
    // set position offset target
    _pos_offset_target_neu_cm.xy() = pos_offset_target_ne_cm;

    // set velocity offset target
    _vel_offset_target_neu_cms.xy() = vel_offset_target_ne_cms;

    // set acceleration offset target
    _accel_offset_target_neu_cmss.xy() = accel_offset_target_ne_cmss;

    // record time of update so we can detect timeouts
    _posvelaccel_offset_target_ne_ms = AP_HAL::millis();
}

/// set the vertical position, velocity and acceleration offset targets in cm, cms and cm/s/s from EKF origin in NE frame
/// these must be set every 3 seconds (or less) or they will timeout and return to zero
void AC_PosControl::set_posvelaccel_offset_target_U_cm(float pos_offset_target_u_cm, float vel_offset_target_u_cms, const float accel_offset_target_u_cmss)
{
    // set position offset target
    _pos_offset_target_neu_cm.z = pos_offset_target_u_cm;

    // set velocity offset target
    _vel_offset_target_neu_cms.z = vel_offset_target_u_cms;

    // set acceleration offset target
    _accel_offset_target_neu_cmss.z = accel_offset_target_u_cmss;

    // record time of update so we can detect timeouts
    _posvelaccel_offset_target_u_ms = AP_HAL::millis();
}

// returns the NED target acceleration vector for attitude control
Vector3f AC_PosControl::get_thrust_vector() const
{
    Vector3f accel_target_neu_cmss = get_accel_target_NEU_cmss();
    accel_target_neu_cmss.z = -GRAVITY_MSS * 100.0f;
    return accel_target_neu_cmss;
}

/// get_stopping_point_NE_cm - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration
///    function does not change the z axis
void AC_PosControl::get_stopping_point_NE_cm(Vector2p &stopping_point_neu_cm) const
{
    // todo: we should use the current target position and velocity if we are currently running the position controller
    stopping_point_neu_cm = _pos_estimate_neu_cm.xy();
    stopping_point_neu_cm -= _pos_offset_neu_cm.xy();

    Vector2f curr_vel = _vel_estimate_neu_cms.xy();
    curr_vel -= _vel_offset_neu_cms.xy();

    // calculate current velocity
    float vel_total = curr_vel.length();

    if (!is_positive(vel_total)) {
        return;
    }
    
    float kP = _p_pos_ne.kP();
    const float stopping_dist = stopping_distance(constrain_float(vel_total, 0.0, _vel_max_ne_cms), kP, _accel_max_ne_cmss);
    if (!is_positive(stopping_dist)) {
        return;
    }

    // convert the stopping distance into a stopping point using velocity vector
    // todo: convert velocity to a unit vector instead.
    const float t = stopping_dist / vel_total;
    stopping_point_neu_cm += (curr_vel * t).topostype();
}

/// get_stopping_point_U_cm - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration
void AC_PosControl::get_stopping_point_U_cm(postype_t &stopping_point_u_cm) const
{
    float curr_pos_u_cm = _pos_estimate_neu_cm.z;
    curr_pos_u_cm -= _pos_offset_neu_cm.z;

    float curr_vel_u_cms = _vel_estimate_neu_cms.z;
    curr_vel_u_cms -= _vel_offset_neu_cms.z;

    // avoid divide by zero by using current position if kP is very low or acceleration is zero
    if (!is_positive(_p_pos_u.kP()) || !is_positive(_accel_max_u_cmss)) {
        stopping_point_u_cm = curr_pos_u_cm;
        return;
    }

    stopping_point_u_cm = curr_pos_u_cm + constrain_float(stopping_distance(curr_vel_u_cms, _p_pos_u.kP(), _accel_max_u_cmss), - POSCONTROL_STOPPING_DIST_DOWN_MAX, POSCONTROL_STOPPING_DIST_UP_MAX);
}

/// get_bearing_to_target_cd - get bearing to target position in centi-degrees
int32_t AC_PosControl::get_bearing_to_target_cd() const
{
    return get_bearing_cd(Vector2f{0.0, 0.0}, (_pos_target_neu_cm.xy() - _pos_estimate_neu_cm.xy()).tofloat());
}


///
/// System methods
///

// Updates internal position and velocity estimates in the NED frame.
// Falls back to vertical-only estimates if full NED data is unavailable.
// When high_vibes is true, forces use of vertical fallback for velocity.
void AC_PosControl::update_estimates(bool high_vibes)
{
    Vector3p pos_estimate_ned_m;
    if (!AP::ahrs().get_relative_position_NED_origin(pos_estimate_ned_m)) {
        float posD;
        if (AP::ahrs().get_relative_position_D_origin_float(posD)) {
            pos_estimate_ned_m.z = posD;
        }
    }
    _pos_estimate_neu_cm.xy() = pos_estimate_ned_m.xy() * 100.0;
    _pos_estimate_neu_cm.z = -pos_estimate_ned_m.z * 100.0;

    Vector3f vel_estimate_ned_ms;
    if (!AP::ahrs().get_velocity_NED(vel_estimate_ned_ms) || high_vibes) {
        float rate_z;
        if (AP::ahrs().get_vert_pos_rate_D(rate_z)) {
            vel_estimate_ned_ms.z = rate_z;
        }
    }
    _vel_estimate_neu_cms.xy() = vel_estimate_ned_ms.xy() * 100.0;
    _vel_estimate_neu_cms.z = -vel_estimate_ned_ms.z * 100.0;
}

// get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain)
float AC_PosControl::get_throttle_with_vibration_override()
{
    const float thr_per_accel_u_cmss = _motors.get_throttle_hover() / (GRAVITY_MSS * 100.0f);
    // during vibration compensation use feed forward with manually calculated gain
    // ToDo: clear pid_info P, I and D terms for logging
    if (!(_motors.limit.throttle_lower || _motors.limit.throttle_upper) || ((is_positive(_pid_accel_u.get_i()) && is_negative(_pid_vel_u.get_error())) || (is_negative(_pid_accel_u.get_i()) && is_positive(_pid_vel_u.get_error())))) {
        _pid_accel_u.set_integrator(_pid_accel_u.get_i() + _dt * thr_per_accel_u_cmss * 1000.0f * _pid_vel_u.get_error() * _pid_vel_u.kP() * POSCONTROL_VIBE_COMP_I_GAIN);
    }
    return POSCONTROL_VIBE_COMP_P_GAIN * thr_per_accel_u_cmss * _accel_target_neu_cmss.z + _pid_accel_u.get_i() * 0.001f;
}

/// standby_NEU_reset - resets I terms and removes position error
///     This function will let Loiter and Alt Hold continue to operate
///     in the event that the flight controller is in control of the
///     aircraft when in standby.
void AC_PosControl::standby_NEU_reset()
{
    // Set _pid_accel_u integrator to zero.
    _pid_accel_u.set_integrator(0.0f);

    // Set the target position to the current pos.
    _pos_target_neu_cm = _pos_estimate_neu_cm;

    // Set _pid_vel_ne integrator and derivative to zero.
    _pid_vel_ne.reset_filter();

    // initialise ekf xy reset handler
    init_ekf_NE_reset();
}

#if HAL_LOGGING_ENABLED
// write PSC and/or PSCZ logs
void AC_PosControl::write_log()
{
    if (is_active_NE()) {
        float accel_n_cmss, accel_e_cmss;
        lean_angles_to_accel_NE_cmss(accel_n_cmss, accel_e_cmss);
        Write_PSCN(_pos_desired_neu_cm.x, _pos_target_neu_cm.x, _pos_estimate_neu_cm.x ,
                   _vel_desired_neu_cms.x, _vel_target_neu_cms.x, _vel_estimate_neu_cms.x,
                   _accel_desired_neu_cmss.x, _accel_target_neu_cmss.x, accel_n_cmss);
        Write_PSCE(_pos_desired_neu_cm.y, _pos_target_neu_cm.y, _pos_estimate_neu_cm.y,
                   _vel_desired_neu_cms.y, _vel_target_neu_cms.y, _vel_estimate_neu_cms.y,
                   _accel_desired_neu_cmss.y, _accel_target_neu_cmss.y, accel_e_cmss);

        // log offsets if they are being used
        if (!_pos_offset_neu_cm.xy().is_zero()) {
            Write_PSON(_pos_offset_target_neu_cm.x, _pos_offset_neu_cm.x, _vel_offset_target_neu_cms.x, _vel_offset_neu_cms.x, _accel_offset_target_neu_cmss.x, _accel_offset_neu_cmss.x);
            Write_PSOE(_pos_offset_target_neu_cm.y, _pos_offset_neu_cm.y, _vel_offset_target_neu_cms.y, _vel_offset_neu_cms.y, _accel_offset_target_neu_cmss.y, _accel_offset_neu_cmss.y);
        }
    }

    if (is_active_U()) {
        Write_PSCD(-_pos_desired_neu_cm.z, -_pos_target_neu_cm.z, -_pos_estimate_neu_cm.z,
                   -_vel_desired_neu_cms.z, -_vel_target_neu_cms.z, -_vel_estimate_neu_cms.z,
                   -_accel_desired_neu_cmss.z, -_accel_target_neu_cmss.z, -get_measured_accel_U_cmss());

        // log down and terrain offsets if they are being used
        if (!is_zero(_pos_offset_neu_cm.z)) {
            Write_PSOD(-_pos_offset_target_neu_cm.z, -_pos_offset_neu_cm.z, -_vel_offset_target_neu_cms.z, -_vel_offset_neu_cms.z, -_accel_offset_target_neu_cmss.z, -_accel_offset_neu_cmss.z);
        }
        if (!is_zero(_pos_terrain_u_cm)) {
            Write_PSOT(-_pos_terrain_target_u_cm, -_pos_terrain_u_cm, 0, -_vel_terrain_u_cms, 0, -_accel_terrain_u_cmss);
        }
    }
}
#endif  // HAL_LOGGING_ENABLED

/// crosstrack_error - returns horizontal error to the closest point to the current track
float AC_PosControl::crosstrack_error() const
{
    const Vector2f pos_error = (_pos_target_neu_cm.xy() - _pos_estimate_neu_cm.xy()).tofloat();
    if (is_zero(_vel_desired_neu_cms.xy().length_squared())) {
        // crosstrack is the horizontal distance to target when stationary
        return pos_error.length();
    } else {
        // crosstrack is the horizontal distance to the closest point to the current track
        const Vector2f vel_unit = _vel_desired_neu_cms.xy().normalized();
        const float dot_error = pos_error * vel_unit;

        // todo: remove MAX of zero when safe_sqrt fixed
        return safe_sqrt(MAX(pos_error.length_squared() - sq(dot_error), 0.0));
    }
}

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
/// returns true when the forward pitch demand is limited by the maximum allowed tilt
bool AC_PosControl::get_fwd_pitch_is_limited() const 
{
    if (_limit_vector.xy().is_zero()) {  
        return false;  
    }  
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_cmss = angle_rad_to_accel_mss(angle_max_rad) * 100.0;
    // Check for pitch limiting in the forward direction
    const float accel_fwd_unlimited = _limit_vector.x * _ahrs.cos_yaw() + _limit_vector.y * _ahrs.sin_yaw();
    const float pitch_target_unlimited_cd = accel_mss_to_angle_deg(- MIN(accel_fwd_unlimited, accel_max_cmss) * 0.01f) * 100;
    const float accel_fwd_limited = _accel_target_neu_cmss.x * _ahrs.cos_yaw() + _accel_target_neu_cmss.y * _ahrs.sin_yaw();
    const float pitch_target_limited_cd = accel_mss_to_angle_deg(- accel_fwd_limited * 0.01f) * 100;

    return is_negative(pitch_target_unlimited_cd) && pitch_target_unlimited_cd < pitch_target_limited_cd;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)

///
/// private methods
///

/// Terrain

/// update_z_offsets - updates the vertical offsets used by terrain following
void AC_PosControl::update_terrain()
{
    // update position, velocity, acceleration offsets for this iteration
    postype_t pos_terrain_u_cm = _pos_terrain_u_cm;
    update_pos_vel_accel(pos_terrain_u_cm, _vel_terrain_u_cms, _accel_terrain_u_cmss, _dt, MIN(_limit_vector.z, 0.0f), _p_pos_u.get_error(), _pid_vel_u.get_error());
    _pos_terrain_u_cm = pos_terrain_u_cm;

    // input shape horizontal position, velocity and acceleration offsets
    shape_pos_vel_accel(_pos_terrain_target_u_cm, 0.0, 0.0,
        _pos_terrain_u_cm, _vel_terrain_u_cms, _accel_terrain_u_cmss,
        get_max_speed_down_cms(), get_max_speed_up_cms(),
        -get_max_accel_U_cmss(), get_max_accel_U_cmss(),
        _jerk_max_u_cmsss, _dt, false);

    // we do not have to update _pos_terrain_target_u_cm because we assume the target velocity and acceleration are zero
    // if we know how fast the terain altitude is changing we would add update_pos_vel_accel for _pos_terrain_target_u_cm here
}

// get_lean_angles_to_accel - convert NE frame accelerations in cm/s/s to roll, pitch lean angles in centi-degrees
void AC_PosControl::accel_NE_cmss_to_lean_angles_rad(float accel_n_cmss, float accel_e_cmss, float& roll_target_rad, float& pitch_target_rad) const
{
    // rotate accelerations into body forward-right frame
    const float accel_forward_mss = (accel_n_cmss * _ahrs.cos_yaw() + accel_e_cmss * _ahrs.sin_yaw()) * 0.01;
    const float accel_right_mss = (-accel_n_cmss * _ahrs.sin_yaw() + accel_e_cmss * _ahrs.cos_yaw()) * 0.01;

    // update angle targets that will be passed to stabilize controller
    pitch_target_rad = accel_mss_to_angle_rad(-accel_forward_mss);
    float cos_pitch_target = cosf(pitch_target_rad);
    roll_target_rad = accel_mss_to_angle_rad(accel_right_mss * cos_pitch_target);
}

// lean_angles_to_accel_NE_cmss - convert roll, pitch lean target angles to NE frame accelerations in cm/s/s
// todo: this should be based on thrust vector attitude control
void AC_PosControl::lean_angles_to_accel_NE_cmss(float& accel_n_cmss, float& accel_e_cmss) const
{
    // rotate our roll, pitch angles into lat/lon frame
    Vector3f att_target_euler_rad = _attitude_control.get_att_target_euler_rad();
    att_target_euler_rad.z = _ahrs.yaw;
    Vector3f accel_ne_cmss = lean_angles_to_accel_NEU_cmss(att_target_euler_rad);

    accel_n_cmss = accel_ne_cmss.x;
    accel_e_cmss = accel_ne_cmss.y;
}

// calculate_yaw_and_rate_yaw - update the calculated the vehicle yaw and rate of yaw.
void AC_PosControl::calculate_yaw_and_rate_yaw()
{
    // Calculate the turn rate
    float turn_rate_rads = 0.0f;
    const float vel_desired_length_ne_cms = _vel_desired_neu_cms.xy().length();
    if (is_positive(vel_desired_length_ne_cms)) {
        const float accel_forward_cmss = (_accel_desired_neu_cmss.x * _vel_desired_neu_cms.x + _accel_desired_neu_cmss.y * _vel_desired_neu_cms.y) / vel_desired_length_ne_cms;
        const Vector2f accel_turn_ne_cmss = _accel_desired_neu_cmss.xy() - _vel_desired_neu_cms.xy() * accel_forward_cmss / vel_desired_length_ne_cms;
        const float accel_turn_length_ne_cmss = accel_turn_ne_cmss.length();
        turn_rate_rads = accel_turn_length_ne_cmss / vel_desired_length_ne_cms;
        if ((accel_turn_ne_cmss.y * _vel_desired_neu_cms.x - accel_turn_ne_cmss.x * _vel_desired_neu_cms.y) < 0.0) {
            turn_rate_rads = -turn_rate_rads;
        }
    }

    // update the target yaw if velocity is greater than 5% _vel_max_ne_cms
    if (vel_desired_length_ne_cms > _vel_max_ne_cms * 0.05f) {
        _yaw_target_rad = _vel_desired_neu_cms.xy().angle();
        _yaw_rate_target_rads = turn_rate_rads;
        return;
    }

    // use the current attitude controller yaw target
    _yaw_target_rad = _attitude_control.get_att_target_euler_rad().z;
    _yaw_rate_target_rads = 0;
}

// calculate_overspeed_gain - calculated increased maximum acceleration and jerk if over speed condition is detected
float AC_PosControl::calculate_overspeed_gain()
{
    if (_vel_desired_neu_cms.z < _vel_max_down_cms && !is_zero(_vel_max_down_cms)) {
        return POSCONTROL_OVERSPEED_GAIN_U * _vel_desired_neu_cms.z / _vel_max_down_cms;
    }
    if (_vel_desired_neu_cms.z > _vel_max_up_cms && !is_zero(_vel_max_up_cms)) {
        return POSCONTROL_OVERSPEED_GAIN_U * _vel_desired_neu_cms.z / _vel_max_up_cms;
    }
    return 1.0;
}

/// initialise ekf xy position reset check
void AC_PosControl::init_ekf_NE_reset()
{
    Vector2f pos_shift;
    _ekf_ne_reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
}

/// handle_ekf_NE_reset - check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::handle_ekf_NE_reset()
{
    // check for position shift
    Vector2f pos_shift;
    uint32_t reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
    if (reset_ms != _ekf_ne_reset_ms) {

        // ToDo: move EKF steps into the offsets for modes setting absolute position and velocity
        // for this we need some sort of switch to select what type of EKF handling we want to use

        // To zero real position shift during relative position modes like Loiter, PosHold, Guided velocity and accleration control.
        _pos_target_neu_cm.xy() = _pos_estimate_neu_cm.xy() + _p_pos_ne.get_error().topostype();
        _pos_desired_neu_cm.xy() = _pos_target_neu_cm.xy() - _pos_offset_neu_cm.xy();
        _vel_target_neu_cms.xy() = _vel_estimate_neu_cms.xy() + _pid_vel_ne.get_error();
        _vel_desired_neu_cms.xy() = _vel_target_neu_cms.xy() - _vel_offset_neu_cms.xy();

        _ekf_ne_reset_ms = reset_ms;
    }
}

/// initialise ekf z axis reset check
void AC_PosControl::init_ekf_U_reset()
{
    float alt_shift_d_m;
    _ekf_u_reset_ms = _ahrs.getLastPosDownReset(alt_shift_d_m);
}

/// handle_ekf_U_reset - check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::handle_ekf_U_reset()
{
    // check for position shift
    float alt_shift_d_m;
    uint32_t reset_ms = _ahrs.getLastPosDownReset(alt_shift_d_m);
    if (reset_ms != 0 && reset_ms != _ekf_u_reset_ms) {

        // ToDo: move EKF steps into the offsets for modes setting absolute position and velocity
        // for this we need some sort of switch to select what type of EKF handling we want to use

        // To zero real position shift during relative position modes like Loiter, PosHold, Guided velocity and accleration control.
        _pos_target_neu_cm.z = _pos_estimate_neu_cm.z + _p_pos_u.get_error();
        _pos_desired_neu_cm.z = _pos_target_neu_cm.z - (_pos_offset_neu_cm.z + _pos_terrain_u_cm);
        _vel_target_neu_cms.z = _vel_estimate_neu_cms.z + _pid_vel_u.get_error();
        _vel_desired_neu_cms.z = _vel_target_neu_cms.z - (_vel_offset_neu_cms.z + _vel_terrain_u_cms);

        _ekf_u_reset_ms = reset_ms;
    }
}

bool AC_PosControl::pre_arm_checks(const char *param_prefix,
                                   char *failure_msg,
                                   const uint8_t failure_msg_len)
{
    if (!is_positive(get_pos_NE_p().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_POSXY_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_pos_U_p().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_POSZ_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_vel_U_pid().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_VELZ_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_accel_U_pid().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ACCZ_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_accel_U_pid().kI())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ACCZ_I must be > 0", param_prefix);
        return false;
    }

    return true;
}

// return true if on a real vehicle or SITL with lock-step scheduling
bool AC_PosControl::has_good_timing(void) const
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    auto *sitl = AP::sitl();
    if (sitl) {
        return sitl->state.is_lock_step_scheduled;
    }
#endif
    // real boards are assumed to have good timing
    return true;
}
