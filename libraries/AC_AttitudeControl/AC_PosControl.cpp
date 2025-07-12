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
 # define POSCONTROL_VEL_XY_D                   0.25f   // horizontal velocity controller D gain default
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
    AP_SUBGROUPINFO(_p_pos_u_m, "_POSZ_", 2, AC_PosControl, AC_P_1D),

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
    AP_SUBGROUPINFO(_pid_vel_u_cm, "_VELZ_", 3, AC_PosControl, AC_PID_Basic),

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

    AP_SUBGROUPINFO(_pid_accel_u_cm_to_kt, "_ACCZ_", 4, AC_PosControl, AC_PID),

    // @Param: _POSXY_P
    // @DisplayName: Position (horizontal) controller P gain
    // @Description: Position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_ne_m, "_POSXY_", 5, AC_PosControl, AC_P_2D),

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
    // @Units: cm/s²
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
    AP_SUBGROUPINFO(_pid_vel_ne_cm, "_VELXY_", 6, AC_PosControl, AC_PID_2D),

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
    // @Units: m/s³
    // @Range: 1 20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_JERK_XY", 10, AC_PosControl, _shaping_jerk_ne_msss, POSCONTROL_JERK_NE_MSSS),

    // @Param: _JERK_Z
    // @DisplayName: Jerk limit for the vertical kinematic input shaping
    // @Description: Jerk limit of the vertical kinematic path generation used to determine how quickly the aircraft varies the acceleration target
    // @Units: m/s³
    // @Range: 5 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_JERK_Z", 11, AC_PosControl, _shaping_jerk_u_msss, POSCONTROL_JERK_U_MSSS),

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
    _p_pos_ne_m(POSCONTROL_POS_XY_P),
    _p_pos_u_m(POSCONTROL_POS_Z_P),
    _pid_vel_ne_cm(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, 0.0f, POSCONTROL_VEL_XY_IMAX, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ),
    _pid_vel_u_cm(POSCONTROL_VEL_Z_P, 0.0f, 0.0f, 0.0f, POSCONTROL_VEL_Z_IMAX, POSCONTROL_VEL_Z_FILT_HZ, POSCONTROL_VEL_Z_FILT_D_HZ),
    _pid_accel_u_cm_to_kt(POSCONTROL_ACC_Z_P, POSCONTROL_ACC_Z_I, POSCONTROL_ACC_Z_D, 0.0f, POSCONTROL_ACC_Z_IMAX, 0.0f, POSCONTROL_ACC_Z_FILT_HZ, 0.0f),
    _vel_max_ne_ms(POSCONTROL_SPEED_MS),
    _vel_max_up_ms(POSCONTROL_SPEED_UP_MS),
    _vel_max_down_ms(POSCONTROL_SPEED_DOWN_MS),
    _accel_max_ne_mss(POSCONTROL_ACCEL_NE_MSS),
    _accel_max_u_mss(POSCONTROL_ACCEL_U_MSS),
    _jerk_max_ne_msss(POSCONTROL_JERK_NE_MSSS),
    _jerk_max_u_msss(POSCONTROL_JERK_U_MSSS)
{
    AP_Param::setup_object_defaults(this, var_info);

    _singleton = this;
}


///
/// 3D position shaper
///

// Sets a new NEU position target in centimeters and computes a jerk-limited trajectory.
// Also updates vertical buffer logic using terrain altitude target.
// See input_pos_NEU_m() for full details.
void AC_PosControl::input_pos_NEU_cm(const Vector3p& pos_neu_cm, float pos_terrain_target_u_cm, float terrain_buffer_cm)
{
    input_pos_NEU_m(pos_neu_cm * 0.01, pos_terrain_target_u_cm * 0.01, terrain_buffer_cm * 0.01);
}

// Sets a new NEU position target in meters and computes a jerk-limited trajectory.
// Updates internal acceleration commands using a smooth kinematic path constrained
// by configured acceleration and jerk limits. Terrain margin is used to constrain
// horizontal velocity to avoid vertical buffer violation.
void AC_PosControl::input_pos_NEU_m(const Vector3p& pos_neu_m, float pos_terrain_target_u_m, float terrain_buffer_m)
{
    // Terrain following velocity scalar must be calculated before we remove the position offset
    const float offset_u_scalar = pos_terrain_U_scaler_m(pos_terrain_target_u_m, terrain_buffer_m);
    set_pos_terrain_target_U_m(pos_terrain_target_u_m);

    // calculated increased maximum acceleration and jerk if over speed
    const float overspeed_gain = calculate_overspeed_gain();
    const float accel_max_u_mss = _accel_max_u_mss * overspeed_gain;
    const float jerk_max_u_msss = _jerk_max_u_msss * overspeed_gain;

    update_pos_vel_accel_xy(_pos_desired_neu_m.xy(), _vel_desired_neu_ms.xy(), _accel_desired_neu_mss.xy(), _dt_s, _limit_vector_neu.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_cm.get_error());

    // adjust desired altitude if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_neu_m.z, _vel_desired_neu_ms.z, _accel_desired_neu_mss.z, _dt_s, _limit_vector_neu.z, _p_pos_u_m.get_error(), _pid_vel_u_cm.get_error());

    // calculate the horizontal and vertical velocity limits to travel directly to the destination defined by pos_ne_m
    float vel_max_ne_ms = 0.0f;
    float vel_max_u_ms = 0.0f;
    Vector3f travel_dir_unit = (pos_neu_m - _pos_desired_neu_m).tofloat();
    if (is_positive(travel_dir_unit.length_squared()) ) {
        travel_dir_unit.normalize();
        float travel_dir_unit_ne_length = travel_dir_unit.xy().length();

        float vel_max_ms = kinematic_limit(travel_dir_unit, _vel_max_ne_ms, _vel_max_up_ms, _vel_max_down_ms);
        vel_max_ne_ms = vel_max_ms * travel_dir_unit_ne_length;
        vel_max_u_ms = fabsf(vel_max_ms * travel_dir_unit.z);
    }

    // reduce speed if we are reaching the edge of our vertical buffer
    vel_max_ne_ms *= offset_u_scalar;

    Vector2f vel_ne_ms;
    Vector2f accel_ne_mss;
    shape_pos_vel_accel_xy(pos_neu_m.xy(), vel_ne_ms, accel_ne_mss, _pos_desired_neu_m.xy(), _vel_desired_neu_ms.xy(), _accel_desired_neu_mss.xy(),
                           vel_max_ne_ms, _accel_max_ne_mss, _jerk_max_ne_msss, _dt_s, false);

    float pos_u_m = pos_neu_m.z;
    shape_pos_vel_accel(pos_u_m, 0, 0,
                        _pos_desired_neu_m.z, _vel_desired_neu_ms.z, _accel_desired_neu_mss.z,
                        -vel_max_u_ms, vel_max_u_ms,
                        -constrain_float(accel_max_u_mss, 0.0, 7.5), accel_max_u_mss,
                        jerk_max_u_msss, _dt_s, false);
}

// Returns a scaling factor for horizontal velocity in cm/s to respect vertical terrain buffer.
// See pos_terrain_U_scaler_m() for full details.
float AC_PosControl::pos_terrain_U_scaler_cm(float pos_terrain_u_cm, float pos_terrain_u_buffer_cm) const
{
    return pos_terrain_U_scaler_m(pos_terrain_u_cm * 0.01, pos_terrain_u_buffer_cm * 0.01);
}

// Returns a scaling factor for horizontal velocity in m/s to ensure
// the vertical controller maintains a safe distance above terrain.
float AC_PosControl::pos_terrain_U_scaler_m(float pos_terrain_u_m, float pos_terrain_u_buffer_m) const
{
    if (is_zero(pos_terrain_u_buffer_m)) {
        return 1.0;
    }
    float pos_offset_error_u_m = _pos_estimate_neu_m.z - (_pos_target_neu_m.z + (pos_terrain_u_m - _pos_terrain_u_m));
    return constrain_float((1.0 - (fabsf(pos_offset_error_u_m) - 0.5 * pos_terrain_u_buffer_m) / (0.5 * pos_terrain_u_buffer_m)), 0.01, 1.0);
}

///
/// Lateral position controller
///

// Sets maximum horizontal speed (cm/s) and acceleration (cm/s²) for NE-axis shaping.
// Can be called anytime; transitions are handled smoothly.
// See set_max_speed_accel_NE_m() for full details.
void AC_PosControl::set_max_speed_accel_NE_cm(float speed_ne_cms, float accel_ne_cmss)
{
    set_max_speed_accel_NE_m(speed_ne_cms * 0.01, accel_ne_cmss * 0.01);
}

// Sets maximum horizontal speed (m/s) and acceleration (m/s²) for NE-axis shaping.
// These values constrain the kinematic trajectory used by the lateral controller.
void AC_PosControl::set_max_speed_accel_NE_m(float speed_ne_ms, float accel_ne_mss)
{
    _vel_max_ne_ms = speed_ne_ms;
    _accel_max_ne_mss = accel_ne_mss;

    // ensure the horizontal jerk is less than the vehicle is capable of
    const float jerk_max_msss = MIN(_attitude_control.get_ang_vel_roll_max_rads(), _attitude_control.get_ang_vel_pitch_max_rads()) * GRAVITY_MSS;
    const float snap_max_mssss = MIN(_attitude_control.get_accel_roll_max_radss(), _attitude_control.get_accel_pitch_max_radss()) * GRAVITY_MSS;

    // get specified jerk limit
    _jerk_max_ne_msss = _shaping_jerk_ne_msss;

    // limit maximum jerk based on maximum angular rate
    if (is_positive(jerk_max_msss) && _attitude_control.get_bf_feedforward()) {
        _jerk_max_ne_msss = MIN(_jerk_max_ne_msss, jerk_max_msss);
    }

    // limit maximum jerk to maximum possible average jerk based on angular acceleration
    if (is_positive(snap_max_mssss) && _attitude_control.get_bf_feedforward()) {
        _jerk_max_ne_msss = MIN(0.5 * safe_sqrt(_accel_max_ne_mss * snap_max_mssss), _jerk_max_ne_msss);
    }
}

// Sets horizontal correction limits for velocity (cm/s) and acceleration (cm/s²).
// Should be called only during initialization to avoid control discontinuities.
// See set_correction_speed_accel_NE_m() for full details.
void AC_PosControl::set_correction_speed_accel_NE_cm(float speed_ne_cms, float accel_ne_cmss)
{
    set_correction_speed_accel_NE_m(speed_ne_cms * 0.01, accel_ne_cmss * 0.01);
}

// Sets horizontal correction limits for velocity (m/s) and acceleration (m/s²).
// These values constrain the PID correction path, not the desired trajectory.
void AC_PosControl::set_correction_speed_accel_NE_m(float speed_ne_ms, float accel_ne_mss)
{
    _p_pos_ne_m.set_limits(speed_ne_ms, accel_ne_mss, 0.0f);
}

// Initializes NE controller to a stationary stopping point with zero velocity and acceleration.
// Use when the expected trajectory begins at rest but the starting position is unspecified.
// The starting position can be retrieved with get_pos_target_NEU_m().
void AC_PosControl::init_NE_controller_stopping_point()
{
    init_NE_controller();

    get_stopping_point_NE_m(_pos_desired_neu_m.xy());
    _pos_target_neu_m.xy() = _pos_desired_neu_m.xy() + _pos_offset_neu_m.xy();
    _vel_desired_neu_ms.xy().zero();
    _accel_desired_neu_mss.xy().zero();
}

// Smoothly decays NE acceleration over time to zero while maintaining current velocity and position.
// Reduces output acceleration by ~95% over 0.5 seconds to avoid abrupt transitions.
void AC_PosControl::relax_velocity_controller_NE()
{
    // decay acceleration and therefore current attitude target to zero
    // this will be reset by init_NE_controller() if !is_active_NE()
    if (is_positive(_dt_s)) {
        float decay = 1.0 - _dt_s / (_dt_s + POSCONTROL_RELAX_TC);
        _accel_target_neu_mss.xy() *= decay;
    }

    init_NE_controller();
}

// Softens NE controller for landing by reducing position error and suppressing I-term windup.
// Used to make descent behavior more stable near ground contact.
void AC_PosControl::soften_for_landing_NE()
{
    // decay position error to zero
    if (is_positive(_dt_s)) {
        _pos_target_neu_m.xy() += (_pos_estimate_neu_m.xy() - _pos_target_neu_m.xy()) * (_dt_s / (_dt_s + POSCONTROL_RELAX_TC));
        _pos_desired_neu_m.xy() = _pos_target_neu_m.xy() - _pos_offset_neu_m.xy();
    }

    // Prevent I term build up in xy velocity controller.
    // Note that this flag is reset on each loop in update_NE_controller()
    set_externally_limited_NE();
}

// Fully initializes the NE controller with current position, velocity, acceleration, and attitude.
// Intended for normal startup when the full state is known.
// Private function shared by other NE initializers.
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

    _pos_target_neu_m.xy() = _pos_estimate_neu_m.xy();
    _pos_desired_neu_m.xy() = _pos_target_neu_m.xy() - _pos_offset_neu_m.xy();

    _vel_target_neu_ms.xy() = _vel_estimate_neu_ms.xy();
    _vel_desired_neu_ms.xy() = _vel_target_neu_ms.xy() - _vel_offset_neu_ms.xy();

    // Set desired acceleration to zero because raw acceleration is prone to noise
    _accel_desired_neu_mss.xy().zero();

    if (!is_active_NE()) {
        lean_angles_to_accel_NE_mss(_accel_target_neu_mss.x, _accel_target_neu_mss.y);
    }

    // limit acceleration using maximum lean angles
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_mss = angle_rad_to_accel_mss(angle_max_rad);
    _accel_target_neu_mss.xy().limit_length(accel_max_mss);

    // initialise I terms from lean angles
    _pid_vel_ne_cm.reset_filter();
    // initialise the I term to (_accel_target_neu_mss - _accel_desired_neu_mss)
    // _accel_desired_neu_mss is zero and can be removed from the equation
    _pid_vel_ne_cm.set_integrator((_accel_target_neu_mss.xy() - _vel_target_neu_ms.xy() * _pid_vel_ne_cm.ff()) * 100.0);

    // initialise ekf xy reset handler
    init_ekf_NE_reset();

    // initialise z_controller time out
    _last_update_ne_ticks = AP::scheduler().ticks32();
}

// Sets the desired NE-plane acceleration in cm/s² using jerk-limited shaping.
// See input_accel_NE_m() for full details.
void AC_PosControl::input_accel_NE_cm(const Vector3f& accel_neu_cmss)
{
    input_accel_NE_m(accel_neu_cmss * 0.01);
}

// Sets the desired NE-plane acceleration in m/s² using jerk-limited shaping.
// Smoothly transitions to the specified acceleration from current kinematic state.
// Constraints: max acceleration and jerk set via set_max_speed_accel_NE_m().
void AC_PosControl::input_accel_NE_m(const Vector3f& accel_neu_mss)
{
    update_pos_vel_accel_xy(_pos_desired_neu_m.xy(), _vel_desired_neu_ms.xy(), _accel_desired_neu_mss.xy(), _dt_s, _limit_vector_neu.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_cm.get_error());
    shape_accel_xy(accel_neu_mss.xy(), _accel_desired_neu_mss.xy(), _jerk_max_ne_msss, _dt_s);
}

// Sets desired NE-plane velocity and acceleration (cm/s, cm/s²) using jerk-limited shaping.
// See input_vel_accel_NE_m() for full details.
void AC_PosControl::input_vel_accel_NE_cm(Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output)
{
    Vector2f vel_ne_ms = vel_ne_cms * 0.01;
    input_vel_accel_NE_m(vel_ne_ms, accel_ne_cmss * 0.01, limit_output);
    vel_ne_cms = vel_ne_ms * 100.0;
}

// Sets desired NE-plane velocity and acceleration (m/s, m/s²) using jerk-limited shaping.
// Calculates target acceleration using current kinematics constrained by acceleration and jerk limits.
// If `limit_output` is true, applies limits to total command (desired + correction).
void AC_PosControl::input_vel_accel_NE_m(Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss, bool limit_output)
{
    update_pos_vel_accel_xy(_pos_desired_neu_m.xy(), _vel_desired_neu_ms.xy(), _accel_desired_neu_mss.xy(), _dt_s, _limit_vector_neu.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_cm.get_error());

    shape_vel_accel_xy(vel_ne_ms, accel_ne_mss, _vel_desired_neu_ms.xy(), _accel_desired_neu_mss.xy(),
        _accel_max_ne_mss, _jerk_max_ne_msss, _dt_s, limit_output);

    update_vel_accel_xy(vel_ne_ms, accel_ne_mss, _dt_s, Vector2f(), Vector2f());
}

// Sets desired NE position, velocity, and acceleration (cm, cm/s, cm/s²) with jerk-limited shaping.
// See input_pos_vel_accel_NE_m() for full details.
void AC_PosControl::input_pos_vel_accel_NE_cm(Vector2p& pos_ne_cm, Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output)
{
    Vector2p pos_ne_m = pos_ne_cm * 0.01; 
    Vector2f vel_ne_ms = vel_ne_cms * 0.01;
    input_pos_vel_accel_NE_m(pos_ne_m, vel_ne_ms, accel_ne_cmss * 0.01, limit_output);
    pos_ne_cm = pos_ne_m * 100.0; 
    vel_ne_cms = vel_ne_ms * 100.0;
}

// Sets desired NE position, velocity, and acceleration (m, m/s, m/s²) with jerk-limited shaping.
// Calculates acceleration trajectory based on current kinematics and constraints.
// If `limit_output` is true, limits apply to full command (desired + correction).
void AC_PosControl::input_pos_vel_accel_NE_m(Vector2p& pos_ne_m, Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss, bool limit_output)
{
    update_pos_vel_accel_xy(_pos_desired_neu_m.xy(), _vel_desired_neu_ms.xy(), _accel_desired_neu_mss.xy(), _dt_s, _limit_vector_neu.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_cm.get_error());

    shape_pos_vel_accel_xy(pos_ne_m, vel_ne_ms, accel_ne_mss, _pos_desired_neu_m.xy(), _vel_desired_neu_ms.xy(), _accel_desired_neu_mss.xy(),
                           _vel_max_ne_ms, _accel_max_ne_mss, _jerk_max_ne_msss, _dt_s, limit_output);

    update_pos_vel_accel_xy(pos_ne_m, vel_ne_ms, accel_ne_mss, _dt_s, Vector2f(), Vector2f(), Vector2f());
}

// Updates NE offsets by gradually moving them toward their targets.
void AC_PosControl::update_offsets_NE()
{
    // Check if NE offset targets have timed out
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_ne_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        // Timeout: reset all NE offset targets to zero
        _pos_offset_target_neu_m.xy().zero();
        _vel_offset_target_neu_ms.xy().zero();
        _accel_offset_target_neu_mss.xy().zero();
    }

    // Advance offset target kinematic state (position, velocity, accel)
    update_pos_vel_accel_xy(_pos_offset_target_neu_m.xy(), _vel_offset_target_neu_ms.xy(), _accel_offset_target_neu_mss.xy(), _dt_s, Vector2f(), Vector2f(), Vector2f());
    update_pos_vel_accel_xy(_pos_offset_neu_m.xy(), _vel_offset_neu_ms.xy(), _accel_offset_neu_mss.xy(), _dt_s, _limit_vector_neu.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_cm.get_error());

    // Shape the offset path from current to target using jerk-limited smoothing
    shape_pos_vel_accel_xy(_pos_offset_target_neu_m.xy(), _vel_offset_target_neu_ms.xy(), _accel_offset_target_neu_mss.xy(),
                            _pos_offset_neu_m.xy(), _vel_offset_neu_ms.xy(), _accel_offset_neu_mss.xy(),
                            _vel_max_ne_ms, _accel_max_ne_mss, _jerk_max_ne_msss, _dt_s, false);
}

// Disables NE position correction by setting the target position to the current position.
// Useful to freeze positional control without disrupting velocity control.
void AC_PosControl::stop_pos_NE_stabilisation()
{
    _pos_target_neu_m.xy() = _pos_estimate_neu_m.xy();
    _pos_desired_neu_m.xy() = _pos_target_neu_m.xy() - _pos_offset_neu_m.xy();
}

// Disables NE position and velocity correction by setting target values to current state.
// Useful to prevent further corrections and freeze motion stabilization in NE axes.
void AC_PosControl::stop_vel_NE_stabilisation()
{
    _pos_target_neu_m.xy() =  _pos_estimate_neu_m.xy();
    _pos_desired_neu_m.xy() = _pos_target_neu_m.xy() - _pos_offset_neu_m.xy();
    
    _vel_target_neu_ms.xy() = _vel_estimate_neu_ms.xy();
    _vel_desired_neu_ms.xy() = _vel_target_neu_ms.xy() - _vel_offset_neu_ms.xy();

    // initialise I terms from lean angles
    _pid_vel_ne_cm.reset_filter();
    _pid_vel_ne_cm.reset_I();
}

// Returns true if the NE position controller has run in the last 5 control loop cycles.
bool AC_PosControl::is_active_NE() const
{
    const uint32_t dt_ticks = AP::scheduler().ticks32() - _last_update_ne_ticks;
    return dt_ticks <= 1;
}

// Uses P and PID controllers to generate corrections which are added to feedforward velocity/acceleration.
// Requires all desired targets to be pre-set using the input_* or set_* methods.
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

    // Update lateral position, velocity, and acceleration offsets using path shaping
    update_offsets_NE();

    // Position Controller

    // Combine position target with active NE offset to get absolute target
    _pos_target_neu_m.xy() = _pos_desired_neu_m.xy() + _pos_offset_neu_m.xy();

    // determine the combined position of the actual position and the disturbance from system ID mode
    // calculate the target velocity correction
    Vector2p comb_pos_ne_m = _pos_estimate_neu_m.xy();
    comb_pos_ne_m += _disturb_pos_ne_m.topostype();

    // Run P controller to compute velocity setpoint from position error
    Vector2f vel_target_ne_ms = _p_pos_ne_m.update_all(_pos_target_neu_m.xy(), comb_pos_ne_m);
    _pos_desired_neu_m.xy() = _pos_target_neu_m.xy() - _pos_offset_neu_m.xy();

    // Velocity Controller

    // Apply AHRS scaling (e.g. for optical flow noise compensation)
    vel_target_ne_ms *= ahrsControlScaleXY;
    vel_target_ne_ms *= _ne_control_scale_factor;

    _vel_target_neu_ms.xy() = vel_target_ne_ms;
    _vel_target_neu_ms.xy() += _vel_desired_neu_ms.xy() + _vel_offset_neu_ms.xy();

    // Velocity Controller

    // determine the combined velocity of the actual velocity and the disturbance from system ID mode
    Vector2f comb_vel_ne_ms = _vel_estimate_neu_ms.xy();
    comb_vel_ne_ms += _disturb_vel_ne_ms;

    // Run velocity PID controller and scale result for control authority
    Vector2f accel_target_ne_mss = _pid_vel_ne_cm.update_all(_vel_target_neu_ms.xy() * 100.0, comb_vel_ne_ms * 100.0, _dt_s, _limit_vector_neu.xy()) * 0.01;

    // Acceleration Controller
    
    // Apply AHRS scaling again to correct for measurement distortions
    accel_target_ne_mss *= ahrsControlScaleXY;
    accel_target_ne_mss *= _ne_control_scale_factor;

    _ne_control_scale_factor = 1.0;

    // pass the correction acceleration to the target acceleration output
    _accel_target_neu_mss.xy() = accel_target_ne_mss;
    _accel_target_neu_mss.xy() += _accel_desired_neu_mss.xy() + _accel_offset_neu_mss.xy();

    // limit acceleration using maximum lean angles
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_mss = angle_rad_to_accel_mss(angle_max_rad);
    // Save unbounded target for use in "limited" check (not unit-consistent with z!)
    _limit_vector_neu.xy() = _accel_target_neu_mss.xy();
    if (!limit_accel_xy(_vel_desired_neu_ms.xy(), _accel_target_neu_mss.xy(), accel_max_mss)) {
        // _accel_target_neu_mss was not limited so we can zero the xy limit vector
        _limit_vector_neu.xy().zero();
    }

    // Convert acceleration to roll/pitch angle targets (used by attitude controller)
    accel_NE_mss_to_lean_angles_rad(_accel_target_neu_mss.x, _accel_target_neu_mss.y, _roll_target_rad, _pitch_target_rad);

    // Update yaw and yaw rate targets to match heading of motion
    calculate_yaw_and_rate_yaw();

    // reset the disturbance from system ID mode to zero
    _disturb_pos_ne_m.zero();
    _disturb_vel_ne_ms.zero();
}


///
/// Vertical position controller
///

// Sets maximum climb/descent rate (cm/s) and vertical acceleration (cm/s²) for the U-axis.
// Descent rate may be positive or negative and is always interpreted as a descent.
// See set_max_speed_accel_U_m() for full details.
void AC_PosControl::set_max_speed_accel_U_cm(float speed_down_cms, float speed_up_cms, float accel_cmss)
{
    set_max_speed_accel_U_m(speed_down_cms * 0.01, speed_up_cms * 0.01, accel_cmss * 0.01);
}

// Sets maximum climb/descent rate (m/s) and vertical acceleration (m/s²) for the U-axis.
// These values are used for jerk-limited kinematic shaping of the vertical trajectory.
void AC_PosControl::set_max_speed_accel_U_m(float speed_down_ms, float speed_up_ms, float accel_mss)
{
    // ensure speed_down_ms is always negative
    speed_down_ms = -fabsf(speed_down_ms);

    // sanity check and update
    if (is_negative(speed_down_ms)) {
        _vel_max_down_ms = speed_down_ms;
    }
    if (is_positive(speed_up_ms)) {
        _vel_max_up_ms = speed_up_ms;
    }
    if (is_positive(accel_mss)) {
        _accel_max_u_mss = accel_mss;
    }

    // ensure the vertical Jerk is not limited by the filters in the Z acceleration PID object
    _jerk_max_u_msss = _shaping_jerk_u_msss;
    if (is_positive(_pid_accel_u_cm_to_kt.filt_T_hz())) {
        _jerk_max_u_msss = MIN(_jerk_max_u_msss, MIN(GRAVITY_MSS, _accel_max_u_mss) * (M_2PI * _pid_accel_u_cm_to_kt.filt_T_hz()) / 5.0);
    }
    if (is_positive(_pid_accel_u_cm_to_kt.filt_E_hz())) {
        _jerk_max_u_msss = MIN(_jerk_max_u_msss, MIN(GRAVITY_MSS, _accel_max_u_mss) * (M_2PI * _pid_accel_u_cm_to_kt.filt_E_hz()) / 5.0);
    }
}

// Sets vertical correction velocity and acceleration limits (cm/s, cm/s²).
// Should only be called during initialization to avoid discontinuities.
// See set_correction_speed_accel_U_mss() for full details.
void AC_PosControl::set_correction_speed_accel_U_cmss(float speed_down_cms, float speed_up_cms, float accel_cmss)
{
    set_correction_speed_accel_U_mss(speed_down_cms * 0.01, speed_up_cms * 0.01, accel_cmss * 0.01);
}

// Sets vertical correction velocity and acceleration limits (m/s, m/s²).
// These values constrain the correction output of the PID controller.
void AC_PosControl::set_correction_speed_accel_U_mss(float speed_down_ms, float speed_up_ms, float accel_mss)
{
    // define maximum position error and maximum first and second differential limits
    _p_pos_u_m.set_limits(-fabsf(speed_down_ms), speed_up_ms, accel_mss, 0.0f);
}

// Initializes U-axis controller to current position, velocity, and acceleration, disallowing descent.
// Used for takeoff or hold scenarios where downward motion is prohibited.
void AC_PosControl::init_U_controller_no_descent()
{
    // Initialise the position controller to the current throttle, position, velocity and acceleration.
    init_U_controller();

    // remove all descent if present
    _vel_target_neu_ms.z = MAX(0.0, _vel_target_neu_ms.z);
    _vel_desired_neu_ms.z = MAX(0.0, _vel_desired_neu_ms.z);
    _vel_terrain_u_ms = MAX(0.0, _vel_terrain_u_ms);
    _vel_offset_neu_ms.z = MAX(0.0, _vel_offset_neu_ms.z);
    _accel_target_neu_mss.z = MAX(0.0, _accel_target_neu_mss.z);
    _accel_desired_neu_mss.z = MAX(0.0, _accel_desired_neu_mss.z);
    _accel_terrain_u_mss = MAX(0.0, _accel_terrain_u_mss);
    _accel_offset_neu_mss.z = MAX(0.0, _accel_offset_neu_mss.z);
}

// Initializes U-axis controller to a stationary stopping point with zero velocity and acceleration.
// Used when the trajectory starts at rest but the initial altitude is unspecified.
// The resulting position target can be retrieved with get_pos_target_NEU_m().
void AC_PosControl::init_U_controller_stopping_point()
{
    // Initialise the position controller to the current throttle, position, velocity and acceleration.
    init_U_controller();

    get_stopping_point_U_m(_pos_desired_neu_m.z);
    _pos_target_neu_m.z = _pos_desired_neu_m.z + _pos_offset_neu_m.z;
    _vel_desired_neu_ms.z = 0.0f;
    _accel_desired_neu_mss.z = 0.0f;
}

// Smoothly decays U-axis acceleration to zero over time while maintaining current vertical velocity.
// Reduces requested acceleration by ~95% every 0.5 seconds to avoid abrupt transitions.
// `throttle_setting` is used to determine whether to preserve positive acceleration in low-thrust cases.
void AC_PosControl::relax_U_controller(float throttle_setting)
{
    // Initialise the position controller to the current position, velocity and acceleration.
    init_U_controller();

    // init_U_controller has set the acceleration PID I term to generate the current throttle set point
    // Use relax_integrator to decay the throttle set point to throttle_setting
    _pid_accel_u_cm_to_kt.relax_integrator((throttle_setting - _motors.get_throttle_hover()) * 10.0 * 100.0, _dt_s, POSCONTROL_RELAX_TC);
}

// Fully initializes the U-axis controller with current position, velocity, acceleration, and attitude.
// Used during standard controller activation when full state is known.
// Private function shared by other vertical initializers.
void AC_PosControl::init_U_controller()
{
    // initialise terrain targets and offsets to zero
    init_terrain();

    // initialise offsets to target offsets and ensure offset targets are zero if they have not been updated.
    init_offsets_U();

    _pos_target_neu_m.z = _pos_estimate_neu_m.z;
    _pos_desired_neu_m.z = _pos_target_neu_m.z - _pos_offset_neu_m.z;

    _vel_target_neu_ms.z = _vel_estimate_neu_ms.z;
    _vel_desired_neu_ms.z = _vel_target_neu_ms.z - _vel_offset_neu_ms.z;

    // Reset I term of velocity PID
    _pid_vel_u_cm.reset_filter();
    _pid_vel_u_cm.set_integrator(0.0f);

    _accel_target_neu_mss.z = constrain_float(get_measured_accel_U_mss(), -_accel_max_u_mss, _accel_max_u_mss);
    _accel_desired_neu_mss.z = _accel_target_neu_mss.z - (_accel_offset_neu_mss.z + _accel_terrain_u_mss);
    _pid_accel_u_cm_to_kt.reset_filter();

    // Set acceleration PID I term based on the current throttle
    // Remove the expected P term due to _accel_desired_neu_mss.z being constrained to _accel_max_u_mss
    // Remove the expected FF term due to non-zero _accel_target_neu_mss.z
    _pid_accel_u_cm_to_kt.set_integrator((_attitude_control.get_throttle_in() - _motors.get_throttle_hover()) * 10.0 * 100.0
        - _pid_accel_u_cm_to_kt.kP() * (_accel_target_neu_mss.z - get_measured_accel_U_mss()) * 100.0
        - _pid_accel_u_cm_to_kt.ff() * _accel_target_neu_mss.z * 100.0);

    // initialise ekf z reset handler
    init_ekf_U_reset();

    // initialise z_controller time out
    _last_update_u_ticks = AP::scheduler().ticks32();
}

// Sets the desired vertical acceleration in cm/s² using jerk-limited shaping.
// See input_accel_U_m() for full details.
void AC_PosControl::input_accel_U_cm(float accel_cmss)
{
    input_accel_U_m(accel_cmss * 0.01);
}

// Sets the desired vertical acceleration in m/s² using jerk-limited shaping.
// Smoothly transitions to the target acceleration from current kinematic state.
// Constraints: max acceleration and jerk set via set_max_speed_accel_U_m().
void AC_PosControl::input_accel_U_m(float accel_mss)
{
    // calculated increased maximum jerk if over speed
    float jerk_max_u_msss = _jerk_max_u_msss * calculate_overspeed_gain();

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_neu_m.z, _vel_desired_neu_ms.z, _accel_desired_neu_mss.z, _dt_s, _limit_vector_neu.z, _p_pos_u_m.get_error(), _pid_vel_u_cm.get_error());

    shape_accel(accel_mss, _accel_desired_neu_mss.z, jerk_max_u_msss, _dt_s);
}

// Sets desired vertical velocity and acceleration (cm/s, cm/s²) using jerk-limited shaping.
// See input_vel_accel_U_m() for full details.
void AC_PosControl::input_vel_accel_U_cm(float &vel_u_cms, float accel_cmss, bool limit_output)
{
    float vel_u_ms = vel_u_cms * 0.01; 
    input_vel_accel_U_m(vel_u_ms, accel_cmss * 0.01, limit_output);
    vel_u_cms = vel_u_ms * 100.0;
}

// Sets desired vertical velocity and acceleration (m/s, m/s²) using jerk-limited shaping.
// Calculates required acceleration using current vertical kinematics.
// If `limit_output` is true, limits apply to the combined (desired + correction) command.
void AC_PosControl::input_vel_accel_U_m(float &vel_u_ms, float accel_mss, bool limit_output)
{
    // calculated increased maximum acceleration and jerk if over speed
    const float overspeed_gain = calculate_overspeed_gain();
    const float accel_max_u_mss = _accel_max_u_mss * overspeed_gain;
    const float jerk_max_u_msss = _jerk_max_u_msss * overspeed_gain;

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_neu_m.z, _vel_desired_neu_ms.z, _accel_desired_neu_mss.z, _dt_s, _limit_vector_neu.z, _p_pos_u_m.get_error(), _pid_vel_u_cm.get_error());

    shape_vel_accel(vel_u_ms, accel_mss,
                    _vel_desired_neu_ms.z, _accel_desired_neu_mss.z,
                    -constrain_float(accel_max_u_mss, 0.0, 7.5), accel_max_u_mss,
                    jerk_max_u_msss, _dt_s, limit_output);

    update_vel_accel(vel_u_ms, accel_mss, _dt_s, 0.0, 0.0);
}

// Generates a vertical trajectory using the given climb rate in cm/s and jerk-limited shaping.
// Adjusts the internal target altitude based on integrated climb rate.
// See set_pos_target_U_from_climb_rate_m() for full details.
void AC_PosControl::set_pos_target_U_from_climb_rate_cm(float vel_u_cms)
{
    set_pos_target_U_from_climb_rate_m(vel_u_cms * 0.01);
}

// Generates a vertical trajectory using the given climb rate in m/s and jerk-limited shaping.
// Target altitude is updated over time by integrating the climb rate.
void AC_PosControl::set_pos_target_U_from_climb_rate_m(float vel_u_ms)
{
    input_vel_accel_U_m(vel_u_ms, 0.0);
}

// Descends at a given rate (cm/s) using jerk-limited shaping for landing.
// If `ignore_descent_limit` is true, descent output is not limited by the configured max.
// See land_at_climb_rate_m() for full details.
void AC_PosControl::land_at_climb_rate_cm(float vel_u_cms, bool ignore_descent_limit)
{
    land_at_climb_rate_m(vel_u_cms * 0.01, ignore_descent_limit);
}

// Descends at a given rate (m/s) using jerk-limited shaping for landing.
// Used during final descent phase to ensure smooth touchdown.
void AC_PosControl::land_at_climb_rate_m(float vel_u_ms, bool ignore_descent_limit)
{
    if (ignore_descent_limit) {
        // turn off limits in the negative z direction
        _limit_vector_neu.z = MAX(_limit_vector_neu.z, 0.0f);
    }

    input_vel_accel_U_m(vel_u_ms, 0.0);
}

// Sets vertical position, velocity, and acceleration in cm using jerk-limited shaping.
// See input_pos_vel_accel_U_m() for full details.
void AC_PosControl::input_pos_vel_accel_U_cm(float &pos_u_cm, float &vel_u_cms, float accel_cmss, bool limit_output)
{
    float pos_u_m = pos_u_cm * 0.01;
    float vel_u_ms = vel_u_cms * 0.01;
    input_pos_vel_accel_U_m(pos_u_m, vel_u_ms, accel_cmss * 0.01, limit_output);
    pos_u_cm = pos_u_m * 100.0;
    vel_u_cms = vel_u_ms * 100.0;
}

// Sets vertical position, velocity, and acceleration in meters using jerk-limited shaping.
// Calculates required acceleration using current state and constraints.
// If `limit_output` is true, limits are applied to combined (desired + correction) command.
void AC_PosControl::input_pos_vel_accel_U_m(float &pos_u_m, float &vel_u_ms, float accel_mss, bool limit_output)
{
    // calculated increased maximum acceleration and jerk if over speed
    const float overspeed_gain = calculate_overspeed_gain();
    const float accel_max_u_mss = _accel_max_u_mss * overspeed_gain;
    const float jerk_max_u_msss = _jerk_max_u_msss * overspeed_gain;

    // adjust desired altitude if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_neu_m.z, _vel_desired_neu_ms.z, _accel_desired_neu_mss.z, _dt_s, _limit_vector_neu.z, _p_pos_u_m.get_error(), _pid_vel_u_cm.get_error());

    shape_pos_vel_accel(pos_u_m, vel_u_ms, accel_mss,
                        _pos_desired_neu_m.z, _vel_desired_neu_ms.z, _accel_desired_neu_mss.z,
                        _vel_max_down_ms, _vel_max_up_ms,
                        -constrain_float(accel_max_u_mss, 0.0, 7.5), accel_max_u_mss,
                        jerk_max_u_msss, _dt_s, limit_output);

    postype_t posp = pos_u_m;
    update_pos_vel_accel(posp, vel_u_ms, accel_mss, _dt_s, 0.0, 0.0, 0.0);
    pos_u_m = posp;
}

// Sets target altitude in cm using jerk-limited shaping to gradually move to the new position.
// See set_alt_target_with_slew_m() for full details.
void AC_PosControl::set_alt_target_with_slew_cm(float pos_u_cm)
{
    set_alt_target_with_slew_m(pos_u_cm * 0.01);
}

// Sets target altitude in meters using jerk-limited shaping.
void AC_PosControl::set_alt_target_with_slew_m(float pos_u_m)
{
    float zero = 0;
    input_pos_vel_accel_U_m(pos_u_m, zero, 0);
}

// Updates vertical (U) offsets by gradually moving them toward their targets.
void AC_PosControl::update_offsets_U()
{
    // Check if vertical offset targets have timed out
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_u_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        // Timeout: reset U-axis offset targets to zero
        _pos_offset_target_neu_m.z = 0.0;
        _vel_offset_target_neu_ms.z = 0.0;
        _accel_offset_target_neu_mss.z = 0.0;
    }

    // Advance current offset state using PID-derived feedback and vertical limits
    postype_t p_offset_u_m = _pos_offset_neu_m.z;
    update_pos_vel_accel(p_offset_u_m, _vel_offset_neu_ms.z, _accel_offset_neu_mss.z, _dt_s, MIN(_limit_vector_neu.z, 0.0f), _p_pos_u_m.get_error(), _pid_vel_u_cm.get_error());
    _pos_offset_neu_m.z = p_offset_u_m;

    // Shape offset trajectory (position/velocity/acceleration) using jerk-limited smoothing
    shape_pos_vel_accel(_pos_offset_target_neu_m.z, _vel_offset_target_neu_ms.z, _accel_offset_target_neu_mss.z,
        _pos_offset_neu_m.z, _vel_offset_neu_ms.z, _accel_offset_neu_mss.z,
        get_max_speed_down_ms(), get_max_speed_up_ms(),
        -get_max_accel_U_mss(), get_max_accel_U_mss(),
        _jerk_max_u_msss, _dt_s, false);

    // Update target state forward in time with assumed zero velocity/acceleration targets
    p_offset_u_m = _pos_offset_target_neu_m.z;
    update_pos_vel_accel(p_offset_u_m, _vel_offset_target_neu_ms.z, _accel_offset_target_neu_mss.z, _dt_s, 0.0, 0.0, 0.0);
    _pos_offset_target_neu_m.z = p_offset_u_m;
}

// Returns true if the U-axis controller has run in the last 5 control loop cycles.
bool AC_PosControl::is_active_U() const
{
    const uint32_t dt_ticks = AP::scheduler().ticks32() - _last_update_u_ticks;
    return dt_ticks <= 1;
}

// Runs the vertical (U-axis) position controller.
// Computes output acceleration based on position and velocity errors using PID correction.
// Feedforward velocity and acceleration are combined with corrections to produce a smooth vertical command.
// Desired position, velocity, and acceleration must be set before calling.
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

    // Update vertical offset targets and terrain estimate
    update_offsets_U();
    update_terrain();

    // Position Controller

    // Combine desired + offset + terrain for final position target
    _pos_target_neu_m.z = _pos_desired_neu_m.z + _pos_offset_neu_m.z + _pos_terrain_u_m;

    // calculate the target velocity correction
    float pos_target_zf = _pos_target_neu_m.z;

    // P controller: convert position error to velocity target
    _vel_target_neu_ms.z = _p_pos_u_m.update_all(pos_target_zf, _pos_estimate_neu_m.z);
    _vel_target_neu_ms.z *= AP::ahrs().getControlScaleZ();

    _pos_target_neu_m.z = pos_target_zf;
    _pos_desired_neu_m.z = _pos_target_neu_m.z - (_pos_offset_neu_m.z + _pos_terrain_u_m);

    // add feed forward component
    _vel_target_neu_ms.z += _vel_desired_neu_ms.z + _vel_offset_neu_ms.z + _vel_terrain_u_ms;

    // Velocity Controller

    // PID controller: convert velocity error to acceleration
    _accel_target_neu_mss.z = _pid_vel_u_cm.update_all(_vel_target_neu_ms.z * 100.0, _vel_estimate_neu_ms.z * 100.0, _dt_s, _motors.limit.throttle_lower, _motors.limit.throttle_upper) * 0.01;
    _accel_target_neu_mss.z *= AP::ahrs().getControlScaleZ();

    // add feed forward component
    _accel_target_neu_mss.z += _accel_desired_neu_mss.z + _accel_offset_neu_mss.z + _accel_terrain_u_mss;

    // Acceleration Controller

    // Gravity-compensated vertical acceleration measurement (positive = up)
    const float measured_accel_u_mss = get_measured_accel_U_mss();

    // Ensure integrator can produce enough thrust to overcome hover throttle
    if (_motors.get_throttle_hover() * 1000.0 > _pid_accel_u_cm_to_kt.imax()) {
        _pid_accel_u_cm_to_kt.set_imax(_motors.get_throttle_hover() * 1000.0);
    }
    float thr_out;
    if (_vibe_comp_enabled) {
        // Use vibration-resistant throttle estimator (feedforward + scaled integrator)
        thr_out = get_throttle_with_vibration_override();
    } else {
        // Standard PID update using vertical acceleration error
        thr_out = _pid_accel_u_cm_to_kt.update_all(_accel_target_neu_mss.z * 100.0, measured_accel_u_mss * 100.0, _dt_s, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001;
        // Include FF contribution to reduce delay
        thr_out += _pid_accel_u_cm_to_kt.get_ff() * 0.001;
    }
    thr_out += _motors.get_throttle_hover();

    // Actuator commands

    // Send final throttle output to attitude controller (includes angle boost)
    _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);

    // Check for vertical controller health

    // Update health indicator based on error magnitude vs configured speed range
    float error_ratio = _pid_vel_u_cm.get_error() * 0.01 / _vel_max_down_ms;
    _vel_u_control_ratio += _dt_s * 0.1f * (0.5 - error_ratio);
    _vel_u_control_ratio = constrain_float(_vel_u_control_ratio, 0.0f, 2.0f);

    // set vertical component of the limit vector
    if (_motors.limit.throttle_upper) {
        _limit_vector_neu.z = 1.0f;
    } else if (_motors.limit.throttle_lower) {
        _limit_vector_neu.z = -1.0f;
    } else {
        _limit_vector_neu.z = 0.0f;
    }
}


///
/// Accessors
///

// Returns the maximum allowed roll/pitch angle in radians.
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

// Sets externally computed NEU position, velocity, and acceleration in centimeters, cm/s, and cm/s².
// See set_pos_vel_accel_NEU_m() for full details.
void AC_PosControl::set_pos_vel_accel_NEU_cm(const Vector3p& pos_neu_cm, const Vector3f& vel_neu_cms, const Vector3f& accel_neu_cmss)
{
    set_pos_vel_accel_NEU_m(pos_neu_cm * 0.01, vel_neu_cms * 0.01, accel_neu_cmss * 0.01);
}

// Sets externally computed NEU position, velocity, and acceleration in meters, m/s, and m/s².
// Use when path planning or shaping is done outside this controller.
void AC_PosControl::set_pos_vel_accel_NEU_m(const Vector3p& pos_neu_m, const Vector3f& vel_neu_ms, const Vector3f& accel_neu_mss)
{
    _pos_desired_neu_m = pos_neu_m;
    _vel_desired_neu_ms = vel_neu_ms;
    _accel_desired_neu_mss = accel_neu_mss;
}

// Sets externally computed NE position, velocity, and acceleration in centimeters, cm/s, and cm/s².
// See set_pos_vel_accel_NE_m() for full details.
void AC_PosControl::set_pos_vel_accel_NE_cm(const Vector2p& pos_ne_cm, const Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss)
{
    set_pos_vel_accel_NE_m(pos_ne_cm * 0.01, vel_ne_cms * 0.01, accel_ne_cmss * 0.01);
}

// Sets externally computed NE position, velocity, and acceleration in meters, m/s, and m/s².
// Use when path planning or shaping is done outside this controller.
void AC_PosControl::set_pos_vel_accel_NE_m(const Vector2p& pos_ne_m, const Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss)
{
    _pos_desired_neu_m.xy() = pos_ne_m;
    _vel_desired_neu_ms.xy() = vel_ne_ms;
    _accel_desired_neu_mss.xy() = accel_ne_mss;
}

// Converts lean angles (rad) to NEU acceleration in cm/s².
// See lean_angles_rad_to_accel_NEU_mss() for full details.
Vector3f AC_PosControl::lean_angles_rad_to_accel_NEU_cmss(const Vector3f& att_target_euler_rad) const
{
    return lean_angles_rad_to_accel_NEU_mss(att_target_euler_rad) * 100.0;
}

// Converts lean angles (rad) to NEU acceleration in m/s².
Vector3f AC_PosControl::lean_angles_rad_to_accel_NEU_mss(const Vector3f& att_target_euler_rad) const
{
    // rotate our roll, pitch angles into lat/lon frame
    const float sin_roll = sinf(att_target_euler_rad.x);
    const float cos_roll = cosf(att_target_euler_rad.x);
    const float sin_pitch = sinf(att_target_euler_rad.y);
    const float cos_pitch = cosf(att_target_euler_rad.y);
    const float sin_yaw = sinf(att_target_euler_rad.z);
    const float cos_yaw = cosf(att_target_euler_rad.z);

    return Vector3f{
        GRAVITY_MSS * (-cos_yaw * sin_pitch * cos_roll - sin_yaw * sin_roll) / MAX(cos_roll * cos_pitch, 0.1f),
        GRAVITY_MSS * (-sin_yaw * sin_pitch * cos_roll + cos_yaw * sin_roll) / MAX(cos_roll * cos_pitch, 0.1f),
        GRAVITY_MSS
    };
}

/// Terrain

// Initializes terrain position, velocity, and acceleration to match the terrain target.
void AC_PosControl::init_terrain()
{
    // set terrain position and target to zero
    _pos_terrain_target_u_m = 0.0;
    _pos_terrain_u_m = 0.0;

    // set velocity offset to zero
    _vel_terrain_u_ms = 0.0;

    // set acceleration offset to zero
    _accel_terrain_u_mss = 0.0;
}

// Initializes terrain altitude and terrain target to the same value (in cm).
// See init_pos_terrain_U_m() for full details.
void AC_PosControl::init_pos_terrain_U_cm(float pos_terrain_u_cm)
{
    init_pos_terrain_U_m(pos_terrain_u_cm * 0.01);
}

// Initializes terrain altitude and terrain target to the same value (in meters).
void AC_PosControl::init_pos_terrain_U_m(float pos_terrain_u_m)
{
    _pos_desired_neu_m.z -= (pos_terrain_u_m - _pos_terrain_u_m);
    _pos_terrain_target_u_m = pos_terrain_u_m;
    _pos_terrain_u_m = pos_terrain_u_m;
}


/// Offsets

// Initializes NE position/velocity/acceleration offsets to match their respective targets.
void AC_PosControl::init_offsets_NE()
{
    // check for offset target timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_ne_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        _pos_offset_target_neu_m.xy().zero();
        _vel_offset_target_neu_ms.xy().zero();
        _accel_offset_target_neu_mss.xy().zero();
    }

    // set position offset to target
    _pos_offset_neu_m.xy() = _pos_offset_target_neu_m.xy();

    // set velocity offset to target
    _vel_offset_neu_ms.xy() = _vel_offset_target_neu_ms.xy();

    // set acceleration offset to target
    _accel_offset_neu_mss.xy() = _accel_offset_target_neu_mss.xy();
}

// Initializes vertical (U) offsets to match their respective targets.
void AC_PosControl::init_offsets_U()
{
    // check for offset target timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_u_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        _pos_offset_target_neu_m.z = 0.0;
        _vel_offset_target_neu_ms.z = 0.0;
        _accel_offset_target_neu_mss.z = 0.0;
    }

    // set position offset to target
    _pos_offset_neu_m.z = _pos_offset_target_neu_m.z;

    // set velocity offset to target
    _vel_offset_neu_ms.z = _vel_offset_target_neu_ms.z;

    // set acceleration offset to target
    _accel_offset_neu_mss.z = _accel_offset_target_neu_mss.z;
}

#if AP_SCRIPTING_ENABLED
// Sets additional position, velocity, and acceleration offsets in meters (NED frame) for scripting.
// Offsets are added to the controller’s internal target.
// Used in LUA
bool AC_PosControl::set_posvelaccel_offset(const Vector3f &pos_offset_NED_m, const Vector3f &vel_offset_NED_ms, const Vector3f &accel_offset_NED_mss)
{
    // Convert NED inputs to NEU frame: Z is inverted
    set_posvelaccel_offset_target_NE_m(pos_offset_NED_m.topostype().xy(), vel_offset_NED_ms.xy(), accel_offset_NED_mss.xy());
    set_posvelaccel_offset_target_U_m(-pos_offset_NED_m.topostype().z, -vel_offset_NED_ms.z, -accel_offset_NED_mss.z);
    return true;
}

// Retrieves current scripted offsets in meters (NED frame).
// Used in LUA
bool AC_PosControl::get_posvelaccel_offset(Vector3f &pos_offset_NED_m, Vector3f &vel_offset_NED_ms, Vector3f &accel_offset_NED_mss)
{
    // Convert from internal NEU to NED by inverting Z
    pos_offset_NED_m.xy() = _pos_offset_target_neu_m.xy().tofloat();
    pos_offset_NED_m.z = -_pos_offset_target_neu_m.z;

    vel_offset_NED_ms.xy() = _vel_offset_target_neu_ms.xy();
    vel_offset_NED_ms.z = -_vel_offset_target_neu_ms.z;

    accel_offset_NED_mss.xy() = _accel_offset_target_neu_mss.xy();
    accel_offset_NED_mss.z = -_accel_offset_target_neu_mss.z;
    return true;
}

// Retrieves current target velocity (NED frame, m/s) including any scripted offset.
// Used in LUA
bool AC_PosControl::get_vel_target(Vector3f &vel_target_NED_ms)
{
    if (!is_active_NE() || !is_active_U()) {
        return false;
    }

    // Convert NEU → NED by inverting Z
    vel_target_NED_ms.xy() = _vel_target_neu_ms.xy();
    vel_target_NED_ms.z = -_vel_target_neu_ms.z;
    return true;
}

// Retrieves current target acceleration (NED frame, m/s²) including any scripted offset.
// Used in LUA
bool AC_PosControl::get_accel_target(Vector3f &accel_target_NED_mss)
{
    if (!is_active_NE() || !is_active_U()) {
        return false;
    }

    // Convert NEU → NED by inverting Z
    accel_target_NED_mss.xy() = _accel_target_neu_mss.xy();
    accel_target_NED_mss.z = -_accel_target_neu_mss.z;
    return true;
}
#endif

// Sets NE offset targets (position [cm], velocity [cm/s], acceleration [cm/s²]) from EKF origin.
// Offsets must be refreshed at least every 3 seconds to remain active.
// See set_posvelaccel_offset_target_NE_m() for full details.
void AC_PosControl::set_posvelaccel_offset_target_NE_cm(const Vector2p& pos_offset_target_ne_cm, const Vector2f& vel_offset_target_ne_cms, const Vector2f& accel_offset_target_ne_cmss)
{
    set_posvelaccel_offset_target_NE_m(pos_offset_target_ne_cm * 0.01, vel_offset_target_ne_cms * 0.01, accel_offset_target_ne_cmss * 0.01);
}

// Sets NE offset targets in meters, m/s, and m/s².
void AC_PosControl::set_posvelaccel_offset_target_NE_m(const Vector2p& pos_offset_target_ne_m, const Vector2f& vel_offset_target_ne_ms, const Vector2f& accel_offset_target_ne_mss)
{
    // set position offset target
    _pos_offset_target_neu_m.xy() = pos_offset_target_ne_m;

    // set velocity offset target
    _vel_offset_target_neu_ms.xy() = vel_offset_target_ne_ms;

    // set acceleration offset target
    _accel_offset_target_neu_mss.xy() = accel_offset_target_ne_mss;

    // record time of update so we can detect timeouts
    _posvelaccel_offset_target_ne_ms = AP_HAL::millis();
}

// Sets vertical offset targets (cm, cm/s, cm/s²) from EKF origin.
// See set_posvelaccel_offset_target_U_m() for full details.
void AC_PosControl::set_posvelaccel_offset_target_U_cm(float pos_offset_target_u_cm, float vel_offset_target_u_cms, const float accel_offset_target_u_cmss)
{
    set_posvelaccel_offset_target_U_m(pos_offset_target_u_cm * 0.01, vel_offset_target_u_cms * 0.01, accel_offset_target_u_cmss * 0.01);
}

// Sets vertical offset targets (m, m/s, m/s²) from EKF origin.
void AC_PosControl::set_posvelaccel_offset_target_U_m(float pos_offset_target_u_m, float vel_offset_target_u_ms, const float accel_offset_target_u_mss)
{
    // set position offset target
    _pos_offset_target_neu_m.z = pos_offset_target_u_m;

    // set velocity offset target
    _vel_offset_target_neu_ms.z = vel_offset_target_u_ms;

    // set acceleration offset target
    _accel_offset_target_neu_mss.z = accel_offset_target_u_mss;

    // record time of update so we can detect timeouts
    _posvelaccel_offset_target_u_ms = AP_HAL::millis();
}

// Returns desired thrust direction as a unit vector in the body frame.
Vector3f AC_PosControl::get_thrust_vector() const
{
    Vector3f accel_target_neu_mss = get_accel_target_NEU_mss();
    accel_target_neu_mss.z = -GRAVITY_MSS;
    return accel_target_neu_mss;
}

// Computes NE stopping point in centimeters based on current position, velocity, and acceleration.
// See get_stopping_point_NE_m() for full details.
void AC_PosControl::get_stopping_point_NE_cm(Vector2p &stopping_point_neu_cm) const
{
    Vector2p stopping_point_neu_m = stopping_point_neu_cm * 0.01;
    get_stopping_point_NE_m(stopping_point_neu_m);
    stopping_point_neu_cm = stopping_point_neu_m * 100.0;
}

// Computes NE stopping point in meters based on current position, velocity, and acceleration.
void AC_PosControl::get_stopping_point_NE_m(Vector2p &stopping_point_neu_m) const
{
    // Start from estimated NE position with offset removed
    // todo: we should use the current target position and velocity if we are currently running the position controller
    stopping_point_neu_m = _pos_estimate_neu_m.xy();
    stopping_point_neu_m -= _pos_offset_neu_m.xy();

    Vector2f curr_vel = _vel_estimate_neu_ms.xy();
    curr_vel -= _vel_offset_neu_ms.xy();

    // Compute velocity magnitude
    float vel_total = curr_vel.length();

    if (!is_positive(vel_total)) {
        return;
    }

    // Use current P gain and max accel to estimate stopping distance
    float kP = _p_pos_ne_m.kP();
    const float stopping_dist = stopping_distance(constrain_float(vel_total, 0.0, _vel_max_ne_ms), kP, _accel_max_ne_mss);
    if (!is_positive(stopping_dist)) {
        return;
    }

    // Project stopping distance along current velocity direction
    // todo: convert velocity to a unit vector instead.
    const float t = stopping_dist / vel_total;
    stopping_point_neu_m += (curr_vel * t).topostype();
}

// Computes vertical stopping point in centimeters based on current velocity and acceleration.
// See get_stopping_point_U_m() for full details.
void AC_PosControl::get_stopping_point_U_cm(postype_t &stopping_point_u_cm) const
{
    postype_t stopping_point_u_m = stopping_point_u_cm * 0.01;
    get_stopping_point_U_m(stopping_point_u_m);
    stopping_point_u_cm = stopping_point_u_m * 100.0;
}

// Computes vertical stopping point in meters based on current velocity and acceleration.
void AC_PosControl::get_stopping_point_U_m(postype_t &stopping_point_u_m) const
{
    float curr_pos_u_m = _pos_estimate_neu_m.z;
    curr_pos_u_m -= _pos_offset_neu_m.z;

    float curr_vel_u_ms = _vel_estimate_neu_ms.z;
    curr_vel_u_ms -= _vel_offset_neu_ms.z;

    // If controller is unconfigured or disabled, return current position
    if (!is_positive(_p_pos_u_m.kP()) || !is_positive(_accel_max_u_mss)) {
        stopping_point_u_m = curr_pos_u_m;
        return;
    }

    // Estimate stopping point using current velocity, P gain, and max vertical acceleration
    stopping_point_u_m = curr_pos_u_m + constrain_float(stopping_distance(curr_vel_u_ms, _p_pos_u_m.kP(), _accel_max_u_mss), - POSCONTROL_STOPPING_DIST_DOWN_MAX_M, POSCONTROL_STOPPING_DIST_UP_MAX_M);
}

// Returns bearing from current position to position target in radians.
// 0 = North, positive = clockwise.
float AC_PosControl::get_bearing_to_target_rad() const
{
    return (_pos_target_neu_m.xy() - _pos_estimate_neu_m.xy()).angle();
}


///
/// System methods
///

// Updates internal NEU position and velocity estimates from AHRS.
// Falls back to vertical-only data if horizontal velocity or position is invalid or vibration forces it.
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
    _pos_estimate_neu_m.xy() = pos_estimate_ned_m.xy();
    _pos_estimate_neu_m.z = -pos_estimate_ned_m.z;

    Vector3f vel_estimate_ned_ms;
    if (!AP::ahrs().get_velocity_NED(vel_estimate_ned_ms) || high_vibes) {
        float rate_z;
        if (AP::ahrs().get_vert_pos_rate_D(rate_z)) {
            vel_estimate_ned_ms.z = rate_z;
        }
    }
    _vel_estimate_neu_ms.xy() = vel_estimate_ned_ms.xy();
    _vel_estimate_neu_ms.z = -vel_estimate_ned_ms.z;
}

// Calculates vertical throttle using vibration-resistant feedforward estimation.
// Returns throttle output using manual feedforward gain for vibration compensation mode.
// Integrator is adjusted using velocity error when PID is being overridden.
float AC_PosControl::get_throttle_with_vibration_override()
{
    const float thr_per_accel_u_mss = _motors.get_throttle_hover() / GRAVITY_MSS;
    // Estimate throttle based on desired acceleration (manual feedforward gain).
    // Used when IMU vibrations corrupt raw acceleration measurements.
    // Allow integrator to compensate for velocity error only if not thrust-limited,
    // or if integrator is actively helping counteract velocity error direction.
    // ToDo: clear pid_info P, I and D terms for logging
    if (!(_motors.limit.throttle_lower || _motors.limit.throttle_upper) || ((is_positive(_pid_accel_u_cm_to_kt.get_i()) && is_negative(_pid_vel_u_cm.get_error())) || (is_negative(_pid_accel_u_cm_to_kt.get_i()) && is_positive(_pid_vel_u_cm.get_error())))) {
        // Adjust integrator to help reduce velocity error.
        // Note: scale by velocity P-gain and an override-specific I gain.
        _pid_accel_u_cm_to_kt.set_integrator(_pid_accel_u_cm_to_kt.get_i() + _dt_s * (thr_per_accel_u_mss / 100.0) * 1000.0 * _pid_vel_u_cm.get_error() * _pid_vel_u_cm.kP() * POSCONTROL_VIBE_COMP_I_GAIN);
    }
    // Final throttle = P term (feedforward) + scaled I term.
    return POSCONTROL_VIBE_COMP_P_GAIN * thr_per_accel_u_mss * _accel_target_neu_mss.z + _pid_accel_u_cm_to_kt.get_i() * 0.001;
}

// Resets NEU position controller state to prevent transients when exiting standby.
// Zeros I-terms and aligns targets to current position.
void AC_PosControl::standby_NEU_reset()
{
    // Reset vertical acceleration controller_cm_to_kt integrator to prevent throttle bias on reentry
    _pid_accel_u_cm_to_kt.set_integrator(0.0f);

    // Reset position controller targets to match current estimate — avoids position jumps
    _pos_target_neu_m = _pos_estimate_neu_m;

    // Reset horizontal velocity controller_cm integrator and derivative filter
    _pid_vel_ne_cm.reset_filter();

    // Reset EKF XY position reset tracking for NE controller
    init_ekf_NE_reset();
}

#if HAL_LOGGING_ENABLED
// Writes position controller diagnostic logs (PSCN, PSCE, etc).
void AC_PosControl::write_log()
{
    if (is_active_NE()) {
        float accel_n_mss, accel_e_mss;
        lean_angles_to_accel_NE_mss(accel_n_mss, accel_e_mss);

        // Log North-axis position control (PSCN): desired, target, and actual
        Write_PSCN(_pos_desired_neu_m.x, _pos_target_neu_m.x, _pos_estimate_neu_m.x ,
                   _vel_desired_neu_ms.x, _vel_target_neu_ms.x, _vel_estimate_neu_ms.x,
                   _accel_desired_neu_mss.x, _accel_target_neu_mss.x, accel_n_mss);

        // Log East-axis position control (PSCE): desired, target, and actual
        Write_PSCE(_pos_desired_neu_m.y, _pos_target_neu_m.y, _pos_estimate_neu_m.y,
                   _vel_desired_neu_ms.y, _vel_target_neu_ms.y, _vel_estimate_neu_ms.y,
                   _accel_desired_neu_mss.y, _accel_target_neu_mss.y, accel_e_mss);

        // log offsets if they are being used
        if (!_pos_offset_neu_m.xy().is_zero()) {
            // Log North offset tracking (PSON)
            Write_PSON(_pos_offset_target_neu_m.x, _pos_offset_neu_m.x, _vel_offset_target_neu_ms.x, _vel_offset_neu_ms.x, _accel_offset_target_neu_mss.x, _accel_offset_neu_mss.x);

            // Log East offset tracking (PSOE)
            Write_PSOE(_pos_offset_target_neu_m.y, _pos_offset_neu_m.y, _vel_offset_target_neu_ms.y, _vel_offset_neu_ms.y, _accel_offset_target_neu_mss.y, _accel_offset_neu_mss.y);
        }
    }

    if (is_active_U()) {
        // Log Down-axis position control (PSCD)
        Write_PSCD(-_pos_desired_neu_m.z, -_pos_target_neu_m.z, -_pos_estimate_neu_m.z,
                   -_vel_desired_neu_ms.z, -_vel_target_neu_ms.z, -_vel_estimate_neu_ms.z,
                   -_accel_desired_neu_mss.z, -_accel_target_neu_mss.z, -get_measured_accel_U_mss());

        // log down and terrain offsets if they are being used
        if (!is_zero(_pos_offset_neu_m.z)) {
            // Log Down offset tracking (PSOD)
            Write_PSOD(-_pos_offset_target_neu_m.z, -_pos_offset_neu_m.z, -_vel_offset_target_neu_ms.z, -_vel_offset_neu_ms.z, -_accel_offset_target_neu_mss.z, -_accel_offset_neu_mss.z);
        }
        if (!is_zero(_pos_terrain_u_m)) {
            // Log terrain-following offset (PSOT)
            Write_PSOT(-_pos_terrain_target_u_m, -_pos_terrain_u_m, 0, -_vel_terrain_u_ms, 0, -_accel_terrain_u_mss);
        }
    }
}
#endif  // HAL_LOGGING_ENABLED

// Returns lateral distance to closest point on active trajectory in meters.
// Used to assess horizontal deviation from path.
float AC_PosControl::crosstrack_error() const
{
    const Vector2f pos_error = (_pos_target_neu_m.xy() - _pos_estimate_neu_m.xy()).tofloat();
    if (is_zero(_vel_desired_neu_ms.xy().length_squared())) {
        // No desired velocity → return direct distance to target
        return pos_error.length();
    } else {
        // Project position error onto desired velocity vector
        const Vector2f vel_unit = _vel_desired_neu_ms.xy().normalized();
        const float dot_error = pos_error * vel_unit;

        // Use Pythagorean difference to isolate perpendicular (cross-track) component
        // todo: remove MAX of zero when safe_sqrt fixed
        return safe_sqrt(MAX(pos_error.length_squared() - sq(dot_error), 0.0));
    }
}

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // Returns true if the requested forward pitch is limited by the configured tilt constraint.
bool AC_PosControl::get_fwd_pitch_is_limited() const 
{
    if (_limit_vector_neu.xy().is_zero()) {  
        return false;  
    }  
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_mss = angle_rad_to_accel_mss(angle_max_rad);
    // Check for pitch limiting in the forward direction
    const float accel_fwd_unlimited_mss = _limit_vector_neu.x * _ahrs.cos_yaw() + _limit_vector_neu.y * _ahrs.sin_yaw();
    const float pitch_target_unlimited_deg = accel_mss_to_angle_deg(- MIN(accel_fwd_unlimited_mss, accel_max_mss));
    const float accel_fwd_limited = _accel_target_neu_mss.x * _ahrs.cos_yaw() + _accel_target_neu_mss.y * _ahrs.sin_yaw();
    const float pitch_target_limited_deg = accel_mss_to_angle_deg(- accel_fwd_limited);

    return is_negative(pitch_target_unlimited_deg) && pitch_target_unlimited_deg < pitch_target_limited_deg;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)

///
/// private methods
///

/// Terrain

// Updates terrain estimate (_pos_terrain_u_m) toward target using filter time constants.
void AC_PosControl::update_terrain()
{
    // update position, velocity, acceleration offsets for this iteration
    postype_t pos_terrain_u_m = _pos_terrain_u_m;
    update_pos_vel_accel(pos_terrain_u_m, _vel_terrain_u_ms, _accel_terrain_u_mss, _dt_s, MIN(_limit_vector_neu.z, 0.0f), _p_pos_u_m.get_error(), _pid_vel_u_cm.get_error());
    _pos_terrain_u_m = pos_terrain_u_m;

    // input shape horizontal position, velocity and acceleration offsets
    shape_pos_vel_accel(_pos_terrain_target_u_m, 0.0, 0.0,
        _pos_terrain_u_m, _vel_terrain_u_ms, _accel_terrain_u_mss,
        get_max_speed_down_ms(), get_max_speed_up_ms(),
        -get_max_accel_U_mss(), get_max_accel_U_mss(),
        _jerk_max_u_msss, _dt_s, false);

    // we do not have to update _pos_terrain_target_u_m because we assume the target velocity and acceleration are zero
    // if we know how fast the terain altitude is changing we would add update_pos_vel_accel for _pos_terrain_target_u_m here
}

    // Converts horizontal acceleration (m/s²) to roll/pitch lean angles in radians.
void AC_PosControl::accel_NE_mss_to_lean_angles_rad(float accel_n_mss, float accel_e_mss, float& roll_target_rad, float& pitch_target_rad) const
{
    // rotate accelerations into body forward-right frame
    const float accel_forward_mss = accel_n_mss * _ahrs.cos_yaw() + accel_e_mss * _ahrs.sin_yaw();
    const float accel_right_mss = -accel_n_mss * _ahrs.sin_yaw() + accel_e_mss * _ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    pitch_target_rad = accel_mss_to_angle_rad(-accel_forward_mss);
    float cos_pitch_target = cosf(pitch_target_rad);
    roll_target_rad = accel_mss_to_angle_rad(accel_right_mss * cos_pitch_target);
}

// Converts current target lean angles to NE acceleration in m/s².
void AC_PosControl::lean_angles_to_accel_NE_mss(float& accel_n_mss, float& accel_e_mss) const
{
    // rotate our roll, pitch angles into lat/lon frame
    Vector3f att_target_euler_rad = _attitude_control.get_att_target_euler_rad();
    att_target_euler_rad.z = _ahrs.yaw;
    Vector3f accel_ne_mss = lean_angles_rad_to_accel_NEU_mss(att_target_euler_rad);

    accel_n_mss = accel_ne_mss.x;
    accel_e_mss = accel_ne_mss.y;
}

// Computes desired yaw and yaw rate based on the NE acceleration and velocity vectors.
// Aligns yaw with the direction of travel if speed exceeds 5% of maximum.
void AC_PosControl::calculate_yaw_and_rate_yaw()
{
    // Calculate the turn rate
    float turn_rate_rads = 0.0f;
    const float vel_desired_length_ne_ms = _vel_desired_neu_ms.xy().length();
    if (is_positive(vel_desired_length_ne_ms)) {
        // Project acceleration vector into velocity direction to extract forward acceleration component
        const float accel_forward_mss = (_accel_desired_neu_mss.x * _vel_desired_neu_ms.x + _accel_desired_neu_mss.y * _vel_desired_neu_ms.y) / vel_desired_length_ne_ms;
        // Subtract forward component to isolate turn acceleration perpendicular to velocity vector
        const Vector2f accel_turn_ne_mss = _accel_desired_neu_mss.xy() - _vel_desired_neu_ms.xy() * accel_forward_mss / vel_desired_length_ne_ms;
        // Compute turn rate from lateral acceleration and velocity (centripetal formula)
        const float accel_turn_length_ne_mss = accel_turn_ne_mss.length();
        turn_rate_rads = accel_turn_length_ne_mss / vel_desired_length_ne_ms;
        // Determine turn direction: positive = clockwise (right)
        if ((accel_turn_ne_mss.y * _vel_desired_neu_ms.x - accel_turn_ne_mss.x * _vel_desired_neu_ms.y) < 0.0) {
            turn_rate_rads = -turn_rate_rads;
        }
    }

    // If vehicle is moving significantly, align yaw to velocity vector and apply computed turn rate
    if (vel_desired_length_ne_ms > _vel_max_ne_ms * 0.05f) {
        _yaw_target_rad = _vel_desired_neu_ms.xy().angle();
        _yaw_rate_target_rads = turn_rate_rads;
        return;
    }

    // If motion is too slow, retain last yaw target from attitude controller
    _yaw_target_rad = _attitude_control.get_att_target_euler_rad().z;
    _yaw_rate_target_rads = 0;
}

// Computes scaling factor to increase max vertical accel/jerk if vertical speed exceeds configured limits.
float AC_PosControl::calculate_overspeed_gain()
{
    // If desired descent speed exceeds configured max, scale acceleration/jerk proportionally
    if (_vel_desired_neu_ms.z < _vel_max_down_ms && !is_zero(_vel_max_down_ms)) {
        return POSCONTROL_OVERSPEED_GAIN_U * _vel_desired_neu_ms.z / _vel_max_down_ms;
    }

    // If desired climb speed exceeds configured max, scale acceleration/jerk proportionally
    if (_vel_desired_neu_ms.z > _vel_max_up_ms && !is_zero(_vel_max_up_ms)) {
        return POSCONTROL_OVERSPEED_GAIN_U * _vel_desired_neu_ms.z / _vel_max_up_ms;
    }

    // Within normal speed limits — use nominal acceleration and jerk
    return 1.0;
}

// Initializes tracking of NE EKF position resets.
void AC_PosControl::init_ekf_NE_reset()
{
    Vector2f pos_shift;
    _ekf_ne_reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
}

// Handles NE position reset detection and response (e.g., clearing accumulated errors).
void AC_PosControl::handle_ekf_NE_reset()
{
    // Check for EKF-reported NE position shift since last update
    Vector2f pos_shift;
    uint32_t reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
    if (reset_ms != _ekf_ne_reset_ms) {
        // Reset NE controller to preserve relative position control during Loiter, PosHold, etc.
        // This ensures controller output remains continuous after EKF realigns the origin.

        // ToDo: move EKF steps into the offsets for modes setting absolute position and velocity
        // for this we need some sort of switch to select what type of EKF handling we want to use

        // Reconstruct target and desired positions relative to new EKF origin
        _pos_target_neu_m.xy() = _pos_estimate_neu_m.xy() + _p_pos_ne_m.get_error().topostype();
        _pos_desired_neu_m.xy() = _pos_target_neu_m.xy() - _pos_offset_neu_m.xy();

        // Reconstruct velocity targets based on PID error and velocity estimate
        _vel_target_neu_ms.xy() = _vel_estimate_neu_ms.xy() + _pid_vel_ne_cm.get_error() * 0.01;
        _vel_desired_neu_ms.xy() = _vel_target_neu_ms.xy() - _vel_offset_neu_ms.xy();

        _ekf_ne_reset_ms = reset_ms;
    }
}

// Initializes tracking of vertical (U) EKF resets.
void AC_PosControl::init_ekf_U_reset()
{
    float alt_shift_d_m;
    _ekf_u_reset_ms = _ahrs.getLastPosDownReset(alt_shift_d_m);
}

// Handles U EKF reset detection and response.
void AC_PosControl::handle_ekf_U_reset()
{
    // Check for EKF-reported Down-axis shift since last update
    float alt_shift_d_m;
    uint32_t reset_ms = _ahrs.getLastPosDownReset(alt_shift_d_m);
    if (reset_ms != 0 && reset_ms != _ekf_u_reset_ms) {
        // Reset U controller to preserve continuity during relative-altitude modes (e.g., Loiter, PosHold).
        // Compensates for EKF origin shift without abrupt position or velocity discontinuities.

        // ToDo: move EKF steps into the offsets for modes setting absolute position and velocity
        // for this we need some sort of switch to select what type of EKF handling we want to use

        // Reconstruct vertical position targets from measured altitude + P error
        _pos_target_neu_m.z = _pos_estimate_neu_m.z + _p_pos_u_m.get_error();
        _pos_desired_neu_m.z = _pos_target_neu_m.z - (_pos_offset_neu_m.z + _pos_terrain_u_m);

        // Reconstruct vertical velocity targets from measured velocity + PID error
        _vel_target_neu_ms.z = _vel_estimate_neu_ms.z + _pid_vel_u_cm.get_error() * 0.01;
        _vel_desired_neu_ms.z = _vel_target_neu_ms.z - (_vel_offset_neu_ms.z + _vel_terrain_u_ms);

        _ekf_u_reset_ms = reset_ms;
    }
}

// Performs pre-arm checks for position control parameters and EKF readiness.
// Returns false if failure_msg is populated.
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
