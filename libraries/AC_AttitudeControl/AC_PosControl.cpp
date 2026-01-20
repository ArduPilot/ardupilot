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
 # define POSCONTROL_D_POS_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_D_VEL_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_D_VEL_IMAX                 10.0f   // vertical velocity controller IMAX gain default
 # define POSCONTROL_D_VEL_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_D_VEL_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_D_ACC_P                    0.03f   // vertical acceleration controller P gain default
 # define POSCONTROL_D_ACC_I                    0.1f    // vertical acceleration controller I gain default
 # define POSCONTROL_D_ACC_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_D_ACC_IMAX                 0.8f    // vertical acceleration controller IMAX gain default
 # define POSCONTROL_D_ACC_FILT_HZ              10.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_D_ACC_DT                   0.02f   // vertical acceleration controller dt default
 # define POSCONTROL_NE_POS_P                   0.5f    // horizontal position controller P gain default
 # define POSCONTROL_NE_VEL_P                   0.7f    // horizontal velocity controller P gain default
 # define POSCONTROL_NE_VEL_I                   0.35f   // horizontal velocity controller I gain default
 # define POSCONTROL_NE_VEL_D                   0.17f   // horizontal velocity controller D gain default
 # define POSCONTROL_NE_VEL_IMAX                10.0f   // horizontal velocity controller IMAX gain default
 # define POSCONTROL_NE_VEL_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_NE_VEL_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
 // default gains for Sub
 # define POSCONTROL_D_POS_P                    3.0f    // vertical position controller P gain default
 # define POSCONTROL_D_VEL_P                    8.0f    // vertical velocity controller P gain default
 # define POSCONTROL_D_VEL_IMAX                 10.0f   // vertical velocity controller IMAX gain default
 # define POSCONTROL_D_VEL_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_D_VEL_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_D_ACC_P                    0.05f   // vertical acceleration controller P gain default
 # define POSCONTROL_D_ACC_I                    0.01f   // vertical acceleration controller I gain default
 # define POSCONTROL_D_ACC_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_D_ACC_IMAX                 0.1f    // vertical acceleration controller IMAX gain default
 # define POSCONTROL_D_ACC_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_D_ACC_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_NE_POS_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_NE_VEL_P                   1.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_NE_VEL_I                   0.5f    // horizontal velocity controller I gain default
 # define POSCONTROL_NE_VEL_D                   0.0f    // horizontal velocity controller D gain default
 # define POSCONTROL_NE_VEL_IMAX                10.0f   // horizontal velocity controller IMAX gain default
 # define POSCONTROL_NE_VEL_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_NE_VEL_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#else
 // default gains for Copter / TradHeli
 # define POSCONTROL_D_POS_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_D_VEL_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_D_VEL_IMAX                 10.0f   // vertical velocity controller IMAX gain default
 # define POSCONTROL_D_VEL_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_D_VEL_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_D_ACC_P                    0.05f   // vertical acceleration controller P gain default
 # define POSCONTROL_D_ACC_I                    0.1f    // vertical acceleration controller I gain default
 # define POSCONTROL_D_ACC_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_D_ACC_IMAX                 0.8f    // vertical acceleration controller IMAX gain default
 # define POSCONTROL_D_ACC_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_D_ACC_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_NE_POS_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_NE_VEL_P                   2.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_NE_VEL_I                   1.0f    // horizontal velocity controller I gain default
 # define POSCONTROL_NE_VEL_D                   0.25f   // horizontal velocity controller D gain default
 # define POSCONTROL_NE_VEL_IMAX                10.0f   // horizontal velocity controller IMAX gain default
 # define POSCONTROL_NE_VEL_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_NE_VEL_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#endif

// vibration compensation gains
#define POSCONTROL_VIBE_COMP_P_GAIN 0.250f
#define POSCONTROL_VIBE_COMP_I_GAIN 0.125f

// velocity offset targets timeout if not updated within 3 seconds
#define POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS 3000

AC_PosControl *AC_PosControl::_singleton;

const AP_Param::GroupInfo AC_PosControl::var_info[] = {
    // 0 was used for HOVER

    // POS_ACC_XY_FILT was here.

    // @Param: _D_POS_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain. Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller. Previously _POSZ_P.
    // @Range: 0.50 4.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_d_m, "_D_POS_", 2, AC_PosControl, AC_P_1D),

    // 3 was _VELZ_ which has become _D_VEL_

    // 4 was _ACCZ_ which has become _D_ACC_

    // @Param: _NE_POS_P
    // @DisplayName: Position (horizontal) controller P gain
    // @Description: Position controller P gain. Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller. Previously _POSXY_P.
    // @Range: 0.50 4.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_ne_m, "_NE_POS_", 5, AC_PosControl, AC_P_2D),

    // 6 was _VELXY_ which has become _NE_VEL_

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request. Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_ANGLE_MAX", 7, AC_PosControl, _lean_angle_max_deg, 0.0f),

    // IDs 8,9 used for _TC_XY and _TC_Z in beta release candidate

    // @Param: _JERK_NE
    // @DisplayName: Jerk limit for the horizontal kinematic input shaping
    // @Description: Jerk limit of the horizontal kinematic path generation used to determine how quickly the aircraft varies the acceleration target
    // @Units: m/s/s/s
    // @Range: 1 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_JERK_NE", 10, AC_PosControl, _shaping_jerk_ne_msss, POSCONTROL_JERK_NE_MSSS),

    // @Param: _JERK_D
    // @DisplayName: Jerk limit for the vertical kinematic input shaping
    // @Description: Jerk limit of the vertical kinematic path generation used to determine how quickly the aircraft varies the acceleration target
    // @Units: m/s/s/s
    // @Range: 1 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_JERK_D", 11, AC_PosControl, _shaping_jerk_d_msss, POSCONTROL_JERK_D_MSSS),

    // @Param: _D_VEL_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain. Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller. Previously _VELZ_P.
    // @Range: 1.0 10.0
    // @Increment: 0.1
    // @User: Standard

    // @Param: _D_VEL_I
    // @DisplayName: Velocity (vertical) controller I gain
    // @Description: Velocity (vertical) controller I gain. Corrects long-term difference in desired velocity to a target acceleration. Previously _VELZ_I.
    // @Range: 0.00 10.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _D_VEL_IMAX
    // @DisplayName: Velocity (vertical) controller I gain maximum
    // @Description: Velocity (vertical) controller I gain maximum. Constrains the target acceleration that the I gain will output. If upgrading from 4.6 this is _VELZ_IMAX * 0.01.
    // @Range: 1.000 10.000
    // @Increment: 0.1
    // @User: Standard

    // @Param: _D_VEL_D
    // @DisplayName: Velocity (vertical) controller D gain
    // @Description: Velocity (vertical) controller D gain. Corrects short-term changes in velocity. Previously _VELZ_D.
    // @Range: 0.00 2.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _D_VEL_FF
    // @DisplayName: Velocity (vertical) controller Feed Forward gain
    // @Description: Velocity (vertical) controller Feed Forward gain. Produces an output that is proportional to the magnitude of the target. Previously _VELZ_FF.
    // @Range: 0.00 2.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _D_VEL_FLTE
    // @DisplayName: Velocity (vertical) error filter
    // @Description: Velocity (vertical) error filter. This filter (in Hz) is applied to the input for P and I terms. Previously _VELZ_FLTE.
    // @Range: 0 100
    // @Increment: 1.0
    // @Units: Hz
    // @User: Advanced

    // @Param: _D_VEL_FLTD
    // @DisplayName: Velocity (vertical) input filter for D term
    // @Description: Velocity (vertical) input filter for D term. This filter (in Hz) is applied to the input for D terms. Previously _VELZ_FLTD.
    // @Range: 0 100
    // @Increment: 1.0
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_d_m, "_D_VEL_", 12, AC_PosControl, AC_PID_Basic),

    // @Param: _D_ACC_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain. Converts the difference between desired vertical acceleration and actual acceleration into a motor output. If upgrading from 4.6 this is _ACCZ_P * 0.1.
    // @Range: 0.010 0.250
    // @Increment: 0.001
    // @User: Standard

    // @Param: _D_ACC_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain. Corrects long-term difference in desired vertical acceleration and actual acceleration. If upgrading from 4.6 this is _ACCZ_I * 0.1.
    // @Range: 0.000 0.500
    // @Increment: 0.001
    // @User: Standard

    // @Param: _D_ACC_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum. Constrains the maximum pwm that the I term will generate. If upgrading from 4.6 this is _ACCZ_IMAX * 0.001.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @Units: d%
    // @User: Standard

    // @Param: _D_ACC_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain. Compensates for short-term change in desired vertical acceleration vs actual acceleration. If upgrading from 4.6 this is _ACCZ_D * 0.1.
    // @Range: 0.000 0.100
    // @Increment: 0.001
    // @User: Standard

    // @Param: _D_ACC_FF
    // @DisplayName: Acceleration (vertical) controller feed forward
    // @Description: Acceleration (vertical) controller feed forward. If upgrading from 4.6 this is _ACCZ_FF * 0.1.
    // @Range: 0.000 0.100
    // @Increment: 0.001
    // @User: Standard

    // @Param: _D_ACC_FLTT
    // @DisplayName: Acceleration (vertical) controller target frequency in Hz
    // @Description: Acceleration (vertical) controller target frequency in Hz. Previously _ACCZ_FLTT.
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _D_ACC_FLTE
    // @DisplayName: Acceleration (vertical) controller error frequency in Hz
    // @Description: Acceleration (vertical) controller error frequency in Hz. Previously _ACCZ_FLTE.
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _D_ACC_FLTD
    // @DisplayName: Acceleration (vertical) controller derivative frequency in Hz
    // @Description: Acceleration (vertical) controller derivative frequency in Hz. Previously _ACCZ_FLTD.
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _D_ACC_SMAX
    // @DisplayName: Accel (vertical) slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _D_ACC_PDMX
    // @DisplayName: Acceleration (vertical) controller PD sum maximum
    // @Description: Acceleration (vertical) controller PD sum maximum. The maximum/minimum value that the sum of the P and D term can output. If upgrading from 4.6 this is _ACCZ_P * 0.1.
    // @Range: 0.00 1.00
    // @Increment: 0.01
    // @Units: d%

    // @Param: _D_ACC_D_FF
    // @DisplayName: Accel (vertical) Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target. If upgrading from 4.6 this is _ACCZ_P * 0.1.
    // @Range: 0.000 0.050
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _D_ACC_NTF
    // @DisplayName: Accel (vertical) Target notch filter index
    // @Description: Accel (vertical) Target notch filter index. If upgrading from 4.6 this is Previously _ACCZ_NTF.
    // @Range: 1 8
    // @User: Advanced

    // @Param: _D_ACC_NEF
    // @DisplayName: Accel (vertical) Error notch filter index
    // @Description: Accel (vertical) Error notch filter index. If upgrading from 4.6 this is Previously _ACCZ_NEF.
    // @Range: 1 8
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_accel_d_m, "_D_ACC_", 13, AC_PosControl, AC_PID),

    // @Param: _NE_VEL_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain. Converts the difference between desired and actual velocity to a target acceleration. Previously _VELXY_P.
    // @Range: 0.10 10.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _NE_VEL_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain. Corrects long-term difference between desired and actual velocity to a target acceleration. Previously _VELXY_I.
    // @Range: 0.10 10.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _NE_VEL_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain. Corrects short-term changes in velocity. Previously _VELXY_D.
    // @Range: 0.00 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _NE_VEL_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum. Constrains the target acceleration that the I gain will output. If upgrading from 4.6 this is _VELXY_IMAX * 0.01.
    // @Range: 0 10
    // @Increment: 1
    // @Units: m/s/s
    // @User: Advanced

    // @Param: _NE_VEL_FLTE
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter. This filter (in Hz) is applied to the input for P and I terms. Previously _VELXY_FLTE.
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Advanced

    // @Param: _NE_VEL_FLTD
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter. This filter (in Hz) is applied to the input for D term. Previously _VELXY_FLTD.
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Advanced

    // @Param: _NE_VEL_FF
    // @DisplayName: Velocity (horizontal) feed forward gain
    // @Description: Velocity (horizontal) feed forward gain. Converts the difference between desired velocity to a target acceleration. Previously _VELXY_FF.
    // @Range: 0.10 10.00
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_ne_m, "_NE_VEL_", 14, AC_PosControl, AC_PID_2D),

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
    _p_pos_ne_m(POSCONTROL_NE_POS_P),
    _p_pos_d_m(POSCONTROL_D_POS_P),
    _pid_vel_ne_m(POSCONTROL_NE_VEL_P, POSCONTROL_NE_VEL_I, POSCONTROL_NE_VEL_D, 0.0f, POSCONTROL_NE_VEL_IMAX, POSCONTROL_NE_VEL_FILT_HZ, POSCONTROL_NE_VEL_FILT_D_HZ),
    _pid_vel_d_m(POSCONTROL_D_VEL_P, 0.0f, 0.0f, 0.0f, POSCONTROL_D_VEL_IMAX, POSCONTROL_D_VEL_FILT_HZ, POSCONTROL_D_VEL_FILT_D_HZ),
    _pid_accel_d_m(POSCONTROL_D_ACC_P, POSCONTROL_D_ACC_I, POSCONTROL_D_ACC_D, 0.0f, POSCONTROL_D_ACC_IMAX, 0.0f, POSCONTROL_D_ACC_FILT_HZ, 0.0f),
    _vel_max_ne_ms(POSCONTROL_SPEED_MS),
    _vel_max_up_ms(POSCONTROL_SPEED_UP_MS),
    _vel_max_down_ms(POSCONTROL_SPEED_DOWN_MS),
    _accel_max_ne_mss(POSCONTROL_ACCEL_NE_MSS),
    _accel_max_d_mss(POSCONTROL_ACCEL_D_MSS),
    _jerk_max_ne_msss(POSCONTROL_JERK_NE_MSSS),
    _jerk_max_d_msss(POSCONTROL_JERK_D_MSSS)
{
    AP_Param::setup_object_defaults(this, var_info);

    _singleton = this;
}


///
/// 3D position shaper
///

// Sets a new NED position target in meters and computes a jerk-limited trajectory.
// Updates internal acceleration commands using a smooth kinematic path constrained
// by configured acceleration and jerk limits. 
// The path can be offset vertically to follow the terrain by providing the current 
// terrain level in the NED frame and the terrain margin. Terrain margin is used to
// constrain horizontal velocity to avoid vertical buffer violation.
void AC_PosControl::input_pos_NED_m(const Vector3p& pos_ned_m, float pos_terrain_target_d_m, float terrain_margin_m)
{
    // Terrain following velocity scalar must be calculated before we remove the position offset
    const float offset_d_scalar = terrain_scaler_D_m(pos_terrain_target_d_m, terrain_margin_m);
    set_pos_terrain_target_D_m(pos_terrain_target_d_m);

    // calculated increased maximum acceleration and jerk if over speed
    const float overspeed_gain = calculate_overspeed_gain();
    const float accel_max_d_mss = _accel_max_d_mss * overspeed_gain;
    const float jerk_max_d_msss = _jerk_max_d_msss * overspeed_gain;

    update_pos_vel_accel_xy(_pos_desired_ned_m.xy(), _vel_desired_ned_ms.xy(), _accel_desired_ned_mss.xy(), _dt_s, _limit_vector_ned.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_m.get_error());

    // adjust desired altitude if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_ned_m.z, _vel_desired_ned_ms.z, _accel_desired_ned_mss.z, _dt_s, _limit_vector_ned.z, _p_pos_d_m.get_error(), _pid_vel_d_m.get_error());

    // calculate the horizontal and vertical velocity limits to travel directly to the destination defined by pos_ne_m
    float vel_max_ne_ms = 0.0f;
    float vel_max_d_ms = 0.0f;
    Vector3f travel_dir_unit = (pos_ned_m - _pos_desired_ned_m).tofloat();
    if (is_positive(travel_dir_unit.length_squared()) ) {
        travel_dir_unit.normalize();
        float travel_dir_unit_ne_length = travel_dir_unit.xy().length();

        float vel_max_ms = kinematic_limit(travel_dir_unit, _vel_max_ne_ms, _vel_max_up_ms, _vel_max_down_ms);
        vel_max_ne_ms = vel_max_ms * travel_dir_unit_ne_length;
        vel_max_d_ms = fabsf(vel_max_ms * travel_dir_unit.z);
    }

    // reduce speed if we are reaching the edge of our vertical buffer
    vel_max_ne_ms *= offset_d_scalar;

    Vector2f vel_ne_ms;
    Vector2f accel_ne_mss;
    shape_pos_vel_accel_xy(pos_ned_m.xy(), vel_ne_ms, accel_ne_mss, _pos_desired_ned_m.xy(), _vel_desired_ned_ms.xy(), _accel_desired_ned_mss.xy(),
                           vel_max_ne_ms, _accel_max_ne_mss, _jerk_max_ne_msss, _dt_s, false);

    float pos_d_m = pos_ned_m.z;
    shape_pos_vel_accel(pos_d_m, 0, 0,
                        _pos_desired_ned_m.z, _vel_desired_ned_ms.z, _accel_desired_ned_mss.z,
                        -vel_max_d_ms, vel_max_d_ms,
                        -accel_max_d_mss, constrain_float(accel_max_d_mss, 0.0, 7.5),
                        jerk_max_d_msss, _dt_s, false);
}

// Returns a scaling factor for horizontal velocity in m/s to ensure
// the vertical controller maintains a safe distance above terrain.
float AC_PosControl::terrain_scaler_D_m(float pos_terrain_d_m, float terrain_margin_m) const
{
    if (is_zero(terrain_margin_m)) {
        return 1.0;
    }
    float pos_offset_error_d_m = _pos_estimate_ned_m.z - (_pos_target_ned_m.z + (pos_terrain_d_m - _pos_terrain_d_m));
    return constrain_float((1.0 - (fabsf(pos_offset_error_d_m) - 0.5 * terrain_margin_m) / (0.5 * terrain_margin_m)), 0.01, 1.0);
}

///
/// Lateral position controller
///

// Sets maximum horizontal speed (cm/s) and acceleration (cm/s²) for NE-axis shaping.
// Can be called anytime; transitions are handled smoothly.
// All arguments should be positive.
// See NE_set_max_speed_accel_m() for full details.
void AC_PosControl::NE_set_max_speed_accel_cm(float speed_ne_cms, float accel_ne_cmss)
{
    NE_set_max_speed_accel_m(speed_ne_cms * 0.01, accel_ne_cmss * 0.01);
}

// Sets maximum horizontal speed (m/s) and acceleration (m/s²) for NE-axis shaping.
// These values constrain the kinematic trajectory used by the lateral controller.
// All arguments should be positive.
void AC_PosControl::NE_set_max_speed_accel_m(float speed_ne_ms, float accel_ne_mss)
{
    _vel_max_ne_ms = fabsf(speed_ne_ms);
    _accel_max_ne_mss = fabsf(accel_ne_mss);

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
// All arguments should be positive.
// See NE_set_correction_speed_accel_m() for full details.
void AC_PosControl::NE_set_correction_speed_accel_cm(float speed_ne_cms, float accel_ne_cmss)
{
    NE_set_correction_speed_accel_m(speed_ne_cms * 0.01, accel_ne_cmss * 0.01);
}

// Sets horizontal correction limits for velocity (m/s) and acceleration (m/s²).
// These values constrain the PID correction path, not the desired trajectory.
// All arguments should be positive.
void AC_PosControl::NE_set_correction_speed_accel_m(float speed_ne_ms, float accel_ne_mss)
{
    // limits that are not positive are ignored
    _p_pos_ne_m.set_limits(speed_ne_ms, accel_ne_mss, 0.0f);
}

// Initializes NE controller to a stationary stopping point with zero velocity and acceleration.
// Use when the expected trajectory begins at rest but the starting position is unspecified.
// The starting position can be retrieved with get_pos_target_NED_m().
void AC_PosControl::NE_init_controller_stopping_point()
{
    NE_init_controller();

    get_stopping_point_NE_m(_pos_desired_ned_m.xy());
    _pos_target_ned_m.xy() = _pos_desired_ned_m.xy() + _pos_offset_ned_m.xy();
    _vel_desired_ned_ms.xy().zero();
    _accel_desired_ned_mss.xy().zero();
}

// Smoothly decays NE acceleration over time to zero while maintaining current velocity and position.
// Reduces output acceleration by ~95% over 0.5 seconds to avoid abrupt transitions.
void AC_PosControl::NE_relax_velocity_controller()
{
    // decay acceleration and therefore current attitude target to zero
    // this will be reset by NE_init_controller() if !NE_is_active()
    if (is_positive(_dt_s)) {
        float decay = 1.0 - _dt_s / (_dt_s + POSCONTROL_RELAX_TC);
        _accel_target_ned_mss.xy() *= decay;
    }

    NE_init_controller();
}

// Softens NE controller for landing by reducing position error and suppressing I-term windup.
// Used to make descent behavior more stable near ground contact.
void AC_PosControl::NE_soften_for_landing()
{
    // decay position error to zero
    if (is_positive(_dt_s)) {
        _pos_target_ned_m.xy() += (_pos_estimate_ned_m.xy() - _pos_target_ned_m.xy()) * (_dt_s / (_dt_s + POSCONTROL_RELAX_TC));
        _pos_desired_ned_m.xy() = _pos_target_ned_m.xy() - _pos_offset_ned_m.xy();
    }

    // Prevent I term build up in xy velocity controller.
    // Note that this flag is reset on each loop in NE_update_controller()
    NE_set_externally_limited();
}

// Fully initializes the NE controller with current position, velocity, acceleration, and attitude.
// Intended for normal startup when the full state is known.
// Private function shared by other NE initializers.
void AC_PosControl::NE_init_controller()
{
    // initialise offsets to target offsets and ensure offset targets are zero if they have not been updated.
    NE_init_offsets();
    
    // set roll, pitch lean angle targets to current attitude
    const Vector3f &att_target_euler_rad = _attitude_control.get_att_target_euler_rad();
    _roll_target_rad = att_target_euler_rad.x;
    _pitch_target_rad = att_target_euler_rad.y;
    _yaw_target_rad = att_target_euler_rad.z; // todo: this should be thrust vector heading, not yaw.
    _yaw_rate_target_rads = 0.0f;
    _angle_max_override_rad = 0.0;

    _pos_target_ned_m.xy() = _pos_estimate_ned_m.xy();
    _pos_desired_ned_m.xy() = _pos_target_ned_m.xy() - _pos_offset_ned_m.xy();

    _vel_target_ned_ms.xy() = _vel_estimate_ned_ms.xy();
    _vel_desired_ned_ms.xy() = _vel_target_ned_ms.xy() - _vel_offset_ned_ms.xy();

    // Set desired acceleration to zero because raw acceleration is prone to noise
    _accel_desired_ned_mss.xy().zero();

    if (!NE_is_active()) {
        lean_angles_to_accel_NE_mss(_accel_target_ned_mss.x, _accel_target_ned_mss.y);
    }

    // limit acceleration using maximum lean angles
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_mss = angle_rad_to_accel_mss(angle_max_rad);
    _accel_target_ned_mss.xy().limit_length(accel_max_mss);

    // initialise I terms from lean angles
    _pid_vel_ne_m.reset_filter();
    // initialise the I term to (_accel_target_ned_mss - _accel_desired_ned_mss)
    // _accel_desired_ned_mss is zero and can be removed from the equation
    _pid_vel_ne_m.set_integrator((_accel_target_ned_mss.xy() - _vel_target_ned_ms.xy() * _pid_vel_ne_m.ff()));

    // initialise ekf xy reset handler
    NE_init_ekf_reset();

    // initialise z_controller time out
    _last_update_ne_ticks = AP::scheduler().ticks32();
}

// Sets the desired NE-plane acceleration in m/s² using jerk-limited shaping.
// Smoothly transitions to the specified acceleration from current kinematic state.
// Constraints: max acceleration and jerk set via NE_set_max_speed_accel_m().
void AC_PosControl::input_accel_NE_m(const Vector2f& accel_ne_mss)
{
    update_pos_vel_accel_xy(_pos_desired_ned_m.xy(), _vel_desired_ned_ms.xy(), _accel_desired_ned_mss.xy(), _dt_s, _limit_vector_ned.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_m.get_error());
    shape_accel_xy(accel_ne_mss, _accel_desired_ned_mss.xy(), _jerk_max_ne_msss, _dt_s);
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
    update_pos_vel_accel_xy(_pos_desired_ned_m.xy(), _vel_desired_ned_ms.xy(), _accel_desired_ned_mss.xy(), _dt_s, _limit_vector_ned.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_m.get_error());

    shape_vel_accel_xy(vel_ne_ms, accel_ne_mss, _vel_desired_ned_ms.xy(), _accel_desired_ned_mss.xy(),
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
    update_pos_vel_accel_xy(_pos_desired_ned_m.xy(), _vel_desired_ned_ms.xy(), _accel_desired_ned_mss.xy(), _dt_s, _limit_vector_ned.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_m.get_error());

    shape_pos_vel_accel_xy(pos_ne_m, vel_ne_ms, accel_ne_mss, _pos_desired_ned_m.xy(), _vel_desired_ned_ms.xy(), _accel_desired_ned_mss.xy(),
                           _vel_max_ne_ms, _accel_max_ne_mss, _jerk_max_ne_msss, _dt_s, limit_output);

    update_pos_vel_accel_xy(pos_ne_m, vel_ne_ms, accel_ne_mss, _dt_s, Vector2f(), Vector2f(), Vector2f());
}

// Updates NE offsets by gradually moving them toward their targets.
void AC_PosControl::NE_update_offsets()
{
    // Check if NE offset targets have timed out
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_ne_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        // Timeout: reset all NE offset targets to zero
        _pos_offset_target_ned_m.xy().zero();
        _vel_offset_target_ned_ms.xy().zero();
        _accel_offset_target_ned_mss.xy().zero();
    }

    // Advance offset target kinematic state (position, velocity, accel)
    update_pos_vel_accel_xy(_pos_offset_target_ned_m.xy(), _vel_offset_target_ned_ms.xy(), _accel_offset_target_ned_mss.xy(), _dt_s, Vector2f(), Vector2f(), Vector2f());
    update_pos_vel_accel_xy(_pos_offset_ned_m.xy(), _vel_offset_ned_ms.xy(), _accel_offset_ned_mss.xy(), _dt_s, _limit_vector_ned.xy(), _p_pos_ne_m.get_error(), _pid_vel_ne_m.get_error());

    // Shape the offset path from current to target using jerk-limited smoothing
    shape_pos_vel_accel_xy(_pos_offset_target_ned_m.xy(), _vel_offset_target_ned_ms.xy(), _accel_offset_target_ned_mss.xy(),
                            _pos_offset_ned_m.xy(), _vel_offset_ned_ms.xy(), _accel_offset_ned_mss.xy(),
                            _vel_max_ne_ms, _accel_max_ne_mss, _jerk_max_ne_msss, _dt_s, false);
}

// Disables NE position correction by setting the target position to the current position.
// Useful to freeze positional control without disrupting velocity control.
void AC_PosControl::NE_stop_pos_stabilisation()
{
    _pos_target_ned_m.xy() = _pos_estimate_ned_m.xy();
    _pos_desired_ned_m.xy() = _pos_target_ned_m.xy() - _pos_offset_ned_m.xy();
}

// Disables NE position and velocity correction by setting target values to current state.
// Useful to prevent further corrections and freeze motion stabilization in NE axes.
void AC_PosControl::NE_stop_vel_stabilisation()
{
    _pos_target_ned_m.xy() =  _pos_estimate_ned_m.xy();
    _pos_desired_ned_m.xy() = _pos_target_ned_m.xy() - _pos_offset_ned_m.xy();
    
    _vel_target_ned_ms.xy() = _vel_estimate_ned_ms.xy();
    _vel_desired_ned_ms.xy() = _vel_target_ned_ms.xy() - _vel_offset_ned_ms.xy();

    // reset I terms
    _pid_vel_ne_m.reset_filter();
    _pid_vel_ne_m.reset_I();
}

// Returns true if the NE position controller has run in the last 5 control loop cycles.
bool AC_PosControl::NE_is_active() const
{
    const uint32_t dt_ticks = AP::scheduler().ticks32() - _last_update_ne_ticks;
    return dt_ticks <= 1;
}

// Uses P and PID controllers to generate corrections which are added to feedforward velocity/acceleration.
// Requires all desired targets to be pre-set using the input_* or set_* methods.
void AC_PosControl::NE_update_controller()
{
    // check for ekf xy position reset
    NE_handle_ekf_reset();

    // Check for position control time out
    if (!NE_is_active()) {
        NE_init_controller();
        if (has_good_timing()) {
            // call internal error because initialisation has not been done
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }
    _last_update_ne_ticks = AP::scheduler().ticks32();

    float ahrsGndSpdLimit, ahrsControlScaleXY;
    AP::ahrs().getControlLimits(ahrsGndSpdLimit, ahrsControlScaleXY);

    // Update lateral position, velocity, and acceleration offsets using path shaping
    NE_update_offsets();

    // Position Controller

    // Combine position target with active NE offset to get absolute target
    _pos_target_ned_m.xy() = _pos_desired_ned_m.xy() + _pos_offset_ned_m.xy();

    // determine the combined position of the actual position and the disturbance from system ID mode
    // calculate the target velocity correction
    Vector2p comb_pos_ne_m = _pos_estimate_ned_m.xy();
    comb_pos_ne_m += _disturb_pos_ne_m.topostype();

    // Run P controller to compute velocity setpoint from position error
    Vector2f vel_target_ne_ms = _p_pos_ne_m.update_all(_pos_target_ned_m.xy(), comb_pos_ne_m);
    _pos_desired_ned_m.xy() = _pos_target_ned_m.xy() - _pos_offset_ned_m.xy();

    // Velocity Controller

    // Apply AHRS scaling (e.g. for optical flow noise compensation)
    vel_target_ne_ms *= ahrsControlScaleXY;
    vel_target_ne_ms *= _ne_control_scale_factor;

    _vel_target_ned_ms.xy() = vel_target_ne_ms;
    _vel_target_ned_ms.xy() += _vel_desired_ned_ms.xy() + _vel_offset_ned_ms.xy();

    // Velocity Controller

    // determine the combined velocity of the actual velocity and the disturbance from system ID mode
    Vector2f comb_vel_ne_ms = _vel_estimate_ned_ms.xy();
    comb_vel_ne_ms += _disturb_vel_ne_ms;

    // Run velocity PID controller and scale result for control authority
    Vector2f accel_target_ne_mss = _pid_vel_ne_m.update_all(_vel_target_ned_ms.xy(), comb_vel_ne_ms, _dt_s, _limit_vector_ned.xy());

    // Acceleration Controller
    
    // Apply AHRS scaling again to correct for measurement distortions
    accel_target_ne_mss *= ahrsControlScaleXY;
    accel_target_ne_mss *= _ne_control_scale_factor;

    _ne_control_scale_factor = 1.0;

    // pass the correction acceleration to the target acceleration output
    _accel_target_ned_mss.xy() = accel_target_ne_mss;
    _accel_target_ned_mss.xy() += _accel_desired_ned_mss.xy() + _accel_offset_ned_mss.xy();

    // limit acceleration using maximum lean angles
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_mss = angle_rad_to_accel_mss(angle_max_rad);
    // Save unbounded target for use in "limited" check (not unit-consistent with z!)
    _limit_vector_ned.xy() = _accel_target_ned_mss.xy();
    if (!limit_accel_xy(_vel_desired_ned_ms.xy(), _accel_target_ned_mss.xy(), accel_max_mss)) {
        // _accel_target_ned_mss was not limited so we can zero the xy limit vector
        _limit_vector_ned.xy().zero();
    }

    // Convert acceleration to roll/pitch angle targets (used by attitude controller)
    accel_NE_mss_to_lean_angles_rad(_accel_target_ned_mss.x, _accel_target_ned_mss.y, _roll_target_rad, _pitch_target_rad);

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
// See D_set_max_speed_accel_m() for full details.
// All values must be positive.
void AC_PosControl::D_set_max_speed_accel_cm(float decent_speed_max_cms, float climb_speed_max_cms, float accel_max_u_cmss)
{
    D_set_max_speed_accel_m(decent_speed_max_cms * 0.01, climb_speed_max_cms * 0.01, accel_max_u_cmss * 0.01);
}

// Sets maximum climb/descent rate (m/s) and vertical acceleration (m/s²) for the U-axis.
// These values are used for jerk-limited kinematic shaping of the vertical trajectory.
// All values must be positive.
void AC_PosControl::D_set_max_speed_accel_m(float decent_speed_max_ms, float climb_speed_max_ms, float accel_max_d_mss)
{
    // sanity check and update
    if (!is_zero(decent_speed_max_ms)) {
        _vel_max_down_ms = fabsf(decent_speed_max_ms);
    }
    if (!is_zero(climb_speed_max_ms)) {
        _vel_max_up_ms = fabsf(climb_speed_max_ms);
    }
    if (!is_zero(accel_max_d_mss)) {
        _accel_max_d_mss = fabsf(accel_max_d_mss);
    }

    // ensure the vertical Jerk is not limited by the filters in the Z acceleration PID object
    _jerk_max_d_msss = _shaping_jerk_d_msss;
    if (is_positive(_pid_accel_d_m.filt_T_hz())) {
        _jerk_max_d_msss = MIN(_jerk_max_d_msss, MIN(GRAVITY_MSS, _accel_max_d_mss) * (M_2PI * _pid_accel_d_m.filt_T_hz()) / 5.0);
    }
    if (is_positive(_pid_accel_d_m.filt_E_hz())) {
        _jerk_max_d_msss = MIN(_jerk_max_d_msss, MIN(GRAVITY_MSS, _accel_max_d_mss) * (M_2PI * _pid_accel_d_m.filt_E_hz()) / 5.0);
    }
}

// Sets vertical correction velocity and acceleration limits (cm/s, cm/s²).
// Should only be called during initialization to avoid discontinuities.
// See set_correction_speed_accel_U_m() for full details.
// All values must be positive.
void AC_PosControl::D_set_correction_speed_accel_cm(float decent_speed_max_cms, float climb_speed_max_cms, float accel_max_u_cmss)
{
    D_set_correction_speed_accel_m(decent_speed_max_cms * 0.01, climb_speed_max_cms * 0.01, accel_max_u_cmss * 0.01);
}

// Sets vertical correction velocity and acceleration limits (m/s, m/s²).
// These values constrain the correction output of the PID controller.
// All values must be positive.
void AC_PosControl::D_set_correction_speed_accel_m(float decent_speed_max_ms, float climb_speed_max_ms, float accel_max_d_mss)
{
    // define maximum position error and maximum first and second differential limits
    _p_pos_d_m.set_limits(-fabsf(decent_speed_max_ms), fabsf(climb_speed_max_ms), fabsf(accel_max_d_mss), 0.0f);
}

// Initializes U-axis controller to current position, velocity, and acceleration, disallowing descent.
// Used for takeoff or hold scenarios where downward motion is prohibited.
void AC_PosControl::D_init_controller_no_descent()
{
    // Initialise the position controller to the current throttle, position, velocity and acceleration.
    D_init_controller();

    // remove all descent if present
    _vel_target_ned_ms.z = MIN(0.0, _vel_target_ned_ms.z);
    _vel_desired_ned_ms.z = MIN(0.0, _vel_desired_ned_ms.z);
    _vel_terrain_d_ms = MIN(0.0, _vel_terrain_d_ms);
    _vel_offset_ned_ms.z = MIN(0.0, _vel_offset_ned_ms.z);
    _accel_target_ned_mss.z = MIN(0.0, _accel_target_ned_mss.z);
    _accel_desired_ned_mss.z = MIN(0.0, _accel_desired_ned_mss.z);
    _accel_terrain_d_mss = MIN(0.0, _accel_terrain_d_mss);
    _accel_offset_ned_mss.z = MIN(0.0, _accel_offset_ned_mss.z);
}

// Initializes U-axis controller to a stationary stopping point with zero velocity and acceleration.
// Used when the trajectory starts at rest but the initial altitude is unspecified.
// The resulting position target can be retrieved with get_pos_target_NED_m().
void AC_PosControl::D_init_controller_stopping_point()
{
    // Initialise the position controller to the current throttle, position, velocity and acceleration.
    D_init_controller();

    get_stopping_point_D_m(_pos_desired_ned_m.z);
    _pos_target_ned_m.z = _pos_desired_ned_m.z + _pos_offset_ned_m.z;
    _vel_desired_ned_ms.z = 0.0f;
    _accel_desired_ned_mss.z = 0.0f;
}

// Smoothly decays U-axis acceleration to zero over time while maintaining current vertical velocity.
// Reduces requested acceleration by ~95% every 0.5 seconds to avoid abrupt transitions.
// `throttle_setting` is used to determine whether to preserve positive acceleration in low-thrust cases.
void AC_PosControl::D_relax_controller(float throttle_setting)
{
    // Initialise the position controller to the current position, velocity and acceleration.
    D_init_controller();

    // D_init_controller has set the acceleration PID I term to generate the current throttle set point
    // Use relax_integrator to decay the throttle set point to throttle_setting
    _pid_accel_d_m.relax_integrator(-(throttle_setting - _motors.get_throttle_hover()), _dt_s, POSCONTROL_RELAX_TC);
}

// Fully initializes the U-axis controller with current position, velocity, acceleration, and attitude.
// Used during standard controller activation when full state is known.
// Private function shared by other vertical initializers.
void AC_PosControl::D_init_controller()
{
    // initialise terrain targets and offsets to zero
    init_terrain();

    // initialise offsets to target offsets and ensure offset targets are zero if they have not been updated.
    D_init_offsets();

    _pos_target_ned_m.z = _pos_estimate_ned_m.z;
    _pos_desired_ned_m.z = _pos_target_ned_m.z - _pos_offset_ned_m.z;

    _vel_target_ned_ms.z = _vel_estimate_ned_ms.z;
    _vel_desired_ned_ms.z = _vel_target_ned_ms.z - _vel_offset_ned_ms.z;

    // Reset I term of velocity PID
    _pid_vel_d_m.reset_filter();
    _pid_vel_d_m.set_integrator(0.0f);

    _accel_target_ned_mss.z = constrain_float(get_estimated_accel_D_mss(), -_accel_max_d_mss, _accel_max_d_mss);
    _accel_desired_ned_mss.z = _accel_target_ned_mss.z - (_accel_offset_ned_mss.z + _accel_terrain_d_mss);
    _pid_accel_d_m.reset_filter();

    // Set acceleration PID I term based on the current throttle
    // Remove the expected P term due to _accel_desired_ned_mss.z being constrained to _accel_max_d_mss
    // Remove the expected FF term due to non-zero _accel_target_ned_mss.z
    _pid_accel_d_m.set_integrator(-(_attitude_control.get_throttle_in() - _motors.get_throttle_hover())
        - _pid_accel_d_m.kP() * (_accel_target_ned_mss.z - get_estimated_accel_D_mss())
        - _pid_accel_d_m.ff() * _accel_target_ned_mss.z);

    // initialise ekf z reset handler
    D_init_ekf_reset();

    // initialise z_controller time out
    _last_update_d_ticks = AP::scheduler().ticks32();
}

// Sets the desired vertical acceleration in m/s² using jerk-limited shaping.
// Smoothly transitions to the target acceleration from current kinematic state.
// Constraints: max acceleration and jerk set via D_set_max_speed_accel_m().
void AC_PosControl::input_accel_D_m(float accel_d_mss)
{
    // calculated increased maximum jerk if over speed
    float jerk_max_d_msss = _jerk_max_d_msss * calculate_overspeed_gain();

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_ned_m.z, _vel_desired_ned_ms.z, _accel_desired_ned_mss.z, _dt_s, _limit_vector_ned.z, _p_pos_d_m.get_error(), _pid_vel_d_m.get_error());

    shape_accel(accel_d_mss, _accel_desired_ned_mss.z, jerk_max_d_msss, _dt_s);
}

// Sets desired vertical velocity and acceleration (m/s, m/s²) using jerk-limited shaping.
// Calculates required acceleration using current vertical kinematics.
// If `limit_output` is true, limits apply to the combined (desired + correction) command.
void AC_PosControl::input_vel_accel_D_m(float &vel_d_ms, float accel_d_mss, bool limit_output)
{
    // calculated increased maximum acceleration and jerk if over speed
    const float overspeed_gain = calculate_overspeed_gain();
    const float accel_max_d_mss = _accel_max_d_mss * overspeed_gain;
    const float jerk_max_d_msss = _jerk_max_d_msss * overspeed_gain;

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_ned_m.z, _vel_desired_ned_ms.z, _accel_desired_ned_mss.z, _dt_s, _limit_vector_ned.z, _p_pos_d_m.get_error(), _pid_vel_d_m.get_error());

    shape_vel_accel(vel_d_ms, accel_d_mss,
                    _vel_desired_ned_ms.z, _accel_desired_ned_mss.z,
                    -accel_max_d_mss, constrain_float(accel_max_d_mss, 0.0, 7.5),
                    jerk_max_d_msss, _dt_s, limit_output);

    update_vel_accel(vel_d_ms, accel_d_mss, _dt_s, 0.0, 0.0);
}

// Generates a vertical trajectory using the given climb rate in cm/s and jerk-limited shaping.
// Adjusts the internal target altitude based on integrated climb rate.
// See set_pos_target_D_from_climb_rate_m() for full details.
void AC_PosControl::D_set_pos_target_from_climb_rate_cms(float climb_rate_cms)
{
    D_set_pos_target_from_climb_rate_ms(climb_rate_cms * 0.01);
}

// Generates a vertical trajectory using the given climb rate in m/s and jerk-limited shaping.
// Target altitude is updated over time by integrating the climb rate.
void AC_PosControl::D_set_pos_target_from_climb_rate_ms(float climb_rate_ms, bool ignore_descent_limit)
{
    if (ignore_descent_limit) {
        // turn off limits in the down (positive z) direction
        _limit_vector_ned.z = MIN(_limit_vector_ned.z, 0.0f);
    }

    float vel_d_ms = -climb_rate_ms;
    input_vel_accel_D_m(vel_d_ms, 0.0);
}

// Sets vertical position, velocity, and acceleration in cm using jerk-limited shaping.
// See input_pos_vel_accel_D_m() for full details.
void AC_PosControl::input_pos_vel_accel_U_cm(float &pos_u_cm, float &vel_u_cms, float accel_u_cmss, bool limit_output)
{
    float pos_d_m = -pos_u_cm * 0.01;
    float vel_d_ms = -vel_u_cms * 0.01;
    const float accel_d_mss = -accel_u_cmss * 0.01;
    input_pos_vel_accel_D_m(pos_d_m, vel_d_ms, accel_d_mss, limit_output);
    pos_u_cm = -pos_d_m * 100.0;
    vel_u_cms = -vel_d_ms * 100.0;
}

// Sets vertical position, velocity, and acceleration in meters using jerk-limited shaping.
// Calculates required acceleration using current state and constraints.
// If `limit_output` is true, limits are applied to combined (desired + correction) command.
void AC_PosControl::input_pos_vel_accel_D_m(float &pos_d_m, float &vel_d_ms, float accel_d_mss, bool limit_output)
{
    // calculated increased maximum acceleration and jerk if over speed
    const float overspeed_gain = calculate_overspeed_gain();
    const float accel_max_d_mss = _accel_max_d_mss * overspeed_gain;
    const float jerk_max_d_msss = _jerk_max_d_msss * overspeed_gain;

    // adjust desired altitude if motors have not hit their limits
    update_pos_vel_accel(_pos_desired_ned_m.z, _vel_desired_ned_ms.z, _accel_desired_ned_mss.z, _dt_s, _limit_vector_ned.z, _p_pos_d_m.get_error(), _pid_vel_d_m.get_error());

    shape_pos_vel_accel(pos_d_m, vel_d_ms, accel_d_mss,
                        _pos_desired_ned_m.z, _vel_desired_ned_ms.z, _accel_desired_ned_mss.z,
                        -_vel_max_up_ms, _vel_max_down_ms,
                        -accel_max_d_mss, constrain_float(accel_max_d_mss, 0.0, 7.5),
                        jerk_max_d_msss, _dt_s, limit_output);

    postype_t posp = pos_d_m;
    update_pos_vel_accel(posp, vel_d_ms, accel_d_mss, _dt_s, 0.0, 0.0, 0.0);
    pos_d_m = posp;
}

// Sets target altitude in cm using jerk-limited shaping to gradually move to the new position.
// See D_set_alt_target_with_slew_m() for full details.
void AC_PosControl::set_alt_target_with_slew_cm(float pos_u_cm)
{
    D_set_alt_target_with_slew_m(pos_u_cm * 0.01);
}

// Sets target altitude in meters using jerk-limited shaping.
void AC_PosControl::D_set_alt_target_with_slew_m(float pos_u_m)
{
    float pos_d_m = -pos_u_m;
    float zero = 0;
    input_pos_vel_accel_D_m(pos_d_m, zero, 0);
}

// Updates vertical (U) offsets by gradually moving them toward their targets.
void AC_PosControl::D_update_offsets()
{
    // Check if vertical offset targets have timed out
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_d_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        // Timeout: reset U-axis offset targets to zero
        _pos_offset_target_ned_m.z = 0.0;
        _vel_offset_target_ned_ms.z = 0.0;
        _accel_offset_target_ned_mss.z = 0.0;
    }

    // Advance current offset state using PID-derived feedback and vertical limits
    postype_t p_offset_d_m = _pos_offset_ned_m.z;
    update_pos_vel_accel(p_offset_d_m, _vel_offset_ned_ms.z, _accel_offset_ned_mss.z, _dt_s, MIN(_limit_vector_ned.z, 0.0f), _p_pos_d_m.get_error(), _pid_vel_d_m.get_error());
    _pos_offset_ned_m.z = p_offset_d_m;

    // Shape offset trajectory (position/velocity/acceleration) using jerk-limited smoothing
    shape_pos_vel_accel(_pos_offset_target_ned_m.z, _vel_offset_target_ned_ms.z, _accel_offset_target_ned_mss.z,
        _pos_offset_ned_m.z, _vel_offset_ned_ms.z, _accel_offset_ned_mss.z,
        -get_max_speed_up_ms(), get_max_speed_down_ms(),
        -D_get_max_accel_mss(), D_get_max_accel_mss(),
        _jerk_max_d_msss, _dt_s, false);

    // Update target state forward in time with assumed zero velocity/acceleration targets
    p_offset_d_m = _pos_offset_target_ned_m.z;
    update_pos_vel_accel(p_offset_d_m, _vel_offset_target_ned_ms.z, _accel_offset_target_ned_mss.z, _dt_s, 0.0, 0.0, 0.0);
    _pos_offset_target_ned_m.z = p_offset_d_m;
}

// Returns true if the U-axis controller has run in the last 5 control loop cycles.
bool AC_PosControl::D_is_active() const
{
    const uint32_t dt_ticks = AP::scheduler().ticks32() - _last_update_d_ticks;
    return dt_ticks <= 1;
}

// Runs the vertical (U-axis) position controller.
// Computes output acceleration based on position and velocity errors using PID correction.
// Feedforward velocity and acceleration are combined with corrections to produce a smooth vertical command.
// Desired position, velocity, and acceleration must be set before calling.
void AC_PosControl::D_update_controller()
{
    // check for ekf z-axis position reset
    D_handle_ekf_reset();

    // Check for z_controller time out
    if (!D_is_active()) {
        D_init_controller();
        if (has_good_timing()) {
            // call internal error because initialisation has not been done
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }
    _last_update_d_ticks = AP::scheduler().ticks32();

    // Update vertical offset targets and terrain estimate
    D_update_offsets();
    update_terrain();

    // Position Controller

    // Combine desired + offset + terrain for final position target
    _pos_target_ned_m.z = _pos_desired_ned_m.z + _pos_offset_ned_m.z + _pos_terrain_d_m;

    // P controller: convert position error to velocity target
    _vel_target_ned_ms.z = _p_pos_d_m.update_all(_pos_target_ned_m.z, _pos_estimate_ned_m.z);
    _vel_target_ned_ms.z *= AP::ahrs().getControlScaleZ();

    _pos_desired_ned_m.z = _pos_target_ned_m.z - (_pos_offset_ned_m.z + _pos_terrain_d_m);

    // add feed forward component
    _vel_target_ned_ms.z += _vel_desired_ned_ms.z + _vel_offset_ned_ms.z + _vel_terrain_d_ms;

    // Velocity Controller

    // PID controller: convert velocity error to acceleration
    _accel_target_ned_mss.z = _pid_vel_d_m.update_all(_vel_target_ned_ms.z, _vel_estimate_ned_ms.z, _dt_s, _motors.limit.throttle_lower, _motors.limit.throttle_upper);
    _accel_target_ned_mss.z *= AP::ahrs().getControlScaleZ();

    // add feed forward component
    _accel_target_ned_mss.z += _accel_desired_ned_mss.z + _accel_offset_ned_mss.z + _accel_terrain_d_mss;

    // Acceleration Controller

    // Gravity-compensated vertical acceleration measurement (positive = up)
    const float estimated_accel_d_mss = get_estimated_accel_D_mss();

    // Ensure integrator can produce enough thrust to overcome hover throttle
    if (_motors.get_throttle_hover() > _pid_accel_d_m.imax()) {
        _pid_accel_d_m.set_imax(_motors.get_throttle_hover());
    }
    float thrust_d_norm;
    if (_vibe_comp_enabled) {
        // Use vibration-resistant throttle estimator (feedforward + scaled integrator)
        thrust_d_norm = get_throttle_with_vibration_override();
    } else {
        // Standard PID update using vertical acceleration error
        thrust_d_norm = _pid_accel_d_m.update_all(_accel_target_ned_mss.z, estimated_accel_d_mss, _dt_s, (_motors.limit.throttle_lower || _motors.limit.throttle_upper));
        // Include FF contribution to reduce delay
        thrust_d_norm += _pid_accel_d_m.get_ff();
    }
    thrust_d_norm -= _motors.get_throttle_hover();

    // Actuator commands

    // Send final throttle output to attitude controller (includes angle boost)
    _attitude_control.set_throttle_out(-thrust_d_norm, true, POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);

    // Check for vertical controller health

    // Update health indicator based on error magnitude vs configured speed range
    float error_ratio = _pid_vel_d_m.get_error() / _vel_max_down_ms;
    _vel_d_control_ratio += _dt_s * 0.1f * (0.5 - error_ratio);
    _vel_d_control_ratio = constrain_float(_vel_d_control_ratio, 0.0f, 2.0f);

    // set vertical component of the limit vector
    if (_motors.limit.throttle_upper) {
        _limit_vector_ned.z = -1.0f;
    } else if (_motors.limit.throttle_lower) {
        _limit_vector_ned.z = 1.0f;
    } else {
        _limit_vector_ned.z = 0.0f;
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

// Sets externally computed NED position, velocity, and acceleration in meters, m/s, and m/s².
// Use when path planning or shaping is done outside this controller.
void AC_PosControl::set_pos_vel_accel_NED_m(const Vector3p& pos_ned_m, const Vector3f& vel_ned_ms, const Vector3f& accel_ned_mss)
{
    _pos_desired_ned_m = pos_ned_m;
    _vel_desired_ned_ms = vel_ned_ms;
    _accel_desired_ned_mss = accel_ned_mss;
}

// Sets externally computed NE position, velocity, and acceleration in meters, m/s, and m/s².
// Use when path planning or shaping is done outside this controller.
void AC_PosControl::set_pos_vel_accel_NE_m(const Vector2p& pos_ne_m, const Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss)
{
    _pos_desired_ned_m.xy() = pos_ne_m;
    _vel_desired_ned_ms.xy() = vel_ne_ms;
    _accel_desired_ned_mss.xy() = accel_ne_mss;
}

// Converts lean angles (rad) to NED acceleration in m/s².
Vector3f AC_PosControl::lean_angles_rad_to_accel_NED_mss(const Vector3f& att_target_euler_rad) const
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
        -GRAVITY_MSS
    };
}

/// Terrain

// Initializes terrain position, velocity, and acceleration to match the terrain target.
void AC_PosControl::init_terrain()
{
    // set terrain position and target to zero
    _pos_terrain_target_d_m = 0.0;
    _pos_terrain_d_m = 0.0;

    // set velocity offset to zero
    _vel_terrain_d_ms = 0.0;

    // set acceleration offset to zero
    _accel_terrain_d_mss = 0.0;
}

// Initializes both the terrain altitude and terrain target to the same value
// (altitude above EKF origin in centimeters, Up-positive).
// See init_pos_terrain_D_m() for full description.
void AC_PosControl::init_pos_terrain_U_cm(float pos_terrain_u_cm)
{
    init_pos_terrain_D_m(-pos_terrain_u_cm * 0.01);
}

// Initializes both the terrain altitude and terrain target to the same value
// (relative to EKF origin in meters, Down-positive).
void AC_PosControl::init_pos_terrain_D_m(float pos_terrain_d_m)
{
    _pos_desired_ned_m.z -= (pos_terrain_d_m - _pos_terrain_d_m);
    _pos_terrain_target_d_m = pos_terrain_d_m;
    _pos_terrain_d_m = pos_terrain_d_m;
}


/// Offsets

// Initializes NE position/velocity/acceleration offsets to match their respective targets.
void AC_PosControl::NE_init_offsets()
{
    // check for offset target timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_ne_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        _pos_offset_target_ned_m.xy().zero();
        _vel_offset_target_ned_ms.xy().zero();
        _accel_offset_target_ned_mss.xy().zero();
    }

    // set position offset to target
    _pos_offset_ned_m.xy() = _pos_offset_target_ned_m.xy();

    // set velocity offset to target
    _vel_offset_ned_ms.xy() = _vel_offset_target_ned_ms.xy();

    // set acceleration offset to target
    _accel_offset_ned_mss.xy() = _accel_offset_target_ned_mss.xy();
}

// Initializes vertical (U) offsets to match their respective targets.
void AC_PosControl::D_init_offsets()
{
    // check for offset target timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _posvelaccel_offset_target_d_ms > POSCONTROL_POSVELACCEL_OFFSET_TARGET_TIMEOUT_MS) {
        _pos_offset_target_ned_m.z = 0.0;
        _vel_offset_target_ned_ms.z = 0.0;
        _accel_offset_target_ned_mss.z = 0.0;
    }

    // set position offset to target
    _pos_offset_ned_m.z = _pos_offset_target_ned_m.z;

    // set velocity offset to target
    _vel_offset_ned_ms.z = _vel_offset_target_ned_ms.z;

    // set acceleration offset to target
    _accel_offset_ned_mss.z = _accel_offset_target_ned_mss.z;
}

#if AP_SCRIPTING_ENABLED
// Sets additional position, velocity, and acceleration offsets in meters (NED frame) for scripting.
// Offsets are added to the controller’s internal target.
// Used in LUA
bool AC_PosControl::set_posvelaccel_offset(const Vector3f &pos_offset_ned_m, const Vector3f &vel_offset_ned_ms, const Vector3f &accel_offset_ned_mss)
{
    set_posvelaccel_offset_target_NE_m(pos_offset_ned_m.topostype().xy(), vel_offset_ned_ms.xy(), accel_offset_ned_mss.xy());
    set_posvelaccel_offset_target_D_m(pos_offset_ned_m.topostype().z, vel_offset_ned_ms.z, accel_offset_ned_mss.z);
    return true;
}

// Retrieves current scripted offsets in meters (NED frame).
// Used in LUA
bool AC_PosControl::get_posvelaccel_offset(Vector3f &pos_offset_ned_m, Vector3f &vel_offset_ned_ms, Vector3f &accel_offset_ned_mss)
{
    pos_offset_ned_m = _pos_offset_target_ned_m.tofloat();
    vel_offset_ned_ms = _vel_offset_target_ned_ms;
    accel_offset_ned_mss = _accel_offset_target_ned_mss;
    return true;
}

// Retrieves current target velocity (NED frame, m/s) including any scripted offset.
// Used in LUA
bool AC_PosControl::get_vel_target(Vector3f &vel_target_ned_ms)
{
    if (!NE_is_active() || !D_is_active()) {
        return false;
    }

    vel_target_ned_ms = _vel_target_ned_ms;
    return true;
}

// Retrieves current target acceleration (NED frame, m/s²) including any scripted offset.
// Used in LUA
bool AC_PosControl::get_accel_target(Vector3f &accel_target_ned_mss)
{
    if (!NE_is_active() || !D_is_active()) {
        return false;
    }

    accel_target_ned_mss = _accel_target_ned_mss;
    return true;
}
#endif

// Sets NE offset targets in meters, m/s, and m/s².
void AC_PosControl::set_posvelaccel_offset_target_NE_m(const Vector2p& pos_offset_target_ne_m, const Vector2f& vel_offset_target_ne_ms, const Vector2f& accel_offset_target_ne_mss)
{
    // set position offset target
    _pos_offset_target_ned_m.xy() = pos_offset_target_ne_m;

    // set velocity offset target
    _vel_offset_target_ned_ms.xy() = vel_offset_target_ne_ms;

    // set acceleration offset target
    _accel_offset_target_ned_mss.xy() = accel_offset_target_ne_mss;

    // record time of update so we can detect timeouts
    _posvelaccel_offset_target_ne_ms = AP_HAL::millis();
}

// Sets vertical offset targets (m, m/s, m/s²) relative to EKF origin in meters, Down-positive.
void AC_PosControl::set_posvelaccel_offset_target_D_m(float pos_offset_target_d_m, float vel_offset_target_d_ms, const float accel_offset_target_d_mss)
{
    // set position offset target
    _pos_offset_target_ned_m.z = pos_offset_target_d_m;

    // set velocity offset target
    _vel_offset_target_ned_ms.z = vel_offset_target_d_ms;

    // set acceleration offset target
    _accel_offset_target_ned_mss.z = accel_offset_target_d_mss;

    // record time of update so we can detect timeouts
    _posvelaccel_offset_target_d_ms = AP_HAL::millis();
}

// Returns desired thrust direction as a unit vector in the body frame.
Vector3f AC_PosControl::get_thrust_vector() const
{
    Vector3f accel_target_ned_mss = get_accel_target_NED_mss();
    accel_target_ned_mss.z = -GRAVITY_MSS;
    return accel_target_ned_mss;
}

// Computes NE stopping point in meters based on current position, velocity, and acceleration.
void AC_PosControl::get_stopping_point_NE_m(Vector2p &stopping_point_ne_m) const
{
    // Start from estimated NE position with offset removed
    // todo: we should use the current target position and velocity if we are currently running the position controller
    stopping_point_ne_m = _pos_estimate_ned_m.xy();
    stopping_point_ne_m -= _pos_offset_ned_m.xy();

    Vector2f vel_estimate_ne_ms = _vel_estimate_ned_ms.xy();
    vel_estimate_ne_ms -= _vel_offset_ned_ms.xy();

    // Compute velocity magnitude
    float speed_ne_ms = vel_estimate_ne_ms.length();

    if (!is_positive(speed_ne_ms)) {
        return;
    }

    // Use current P gain and max accel to estimate stopping distance
    float kP = _p_pos_ne_m.kP();
    const float stopping_dist_m = stopping_distance(constrain_float(speed_ne_ms, 0.0, _vel_max_ne_ms), kP, _accel_max_ne_mss);
    if (!is_positive(stopping_dist_m)) {
        return;
    }

    // Project stopping distance along current velocity direction
    // todo: convert velocity to a unit vector instead.
    const float stopping_time_s = stopping_dist_m / speed_ne_ms;
    stopping_point_ne_m += (vel_estimate_ne_ms * stopping_time_s).topostype();
}

// Computes vertical stopping point in meters based on current velocity and acceleration.
void AC_PosControl::get_stopping_point_D_m(postype_t &stopping_point_d_m) const
{
    float curr_pos_d_m = _pos_estimate_ned_m.z;
    curr_pos_d_m -= _pos_offset_ned_m.z;

    float curr_vel_d_ms = _vel_estimate_ned_ms.z;
    curr_vel_d_ms -= _vel_offset_ned_ms.z;

    // If controller is unconfigured or disabled, return current position
    if (!is_positive(_p_pos_d_m.kP()) || !is_positive(_accel_max_d_mss)) {
        stopping_point_d_m = curr_pos_d_m;
        return;
    }

    // Estimate stopping point using current velocity, P gain, and max vertical acceleration
    stopping_point_d_m = curr_pos_d_m + constrain_float(stopping_distance(curr_vel_d_ms, _p_pos_d_m.kP(), _accel_max_d_mss), - POSCONTROL_STOPPING_DIST_UP_MAX_M, POSCONTROL_STOPPING_DIST_DOWN_MAX_M);
}

// Returns bearing from current position to position target in radians.
// 0 = North, positive = clockwise.
float AC_PosControl::get_bearing_to_target_rad() const
{
    return (_pos_target_ned_m.xy() - _pos_estimate_ned_m.xy()).angle();
}


///
/// System methods
///

// Updates internal NED position and velocity estimates from AHRS.
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
    _pos_estimate_ned_m = pos_estimate_ned_m;

    Vector3f vel_estimate_ned_ms;
    if (!AP::ahrs().get_velocity_NED(vel_estimate_ned_ms) || high_vibes) {
        float rate_z;
        if (AP::ahrs().get_vert_pos_rate_D(rate_z)) {
            vel_estimate_ned_ms.z = rate_z;
        }
    }
    _vel_estimate_ned_ms = vel_estimate_ned_ms;
}

// Calculates vertical throttle using vibration-resistant feedforward estimation.
// Returns throttle output using manual feedforward gain for vibration compensation mode.
// Integrator is adjusted using velocity error when PID is being overridden.
float AC_PosControl::get_throttle_with_vibration_override()
{
    const float thr_per_accel_d_mss = _motors.get_throttle_hover();
    // Estimate throttle based on desired acceleration (manual feedforward gain).
    // Used when IMU vibrations corrupt raw acceleration measurements.
    // Allow integrator to compensate for velocity error only if not thrust-limited,
    // or if integrator is actively helping counteract velocity error direction.
    // ToDo: clear pid_info P, I and D terms for logging
    if (!(_motors.limit.throttle_lower || _motors.limit.throttle_upper) || ((is_positive(_pid_accel_d_m.get_i()) && is_negative(_pid_vel_d_m.get_error())) || (is_negative(_pid_accel_d_m.get_i()) && is_positive(_pid_vel_d_m.get_error())))) {
        // Adjust integrator to help reduce velocity error.
        // Note: scale by velocity P-gain and an override-specific I gain.
        _pid_accel_d_m.set_integrator(_pid_accel_d_m.get_i() + _dt_s * thr_per_accel_d_mss * _pid_vel_d_m.get_error() * _pid_vel_d_m.kP() * POSCONTROL_VIBE_COMP_I_GAIN);
    }
    // Final throttle = P term (feedforward) + scaled I term.
    return POSCONTROL_VIBE_COMP_P_GAIN * thr_per_accel_d_mss * _accel_target_ned_mss.z + _pid_accel_d_m.get_i();
}

// Resets NED position controller state to prevent transients when exiting standby.
// Zeros I-terms and aligns targets to current position.
void AC_PosControl::NED_standby_reset()
{
    // Reset vertical acceleration controller integrator to prevent throttle bias on reentry
    _pid_accel_d_m.set_integrator(0.0f);

    // Reset position controller targets to match current estimate — avoids position jumps
    _pos_target_ned_m = _pos_estimate_ned_m;

    // Reset horizontal velocity controller integrator and derivative filter
    _pid_vel_ne_m.reset_filter();

    // Reset EKF XY position reset tracking for NE controller
    NE_init_ekf_reset();
}

#if HAL_LOGGING_ENABLED
// Writes position controller diagnostic logs (PSCN, PSCE, etc).
void AC_PosControl::write_log()
{
    if (NE_is_active()) {
        float accel_n_mss, accel_e_mss;
        lean_angles_to_accel_NE_mss(accel_n_mss, accel_e_mss);

        // Log North-axis position control (PSCN): desired, target, and actual
        Write_PSCN(_pos_desired_ned_m.x, _pos_target_ned_m.x, _pos_estimate_ned_m.x ,
                   _vel_desired_ned_ms.x, _vel_target_ned_ms.x, _vel_estimate_ned_ms.x,
                   _accel_desired_ned_mss.x, _accel_target_ned_mss.x, accel_n_mss);

        // Log East-axis position control (PSCE): desired, target, and actual
        Write_PSCE(_pos_desired_ned_m.y, _pos_target_ned_m.y, _pos_estimate_ned_m.y,
                   _vel_desired_ned_ms.y, _vel_target_ned_ms.y, _vel_estimate_ned_ms.y,
                   _accel_desired_ned_mss.y, _accel_target_ned_mss.y, accel_e_mss);

        // log offsets if they are being used
        if (!_pos_offset_ned_m.xy().is_zero()) {
            // Log North offset tracking (PSON)
            Write_PSON(_pos_offset_target_ned_m.x, _pos_offset_ned_m.x, _vel_offset_target_ned_ms.x, _vel_offset_ned_ms.x, _accel_offset_target_ned_mss.x, _accel_offset_ned_mss.x);

            // Log East offset tracking (PSOE)
            Write_PSOE(_pos_offset_target_ned_m.y, _pos_offset_ned_m.y, _vel_offset_target_ned_ms.y, _vel_offset_ned_ms.y, _accel_offset_target_ned_mss.y, _accel_offset_ned_mss.y);
        }
    }

    if (D_is_active()) {
        // Log Down-axis position control (PSCD)
        Write_PSCD(_pos_desired_ned_m.z, _pos_target_ned_m.z, _pos_estimate_ned_m.z,
                   _vel_desired_ned_ms.z, _vel_target_ned_ms.z, _vel_estimate_ned_ms.z,
                   _accel_desired_ned_mss.z, _accel_target_ned_mss.z, get_estimated_accel_D_mss());

        // log down and terrain offsets if they are being used
        if (!is_zero(_pos_offset_ned_m.z)) {
            // Log Down offset tracking (PSOD)
            Write_PSOD(_pos_offset_target_ned_m.z, _pos_offset_ned_m.z, _vel_offset_target_ned_ms.z, _vel_offset_ned_ms.z, _accel_offset_target_ned_mss.z, _accel_offset_ned_mss.z);
        }
        if (!is_zero(_pos_terrain_d_m)) {
            // Log terrain-following offset (PSOT)
            Write_PSOT(_pos_terrain_target_d_m, _pos_terrain_d_m, 0, _vel_terrain_d_ms, 0, _accel_terrain_d_mss);
        }
    }
}
#endif  // HAL_LOGGING_ENABLED

// Returns lateral distance to closest point on active trajectory in meters.
// Used to assess horizontal deviation from path.
float AC_PosControl::crosstrack_error_m() const
{
    const Vector2f pos_error = (_pos_target_ned_m.xy() - _pos_estimate_ned_m.xy()).tofloat();
    if (is_zero(_vel_desired_ned_ms.xy().length_squared())) {
        // No desired velocity → return direct distance to target
        return pos_error.length();
    } else {
        // Project position error onto desired velocity vector
        const Vector2f vel_unit = _vel_desired_ned_ms.xy().normalized();
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
    if (_limit_vector_ned.xy().is_zero()) {  
        return false;  
    }  
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_mss = angle_rad_to_accel_mss(angle_max_rad);
    // Check for pitch limiting in the forward direction
    const float accel_fwd_unlimited_mss = _limit_vector_ned.x * _ahrs.cos_yaw() + _limit_vector_ned.y * _ahrs.sin_yaw();
    const float pitch_target_unlimited_deg = accel_mss_to_angle_deg(- MIN(accel_fwd_unlimited_mss, accel_max_mss));
    const float accel_fwd_limited = _accel_target_ned_mss.x * _ahrs.cos_yaw() + _accel_target_ned_mss.y * _ahrs.sin_yaw();
    const float pitch_target_limited_deg = accel_mss_to_angle_deg(- accel_fwd_limited);

    return is_negative(pitch_target_unlimited_deg) && pitch_target_unlimited_deg < pitch_target_limited_deg;
}
#endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane)

///
/// private methods
///

/// Terrain

// Updates terrain estimate (_pos_terrain_d_m) toward target using filter time constants.
void AC_PosControl::update_terrain()
{
    // update position, velocity, acceleration offsets for this iteration
    postype_t pos_terrain_d_m = _pos_terrain_d_m;
    update_pos_vel_accel(pos_terrain_d_m, _vel_terrain_d_ms, _accel_terrain_d_mss, _dt_s, MIN(_limit_vector_ned.z, 0.0f), _p_pos_d_m.get_error(), _pid_vel_d_m.get_error());
    _pos_terrain_d_m = pos_terrain_d_m;

    // input shape horizontal position, velocity and acceleration offsets
    shape_pos_vel_accel(_pos_terrain_target_d_m, 0.0, 0.0,
        _pos_terrain_d_m, _vel_terrain_d_ms, _accel_terrain_d_mss,
        -get_max_speed_up_ms(), get_max_speed_down_ms(),
        -D_get_max_accel_mss(), D_get_max_accel_mss(),
        _jerk_max_d_msss, _dt_s, false);

    // we do not have to update _pos_terrain_target_d_m because we assume the target velocity and acceleration are zero
    // if we know how fast the terrain altitude is changing we would add update_pos_vel_accel for _pos_terrain_target_d_m here
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
    Vector3f accel_ne_mss = lean_angles_rad_to_accel_NED_mss(att_target_euler_rad);

    accel_n_mss = accel_ne_mss.x;
    accel_e_mss = accel_ne_mss.y;
}

// Computes desired yaw and yaw rate based on the NE acceleration and velocity vectors.
// Aligns yaw with the direction of travel if speed exceeds 5% of maximum.
void AC_PosControl::calculate_yaw_and_rate_yaw()
{
    // Calculate the turn rate
    float turn_rate_rads = 0.0f;
    const float vel_desired_length_ne_ms = _vel_desired_ned_ms.xy().length();
    if (is_positive(vel_desired_length_ne_ms)) {
        // Project acceleration vector into velocity direction to extract forward acceleration component
        const float accel_forward_mss = (_accel_desired_ned_mss.x * _vel_desired_ned_ms.x + _accel_desired_ned_mss.y * _vel_desired_ned_ms.y) / vel_desired_length_ne_ms;
        // Subtract forward component to isolate turn acceleration perpendicular to velocity vector
        const Vector2f accel_turn_ne_mss = _accel_desired_ned_mss.xy() - _vel_desired_ned_ms.xy() * accel_forward_mss / vel_desired_length_ne_ms;
        // Compute turn rate from lateral acceleration and velocity (centripetal formula)
        const float accel_turn_length_ne_mss = accel_turn_ne_mss.length();
        turn_rate_rads = accel_turn_length_ne_mss / vel_desired_length_ne_ms;
        // Determine turn direction: positive = clockwise (right)
        if ((accel_turn_ne_mss.y * _vel_desired_ned_ms.x - accel_turn_ne_mss.x * _vel_desired_ned_ms.y) < 0.0) {
            turn_rate_rads = -turn_rate_rads;
        }
    }

    // If vehicle is moving significantly, align yaw to velocity vector and apply computed turn rate
    if (vel_desired_length_ne_ms > _vel_max_ne_ms * 0.05f) {
        _yaw_target_rad = _vel_desired_ned_ms.xy().angle();
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
    if (_vel_desired_ned_ms.z > _vel_max_down_ms && !is_zero(_vel_max_down_ms)) {
        return POSCONTROL_OVERSPEED_GAIN_U * _vel_desired_ned_ms.z / _vel_max_down_ms;
    }

    // If desired climb speed exceeds configured max, scale acceleration/jerk proportionally
    if (_vel_desired_ned_ms.z < -_vel_max_up_ms && !is_zero(_vel_max_up_ms)) {
        return -POSCONTROL_OVERSPEED_GAIN_U * _vel_desired_ned_ms.z / _vel_max_up_ms;
    }

    // Within normal speed limits — use nominal acceleration and jerk
    return 1.0;
}

// Initializes tracking of NE EKF position resets.
void AC_PosControl::NE_init_ekf_reset()
{
    Vector2f pos_shift;
    _ekf_ne_reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
}

// Handles NE position reset detection and response (e.g., clearing accumulated errors).
void AC_PosControl::NE_handle_ekf_reset()
{
    // Check for EKF-reported NE position shift since last update
    Vector2f pos_shift_ne_m;
    uint32_t reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift_ne_m);
    // todo: the actual difference in position and velocity estimation.
    // This will prevent the need to pause error calculation for one cycle.

    if (reset_ms != _ekf_ne_reset_ms) {
        // This ensures controller output remains continuous after EKF realigns the origin.

        // Reconstruct position target relative to the to new EKF estimation to maintain the current position error
        Vector2p delta_pos_estimate_ne_m = _p_pos_ne_m.get_error().topostype() - (_pos_target_ned_m.xy() - _pos_estimate_ned_m.xy());
        _pos_target_ned_m.xy() += delta_pos_estimate_ne_m;

        // Reconstruct velocity target relative to the to new EKF estimation to maintain the current velocity error
        Vector2f delta_vel_estimate_ne_ms = _pid_vel_ne_m.get_error() - (_vel_target_ned_ms.xy() - _vel_estimate_ned_ms.xy());
        _vel_target_ned_ms.xy() += delta_vel_estimate_ne_ms;

        switch (_ekf_reset_method) {
        case EKFResetMethod::MoveTarget:
            // Reset NE controller desired position and velocity to preserve actual position control during Loiter, PosHold, etc.
            _pos_desired_ned_m.xy() += delta_pos_estimate_ne_m;
            _vel_desired_ned_ms.xy() += delta_vel_estimate_ne_ms;
            break;
        case EKFResetMethod::MoveVehicle:
            // Move the change in estimate into the offsest to move the aircraft to our new estimate smoothly during Auto, Guided, etc.
            _pos_offset_ned_m.xy() += delta_pos_estimate_ne_m;
            _vel_offset_ned_ms.xy() += delta_vel_estimate_ne_ms;
            break;
        }
        _ekf_ne_reset_ms = reset_ms;
    }
}

// Initializes tracking of vertical (U) EKF resets.
void AC_PosControl::D_init_ekf_reset()
{
    float alt_shift_d_m;
    _ekf_d_reset_ms = _ahrs.getLastPosDownReset(alt_shift_d_m);
}

// Handles U EKF reset detection and response.
void AC_PosControl::D_handle_ekf_reset()
{
    // Check for EKF-reported Down-axis shift since last update
    float pos_shift_d_m;
    uint32_t reset_ms = _ahrs.getLastPosDownReset(pos_shift_d_m);
    // todo: the actual difference in position and velocity estimation.
    // This will prevent the need to pause error calculation for one cycle.

    if (reset_ms != 0 && reset_ms != _ekf_d_reset_ms) {
        // This ensures controller output remains continuous after EKF realigns the origin.
        // Reconstruct position target relative to the to new EKF estimation to maintain the current position error
        postype_t delta_pos_estimate_d_m = _p_pos_d_m.get_error() - (_pos_target_ned_m.z - _pos_estimate_ned_m.z);
        _pos_target_ned_m.z += delta_pos_estimate_d_m;

        // Reconstruct velocity target relative to the to new EKF estimation to maintain the current velocity error
        float delta_vel_estimate_d_ms = _pid_vel_d_m.get_error() - (_vel_target_ned_ms.z - _vel_estimate_ned_ms.z);
        _vel_target_ned_ms.z += delta_vel_estimate_d_ms;

        switch (_ekf_reset_method) {
        case EKFResetMethod::MoveTarget:
            // Reset U controller desired position and velocity to preserve actual position control during Loiter, PosHold, etc.
            _pos_desired_ned_m.z += delta_pos_estimate_d_m;
            _vel_desired_ned_ms.z += delta_vel_estimate_d_ms;
            break;
        case EKFResetMethod::MoveVehicle:
            // Move the change in estimate into the offsest to move the aircraft to our new estimate smoothly during Auto, Guided, etc.
            _pos_offset_ned_m.z += delta_pos_estimate_d_m;
            _vel_offset_ned_ms.z += delta_vel_estimate_d_ms;
            break;
        }
        _ekf_d_reset_ms = reset_ms;
    }
}

// Performs pre-arm checks for position control parameters and EKF readiness.
// Returns false if failure_msg is populated.
bool AC_PosControl::pre_arm_checks(const char *param_prefix,
                                   char *failure_msg,
                                   const uint8_t failure_msg_len)
{
    if (!is_positive(NE_get_pos_p().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_NE_POS_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(D_get_pos_p().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_D_POS_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(D_get_vel_pid().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_D_VEL_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(D_get_accel_pid().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_D_ACC_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(D_get_accel_pid().kI())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_D_ACC_I must be > 0", param_prefix);
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

// perform any required parameter conversions
void AC_PosControl::convert_parameters()
{
    // find param table key
    uint16_t k_param_psc_key;
    if (!AP_Param::find_key_by_pointer(this, k_param_psc_key)) {
        return;
    }

    // return immediately if parameter conversion has already been performed
    if (_pid_accel_d_m.kP().configured()) {
        return;
    }

    // PARAMETER_CONVERSION - Added: Nov-2024 for 4.7
    // parameters that are simply moved
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    static const AP_Param::ConversionInfo conversion_info[] = {
        { k_param_psc_key, 258257, AP_PARAM_FLOAT, "Q_P_D_VEL_P" },   // Q_P_VELZ_P moved to Q_P_D_VEL_P
        { k_param_psc_key, 4305, AP_PARAM_FLOAT, "Q_P_D_VEL_I" },     // Q_P_VELZ_I moved to Q_P_D_VEL_I
        { k_param_psc_key, 16593, AP_PARAM_FLOAT, "Q_P_D_VEL_D" },    // Q_P_VELZ_D moved to Q_P_D_VEL_D
        { k_param_psc_key, 12497, AP_PARAM_FLOAT, "Q_P_D_VEL_FLTE" }, // Q_P_VELZ_FLTE moved to Q_P_D_VEL_FLTE
        { k_param_psc_key, 20689, AP_PARAM_FLOAT, "Q_P_D_VEL_FLTD" }, // Q_P_VELZ_FLTD moved to Q_P_D_VEL_FLTD
        { k_param_psc_key, 24785, AP_PARAM_FLOAT, "Q_P_D_VEL_FF" },   // Q_P_VELZ_FF moved to Q_P_D_VEL_FF
        { k_param_psc_key, 16657, AP_PARAM_FLOAT, "Q_P_D_ACC_FF" },   // Q_P_ACCZ_FF moved to Q_P_D_ACC_FF
        { k_param_psc_key, 37137, AP_PARAM_FLOAT, "Q_P_D_ACC_FLTT" }, // Q_P_ACCZ_FLTT moved to Q_P_D_ACC_FLTT
        { k_param_psc_key, 41233, AP_PARAM_FLOAT, "Q_P_D_ACC_FLTE" }, // Q_P_ACCZ_FLTE moved to Q_P_D_ACC_FLTE
        { k_param_psc_key, 45329, AP_PARAM_FLOAT, "Q_P_D_ACC_FLTD" }, // Q_P_ACCZ_FLTD moved to Q_P_D_ACC_FLTD
        { k_param_psc_key, 49425, AP_PARAM_FLOAT, "Q_P_D_ACC_SMAX" }, // Q_P_ACCZ_SMAX moved to Q_P_D_ACC_SMAX
        { k_param_psc_key, 53521, AP_PARAM_FLOAT, "Q_P_D_ACC_PDMX" }, // Q_P_ACCZ_PDMX moved to Q_P_D_ACC_PDMX
        { k_param_psc_key, 57617, AP_PARAM_FLOAT, "Q_P_D_ACC_D_FF" }, // Q_P_ACCZ_D_FF moved to Q_P_D_ACC_D_FF
        { k_param_psc_key, 65809, AP_PARAM_INT8, "Q_P_D_ACC_NEF" },   // Q_P_ACCZ_NEF moved to Q_P_D_ACC_NEF
        { k_param_psc_key, 61713, AP_PARAM_INT8, "Q_P_D_ACC_NTF" },   // Q_P_ACCZ_NTF moved to Q_P_D_ACC_NTF
        { k_param_psc_key, 258449, AP_PARAM_FLOAT, "Q_P_NE_VEL_P" },  // Q_P_VELXY_P moved to Q_P_NE_VEL_P
        { k_param_psc_key, 4497, AP_PARAM_FLOAT, "Q_P_NE_VEL_I" },    // Q_P_VELXY_I moved to Q_P_NE_VEL_I
        { k_param_psc_key, 16785, AP_PARAM_FLOAT, "Q_P_NE_VEL_D" },   // Q_P_VELXY_D moved to Q_P_NE_VEL_D
        { k_param_psc_key, 12689, AP_PARAM_FLOAT, "Q_P_NE_VEL_FLTE" },// Q_P_VELXY_FLTE moved to Q_P_NE_VEL_FLTE
        { k_param_psc_key, 20881, AP_PARAM_FLOAT, "Q_P_NE_VEL_FLTD" },// Q_P_VELXY_FLTD moved to Q_P_NE_VEL_FLTD
        { k_param_psc_key, 24977, AP_PARAM_FLOAT, "Q_P_NE_VEL_FF" },  // Q_P_VELXY_FF moved to Q_P_NE_VEL_FF
    };
#else
    static const AP_Param::ConversionInfo conversion_info[] = {
        { k_param_psc_key, 4035, AP_PARAM_FLOAT, "PSC_D_VEL_P" },   // PSC_VELZ_P moved to PSC_D_VEL_P
        { k_param_psc_key, 67, AP_PARAM_FLOAT, "PSC_D_VEL_I" },     // PSC_VELZ_I moved to PSC_D_VEL_I
        { k_param_psc_key, 259, AP_PARAM_FLOAT, "PSC_D_VEL_D" },    // PSC_VELZ_D moved to PSC_D_VEL_D
        { k_param_psc_key, 195, AP_PARAM_FLOAT, "PSC_D_VEL_FLTE" }, // PSC_VELZ_FLTE moved to PSC_D_VEL_FLTE
        { k_param_psc_key, 323, AP_PARAM_FLOAT, "PSC_D_VEL_FLTD" }, // PSC_VELZ_FLTD moved to PSC_D_VEL_FLTD
        { k_param_psc_key, 387, AP_PARAM_FLOAT, "PSC_D_VEL_FF" },   // PSC_VELZ_FF moved to PSC_D_VEL_FF
        { k_param_psc_key, 260, AP_PARAM_FLOAT, "PSC_D_ACC_FF" },   // PSC_ACCZ_FF moved to PSC_D_ACC_FF
        { k_param_psc_key, 580, AP_PARAM_FLOAT, "PSC_D_ACC_FLTT" }, // PSC_ACCZ_FLTT moved to PSC_D_ACC_FLTT
        { k_param_psc_key, 644, AP_PARAM_FLOAT, "PSC_D_ACC_FLTE" }, // PSC_ACCZ_FLTE moved to PSC_D_ACC_FLTE
        { k_param_psc_key, 708, AP_PARAM_FLOAT, "PSC_D_ACC_FLTD" }, // PSC_ACCZ_FLTD moved to PSC_D_ACC_FLTD
        { k_param_psc_key, 772, AP_PARAM_FLOAT, "PSC_D_ACC_SMAX" }, // PSC_ACCZ_SMAX moved to PSC_D_ACC_SMAX
        { k_param_psc_key, 836, AP_PARAM_FLOAT, "PSC_D_ACC_PDMX" }, // PSC_ACCZ_PDMX moved to PSC_D_ACC_PDMX
        { k_param_psc_key, 900, AP_PARAM_FLOAT, "PSC_D_ACC_D_FF" }, // PSC_ACCZ_D_FF moved to PSC_D_ACC_D_FF
        { k_param_psc_key, 1028, AP_PARAM_INT8, "PSC_D_ACC_NEF" },  // PSC_ACCZ_NEF moved to PSC_D_ACC_NEF
        { k_param_psc_key, 964, AP_PARAM_INT8, "PSC_D_ACC_NTF" },   // PSC_ACCZ_NTF moved to PSC_D_ACC_NTF
        { k_param_psc_key, 4038, AP_PARAM_FLOAT, "PSC_NE_VEL_P" },  // PSC_VELXY_P moved to PSC_NE_VEL_P
        { k_param_psc_key, 70, AP_PARAM_FLOAT, "PSC_NE_VEL_I" },    // PSC_VELXY_I moved to PSC_NE_VEL_I
        { k_param_psc_key, 262, AP_PARAM_FLOAT, "PSC_NE_VEL_D" },   // PSC_VELXY_D moved to PSC_NE_VEL_D
        { k_param_psc_key, 198, AP_PARAM_FLOAT, "PSC_NE_VEL_FLTE" },// PSC_VELXY_FLTE moved to PSC_NE_VEL_FLTE
        { k_param_psc_key, 326, AP_PARAM_FLOAT, "PSC_NE_VEL_FLTD" },// PSC_VELXY_FLTD moved to PSC_NE_VEL_FLTD
        { k_param_psc_key, 390, AP_PARAM_FLOAT, "PSC_NE_VEL_FF" },  // PSC_VELXY_FF moved to PSC_NE_VEL_FF
    };
#endif
    AP_Param::convert_old_parameters(conversion_info, ARRAY_SIZE(conversion_info));

    // parameters moved and scaled by 0.1
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    static const AP_Param::ConversionInfo conversion_info_01[] = {
        { k_param_psc_key, 258321, AP_PARAM_FLOAT, "Q_P_D_ACC_P" },   // Q_P_ACCZ_P moved to Q_P_D_ACC_P
        { k_param_psc_key, 4369, AP_PARAM_FLOAT, "Q_P_D_ACC_I" },     // Q_P_ACCZ_I moved to Q_P_D_ACC_I
        { k_param_psc_key, 8465, AP_PARAM_FLOAT, "Q_P_D_ACC_D" },     // Q_P_ACCZ_D moved to Q_P_D_ACC_D
    };
#else
    static const AP_Param::ConversionInfo conversion_info_01[] = {
        { k_param_psc_key, 4036, AP_PARAM_FLOAT, "PSC_D_ACC_P" },   // PSC_ACCZ_P moved to PSC_D_ACC_P
        { k_param_psc_key, 68, AP_PARAM_FLOAT, "PSC_D_ACC_I" },     // PSC_ACCZ_I moved to PSC_D_ACC_I
        { k_param_psc_key, 132, AP_PARAM_FLOAT, "PSC_D_ACC_D" },    // PSC_ACCZ_D moved to PSC_D_ACC_D
    };
#endif
    AP_Param::convert_old_parameters_scaled(conversion_info_01, ARRAY_SIZE(conversion_info_01), 0.1, 0);

    // store PSC_D_ACC_P as flag that parameter conversion was completed
    _pid_accel_d_m.kP().save(true);

    // parameters moved and scaled by 0.01
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    static const AP_Param::ConversionInfo conversion_info_001[] = {
        { k_param_psc_key, 8401, AP_PARAM_FLOAT, "Q_P_D_VEL_IMAX" },   // Q_P_VELZ_IMAX moved to Q_P_D_VEL_IMAX
        { k_param_psc_key, 8593, AP_PARAM_FLOAT, "Q_P_NE_VEL_IMAX" },   // Q_P_VELXY_IMAX moved to Q_P_NE_VEL_IMAX
    };
#else
    static const AP_Param::ConversionInfo conversion_info_001[] = {
        { k_param_psc_key, 131, AP_PARAM_FLOAT, "PSC_D_VEL_IMAX" },     // PSC_VELZ_IMAX moved to PSC_D_VEL_IMAX
        { k_param_psc_key, 134, AP_PARAM_FLOAT, "PSC_NE_VEL_IMAX" },    // PSC_VELXY_IMAX moved to PSC_NE_VEL_IMAX
    };
#endif
    AP_Param::convert_old_parameters_scaled(conversion_info_001, ARRAY_SIZE(conversion_info_001), 0.01, 0);

    // parameters moved and scaled by 0.001
    // PSC_ACCZ_IMAX replaced by PSC_D_ACC_IMAX
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    static const AP_Param::ConversionInfo psc_d_acc_imax_info = { k_param_psc_key, 20753, AP_PARAM_FLOAT, "Q_P_D_ACC_IMAX" };
#else
    static const AP_Param::ConversionInfo psc_d_acc_imax_info = { k_param_psc_key, 324, AP_PARAM_FLOAT, "PSC_D_ACC_IMAX" };
#endif
    AP_Param::convert_old_parameter(&psc_d_acc_imax_info, 0.001f);
}
