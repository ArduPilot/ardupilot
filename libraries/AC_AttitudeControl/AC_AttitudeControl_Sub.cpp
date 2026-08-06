#include "AC_AttitudeControl_Sub.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Sub::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Corrects in proportion to the difference between the desired roll rate vs actual roll rate
    // @Range: 0.0 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.0 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller input frequency in Hz
    // @Description: Roll axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller input frequency in Hz
    // @Description: Roll axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller input frequency in Hz
    // @Description: Roll axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_RLL_PDMX
    // @DisplayName: Roll axis rate controller PD sum maximum
    // @Description: Roll axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_RLL_D_FF
    // @DisplayName: Roll Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_RLL_NTF
    // @DisplayName: Roll Target notch filter index
    // @Description: Roll Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_RLL_NEF
    // @DisplayName: Roll Error notch filter index
    // @Description: Roll Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Sub, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Corrects in proportion to the difference between the desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller input frequency in Hz
    // @Description: Pitch axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller input frequency in Hz
    // @Description: Pitch axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller input frequency in Hz
    // @Description: Pitch axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_PIT_PDMX
    // @DisplayName: Pitch axis rate controller PD sum maximum
    // @Description: Pitch axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_PIT_D_FF
    // @DisplayName: Pitch Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_PIT_NTF
    // @DisplayName: Pitch Target notch filter index
    // @Description: Pitch Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_PIT_NEF
    // @DisplayName: Pitch Error notch filter index
    // @Description: Pitch Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Sub, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Corrects in proportion to the difference between the desired yaw rate vs actual yaw rate
    // @Range: 0.0 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.0 0.05
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller input frequency in Hz
    // @Description: Yaw axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller input frequency in Hz
    // @Description: Yaw axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller input frequency in Hz
    // @Description: Yaw axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_YAW_PDMX
    // @DisplayName: Yaw axis rate controller PD sum maximum
    // @Description: Yaw axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_YAW_D_FF
    // @DisplayName: Yaw Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_YAW_NTF
    // @DisplayName: Yaw Target notch filter index
    // @Description: Yaw Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_YAW_NEF
    // @DisplayName: Yaw Error notch filter index
    // @Description: Yaw Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Sub, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Sub, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Sub, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Sub, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    // @Param: RAT_RLL_FILT
    // @DisplayName: Roll axis rate controller input frequency in Hz
    // @Description: Roll axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FILT
    // @DisplayName: Pitch axis rate controller input frequency in Hz
    // @Description: Pitch axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FILT
    // @DisplayName: Yaw axis rate controller input frequency in Hz
    // @Description: Yaw axis rate controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    AP_GROUPEND
};

AC_AttitudeControl_Sub::AC_AttitudeControl_Sub(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors) :
    AC_AttitudeControl(ahrs, aparm, motors),
    _motors_multi(motors),
    _pid_rate_roll(AC_ATC_SUB_RATE_RP_P, AC_ATC_SUB_RATE_RP_I, AC_ATC_SUB_RATE_RP_D, 0.0f, AC_ATC_SUB_RATE_RP_IMAX, AC_ATC_SUB_RATE_RP_FILT_HZ, 0.0f, AC_ATC_SUB_RATE_RP_FILT_HZ),
    _pid_rate_pitch(AC_ATC_SUB_RATE_RP_P, AC_ATC_SUB_RATE_RP_I, AC_ATC_SUB_RATE_RP_D, 0.0f, AC_ATC_SUB_RATE_RP_IMAX, AC_ATC_SUB_RATE_RP_FILT_HZ, 0.0f, AC_ATC_SUB_RATE_RP_FILT_HZ),
	_pid_rate_yaw(AC_ATC_SUB_RATE_YAW_P, AC_ATC_SUB_RATE_YAW_I, AC_ATC_SUB_RATE_YAW_D, 0.0f, AC_ATC_SUB_RATE_YAW_IMAX, AC_ATC_SUB_RATE_YAW_FILT_HZ, 0.0f, AC_ATC_SUB_RATE_YAW_FILT_HZ)
{
    AP_Param::setup_object_defaults(this, var_info);

    // Sub-specific defaults for parent class
    _p_angle_roll.kP().set_default(AC_ATC_SUB_ANGLE_P);
    _p_angle_pitch.kP().set_default(AC_ATC_SUB_ANGLE_P);
    _p_angle_yaw.kP().set_default(AC_ATC_SUB_ANGLE_P);

    _accel_yaw_max.set_default(AC_ATC_SUB_ACCEL_Y_MAX);
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Sub::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acosf(constrain_float(throttle_in/(AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt/(_dt+_angle_limit_tc))*(althold_lean_angle_max-_althold_lean_angle_max);
}

void AC_AttitudeControl_Sub::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Sub::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(2.0f*cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f/constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = throttle_in*inverted_factor*boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in,-1.0f,1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Sub::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in*MAX(0.0f,1.0f-_throttle_rpy_mix)+_motors.get_throttle_hover()*_throttle_rpy_mix);
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Sub::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f*_dt, _throttle_rpy_mix_desired-_throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f*_dt, _throttle_rpy_mix-_throttle_rpy_mix_desired);
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}



// void AC_AttitudeControl_Sub::rate_controller_run()
// {
//     update_throttle_rpy_mix();

//     Vector3f gyro_latest = _ahrs.get_gyro_latest();
//     Vector3f target_rate = _ang_vel_body;
//     Vector3f error = target_rate - gyro_latest;

//     // --- PARAMETER FISIKA BOXFISH (Matriks Total: Rigid + Added Mass) ---
//     const float Ixx = 0.4527f;
//     const float Iyy = 0.9375f;
//     const float Izz = 0.8761f;
//     const float D_roll = 1.36f;   // Koefisien quadratic drag K_p|p|
//     // ------------------------------------------------------------------

//     // =========================================================
//     // INTEGRAL SLIDING MODE CONTROL (SUMBU ROLL)
//     // =========================================================
    
//     // 1. Ekstraksi Tuning dari UI
//     float c_roll       = _pid_rate_roll.kP();     // Slope Integral (c)
//     float rho_roll     = _pid_rate_roll.kD();     // Switching Gain (rho)
//     float phi_roll     = _pid_rate_roll.imax() + 1e-6f; // Boundary layer (epsilon)
//     // float lambda_roll  = _pid_rate_roll.kI();     // Opsional: Gain tambahan

// // 2. Filter Target Akselerasi (p_ref_dot)
//     // Menggunakan target_rate (bukan gyro aktual) sehingga bersih dari noise
//     float p_ref_dot = 0.0f;
//     if (is_positive(_dt)) {
//         // Panggil struct info langsung dari objek PID untuk mendapat target sebelumnya
//         p_ref_dot = (target_rate.x - _pid_rate_roll.get_pid_info().target) / _dt;
        
//         // Gunakan filter LPF bawaan ArduSub untuk target
//         p_ref_dot = _pid_rate_roll.get_filt_T_alpha(_dt) * p_ref_dot; 
//     }
//     // 3. Kalkulasi Integral Error dengan Anti-Windup
//     _smc_int_roll += error.x * _dt;
//     // Anti-windup mekanis (opsional, disesuaikan dengan limit aktuator)
//     _smc_int_roll = constrain_float(_smc_int_roll, -1.0f, 1.0f);

//     // 4. Hitung Integral Sliding Surface: s = e + c * int(e)
//     float s_roll = error.x + (c_roll * _smc_int_roll);

//     // 5. Hitung Equivalent Control (u_eq) dengan dinamika Boxfish
//     // u_eq = Ixx*(p_ref_dot + c*e) + (Izz - Iyy)*q*r + Dp*p*|p|
//     float coriolis_roll = (Izz - Iyy) * gyro_latest.y * gyro_latest.z;
//     float drag_roll = D_roll * gyro_latest.x * fabsf(gyro_latest.x);
    
//     float u_eq_roll = Ixx * (p_ref_dot + (c_roll * error.x)) + coriolis_roll + drag_roll;

//     // 6. Hitung Robust Control (v)
//     float v_roll = rho_roll * (s_roll / (fabsf(s_roll) + phi_roll));

//     // 7. Sinyal Kendali Total
//     float roll_out = u_eq_roll + v_roll;

//     // Batasi output total (Saturasi Aktuator) untuk mencegah overflow ke mixer
//     roll_out = constrain_float(roll_out, -1.0f, 1.0f);

//     // =========================================================
//     // INJEKSI DATA KE LOG UNTUK ANALISIS MAVEXPLORER
//     // =========================================================
//     AP_PIDInfo& roll_info = const_cast<AP_PIDInfo&>(_pid_rate_roll.get_pid_info());
//     roll_info.target = target_rate.x;
//     roll_info.actual = gyro_latest.x;
//     roll_info.error  = error.x;
//     roll_info.P      = u_eq_roll;   // Terbaca sebagai u_eq
//     roll_info.D      = v_roll;      // Terbaca sebagai v (switching)
//     roll_info.I      = s_roll;      // Terbaca sebagai sliding surface (s)
    
//     float pitch_out = _pid_rate_pitch.update_all(_ang_vel_body.y, gyro_latest.y, _dt, _motors.limit.pitch);
//     float yaw_out   = _pid_rate_yaw.update_all(_ang_vel_body.z, gyro_latest.z, _dt, _motors.limit.yaw);

//     // =========================================================
//     // MENGIRIM SINYAL FINAL KE MIXER MOTOR
//     // =========================================================
//     _motors.set_roll(roll_out);     // <--- Menggunakan SMC hasil racikan lu
//     _motors.set_pitch(pitch_out);   // <--- Menggunakan PID ArduSub
//     _motors.set_yaw(yaw_out);       // <--- Menggunakan PID ArduSub

//     control_monitor_update();
// }

// void AC_AttitudeControl_Sub::rate_controller_run()
// {
//     update_throttle_rpy_mix();
//     Vector3f gyro_latest = _ahrs.get_gyro_latest();
//     Vector3f target_rate = _ang_vel_body;
//     Vector3f error = target_rate - gyro_latest;

//     // 1. PARAMETER FISIKA UDIN (Rigid + Added Mass)
//     const float Ixx = 0.492558f;  // Rigid + Added Mass (0.408)
//     const float Iyy = 0.758506f;  // Rigid + Added Mass (0.863)
//     const float Izz = 0.919455f;  // Rigid + Added Mass (0.725)

//     const float D_roll  = 1.36f; 
//     const float D_pitch = 2.13f; 
//     const float D_yaw   = 0.62f;

//     const float MAX_T_RLL = 15.0f; // Normalisasi Torsi ke PWM
//     const float MAX_T_PIT = 15.0f;
//     const float MAX_T_YAW = 10.0f;

//     // 2. EKSTRAKSI PARAMETER (MAPPING: P=c0, I=alpha, D=lb, IMAX=epsilon)
//     // ROLL
//     float c0_r   = _pid_rate_roll.kP();
//     float alph_r = _pid_rate_roll.kI();
//     float lb_r   = _pid_rate_roll.kD();
//     float eps_r  = _pid_rate_roll.imax() + 1e-6f;

//     // PITCH
//     float c0_p   = _pid_rate_pitch.kP();
//     float alph_p = _pid_rate_pitch.kI();
//     float lb_p   = _pid_rate_pitch.kD();
//     float eps_p  = _pid_rate_pitch.imax() + 1e-6f;

//     // YAW
//     float c0_y   = _pid_rate_yaw.kP();
//     float alph_y = _pid_rate_yaw.kI();
//     float lb_y   = _pid_rate_yaw.kD();
//     float eps_y  = _pid_rate_yaw.imax() + 1e-6f;

//     // 3. TURUNAN TARGET & INTEGRAL ERROR
//     float p_ref_dot = 0.0f, q_ref_dot = 0.0f, r_ref_dot = 0.0f;
//     if (is_positive(_dt)) {
//         p_ref_dot = _pid_rate_roll.get_filt_T_alpha(_dt) * ((target_rate.x - _pid_rate_roll.get_pid_info().target) / _dt);
//         q_ref_dot = _pid_rate_pitch.get_filt_T_alpha(_dt) * ((target_rate.y - _pid_rate_pitch.get_pid_info().target) / _dt);
//         r_ref_dot = _pid_rate_yaw.get_filt_T_alpha(_dt) * ((target_rate.z - _pid_rate_yaw.get_pid_info().target) / _dt);
//     }

//     _smc_int_roll  = constrain_float(_smc_int_roll  + error.x * _dt, -1.0f, 1.0f);
//     _smc_int_pitch = constrain_float(_smc_int_pitch + error.y * _dt, -1.0f, 1.0f);
//     _smc_int_yaw   = constrain_float(_smc_int_yaw   + error.z * _dt, -1.0f, 1.0f);

//     // 4. SLIDING SURFACE (sigma) & SWITCHING GAIN (rho)
//     float sig_r = error.x + (c0_r * _smc_int_roll);
//     float sig_p = error.y + (c0_p * _smc_int_pitch);
//     float sig_y = error.z + (c0_y * _smc_int_yaw);

//     float rho_r = (alph_r / 1.4142f) + lb_r;
//     float rho_p = (alph_p / 1.4142f) + lb_p;
//     float rho_y = (alph_y / 1.4142f) + lb_y;

//     // 5. EQUIVALENT CONTROL (u_eq) - FISIKA & CORIOLIS
//     float p = gyro_latest.x, q = gyro_latest.y, r = gyro_latest.z;

//     float u_eq_r = Ixx * (p_ref_dot + (c0_r * error.x)) + (Izz - Iyy)*q*r + D_roll*p*fabsf(p);
//     float u_eq_p = Iyy * (q_ref_dot + (c0_p * error.y)) + (Ixx - Izz)*p*r + D_pitch*q*fabsf(q);
//     float u_eq_y = Izz * (r_ref_dot + (c0_y * error.z)) + (Iyy - Ixx)*p*q + D_yaw*r*fabsf(r);

//     // 6. SATURATION FUNCTION
//     float v_r = -rho_r * (sig_r / (fabsf(sig_r) + eps_r));
//     float v_p = -rho_p * (sig_p / (fabsf(sig_p) + eps_p));
//     float v_y = -rho_y * (sig_y / (fabsf(sig_y) + eps_y));

//     // 7. OUTPUT NORMALIZATION & LOGGING
//     float r_out = constrain_float((u_eq_r - v_r) / MAX_T_RLL, -1.0f, 1.0f);
//     float p_out = constrain_float((u_eq_p - v_p) / MAX_T_PIT, -1.0f, 1.0f);
//     float y_out = constrain_float((u_eq_y - v_y) / MAX_T_YAW, -1.0f, 1.0f);

//     _pid_rate_roll.set_pid_info_custom(target_rate.x, p, error.x, u_eq_r/MAX_T_RLL, -v_r/MAX_T_RLL, sig_r);
//     _pid_rate_pitch.set_pid_info_custom(target_rate.y, q, error.y, u_eq_p/MAX_T_PIT, -v_p/MAX_T_PIT, sig_p);
//     _pid_rate_yaw.set_pid_info_custom(target_rate.z, r, error.z, u_eq_y/MAX_T_YAW, -v_y/MAX_T_YAW, sig_y);

//     _motors.set_roll(r_out);
//     _motors.set_pitch(p_out);
//     _motors.set_yaw(y_out);

//     control_monitor_update();
// }

// void AC_AttitudeControl_Sub::rate_controller_run()
// {
//     // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
//     update_throttle_rpy_mix();

//     Vector3f gyro_latest = _ahrs.get_gyro_latest();
//     _motors.set_pitch(get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y, _dt, _motors.limit.pitch));
//     _motors.set_yaw(get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z, _dt, _motors.limit.yaw));

//     control_monitor_update();
// }

// run lowest level body-frame rate controller and send outputs to the motors
// (HIJACKED FOR SLIDING MODE CONTROL)
void AC_AttitudeControl_Sub::rate_controller_run(){
    // Update throttle mix bawaan (Slew Rate Limit)
    update_throttle_rpy_mix();
    
    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    // Vector3f target_rate = _ang_vel_body;
    // Vector3f error = target_rate - gyro_latest;
    Vector3f target_rate;
    if (is_positive(_dt)) {
        float alpha_r = _pid_rate_roll.get_filt_T_alpha(_dt);
        float alpha_p = _pid_rate_pitch.get_filt_T_alpha(_dt);
        float alpha_y = _pid_rate_yaw.get_filt_T_alpha(_dt);

        _smc_target_roll  += alpha_r * (_ang_vel_body.x - _smc_target_roll);
        _smc_target_pitch += alpha_p * (_ang_vel_body.y - _smc_target_pitch);
        _smc_target_yaw   += alpha_y * (_ang_vel_body.z - _smc_target_yaw);
    }
    target_rate.x = _smc_target_roll;
    target_rate.y = _smc_target_pitch;
    target_rate.z = _smc_target_yaw;

    if (is_positive(_dt)) {
        float alpha_E_r = _pid_rate_roll.get_filt_E_alpha(_dt);
        float alpha_E_p = _pid_rate_pitch.get_filt_E_alpha(_dt);
        float alpha_E_y = _pid_rate_yaw.get_filt_E_alpha(_dt);
        
        float raw_err_r = target_rate.x - gyro_latest.x;
        float raw_err_p = target_rate.y - gyro_latest.y;
        float raw_err_y = target_rate.z - gyro_latest.z;
        
        _smc_error_roll  += alpha_E_r * (raw_err_r - _smc_error_roll);
        _smc_error_pitch += alpha_E_p * (raw_err_p - _smc_error_pitch);
        _smc_error_yaw   += alpha_E_y * (raw_err_y - _smc_error_yaw);
    }
    
    Vector3f error;
    error.x = _smc_error_roll;
    error.y = _smc_error_pitch;
    error.z = _smc_error_yaw;

    // 1. PARAMETER FISIKA ROV (Rigid Body + Added Mass)
    const float Ixx = 0.492558f + 0.16f; 
    const float Iyy = 0.758506f + 0.3f; 
    const float Izz = 0.919455f + 0.3f;
    // const float Ixy = 0.002052f;
    // const float Ixz = 0.019287f;
    // const float Iyz = 0.000148f;


    // Linear Damping
    const float D_lin_roll  = 0.0f; 
    const float D_lin_pitch = 0.0f; 
    const float D_lin_yaw   = 0.8f;

    // Quadratic Damping
    const float D_roll  = 0.4f; 
    const float D_pitch = 1.19f; 
    const float D_yaw   = 0.482f;
    // // Linear Damping
    // const float D_lin_roll  = 0.6f; 
    // const float D_lin_pitch = 1.0f; 
    // const float D_lin_yaw   = 0.4f;

    // // Quadratic Damping
    // const float D_roll  = 8.0f; 
    // const float D_pitch = 8.0f; 
    // const float D_yaw   = 8.0f;

    const float MAX_T_RLL = 10.0f; // Normalisasi Torsi -> PWM
    const float MAX_T_PIT = 10.0;
    const float MAX_T_YAW = 8.0;
    // const float D_roll  = 1.36f; 
    // const float D_pitch = 2.13f; 
    // const float D_yaw   = 1.0f;



    // 2. MAPPING PARAMETER DARI QGROUNDCONTROL (Bajak Sistem PID)
    // ROLL (P = Lambda, I = K Gain, D = Eta/Boundary, IMAX = Epsilon)
    float c0_r   = _pid_rate_roll.kP();
    float alph_r = _pid_rate_roll.kI();
    float lb_r   = _pid_rate_roll.kD();
    float eps_r  = _pid_rate_roll.imax() + 1e-6f; // Cegah divide-by-zero

    // PITCH
    float c0_p   = _pid_rate_pitch.kP();
    float alph_p = _pid_rate_pitch.kI();
    float lb_p   = _pid_rate_pitch.kD();
    float eps_p  = _pid_rate_pitch.imax() + 1e-6f;

    // YAW
    float c0_y   = _pid_rate_yaw.kP();
    float alph_y = _pid_rate_yaw.kI();
    float lb_y   = _pid_rate_yaw.kD();
    float eps_y  = _pid_rate_yaw.imax() + 1e-6f;

    // 3. KALKULASI TURUNAN (LPF) & INTEGRASI (ANTI-WINDUP)
    // if (is_positive(_dt)) {
    //     // Ambil koefisien Low-Pass Filter dari settingan QGC (FLTT)
    //     float alpha_T_r = _pid_rate_roll.get_filt_T_alpha(_dt);
    //     float alpha_T_p = _pid_rate_pitch.get_filt_T_alpha(_dt);
    //     float alpha_T_y = _pid_rate_yaw.get_filt_T_alpha(_dt);

    //     // Hitung Raw Derivative (Target Rate Dot)
    //     float raw_dot_r = (target_rate.x - _pid_rate_roll.get_pid_info().target) / _dt;
    //     float raw_dot_p = (target_rate.y - _pid_rate_pitch.get_pid_info().target) / _dt;
    //     float raw_dot_y = (target_rate.z - _pid_rate_yaw.get_pid_info().target) / _dt;

    //     // Terapkan IIR Low-Pass Filter Orde 1
    //     _smc_pref_dot_roll  += alpha_T_r * (raw_dot_r - _smc_pref_dot_roll);
    //     _smc_pref_dot_pitch += alpha_T_p * (raw_dot_p - _smc_pref_dot_pitch);
    //     _smc_pref_dot_yaw   += alpha_T_y * (raw_dot_y - _smc_pref_dot_yaw);
    // }

    if (is_positive(_dt)) {
        float alpha_D_r = _pid_rate_roll.get_filt_D_alpha(_dt);
        float alpha_D_p = _pid_rate_pitch.get_filt_D_alpha(_dt);
        float alpha_D_y = _pid_rate_yaw.get_filt_D_alpha(_dt);

        // Gunakan state sendiri, bukan dari PID internal (Finite difference)
        float raw_dot_r = (target_rate.x - _smc_prev_target_roll)  / _dt;
        float raw_dot_p = (target_rate.y - _smc_prev_target_pitch) / _dt;
        float raw_dot_y = (target_rate.z - _smc_prev_target_yaw)   / _dt;

        // Update prev untuk cycle berikutnya
        _smc_prev_target_roll  = target_rate.x;
        _smc_prev_target_pitch = target_rate.y;
        _smc_prev_target_yaw   = target_rate.z;

        // IIR filter
        _smc_pref_dot_roll  += alpha_D_r * (raw_dot_r - _smc_pref_dot_roll);
        _smc_pref_dot_pitch += alpha_D_p * (raw_dot_p - _smc_pref_dot_pitch);
        _smc_pref_dot_yaw   += alpha_D_y * (raw_dot_y - _smc_pref_dot_yaw);
    }
    
    // Integral Anti-Windup: Hanya akumulasi error jika motor TIDAK sedang mentok (limit = false)
    if (!_motors.limit.roll)  _smc_int_roll  = constrain_float(_smc_int_roll  + error.x * _dt, -1.0f, 1.0f);
    if (!_motors.limit.pitch) _smc_int_pitch = constrain_float(_smc_int_pitch + error.y * _dt, -1.0f, 1.0f);
    if (!_motors.limit.yaw)   _smc_int_yaw   = constrain_float(_smc_int_yaw   + error.z * _dt, -1.0f, 1.0f);

    // 4. SLIDING SURFACE (sigma) & SWITCHING GAIN (rho)
    float sig_r = error.x + (c0_r * _smc_int_roll);
    float sig_p = error.y + (c0_p * _smc_int_pitch);
    float sig_y = error.z + (c0_y * _smc_int_yaw);

    // Kalkulasi Gain Total
    float rho_r = (alph_r / 1.4142f) + lb_r;
    float rho_p = (alph_p / 1.4142f) + lb_p;
    float rho_y = (alph_y / 1.4142f) + lb_y;

    // 5. EQUIVALENT CONTROL (u_eq) FULL 6-DOF COUPLED DYNAMICS
    float p = gyro_latest.x, q = gyro_latest.y, r = gyro_latest.z;

    // Hitung target akselerasi ideal dari Sliding Surface (a*)
    float a_r = _smc_pref_dot_roll  + (c0_r * error.x);
    float a_p = _smc_pref_dot_pitch + (c0_p * error.y);
    float a_y = _smc_pref_dot_yaw   + (c0_y * error.z);

    // Kalkulasi Damping Total (Linear + Quadratic)
    float damp_r = (D_lin_roll * p)  + (D_roll * p * fabsf(p));
    float damp_p = (D_lin_pitch * q) + (D_pitch * q * fabsf(q));
    float damp_y = (D_lin_yaw * r)   + (D_yaw * r * fabsf(r));

    // Kalkulasi Full u_eq (Inertia + Cross-Inertia + Coriolis + Cross-Coriolis + Damping)
    float u_eq_r = (Ixx*a_r) + (Izz - Iyy)*q*r + damp_r;
    float u_eq_p = (Iyy*a_p) + (Ixx - Izz)*p*r + damp_p;
    float u_eq_y = (Izz*a_y) + (Iyy - Ixx)*p*q + damp_y;
    // float u_eq_r = (Ixx*a_r + Ixy*a_p + Ixz*a_y) + (Izz - Iyy)*q*r + Iyz*(q*q - r*r) + Ixz*p*q - Ixy*p*r + damp_r;
    // float u_eq_p = (Ixy*a_r + Iyy*a_p + Iyz*a_y) + (Ixx - Izz)*p*r + Ixz*(r*r - p*p) + Ixy*q*r - Iyz*p*q + damp_p;
    // float u_eq_y = (Ixz*a_r + Iyz*a_p + Izz*a_y) + (Iyy - Ixx)*p*q + Ixy*(p*p - q*q) + Iyz*p*r - Ixz*q*r + damp_y;

    // 6. SWITCHING CONTROL (u_sw) -> Saturation (avoid Chattering)
    float u_sw_r = rho_r * (sig_r / (fabsf(sig_r) + eps_r));
    float u_sw_p = rho_p * (sig_p / (fabsf(sig_p) + eps_p));
    float u_sw_y = rho_y * (sig_y / (fabsf(sig_y) + eps_y));

    // 7. OUTPUT NORMALIZATION (u_eq + u_sw) -> Bounding -1 ke 1
    float r_out = constrain_float((u_eq_r + u_sw_r) / MAX_T_RLL, -1.0f, 1.0f);
    float p_out = constrain_float((u_eq_p + u_sw_p) / MAX_T_PIT, -1.0f, 1.0f);
    float y_out = constrain_float((u_eq_y + u_sw_y) / MAX_T_YAW, -1.0f, 1.0f);

    // 8. Custom logging (By-Pass Data untuk File .BIN pixhawk)
    // ==========================================================
    // Default Format: (Target, Actual, Error, Nilai P, Nilai D, Nilai I)
    // Isi dengan: (Target, Actual, Error, u_eq, u_sw, sigma)
    _pid_rate_roll.set_pid_info_custom(target_rate.x, p, error.x, u_eq_r/MAX_T_RLL, u_sw_r/MAX_T_RLL, sig_r);
    _pid_rate_pitch.set_pid_info_custom(target_rate.y, q, error.y, u_eq_p/MAX_T_PIT, u_sw_p/MAX_T_PIT, sig_p);
    _pid_rate_yaw.set_pid_info_custom(target_rate.z, r, error.z, u_eq_y/MAX_T_YAW, u_sw_y/MAX_T_YAW, sig_y);

    // 9. Kirim SINYAL KE MOTOR MIXER

    const float MAX_SLEW = 0.5f; // maksimal perubahan per cycle, tune ini
    r_out = constrain_float(r_out, _smc_out_roll_prev  - MAX_SLEW * _dt * 400,
                                    _smc_out_roll_prev  + MAX_SLEW * _dt * 400);
    p_out = constrain_float(p_out, _smc_out_pitch_prev - MAX_SLEW * _dt * 400,
                                    _smc_out_pitch_prev + MAX_SLEW * _dt * 400);
    y_out = constrain_float(y_out, _smc_out_yaw_prev   - MAX_SLEW * _dt * 400,
                                    _smc_out_yaw_prev   + MAX_SLEW * _dt * 400);

    _smc_out_roll_prev  = r_out;
    _smc_out_pitch_prev = p_out;
    _smc_out_yaw_prev   = y_out;

    _motors.set_roll(r_out);
    _motors.set_pitch(p_out);
    _motors.set_yaw(y_out);

    // Cek Failsafe
    control_monitor_update();
}

// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Sub::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > AC_ATTITUDE_CONTROL_MAN_LIMIT) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(constrain_float(_thr_mix_man, 0.1, AC_ATTITUDE_CONTROL_MAN_LIMIT));
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > AC_ATTITUDE_CONTROL_MIN_LIMIT) {
        _thr_mix_min.set_and_save(constrain_float(_thr_mix_min, 0.1, AC_ATTITUDE_CONTROL_MIN_LIMIT));
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(constrain_float(_thr_mix_max, 0.5, AC_ATTITUDE_CONTROL_MAX));
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}

// This function ensures that the ROV reaches the target orientation with the desired yaw rate
void AC_AttitudeControl_Sub::input_euler_angle_roll_pitch_slew_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, float target_yaw_rate)
{
    // Convert from centidegrees on public interface to radians
    const float euler_yaw_angle = wrap_PI(radians(euler_yaw_angle_cd * 0.01f));

    const float current_yaw = AP::ahrs().get_yaw();

    // Compute angle error
    const float yaw_error = wrap_PI(euler_yaw_angle - current_yaw);
    
    int direction = 0;
    if (yaw_error < 0){
        direction = -1;
    } else {
        direction = 1;
    }

    target_yaw_rate *= direction;


    if (fabsf(yaw_error) > MAX_YAW_ERROR) {
        // rotate the rov with desired yaw rate towards the target yaw
        input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, target_yaw_rate);
    } else {
        // holds the rov's angles
        input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_angle_cd, true);
    }
}

void AC_AttitudeControl_Sub::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    _pid_rate_roll.set_notch_sample_rate(sample_rate);
    _pid_rate_pitch.set_notch_sample_rate(sample_rate);
    _pid_rate_yaw.set_notch_sample_rate(sample_rate);
#endif
}
