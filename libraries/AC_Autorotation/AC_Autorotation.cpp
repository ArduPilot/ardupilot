#include "AC_Autorotation.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

// Head Speed (HS) controller specific default definitions
#define HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ          2.0f     // low-pass filter on accel error (unit: hz)
#define HS_CONTROLLER_HEADSPEED_P                     0.7f     // Default P gain for head speed controller (unit: -)

// Speed Height controller specific default definitions for autorotation use
#define AP_FW_VEL_P                                   0.9f    // Default forward speed controller P gain
#define TCH_P                                         0.1f    // Default touchdown phase collective controller P gain

// flare controller default definitions
#define AP_ALPHA_TPP                                  20.0f   // (deg) Maximum angle of the Tip Path Plane
#define MIN_TIME_ON_GROUND                            3000    // (ms) Time on ground required before collective is slewed to zero thrust

const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_Autorotation, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_P
    // @DisplayName: P gain for head speed controller
    // @Description: Increase value to increase sensitivity of head speed controller during autonomous autorotation.
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_p_hs, "HS_", 2, AC_Autorotation, AC_P),

    // @Param: HS_SET_PT
    // @DisplayName: Target Head Speed
    // @Description: The target head speed in RPM during autorotation.  Start by setting to desired hover speed and tune from there as necessary.
    // @Units: RPM
    // @Range: 1000 2800
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("HS_SET_PT", 3, AC_Autorotation, _param_head_speed_set_point, 1500),

    // @Param: TARG_SP
    // @DisplayName: Target Glide Ground Speed
    // @Description: Target ground speed in cm/s for the autorotation controller to try and achieve/ maintain during the glide phase.
    // @Units: cm/s
    // @Range: 800 2000
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("TARG_SP", 4, AC_Autorotation, _param_target_speed, 1100),

    // @Param: COL_FILT_E
    // @DisplayName: Entry Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the entry phase.  Acts as a following trim.  Must be higher than AROT_COL_FILT_G.
    // @Units: Hz
    // @Range: 0.2 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_E", 5, AC_Autorotation, _param_col_entry_cutoff_freq, 0.7),

    // @Param: COL_FILT_G
    // @DisplayName: Glide Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the glide phase.  Acts as a following trim.  Must be lower than AROT_COL_FILT_E.
    // @Units: Hz
    // @Range: 0.03 0.15
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_G", 6, AC_Autorotation, _param_col_glide_cutoff_freq, 0.1),

    // @Param: AS_ACC_MAX
    // @DisplayName: Forward Acceleration Limit
    // @Description: Maximum forward acceleration to apply in speed controller.
    // @Units: cm/s/s
    // @Range: 30 60
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("AS_ACC_MAX", 7, AC_Autorotation, _param_accel_max, 60),

    // @Param: BAIL_TIME
    // @DisplayName: Bail Out Timer
    // @Description: Time in seconds from bail out initiated to the exit of autorotation flight mode.
    // @Units: s
    // @Range: 0.5 4
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BAIL_TIME", 8, AC_Autorotation, _param_bail_time, 2.0),

    // @Param: HS_SENSOR
    // @DisplayName: Main Rotor RPM Sensor
    // @Description: Allocate the RPM sensor instance to use for measuring head speed.  RPM1 = 0.  RPM2 = 1.
    // @Units: s
    // @Range: 0.5 3
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("HS_SENSOR", 9, AC_Autorotation, _param_rpm_instance, 0),

    // @Param: FW_V_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Determines the proportion of the target acceleration based on the velocity error.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced
    AP_SUBGROUPINFO(_p_fw_vel, "FW_V_", 10, AC_Autorotation, AC_P),

    // @Param: FW_V_FF
    // @DisplayName: Velocity (horizontal) feed forward
    // @Description: Velocity (horizontal) input filter.  Corrects the target acceleration proportionally to the desired velocity.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("FW_V_FF", 11, AC_Autorotation, _param_fwd_k_ff, 0.15),

    // @Param: TCH_P
    // @DisplayName: P gain for vertical touchdown controller
    // @Description: proportional term based on sink rate error
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_p_coll_tch, "TCH_", 12, AC_Autorotation, AC_P),

    // @Param: COL_FILT_C
    // @DisplayName: Touchdown Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the touchdown phase.  Acts as a following trim.
    // @Units: Hz
    // @Range: 0.2 0.8
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_C", 13, AC_Autorotation, _param_col_touchdown_cutoff_freq, 0.5),

    // @Param: ROT_SOL
    // @DisplayName: rotor solidity
    // @Description: helicopter specific main rotor solidity
    // @Range: 0.001 0.01
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ROT_SOL", 14, AC_Autorotation, _param_solidity, 0.05),

    // @Param: ROT_DIAM
    // @DisplayName: rotor diameter
    // @Description: helicopter specific main rotor diameter
    // @Units: m
    // @Range: 0.001 0.01
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("ROT_DIAM", 15, AC_Autorotation, _param_diameter, 1.25),

    // @Param: T_TCH
    // @DisplayName: time touchdown
    // @Description: time touchdown
    // @Units: s
    // @Range: 0.001 0.01
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("T_TCH", 16, AC_Autorotation, _t_tch, 0.55),

    // @Param: OPTIONS
    // @DisplayName: Autorotation options
    // @Description: Bitmask for autorotation options.
    // @Bitmask: 0: Use stabilize-like controls (roll angle, yaw rate)
    AP_GROUPINFO("OPTIONS", 17, AC_Autorotation, _options, 0),

    AP_GROUPEND
};


// Constructor
AC_Autorotation::AC_Autorotation(AP_InertialNav& inav, AP_AHRS& ahrs) :
    _inav(inav),
    _ahrs(ahrs),
    _p_hs(HS_CONTROLLER_HEADSPEED_P),
    _p_fw_vel(AP_FW_VEL_P),
    _p_coll_tch(TCH_P)
{
    AP_Param::setup_object_defaults(this, var_info);
}


void AC_Autorotation::init(AP_MotorsHeli* motors, float gnd_clear) {

    _motors_heli = motors;
    if (_motors_heli == nullptr) {
        AP_HAL::panic("AROT: _motors_heli is nullptr");
    }

    // Reset z acceleration average variables
    _avg_acc_z = 0.0f;
    _acc_z_sum = 0.0f;
    _index = 0;
    memset(_curr_acc_z, 0, sizeof(_curr_acc_z));

    initial_flare_estimate();

    // Initialisation of head speed controller
    // Set initial collective position to be the zero thrust collective and minimize loss in head speed
    _collective_out = _motors_heli->get_coll_mid();

    // Reset feed forward filter
    col_trim_lpf.reset(_collective_out);

    // Protect against divide by zero
    _param_head_speed_set_point.set(MAX(_param_head_speed_set_point, 500.0));

    // Initialise forward speed controller
    _accel_target = 0.0;

    // Ensure parameter acceleration doesn't exceed hard-coded limit
    _accel_max = MIN(_param_accel_max, 500.0);

    // Reset cmd vel and last accel to sensible values
    _cmd_vel = calc_speed_forward(); //(cm/s)
    _accel_out_last = _cmd_vel * _param_fwd_k_ff;

    // Store the ground clearance on the lidar sensor for use in touch down calculations
    _ground_clearance = gnd_clear;

    // reset on ground timer
    _time_on_ground = 0;
}


void AC_Autorotation::init_entry(void)
{
    // Set following trim low pass cut off frequency
    _col_cutoff_freq = _param_col_entry_cutoff_freq.get();

    // Set desired forward speed target
    _vel_target = _param_target_speed.get();
}

void AC_Autorotation::init_glide(float hs_targ)
{
    // Set following trim low pass cut off frequency
    _col_cutoff_freq = _param_col_glide_cutoff_freq.get();

    // Ensure desired forward speed target is set to param value
    _vel_target = _param_target_speed.get();

    // Update head speed target
    _target_head_speed = hs_targ;
}

void AC_Autorotation::init_flare(float hs_targ)
{
    // Ensure following trim low pass cut off frequency
    _col_cutoff_freq = _param_col_glide_cutoff_freq.get();

    _flare_entry_speed = calc_speed_forward();

    // Update head speed target
    _target_head_speed = hs_targ;
}

void AC_Autorotation::init_touchdown(void)
{
    // Set following trim low pass cut off frequency
    _col_cutoff_freq = _param_col_touchdown_cutoff_freq.get();

    // store the descent speed and height at the start of the touch down
    _entry_sink_rate = _inav.get_velocity_z_up_cms();
    _entry_alt = _gnd_hgt;
}


// Rotor Speed controller for entry, glide and flare phases of autorotation
void AC_Autorotation::update_hs_glide_controller(void)
{
    // Get current rpm and update healthy signal counters
    _current_rpm = get_rpm();

    if (_current_rpm > 0.0) {
        // Normalised head speed
        float head_speed_norm = _current_rpm / _param_head_speed_set_point;

        // Set collective trim low pass filter cut off frequency
        col_trim_lpf.set_cutoff_frequency(_col_cutoff_freq);

        // Calculate the head speed error.  Current rpm is normalised by the set point head speed.
        // Target head speed is defined as a percentage of the set point.
        _head_speed_error = head_speed_norm - _target_head_speed;

        _p_term_hs = _p_hs.get_p(_head_speed_error);

        // Adjusting collective trim using feed forward (not yet been updated, so this value is the previous time steps collective position)
        _ff_term_hs = col_trim_lpf.apply(_collective_out, _dt);

        // Calculate collective position to be set
        _collective_out = constrain_value((_p_term_hs + _ff_term_hs), 0.0f, 1.0f) ;

    } else {
         // RPM sensor is bad, set collective to angle of -2 deg and hope for the best
         _collective_out = _motors_heli->calc_coll_from_ang(-2.0);
    }

    // Send collective to setting to motors output library
    set_collective(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);
}


void AC_Autorotation::update_hover_autorotation_controller()
{
    // Set collective trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_col_cutoff_freq);

    // Use zero thrust collective to minimize rotor speed loss
    _ff_term_hs = col_trim_lpf.apply(_motors_heli->get_coll_mid(), _dt);

    // Calculate collective position to be set
    _collective_out = constrain_value(_ff_term_hs, 0.0f, 1.0f);

    // Send collective to setting to motors output library
    set_collective(_col_cutoff_freq);
}


// Function to set collective and collective filter in motor library
void AC_Autorotation::set_collective(float collective_filter_cutoff) const
{
    _motors_heli->set_throttle_filter_cutoff(collective_filter_cutoff);
    _motors_heli->set_throttle(_collective_out);
}


// Function that gets rpm and does rpm signal checking to ensure signal is reliable
// before using it in the controller
float AC_Autorotation::get_rpm(void)
{
    float current_rpm = 0.0;

#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        // Check requested rpm instance to ensure either 0 or 1. Always defaults to 0.
        if ((_param_rpm_instance > 1) || (_param_rpm_instance < 0)) {
            _param_rpm_instance.set(0);
        }

        // Get RPM value
        uint8_t instance = _param_rpm_instance;

        // Check RPM sensor is returning a healthy status
        if (!rpm->get_rpm(instance, current_rpm)) {
            current_rpm = 0.0;
        }
    }
#endif
    return current_rpm;
}


void AC_Autorotation::initial_flare_estimate(void)
{
    // Estimate hover thrust
    float _col_hover_rad = radians(_motors_heli->get_hover_coll_ang());
    float b = _param_solidity * M_2PI;
    _disc_area = M_PI * sq(_param_diameter * 0.5f);
    float lambda = (-(b / 8.0f) + safe_sqrt((sq(b)) / 64.0f +((b / 3.0f) * _col_hover_rad))) * 0.5f;
    float freq = _motors_heli->get_rpm_setpoint() / 60.0f;
    float tip_speed= freq * M_2PI * _param_diameter * 0.5f;
    _lift_hover = ((SSL_AIR_DENSITY * sq(tip_speed) * (_param_solidity * _disc_area)) * ((_col_hover_rad / 3.0f) - (lambda / 2.0f)) * 5.8f) * 0.5f;

    // Estimate rate of descent
    float omega_auto = (_param_head_speed_set_point / 60.0f) * M_2PI;
    float tip_speed_auto = omega_auto * _param_diameter * 0.5f;
    float c_t = _lift_hover / (0.6125f * _disc_area * sq(tip_speed));
    _est_rod = ((0.25f * (_param_solidity * 0.011f / c_t) * tip_speed_auto) + ((sq(c_t) / (_param_solidity * 0.011f)) * tip_speed_auto));

    // Estimate rotor C_d
    _c = (_lift_hover / (sq(_est_rod) * 0.5f * SSL_AIR_DENSITY * _disc_area)) * 1.15f;
    _c = constrain_float(_c, 0.7f, 1.4f);

    // Calc flare altitude
    float des_spd_fwd = _param_target_speed * 0.01f;
    calc_flare_alt(-_est_rod, des_spd_fwd);

    // Initialize flare bools
    _flare_complete = false;
    _flare_update_check = false;


    gcs().send_text(MAV_SEVERITY_INFO, "Ct/sigma=%f W=%f kg flare_alt=%f C=%f", c_t/_param_solidity, _lift_hover/GRAVITY_MSS, _flare_alt_calc*0.01f, _c);
    gcs().send_text(MAV_SEVERITY_INFO, "est_rod=%f", _est_rod);
}


void AC_Autorotation::calc_flare_alt(float sink_rate, float fwd_speed)
{
    // Compute speed module and glide path angle during descent
    float speed_module = norm(sink_rate, fwd_speed);
    float glide_angle = safe_asin(M_PI / 2 - (fwd_speed / speed_module));

    // Estimate inflow velocity at beginning of flare
    float inflow = - speed_module * sinf(glide_angle + radians(AP_ALPHA_TPP));

    // Estimate flare duration
    float k_1 = fabsf((-sink_rate + 0.001f - safe_sqrt(_lift_hover / _c))/(-sink_rate + 0.001f + safe_sqrt(_lift_hover / _c)));
    float k_2 = fabsf((inflow - safe_sqrt(_lift_hover / _c)) / (inflow + safe_sqrt(_lift_hover / _c)));
    float delta_t_flare = (0.5f * (_lift_hover / (GRAVITY_MSS * _c)) * safe_sqrt(_c / _lift_hover) * logf(k_1)) - (0.5f * (_lift_hover / (GRAVITY_MSS * _c)) * safe_sqrt(_c / _lift_hover) * logf(k_2));

    // Estimate flare delta altitude
    float sq_gravity = sq(GRAVITY_MSS);
    float a = (2 * safe_sqrt(_c * sq_gravity / _lift_hover )) * delta_t_flare + (2 * safe_sqrt(_c * sq_gravity / _lift_hover )) * (0.5f * (_lift_hover / (GRAVITY_MSS * _c)) * safe_sqrt(_c / _lift_hover) * logf(k_1));
    float x = 1 - expf(a);
    float s = 1 - expf((2 * safe_sqrt(_c * sq_gravity / _lift_hover )) * (0.5f * (_lift_hover/(GRAVITY_MSS * _c)) * safe_sqrt(_c / _lift_hover) * logf(k_1)));
    float d = safe_sqrt(_lift_hover / _c);
    float flare_distance = ((2 * d / (2 * safe_sqrt(_c * sq_gravity / _lift_hover ))) * (a - logf(fabsf(x)) - (2 * safe_sqrt(_c * sq_gravity / _lift_hover )) * (0.5f * (_lift_hover / (GRAVITY_MSS * _c)) * safe_sqrt(_c / _lift_hover) * logf(k_1)) + logf(fabsf(s)))) - d * delta_t_flare;
    float delta_h = -flare_distance * cosf(radians(AP_ALPHA_TPP));

    // Estimate altitude to begin collective pull
    _cushion_alt = (-(sink_rate * cosf(radians(AP_ALPHA_TPP))) * _t_tch) * 100.0f;

    // Total delta altitude to ground
    _flare_alt_calc = _cushion_alt + delta_h * 100.0f;
}


#if HAL_LOGGING_ENABLED
void AC_Autorotation::Log_Write_Autorotation(void) const
{
    // @LoggerMessage: AROT
    // @Vehicles: Copter
    // @Description: Helicopter AutoRotation information
    // @Field: TimeUS: Time since system startup
    // @Field: hsp: P-term for headspeed controller response
    // @Field: hse: head speed error; difference between current and desired head speed
    // @Field: co: Collective Out
    // @Field: cff: FF-term for headspeed controller response
    // @Field: sf: current forward speed
    // @Field: dsf: desired forward speed
    // @Field: vp: p-term of velocity response
    // @Field: vff: ff-term of velocity response
    // @Field: az: average z acceleration
    // @Field: dvz: Desired Sink Rate
    // @Field: h: height above ground

    //Write to data flash log
    AP::logger().WriteStreaming("AROT",
                                "TimeUS,hsp,hse,co,cff,sf,dsf,vp,vff,az,dvz,h",
                                "Qfffffffffff",
                                AP_HAL::micros64(),
                                _p_term_hs,
                                _head_speed_error,
                                _collective_out,
                                _ff_term_hs,
                                (_speed_forward*0.01f),
                                (_cmd_vel*0.01f),
                                _vel_p,
                                _vel_ff,
                                _avg_acc_z,
                                _desired_sink_rate,
                                (_gnd_hgt*0.01f));
}
#endif  // HAL_LOGGING_ENABLED


// Sets time delta in seconds for all controllers
void AC_Autorotation::set_dt(float delta_sec)
{
    _dt = delta_sec;
}


// Update speed controller
void AC_Autorotation::update_forward_speed_controller(void)
{
    // Specify forward velocity component and determine delta velocity with respect to time
    _speed_forward = calc_speed_forward(); //(cm/s)

    _delta_speed_fwd = _speed_forward - _speed_forward_last; //(cm/s)
    _speed_forward_last = _speed_forward; //(cm/s)

    // Limiting the target velocity based on the max acceleration limit
    if (_cmd_vel < _vel_target) {
        _cmd_vel += _accel_max * _dt;
        if (_cmd_vel > _vel_target) {
            _cmd_vel = _vel_target;
        }
    } else {
        _cmd_vel -= _accel_max * _dt;
        if (_cmd_vel < _vel_target) {
            _cmd_vel = _vel_target;
        }
    }

    // Get p
    _vel_p = _p_fw_vel.get_p(_cmd_vel - _speed_forward);

    // Get ff
    _vel_ff = _cmd_vel * _param_fwd_k_ff;

    // Calculate acceleration target based on PI controller
    _accel_target = _vel_ff + _vel_p;

    // Filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(10.0f);
    _accel_target_filter.apply(_accel_target, _dt);

    // Limits the maximum change in pitch attitude based on acceleration
    if (_accel_target > _accel_out_last + _accel_max) {
        _accel_target = _accel_out_last + _accel_max;
    } else if (_accel_target < _accel_out_last - _accel_max) {
        _accel_target = _accel_out_last - _accel_max;
    }

    // Limiting acceleration based on velocity gained during the previous time step
    if (fabsf(_delta_speed_fwd) > _accel_max * _dt) {
        _flag_limit_accel = true;
    } else {
        _flag_limit_accel = false;
    }

    if ((_flag_limit_accel && fabsf(_accel_target) < fabsf(_accel_out_last)) || !_flag_limit_accel) {
        _accel_out = _accel_target;
    } else {
        _accel_out = _accel_out_last;
    }
    _accel_out_last = _accel_out;

    if (_gnd_hgt >= _flare_alt_calc * 1.25f) {
        _pitch_target = accel_to_angle(-_accel_out * 0.01) * 100;
    } else {
        _pitch_target = 0.0f;
    }
}


void AC_Autorotation::update_flare_alt(void)
{
    if (!_flare_update_check) {
        float delta_v_z = fabsf((_inav.get_velocity_z_up_cms()) * 0.01f + _est_rod);

        if ((_speed_forward >= 0.8f * _param_target_speed) && (delta_v_z <= 2) && (fabsf(_avg_acc_z+GRAVITY_MSS) <= 0.5f)) {
            float vel_z = _inav.get_velocity_z_up_cms() * 0.01f;
            float spd_fwd = _speed_forward * 0.01f;
            _c = (_lift_hover / (sq(vel_z) * 0.5f * SSL_AIR_DENSITY * _disc_area)) * 1.15f;
            _c = constrain_float(_c, 0.7f, 1.4f);
            calc_flare_alt(vel_z,spd_fwd);
            _flare_update_check = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Flare_alt_updated=%f C_updated=%f",  _flare_alt_calc*0.01f, _c);
        }
    }
}


// Determine the forward ground speed component from measured components
float AC_Autorotation::calc_speed_forward(void)
{
    auto &ahrs = AP::ahrs();
    Vector2f groundspeed_vector = ahrs.groundspeed_vector();
    float speed_forward = (groundspeed_vector.x*ahrs.cos_yaw() + groundspeed_vector.y * ahrs.sin_yaw()) * 100.0; // (cm/s)
    return speed_forward;
}


void AC_Autorotation::flare_controller(void)
{

    // Specify forward velocity component and determine delta velocity with respect to time
    _speed_forward = calc_speed_forward(); // (cm/s)
    _delta_speed_fwd = _speed_forward - _speed_forward_last; // (cm/s)
    _speed_forward_last = _speed_forward; // (cm/s)
    _desired_speed = linear_interpolate(0.0f, _flare_entry_speed, _gnd_hgt, _cushion_alt, _flare_alt_calc);

    // Get p
    _vel_p = _p_fw_vel.get_p(_desired_speed - _speed_forward);

    // Calculate acceleration target based on PI controller
    _accel_target = _vel_p;

    // Filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(10.0f);
    _accel_target_filter.apply(_accel_target, _dt);

    // Limits the maximum change in pitch attitude based on acceleration
    if (_accel_target > _accel_out_last + _accel_max) {
        _accel_target = _accel_out_last + _accel_max;
    } else if (_accel_target < _accel_out_last - _accel_max) {
        _accel_target = _accel_out_last - _accel_max;
    }

    // Limiting acceleration based on velocity gained during the previous time step
    if (fabsf(_delta_speed_fwd) > _accel_max * _dt) {
        _flag_limit_accel = true;
    } else {
        _flag_limit_accel = false;
    }

    if ((_flag_limit_accel && fabsf(_accel_target) < fabsf(_accel_out_last)) || !_flag_limit_accel) {
        _accel_out = _accel_target;
    } else {
        _accel_out = _accel_out_last;
    }
    _accel_out_last = _accel_out;

    // Estimate flare effectiveness
    if (_speed_forward <= (0.6 * _flare_entry_speed) && (fabsf(_avg_acc_z+GRAVITY_MSS) <= 0.5f)) {
        if (!_flare_complete) {
            _flare_complete = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Flare_complete");
        }
    }

    if (!_flare_complete) {
        _pitch_target = atanf(-_accel_out / (GRAVITY_MSS * 100.0f)) * (18000.0f/M_PI);
        _pitch_target = constrain_float(_pitch_target, 0.0f, AP_ALPHA_TPP * 100.0f);
    } else {
        _pitch_target *= 0.9995f;
    }
}


void AC_Autorotation::touchdown_controller(void)
{
    float _current_sink_rate = _inav.get_velocity_z_up_cms();
    if (_gnd_hgt >= _ground_clearance) {
        _desired_sink_rate = linear_interpolate(0.0f, _entry_sink_rate, _gnd_hgt, _ground_clearance, _entry_alt);
    } else {
        _desired_sink_rate = 0.0f;
    }

    // Update forward speed for logging
    _speed_forward = calc_speed_forward(); // (cm/s)

    // Check to see if we think the aircraft is on the ground
    if(_current_sink_rate < 10.0 && _gnd_hgt <= _ground_clearance && _time_on_ground == 0){
        _time_on_ground = AP_HAL::millis();
    }

    // Use a timer to get confidence that the aircraft is on the ground.
    // Note: The landing detector uses the zero thust collective as an indicator for being on the ground. The touch down controller will have
    // driven the collective high to cushion the landing so the landing detector will not trip until we drive the collective back to zero thrust and below.
    if ((_time_on_ground > 0) && ((AP_HAL::millis() - _time_on_ground) > MIN_TIME_ON_GROUND)) {
       // On ground, smoothly lower collective to just bellow zero thrust, to make sure we trip the landing detector
       float desired_col = _motors_heli->get_coll_land_min() * 0.95;
       _collective_out = _collective_out*0.9 + desired_col*0.1;

    } else {
        _collective_out =  constrain_value((_p_coll_tch.get_p(_desired_sink_rate - _current_sink_rate))*0.01f + _ff_term_hs, 0.0f, 1.0f);
        col_trim_lpf.set_cutoff_frequency(_col_cutoff_freq);
        _ff_term_hs = col_trim_lpf.apply(_collective_out, _dt);
    }

    set_collective(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);

    // Smoothly scale the pitch target back to zero (level)
    _pitch_target *= 0.95f;
}


// Determine if we are above the touchdown height using the descent rate and param values
bool AC_Autorotation::should_begin_touchdown(void)
{
    float vz = _inav.get_velocity_z_up_cms();

    // We need to be descending for the touch down controller to interpolate the target
    // sensibly between the entry of the touch down phase zero.
    if (vz >= 0.0) {
        return false;
    }

    float time_to_ground = (-_gnd_hgt / _inav.get_velocity_z_up_cms());

    return time_to_ground <= _t_tch.get();
}


void AC_Autorotation::update_avg_acc_z(void)
{
    // Wrap index
    if (_index >= 10) {
        _index = 0;
    }

    _acc_z_sum -= _curr_acc_z[_index];
    _curr_acc_z[_index] = _ahrs.get_accel_ef().z;
    _acc_z_sum += _curr_acc_z[_index];
    _index = _index + 1;

    _avg_acc_z = _acc_z_sum / 10.0;
}
