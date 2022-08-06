#include "AC_Autorotation.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AHRS/AP_AHRS.h>

//Autorotation controller defaults
#define AROT_BAIL_OUT_TIME                            2.0f     // Default time for bail out controller to run (unit: s)

// Head Speed (HS) controller specific default definitions
#define HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ          2.0f     // low-pass filter on accel error (unit: hz)
#define HS_CONTROLLER_HEADSPEED_P                     0.7f     // Default P gain for head speed controller (unit: -)
#define HS_CONTROLLER_ENTRY_COL_FILTER                0.7f    // Default low pass filter frequency during the entry phase (unit: Hz)
#define HS_CONTROLLER_GLIDE_COL_FILTER                0.1f    // Default low pass filter frequency during the glide phase (unit: Hz)

// Speed Height controller specific default definitions for autorotation use
#define FWD_SPD_CONTROLLER_GND_SPEED_TARGET           1100     // Default target ground speed for speed height controller (unit: cm/s)
#define FWD_SPD_CONTROLLER_MAX_ACCEL                  60      // Default acceleration limit for speed height controller (unit: cm/s/s)
#define AP_FW_VEL_P                       0.9f
#define AP_FW_VEL_FF                      0.15f


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
    AP_GROUPINFO("TARG_SP", 4, AC_Autorotation, _param_target_speed, FWD_SPD_CONTROLLER_GND_SPEED_TARGET),

    // @Param: COL_FILT_E
    // @DisplayName: Entry Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the entry phase.  Acts as a following trim.  Must be higher than AROT_COL_FILT_G.
    // @Units: Hz
    // @Range: 0.2 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_E", 5, AC_Autorotation, _param_col_entry_cutoff_freq, HS_CONTROLLER_ENTRY_COL_FILTER),

    // @Param: COL_FILT_G
    // @DisplayName: Glide Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the glide phase.  Acts as a following trim.  Must be lower than AROT_COL_FILT_E.
    // @Units: Hz
    // @Range: 0.03 0.15
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COL_FILT_G", 6, AC_Autorotation, _param_col_glide_cutoff_freq, HS_CONTROLLER_GLIDE_COL_FILTER),

    // @Param: AS_ACC_MAX
    // @DisplayName: Forward Acceleration Limit
    // @Description: Maximum forward acceleration to apply in speed controller.
    // @Units: cm/s/s
    // @Range: 30 60
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("AS_ACC_MAX", 7, AC_Autorotation, _param_accel_max, FWD_SPD_CONTROLLER_MAX_ACCEL),

    // @Param: BAIL_TIME
    // @DisplayName: Bail Out Timer
    // @Description: Time in seconds from bail out initiated to the exit of autorotation flight mode.
    // @Units: s
    // @Range: 0.5 4
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BAIL_TIME", 8, AC_Autorotation, _param_bail_time, AROT_BAIL_OUT_TIME),

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
    // @Description: Velocity (horizontal) P gain.  Determines the propotion of the target acceleration based on the velocity error.
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
    AP_GROUPINFO("FW_V_FF", 11, AC_Autorotation, _param_fwd_k_ff, AP_FW_VEL_FF),

    AP_GROUPEND
};

// Constructor
AC_Autorotation::AC_Autorotation() :
    _p_hs(HS_CONTROLLER_HEADSPEED_P),
    _p_fw_vel(AP_FW_VEL_P)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

// Initialisation of head speed controller
void AC_Autorotation::init_hs_controller()
{
    // Set initial collective position to be the collective position on initialisation
    _collective_out = 0.4f;

    // Reset feed forward filter
    col_trim_lpf.reset(_collective_out);

    // Reset flags
    _flags.bad_rpm = false;

    // Reset RPM health monitoring
    _unhealthy_rpm_counter = 0;
    _healthy_rpm_counter = 0;

    // Protect against divide by zero
    _param_head_speed_set_point.set(MAX(_param_head_speed_set_point,500));
}


bool AC_Autorotation::update_hs_glide_controller(float dt)
{
    // Reset rpm health flag
    _flags.bad_rpm = false;
    _flags.bad_rpm_warning = false;

    // Get current rpm and update healthly signal counters
    _current_rpm = get_rpm(true);

    if (_unhealthy_rpm_counter <=30) {
        // Normalised head speed
        float head_speed_norm = _current_rpm / _param_head_speed_set_point;

        // Set collective trim low pass filter cut off frequency
        col_trim_lpf.set_cutoff_frequency(_col_cutoff_freq);

        // Calculate the head speed error.  Current rpm is normalised by the set point head speed.  
        // Target head speed is defined as a percentage of the set point.
        _head_speed_error = head_speed_norm - _target_head_speed;

        _p_term_hs = _p_hs.get_p(_head_speed_error);

        // Adjusting collective trim using feed forward (not yet been updated, so this value is the previous time steps collective position)
        _ff_term_hs = col_trim_lpf.apply(_collective_out, dt);

        // Calculate collective position to be set
        _collective_out = _p_term_hs + _ff_term_hs;

    } else {
        // RPM sensor is bad set collective to minimum
        _collective_out = -1.0f;

        _flags.bad_rpm_warning = true;
    }

    // Send collective to setting to motors output library
    set_collective(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);

    return _flags.bad_rpm_warning;
}


// Function to set collective and collective filter in motor library
void AC_Autorotation::set_collective(float collective_filter_cutoff) const
{
    AP_Motors *motors = AP::motors();
    if (motors) {
        motors->set_throttle_filter_cutoff(collective_filter_cutoff);
        motors->set_throttle(_collective_out);
    }
}


//function that gets rpm and does rpm signal checking to ensure signal is reliable
//before using it in the controller
float AC_Autorotation::get_rpm(bool update_counter)
{
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    float current_rpm = 0.0f;

    //Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        //Check requested rpm instance to ensure either 0 or 1.  Always defaults to 0.
        if ((_param_rpm_instance > 1) || (_param_rpm_instance < 0)) {
            _param_rpm_instance.set(0);
        }

        //Get RPM value
        uint8_t instance = _param_rpm_instance;

        //Check RPM sesnor is returning a healthy status
        if (!rpm->get_rpm(instance, current_rpm) || current_rpm <= -1) {
            //unhealthy, rpm unreliable
            _flags.bad_rpm = true;
        }

    } else {
        _flags.bad_rpm = true;
    }

    if (_flags.bad_rpm) {
        //count unhealthy rpm updates and reset healthy rpm counter
        _unhealthy_rpm_counter++;
        _healthy_rpm_counter = 0;

    } else if (!_flags.bad_rpm && _unhealthy_rpm_counter > 0) {
        //poor rpm reading may have cleared.  Wait 10 update cycles to clear.
        _healthy_rpm_counter++;

        if (_healthy_rpm_counter >= 10) {
            //poor rpm health has cleared, reset counters
            _unhealthy_rpm_counter = 0;
            _healthy_rpm_counter = 0;
        }
    }
    return current_rpm;
}


void AC_Autorotation::Log_Write_Autorotation(void) const
{
// @LoggerMessage: AROT
// @Vehicles: Copter
// @Description: Helicopter AutoRotation information
// @Field: TimeUS: Time since system startup
// @Field: P: P-term for headspeed controller response
// @Field: hserr: head speed error; difference between current and desired head speed
// @Field: ColOut: Collective Out
// @Field: FFCol: FF-term for headspeed controller response
// @Field: CRPM: current headspeed RPM
// @Field: SpdF: current forward speed
// @Field: CmdV: desired forward speed
// @Field: p: p-term of velocity response
// @Field: ff: ff-term of velocity response
// @Field: AccO: forward acceleration output
// @Field: AccT: forward acceleration target
// @Field: PitT: pitch target

    //Write to data flash log
    AP::logger().WriteStreaming("AROT",
                       "TimeUS,P,hserr,ColOut,FFCol,CRPM,SpdF,CmdV,p,ff,AccO,AccT,PitT",
                         "Qffffffffffff",
                        AP_HAL::micros64(),
                        (double)_p_term_hs,
                        (double)_head_speed_error,
                        (double)_collective_out,
                        (double)_ff_term_hs,
                        (double)_current_rpm,
                        (double)_speed_forward,
                        (double)_cmd_vel,
                        (double)_vel_p,
                        (double)_vel_ff,
                        (double)_accel_out,
                        (double)_accel_target,
                        (double)_pitch_target);
}


// Initialise forward speed controller
void AC_Autorotation::init_fwd_spd_controller(void)
{
    // Reset I term and acceleration target
    _accel_target = 0.0f;
    
    // Ensure parameter acceleration doesn't exceed hard-coded limit
    _accel_max = MIN(_param_accel_max, 60.0f);

    // Reset cmd vel and last accel to sensible values
    _cmd_vel = calc_speed_forward(); //(cm/s)
    _accel_out_last = _cmd_vel*_param_fwd_k_ff;
}


// set_dt - sets time delta in seconds for all controllers
void AC_Autorotation::set_dt(float delta_sec)
{
    _dt = delta_sec;
}


// update speed controller
void AC_Autorotation::update_forward_speed_controller(void)
{
    // Specify forward velocity component and determine delta velocity with respect to time
    _speed_forward = calc_speed_forward(); //(cm/s)

    _delta_speed_fwd = _speed_forward - _speed_forward_last; //(cm/s)
    _speed_forward_last = _speed_forward; //(cm/s)

    // Limitng the target velocity based on the max acceleration limit
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

    // get p
    _vel_p = _p_fw_vel.get_p(_cmd_vel-_speed_forward);

    // get ff
    _vel_ff = _cmd_vel*_param_fwd_k_ff;

    //calculate acceleration target based on PI controller
    _accel_target = _vel_ff + _vel_p;

    // filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(10.0f);
    _accel_target_filter.apply(_accel_target, _dt);

    //Limits the maximum change in pitch attitude based on acceleration
    if (_accel_target > _accel_out_last + _accel_max) {
        _accel_target = _accel_out_last + _accel_max;
    } else if (_accel_target < _accel_out_last - _accel_max) {
        _accel_target = _accel_out_last - _accel_max;
    }

    //Limiting acceleration based on velocity gained during the previous time step 
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

    // update angle targets that will be passed to stabilize controller
    _pitch_target = accel_to_angle(-_accel_out*0.01) * 100;

}


// Determine the forward ground speed component from measured components
float AC_Autorotation::calc_speed_forward(void)
{
    auto &ahrs = AP::ahrs();
    Vector2f groundspeed_vector = ahrs.groundspeed_vector();
    float speed_forward = (groundspeed_vector.x*ahrs.cos_yaw() + groundspeed_vector.y*ahrs.sin_yaw())* 100; //(c/s)
    return speed_forward;
}

