#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_MotorsHeli_RSC.h>
#include <Filter/LowPassFilter.h>
#include <AC_PID/AC_P.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>

class AC_Autorotation
{
public:

    //Constructor
    AC_Autorotation(AP_AHRS& ahrs, AP_MotorsHeli*& motors, AC_PosControl*& pos_ctrl, AC_AttitudeControl*& att_crtl);

    void init(void);

    bool enabled(void) const { return _param_enable.get() > 0; }

    // Init and run entry phase controller
    void init_entry(void);
    void run_entry(float pilot_norm_accel);

    // Init and run the glide phase controller
    void init_glide(void);
    void run_glide(float pilot_norm_accel);

    // Run the landed phase controller to zero the desired vels and accels
    void run_landed(void);

    // Update controller used to drive head speed with collective
    void update_headspeed_controller(void);

    // Update controller used to control speed via vehicle pitch
    void update_xy_speed_controller(void);

    // Arming checks for autorotation, mostly checking for miss-configurations
    bool arming_checks(size_t buflen, char *buffer) const;

    // Logging of autorotation specific variables
    void log_write_autorotation(void) const;

    // Returns true if we have met the autorotation-specific reasons to think we have landed
    bool check_landed(void);

    // Dynamically update time step used in autorotation controllers
    void set_dt(float delta_sec) { _dt = delta_sec; }

    // Helper to get measured head speed that has been normalised by head speed set point
    bool get_norm_head_speed(float& norm_rpm) const;

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];

    static const uint32_t entry_time_ms = 2000; // (ms) Number of milliseconds that the entry phase operates for

private:

    // Calculates the forward ground speed in the horizontal plane
    float get_speed_forward(void) const;

    // Get measured accel from the EKF and convert it to body frame XY
    Vector2f get_bf_accel(void) const;

    float _dt; // (s) Time step, updated dynamically from vehicle

    // Position controller related variables
    AC_AttitudeControl::HeadingCommand _desired_heading;
    LowPassFilterConstDtVector2f _desired_accel_bf;
    Vector2f _desired_velocity_bf;
    Vector2f _desired_accel_ef;
    Vector2f _desired_velocity_ef;

    // Head speed controller variables
    float _hs_decay;                  // The head speed target acceleration during the entry phase
    float _head_speed_error;          // Error between target head speed and current head speed. Normalised by head speed set point RPM.
    float _target_head_speed;         // Normalised target head speed.  Normalised by head speed set point RPM.
    float _p_term_hs;                 // Proportional contribution to collective setting.
    float _ff_term_hs;                // Following trim feed forward contribution to collective setting.
    LowPassFilterFloat col_trim_lpf;  // Low pass filter for collective trim

    // Flags used to check if we believe the aircraft has landed
    struct {
        bool min_speed;
        bool land_col;
        bool is_still;
    } _landed_reason;

    // Parameter values
    AP_Int8  _param_enable;
    AC_P _p_hs;
    AP_Float _param_head_speed_set_point;
    AP_Float _param_target_speed;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
    AP_Float _param_accel_max;
    AP_Int8  _param_rpm_instance;

    // References to other libraries
    AP_AHRS&           _ahrs;
    AP_MotorsHeli*&    _motors_heli;
    AC_PosControl*&    _pos_control;
    AC_AttitudeControl*& _attitude_control;
};
