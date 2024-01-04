#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_MotorsHeli_RSC.h>
#include <AP_Motors/AP_MotorsHeli.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <AC_PID/AC_P.h>
#include <AP_InertialNav/AP_InertialNav.h>  // Inertial Navigation library

class AC_Autorotation
{
public:

    //Constructor
    AC_Autorotation(AP_InertialNav& inav, AP_AHRS& ahrs);

     // object initialisation
    void init(AP_MotorsHeli* motors);

    void initial_flare_estimate(void);

    // Update head speed controller
    bool update_hs_glide_controller(void);

    // Function just returns the rpm as last read in this library
    float get_rpm(void) const { return _current_rpm; }

    // Function fetches fresh rpm update and continues sensor health monitoring
    float get_rpm(bool update_counter);

    // Sets the normalised target head speed
    void set_target_head_speed(float ths) { _target_head_speed = ths; }

    // Sets the collective low pass filter cut off frequency
    void set_col_cutoff_freq(float freq) { _col_cutoff_freq = freq; }

    int16_t get_hs_set_point(void) { return _param_head_speed_set_point; }

    float get_col_entry_freq(void) { return _param_col_entry_cutoff_freq; }

    float get_col_glide_freq(void) { return _param_col_glide_cutoff_freq; }

    float get_col_cushion_freq(void) { return _param_col_cushion_cutoff_freq; }

    float get_bail_time(void) { return _param_bail_time; }

    float get_last_collective() const { return _collective_out; }

    bool is_enable(void) { return _param_enable; }

    void Log_Write_Autorotation(void) const;

    // Update forward speed controller
    void update_forward_speed_controller(void);

    // Overloaded: Set desired speed for forward controller to parameter value
    void set_desired_fwd_speed(void) { _vel_target = _param_target_speed; }

    // Overloaded: Set desired speed to argument value
    void set_desired_fwd_speed(float speed) { _vel_target = speed; }

    // Get pitch target
    int32_t get_pitch(void) const { return _pitch_target; }

    // Calculates the forward speed in the horizontal plane
    float calc_speed_forward(void);

    // set the loop time
    void set_dt(float delta_sec);

    // Update the flare controller
    void flare_controller(void);

    // Update the touchdown controller
    void touchdown_controller(void);

    void set_ground_distance(float radalt) { _radar_alt = radalt; }

    void get_entry_speed(void);

    float get_ground_distance(void) const { return _radar_alt; }

    float get_time_to_ground(void) const { return _time_to_ground; }

    void time_to_ground(void);

    void set_entry_sink_rate (float sink_rate) { _entry_sink_rate = sink_rate; }

    void set_entry_alt (float entry_alt) { _entry_alt = entry_alt; }

    void set_ground_clearance(float ground_clearance) { _ground_clearance = ground_clearance; }

    void init_est_radar_alt(void);

    void update_est_radar_alt(void);

    float get_est_alt(void) const { return _est_alt; }

    void update_hover_autorotation_controller();

    void update_avg_acc_z(void);

    float get_flare_alt(void) const { return _flare_alt_calc; }

    void update_flare_alt(void);

    void calc_flare_alt(float sink_rate, float fwd_speed);

    float get_t_touchdown(void) const { return _t_tch; }

    float get_cushion_alt(void) const { return _cushion_alt; }

    bool get_flare_status(void) { return _flare_complete; }

    void calc_sink_d_avg(void);

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];

    bool  _using_rfnd;

private:

    //--------Internal Variables--------
    float _current_rpm;
    float _collective_out;
    float _head_speed_error;         // Error between target head speed and current head speed.  Normalised by head speed set point RPM.
    float _col_cutoff_freq;          // Lowpass filter cutoff frequency (Hz) for collective.
    uint8_t _unhealthy_rpm_counter;  // Counter used to track RPM sensor unhealthy signal.
    uint8_t _healthy_rpm_counter;    // Counter used to track RPM sensor healthy signal.
    float _target_head_speed;        // Normalised target head speed.  Normalised by head speed set point RPM.
    float _p_term_hs;                // Proportional contribution to collective setting.
    float _ff_term_hs;               // Following trim feed forward contribution to collective setting.
    float _vel_target;               // Forward velocity target.
    float _pitch_target;             // Pitch angle target.
    float _accel_max;                // Maximum acceleration limit.
    int16_t _speed_forward_last;     // The forward speed calculated in the previous cycle.
    bool _flag_limit_accel;          // Maximum acceleration limit reached flag.
    float _accel_out_last;           // Acceleration value used to calculate pitch target in previous cycle.
    float _cmd_vel;                  // Command velocity, used to get PID values for acceleration calculation.
    float _accel_target;             // Acceleration target, calculated from PID.
    float _delta_speed_fwd;          // Change in forward speed between computation cycles.
    float _dt;                       // Time step.
    int16_t _speed_forward;          // Measured forward speed.
    float _vel_p;                    // Forward velocity P term.
    float _vel_ff;                   // Forward velocity Feed Forward term.
    float _accel_out;                // Acceleration value used to calculate pitch target.
    float _entry_sink_rate;
    float _entry_alt;
    float _radar_alt;
    float _flare_entry_speed;
    float _desired_speed;
    float _time_to_ground;
    float _desired_sink_rate;
    float _ground_clearance;
    float _est_alt;
    float _descent_rate_filtered;
    float _radar_alt_prev;
    float _radar_alt_calc;
    float _avg_acc_z;
    float _acc_z_sum;
    int16_t _index;
    float _curr_acc_z[10]{};
    float _flare_alt_calc;
    float _lift_hover;
    float _c;
    float _cushion_alt;
    float _disc_area;
    float _last_vertical_speed;
    float _sink_deriv;
    float _est_rod;
    float _avg_sink_deriv;
    float _avg_sink_deriv_sum;
    int16_t _index_sink_rate;
    float _curr_sink_deriv[20]{};
    bool  _flare_complete;
    bool  _flare_update_check;

    LowPassFilterFloat _accel_target_filter; // acceleration target filter

    //--------Parameter Values--------
    AP_Int8  _param_enable;
    AC_P _p_hs;
    AC_P _p_fw_vel;
    AC_P _p_coll_tch;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
    AP_Float _param_col_cushion_cutoff_freq;
    AP_Int16 _param_accel_max;
    AP_Float _param_bail_time;
    AP_Int8  _param_rpm_instance;
    AP_Float _param_fwd_k_ff;
    AP_Int16 _param_target_speed;
    AP_Int16 _param_head_speed_set_point;
    AP_Float _param_solidity;
    AP_Float _param_diameter;
    AP_Float _t_tch;


    //--------Internal Flags--------
    struct controller_flags {
            bool bad_rpm             : 1;
            bool bad_rpm_warning     : 1;
    } _flags;

    //--------Internal Functions--------
    // set the collective in the motors library
    void set_collective(float _collective_filter_cutoff) const;

    // low pass filter for collective trim
    LowPassFilterFloat col_trim_lpf;

    // low pass filter for descent rate
    LowPassFilterFloat descent_rate_lpf;

    //--------References to Other Libraries--------
    AP_InertialNav&    _inav;
    AP_AHRS&           _ahrs;
    AP_MotorsHeli*     _motors_heli;
};
