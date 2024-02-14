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
    void init(AP_MotorsHeli* motors, float gnd_clear);

    // Helper to set all necessary variables needed for the entry phase
    void init_entry(void);

    // Helper to set all necessary variables needed for the glide phase
    void init_glide(float hs_targ);

    // Helper to set all necessary variables needed for the flare phase
    void init_flare(float hs_targ);

    // Helper to set all necessary variables needed for the touchdown phase
    void init_touchdown(void);

    void initial_flare_estimate(void);

    // Update head speed controller
    void update_hs_glide_controller(void);

    // Function fetches fresh rpm update and continues sensor health monitoring
    float get_rpm(void);

    // Sets the normalised target head speed
    void set_target_head_speed(float ths) { _target_head_speed = ths; }

    // Sets the collective low pass filter cut off frequency
    void set_col_cutoff_freq(float freq) { _col_cutoff_freq = freq; }

    int16_t get_hs_set_point(void) { return _param_head_speed_set_point; }

    float get_bail_time(void) { return _param_bail_time; }

    float get_last_collective() const { return _collective_out; }

    bool is_enable(void) { return _param_enable; }

    void Log_Write_Autorotation(void) const;

    // Update forward speed controller
    void update_forward_speed_controller(void);

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

    void set_ground_distance(float gnd_dist) { _gnd_hgt = (float)gnd_dist; }

    void get_entry_speed(void);

    void set_entry_sink_rate (float sink_rate) { _entry_sink_rate = sink_rate; }

    void set_entry_alt (float entry_alt) { _entry_alt = entry_alt; }

    bool above_flare_height(void) const { return _gnd_hgt > _flare_alt_calc; }

    void update_hover_autorotation_controller();

    // update rolling average z-accel filter
    void update_avg_acc_z(void);

    void update_flare_alt(void);

    void calc_flare_alt(float sink_rate, float fwd_speed);

    // Estimate the time to impact and compare it with the touchdown time param
    bool should_begin_touchdown(void);

    bool use_stabilise_controls(void) const { return _options.get() & int32_t(OPTION::STABILISE_CONTROLS); }

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    //--------Internal Variables--------
    float _current_rpm;
    float _collective_out;
    float _head_speed_error;         // Error between target head speed and current head speed.  Normalised by head speed set point RPM.
    float _col_cutoff_freq;          // Lowpass filter cutoff frequency (Hz) for collective.
    uint16_t _unhealthy_rpm_counter;  // Counter used to track RPM sensor unhealthy signal.
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
    float _entry_sink_rate;          // Descent rate at beginning of touvhdown collective pull
    float _entry_alt;                // Altitude at beginning of touchdown coll pull
    float _flare_entry_speed;        // Traslational velocity at beginning of flare maneuver
    float _desired_speed;            // Desired traslational velocity during flare
    float _desired_sink_rate;        // Desired vertical velocity during touchdown
    float _ground_clearance;         // Sensor offset distance
    float _gnd_hgt;                  // Height above ground, passed down from copter, can be from lidar or terrain
    float _avg_acc_z;                // Averaged vertical acceleration
    float _acc_z_sum;                // Sum of vertical acceleration samples
    int16_t _index;                  // Index for vertical acceleration rolling average
    float _curr_acc_z[10];           // Array for storing vertical acceleration samples
    float _flare_alt_calc;           // Calculated flare altitude
    float _lift_hover;               // Main rotor thrust in hover condition
    float _c;                        // Main rotor drag coefficient
    float _cushion_alt;              // Altitude for touchdown collective pull
    float _disc_area;                // Main rotor disc area
    float _est_rod;                  // Estimated rate of descent (vertical autorotation)
    bool  _flare_complete;           // Flare completed
    bool  _flare_update_check;       // Check for flare altitude updating
    uint32_t _time_on_ground;        // Time elapsed after touch down

    LowPassFilterFloat _accel_target_filter; // acceleration target filter

    //--------Parameter Values--------
    AP_Int8  _param_enable;
    AC_P _p_hs;
    AC_P _p_fw_vel;
    AC_P _p_coll_tch;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
    AP_Float _param_col_touchdown_cutoff_freq;
    AP_Int16 _param_accel_max;
    AP_Float _param_bail_time;
    AP_Int8  _param_rpm_instance;
    AP_Float _param_fwd_k_ff;
    AP_Int16 _param_target_speed;
    AP_Int16 _param_head_speed_set_point;
    AP_Float _param_solidity;
    AP_Float _param_diameter;
    AP_Float _t_tch;
    AP_Int32 _options;

    enum class OPTION {
        STABILISE_CONTROLS=(1<<0),
    };

    //--------Internal Functions--------
    // set the collective in the motors library
    void set_collective(float _collective_filter_cutoff) const;

    // low pass filter for collective trim
    LowPassFilterFloat col_trim_lpf;

    //--------References to Other Libraries--------
    AP_InertialNav&    _inav;
    AP_AHRS&           _ahrs;
    AP_MotorsHeli*     _motors_heli;
};
