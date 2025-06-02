#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_MotorsHeli_RSC.h>
#include <Filter/LowPassFilter.h>
#include <AC_PID/AC_P.h>
#include <AC_PID/AC_PID_Basic.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_SurfaceDistance/AP_SurfaceDistance.h>
#include <AC_AttitudeControl/AC_PosControl.h>

class AC_Autorotation
{
public:

    //Constructor
    AC_Autorotation(AP_MotorsHeli*& motors, AC_AttitudeControl*& att_crtl, AP_InertialNav& inav, AC_PosControl*& pos_ctrl);

    void init(void);

    bool enabled(void) const { return _param_enable.get() > 0; }

    // Init and run entry phase controller
    void init_entry(void);
    void run_entry(float des_lat_accel_norm);

    // Init and run the glide phase controller
    void init_glide(void);
    void run_glide(float des_lat_accel_norm);

    // Init and run the flare phase controller
    void init_flare(void);
    void run_flare(float des_lat_accel_norm);

    // Init and run the touch down phase controller
    void init_touchdown(void);
    void run_touchdown(float des_lat_accel_norm);

    // Run the landed phase controller to zero the desired vels and accels
    void run_landed(void);

    // Arming checks for autorotation, mostly checking for miss-configurations
    bool arming_checks(size_t buflen, char *buffer) const;

    // Logging of lower rate autorotation specific variables
    void log_write_autorotation(void) const;

    bool below_flare_height(void) const;

    // Determine if we are above the touchdown height using the descent rate and param values
    bool should_begin_touchdown(void) const;

    // Returns true if we have met the autorotation-specific reasons to think we have landed
    bool check_landed(void);

    // Dynamically update time step used in autorotation controllers
    void set_dt(float delta_sec);

    // Update the height above ground estimate in meters
    void update_hagl(void);

    // Functions to support NAV_CONTROLLER_OUTPUT telemetry
    int32_t get_wp_bearing(void) const;
    float wp_distance_m(void) const;
    float crosstrack_error(void) const;

    // A helper function to tidy up on mode exit
    void exit(void);

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];

    static const uint32_t entry_time_ms = 2000; // (ms) Number of milliseconds that the entry phase operates for

private:

    // References to other libraries
    AP_MotorsHeli*&    _motors_heli;
    AC_AttitudeControl*& _attitude_control;
    AC_PosControl*&    _pos_control;

    // A helper class that allows the setting of heights that have a minimum
    // protection value and prevents direct access to the height value
    class GuardedHeight {
        public:
            void set(float hgt);
            float get(void) const { return height; };
            void reset(void) { set(0.0); }
            AP_Float min_height;
            AP_Float max_height;
        private:
            float height;
    };

#if AP_RANGEFINDER_ENABLED
    AP_SurfaceDistance* _ground_surface;
#endif
    uint32_t _last_gnd_surf_update;

    // Determine the body frame speeds in m/s
    float get_bf_speed_forward(void) const;
    float get_bf_speed_down(void) const;
    Vector3f get_bf_vel(void) const;

    // Determine the earth frame forward speed in m/s
    float get_ef_speed_forward(void) const;

    // Get the earth frame vertical velocity in meters, positive is up
    float get_ef_velocity_up(void) const;

    // Helper to get measured head speed that has been normalised by head speed set point
    bool get_norm_head_speed(float& norm_rpm) const;

    float _dt;         // (s) Time step, updated dynamically from vehicle
    float _hagl;       // (m) height above ground
    bool _hagl_valid;   // Ensure that the height measurement we have is valid

    // Parameter values
    AP_Int8  _param_enable;
    AP_Float _param_head_speed_set_point;
    AP_Float _param_target_speed;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
    AP_Float _param_accel_max;
    AP_Int8  _param_rpm_instance;
    AP_Float _param_fwd_k_ff;
    AP_Float _param_solidity;
    AP_Float _param_diameter;
    AP_Float _param_touchdown_time;
    AP_Int8  _param_nav_mode;

    // Navigation controller
    void update_navigation_controller(float des_lat_accel_norm);
    void calc_yaw_rate_from_roll_target(float& yaw_rate_rad, float& lat_accel);
    bool _heading_hold;           // A flag used to tell the navigation controller to maintain the current heading.
    float _desired_vel;           // (m/s) This is the velocity that we want.  This is the variable that is set by the invoking function to request a certain speed
    Vector2f _track_vector;       // The velocity or heading vector stored at init and used as a navigation target for CROSS_TRACK mode
    Vector2f _bearing_vector;     // Unit vector calculated from NE vel targets to report bearing over telemetry

    enum class Nav_Mode : int8_t {
        PILOT_LAT_ACCEL = 0, // Pilot controls direction, using yaw stick to request lateral accelerations, coordinated turns are performed.
        TURN_INTO_WIND  = 1, // Aircraft attempts to turn into wind.
        CROSS_TRACK     = 2, // Aircraft attempts to maintain velocity vector or heading at entry, yaw is used to maintain cross track.
    };

    // Head speed controller variables
    void update_headspeed_controller(void);  // Update controller used to drive head speed with collective
    float _hs_accel;                         // The head speed target acceleration during the entry phase
    float _head_speed_error;                 // Error between target head speed and current head speed. Normalised by head speed set point RPM.
    float _target_head_speed;                // Normalised target head speed.  Normalised by head speed set point RPM.
    LowPassFilterFloat col_trim_lpf;         // Low pass filter for collective trim
    AC_P _p_hs{1.0};                         // head speed-collective p controller

    // Flare controller functions and variables
    void initial_flare_hgt_estimate(void);
    void calc_flare_hgt(const float fwd_speed, float climb_rate);
    void update_flare_hgt(void);
    void set_flare_height(float hgt);
    GuardedHeight _flare_hgt;            // (m) Calculated height above ground at which the flare phase will begin
    float _calculated_flare_hgt;         // (m) Used for logging the calculated flare height so that we can keep track of the calculations output. This value is not used for the flare phase decision.
    float _hover_thrust;                 // (N) Estimated thrust force the aircraft produces in the hover
    float _flare_entry_fwd_speed;        // The measured body frame forward speed of the vehicle as it enters the flare phase
    LowPassFilterFloat _lagged_vel_z;    // (m/s) A slow filter on velocity that purposefully lags behind the latest measurements so that we can get an idea of whether we can be considered to be in steady conditions

    // Touch down controller functions and variables
    GuardedHeight _touch_down_hgt;       // (m) Height above ground for touchdown phase to begin
    float _calculated_touch_down_hgt;    // (m) Used for logging the calculated touch down height so that we can keep track of the calculations output. This value is not used for the touch down phase decision.
    float _touchdown_init_climb_rate;    // (m/s) The measured climb rate (positive up) when the touch down phase is init
    float _touchdown_init_hgt;           // (m) The measured height above the ground when the touch down phase is init
    AC_P _p_col_td{0.2};                 // Touch down collective p controller

    // Flags used to check if we believe the aircraft has landed
    struct {
        bool min_speed;
        bool land_col;
        bool is_still;
    } _landed_reason;

    // Parameter accessors that provide value constraints
    float get_accel_max(void) const { return MAX(_param_accel_max.get(), 0.5); }
    float get_touchdown_time(void) const { return MAX(_param_touchdown_time.get(), 0.3); }
    float get_solidity(void) const { return MAX(_param_solidity.get(), 0.01); }

    const float MIN_MANOEUVERING_SPEED = 2.0; // (m/s)

};
