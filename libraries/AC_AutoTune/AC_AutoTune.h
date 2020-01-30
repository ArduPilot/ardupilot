/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for autotune of multirotors. Based on original autotune code from ArduCopter, written by Leonard Hall
  Converted to a library by Andrew Tridgell
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AC_AttitudeControl/AC_PosControl.h>

class AC_AutoTune {
public:
    // constructor
    AC_AutoTune();

    // main run loop
    virtual void run();

    // save gained, called on disarm
    void save_tuning_gains();

    // stop tune, reverting gains
    void stop();

    // reset Autotune so that gains are not saved again and autotune can be run again.
    void reset() {
        mode = UNINITIALISED;
        axes_completed = 0;
    }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // methods that must be supplied by the vehicle specific subclass
    virtual bool init(void) = 0;

    // get pilot input for desired cimb rate
    virtual float get_pilot_desired_climb_rate_cms(void) const = 0;

    // get pilot input for designed roll and pitch, and yaw rate
    virtual void get_pilot_desired_rp_yrate_cd(float &roll_cd, float &pitch_cd, float &yaw_rate_cds) = 0;

    // init pos controller Z velocity and accel limits
    virtual void init_z_limits() = 0;

    // log PIDs at full rate for during twitch
    virtual void log_pids() = 0;
    
    // start tune - virtual so that vehicle code can add additional pre-conditions
    virtual bool start(void);

    // return true if we have a good position estimate
    virtual bool position_ok();

    enum at_event {
        EVENT_AUTOTUNE_INITIALISED   =  0,
        EVENT_AUTOTUNE_OFF           =  1,
        EVENT_AUTOTUNE_RESTART       =  2,
        EVENT_AUTOTUNE_SUCCESS       =  3,
        EVENT_AUTOTUNE_FAILED        =  4,
        EVENT_AUTOTUNE_REACHED_LIMIT =  5,
        EVENT_AUTOTUNE_PILOT_TESTING =  6,
        EVENT_AUTOTUNE_SAVEDGAINS    =  7
    };

    // write a log event
    virtual void Log_Write_Event(enum at_event id) = 0;

    // internal init function, should be called from init()
    bool init_internals(bool use_poshold,
                        AC_AttitudeControl_Multi *attitude_control,
                        AC_PosControl *pos_control,
                        AP_AHRS_View *ahrs_view,
                        AP_InertialNav *inertial_nav);

private:
    void control_attitude();
    void backup_gains_and_initialise();
    void load_orig_gains();
    void load_tuned_gains();
    void load_intra_test_gains();
    void load_twitch_gains();
    void update_gcs(uint8_t message_id);
    bool roll_enabled();
    bool pitch_enabled();
    bool yaw_enabled();
    void twitching_test_rate(float rate, float rate_target, float &meas_rate_min, float &meas_rate_max);
    void twitching_abort_rate(float angle, float rate, float angle_max, float meas_rate_min);
    void twitching_test_angle(float angle, float rate, float angle_target, float &meas_angle_min, float &meas_angle_max, float &meas_rate_min, float &meas_rate_max);
    void twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max);
    void updating_rate_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);
    void updating_rate_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);
    void updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);
    void updating_angle_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max);
    void updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max);
    void get_poshold_attitude(float &roll_cd, float &pitch_cd, float &yaw_cd);

    void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);

    void send_step_string();
    const char *level_issue_string() const;
    const char * type_string() const;
    void announce_state_to_gcs();
    void do_gcs_announcements();

    enum struct LevelIssue {
        NONE,
        ANGLE_ROLL,
        ANGLE_PITCH,
        ANGLE_YAW,
        RATE_ROLL,
        RATE_PITCH,
        RATE_YAW,
    };
    bool check_level(const enum LevelIssue issue, const float current, const float maximum);
    bool currently_level();

    // autotune modes (high level states)
    enum TuneMode {
        UNINITIALISED = 0,        // autotune has never been run
        TUNING = 1,               // autotune is testing gains
        SUCCESS = 2,              // tuning has completed, user is flight testing the new gains
        FAILED = 3,               // tuning has failed, user is flying on original gains
    };

    // steps performed while in the tuning mode
    enum StepType {
        WAITING_FOR_LEVEL = 0,    // autotune is waiting for vehicle to return to level before beginning the next twitch
        TWITCHING = 1,            // autotune has begun a twitch and is watching the resulting vehicle movement
        UPDATE_GAINS = 2          // autotune has completed a twitch and is updating the gains based on the results
    };

    // things that can be tuned
    enum AxisType {
        ROLL = 0,                 // roll axis is being tuned (either angle or rate)
        PITCH = 1,                // pitch axis is being tuned (either angle or rate)
        YAW = 2,                  // pitch axis is being tuned (either angle or rate)
    };

    // mini steps performed while in Tuning mode, Testing step
    enum TuneType {
        RD_UP = 0,                // rate D is being tuned up
        RD_DOWN = 1,              // rate D is being tuned down
        RP_UP = 2,                // rate P is being tuned up
        SP_DOWN = 3,              // angle P is being tuned down
        SP_UP = 4                 // angle P is being tuned up
    };

    // type of gains to load
    enum GainType {
        GAIN_ORIGINAL   = 0,
        GAIN_TWITCH     = 1,
        GAIN_INTRA_TEST = 2,
        GAIN_TUNED      = 3,
    };
    void load_gains(enum GainType gain_type);

    TuneMode mode                : 2;    // see TuneMode for what modes are allowed
    bool     pilot_override      : 1;    // true = pilot is overriding controls so we suspend tuning temporarily
    AxisType axis                : 2;    // see AxisType for which things can be tuned
    bool     positive_direction  : 1;    // false = tuning in negative direction (i.e. left for roll), true = positive direction (i.e. right for roll)
    StepType step                : 2;    // see StepType for what steps are performed
    TuneType tune_type           : 3;    // see TuneType
    bool     ignore_next         : 1;    // true = ignore the next test
    bool     twitch_first_iter   : 1;    // true on first iteration of a twitch (used to signal we must step the attitude or rate target)
    bool     use_poshold         : 1;    // true = enable position hold
    bool     have_position       : 1;    // true = start_position is value
    Vector3f start_position;
    uint8_t  axes_completed;             // bitmask of completed axes

    // variables
    uint32_t override_time;                         // the last time the pilot overrode the controls
    float    test_rate_min;                         // the minimum angular rate achieved during TESTING_RATE step
    float    test_rate_max;                         // the maximum angular rate achieved during TESTING_RATE step
    float    test_angle_min;                        // the minimum angle achieved during TESTING_ANGLE step
    float    test_angle_max;                        // the maximum angle achieved during TESTING_ANGLE step
    uint32_t step_start_time_ms;                    // start time of current tuning step (used for timeout checks)
    uint32_t level_start_time_ms;                   // start time of waiting for level
    uint32_t step_time_limit_ms;                    // time limit of current autotune process
    int8_t   counter;                               // counter for tuning gains
    float    target_rate, start_rate;               // target and start rate
    float    target_angle, start_angle;             // target and start angles
    float    desired_yaw_cd;                        // yaw heading during tune
    float    rate_max, test_accel_max;              // maximum acceleration variables
    float    step_scaler;                           // scaler to reduce maximum target step
    float    abort_angle;                           // Angle that test is aborted

    LowPassFilterFloat  rotation_rate_filt;         // filtered rotation rate in radians/second

    // backup of currently being tuned parameter values
    float    orig_roll_rp, orig_roll_ri, orig_roll_rd, orig_roll_rff, orig_roll_sp, orig_roll_accel;
    float    orig_pitch_rp, orig_pitch_ri, orig_pitch_rd, orig_pitch_rff, orig_pitch_sp, orig_pitch_accel;
    float    orig_yaw_rp, orig_yaw_ri, orig_yaw_rd, orig_yaw_rff, orig_yaw_rLPF, orig_yaw_sp, orig_yaw_accel;
    bool     orig_bf_feedforward;

    // currently being tuned parameter values
    float    tune_roll_rp, tune_roll_rd, tune_roll_sp, tune_roll_accel;
    float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel;
    float    tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, tune_yaw_accel;

    uint32_t announce_time;
    float lean_angle;
    float rotation_rate;
    float roll_cd, pitch_cd;

    uint32_t last_pilot_override_warning;

    struct {
        LevelIssue issue{LevelIssue::NONE};
        float maximum;
        float current;
    } level_problem;

    AP_Int8  axis_bitmask;
    AP_Float aggressiveness;
    AP_Float min_d;

    // copies of object pointers to make code a bit clearer
    AC_AttitudeControl_Multi *attitude_control;
    AC_PosControl *pos_control;
    AP_AHRS_View *ahrs_view;
    AP_InertialNav *inertial_nav;
    AP_Motors *motors;
};
