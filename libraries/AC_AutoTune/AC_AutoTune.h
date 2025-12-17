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

#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "AC_AutoTune_FreqResp.h"

#define AUTOTUNE_AXIS_BITMASK_ROLL            1
#define AUTOTUNE_AXIS_BITMASK_PITCH           2
#define AUTOTUNE_AXIS_BITMASK_YAW             4
#define AUTOTUNE_AXIS_BITMASK_YAW_D           8

#define AUTOTUNE_SUCCESS_COUNT                4     // The number of successful iterations we need to freeze at current gains

// Auto Tune message ids for ground station
#define AUTOTUNE_MESSAGE_STARTED 0
#define AUTOTUNE_MESSAGE_STOPPED 1
#define AUTOTUNE_MESSAGE_SUCCESS 2
#define AUTOTUNE_MESSAGE_FAILED 3
#define AUTOTUNE_MESSAGE_SAVED_GAINS 4
#define AUTOTUNE_MESSAGE_TESTING 5
#define AUTOTUNE_MESSAGE_TESTING_END 6

#define AUTOTUNE_ANNOUNCE_INTERVAL_MS 2000

class AC_AutoTune
{
public:
    // constructor
    AC_AutoTune();

    // Main update loop for Autotune mode. Handles all states: tuning, testing, or idle.
    // Should be called at >=100Hz for reliable performance.
    virtual void run();

    // Possibly save gains, called on disarm
    void disarmed(const bool in_autotune_mode);

    // stop tune, reverting gains
    void stop();

    // Autotune aux function trigger
    void do_aux_function(const RC_Channel::AuxSwitchPos ch_flag);

protected:

    virtual void save_tuning_gains() = 0;


    // reset Autotune so that gains are not saved again and autotune can be run again.
    void reset() {
        mode = TuneMode::UNINITIALISED;
        axes_completed = 0;
        testing_switch_used = false;
    }

    // axis that can be tuned
    enum class AxisType {
        ROLL = 0,                 // roll axis is being tuned (either angle or rate)
        PITCH = 1,                // pitch axis is being tuned (either angle or rate)
        YAW = 2,                  // yaw axis is being tuned using FLTE (either angle or rate)
        YAW_D = 3,                // yaw axis is being tuned using D (either angle or rate)
    };

    //
    // methods that must be supplied by the vehicle specific subclass
    //
    virtual bool init(void) = 0;

    // get pilot input for desired climb rate
    virtual float get_desired_climb_rate_ms(void) const = 0;

    // get pilot input for designed roll and pitch, and yaw rate
    virtual void get_pilot_desired_rp_yrate_rad(float &roll_rad, float &pitch_rad, float &yaw_rate_rads) = 0;

    // init pos controller Z velocity and accel limits
    virtual void init_z_limits() = 0;

#if HAL_LOGGING_ENABLED
    // log PIDs at full rate for during test
    virtual void log_pids() = 0;
#endif

    //
    // methods to load and save gains
    //

    // backup original gains and prepare for start of tuning
    virtual void backup_gains_and_initialise();

    // switch to use original gains
    virtual void load_orig_gains() = 0;

    // switch to gains found by last successful autotune
    virtual void load_tuned_gains() = 0;

    // load gains used between tests. called during testing mode's update-gains step to set gains ahead of return-to-level step
    virtual void load_intra_test_gains() = 0;

    // load gains for next test.  relies on axis variable being set
    virtual void load_test_gains() = 0;

    // reset the test variables for each vehicle
    virtual void reset_vehicle_test_variables() = 0;

    // reset the update gain variables for each vehicle
    virtual void reset_update_gain_variables() = 0;

    // test initialization and run methods that should be overridden for each vehicle
    virtual void test_init() = 0;
    virtual void test_run(AxisType test_axis, const float dir_sign) = 0;

    // return true if user has enabled autotune for roll, pitch or yaw axis
    bool roll_enabled() const;
    bool pitch_enabled() const;
    bool yaw_enabled() const;
    bool yaw_d_enabled() const;

    // update gains for the rate p up tune type
    virtual void updating_rate_p_up_all(AxisType test_axis)=0;

    // update gains for the rate d up tune type
    virtual void updating_rate_d_up_all(AxisType test_axis)=0;

    // update gains for the rate d down tune type
    virtual void updating_rate_d_down_all(AxisType test_axis)=0;

    // update gains for the angle p up tune type
    virtual void updating_angle_p_up_all(AxisType test_axis)=0;

    // update gains for the angle p down tune type
    virtual void updating_angle_p_down_all(AxisType test_axis)=0;

    // set gains post tune for the tune type
    virtual void set_tuning_gains_with_backoff(AxisType test_axis)=0;

    // reverse the direction of the next test
    virtual bool reverse_test_direction() = 0;


#if HAL_LOGGING_ENABLED
    virtual void Log_AutoTune() = 0;
    virtual void Log_AutoTuneDetails() = 0;
    virtual void Log_AutoTuneSweep() = 0;
#endif

    // internal init function, should be called from init()
    bool init_internals(bool use_poshold,
                        AC_AttitudeControl *attitude_control,
                        AC_PosControl *pos_control,
                        AP_AHRS_View *ahrs_view);

    // send intermittent updates to user on status of tune
    virtual void do_gcs_announcements() = 0;

    // send post test updates to user
    virtual void do_post_test_gcs_announcements() = 0;

    // send message with high level status (e.g. Started, Stopped)
    void update_gcs(uint8_t message_id) const;

    // send lower level step status (e.g. Pilot overrides Active)
    void send_step_string();

    // convert tune type to string for reporting
    const char *get_tune_type_name() const;

    // return current axis string
    const char *get_axis_name() const;

    // report final gains for a given axis to GCS
    virtual void report_final_gains(AxisType test_axis) const = 0;

    // Functions added for heli autotune

    // Add additional updating gain functions specific to heli
    // generic method used by subclasses to update gains for the rate ff up tune type
    virtual void updating_rate_ff_up_all(AxisType test_axis)=0;

    // generic method used by subclasses to update gains for the max gain tune type
    virtual void updating_max_gains_all(AxisType test_axis)=0;

    // steps performed while in the tuning mode
    enum class Step {
        WAITING_FOR_LEVEL   = 0,    // Waiting for the vehicle to stabilize at level before starting a test.
        EXECUTING_TEST      = 1,    // Performing a test and monitoring the vehicle's response.
        UPDATE_GAINS        = 2,    // Updating gains based on test results.
        ABORT               = 3     // Aborting the current test; revert to safe gains and return to WAITING_FOR_LEVEL.
    };
    Step step;              // see StepType for what steps are performed

    // mini steps performed while in Tuning mode, Testing step
    enum class TuneType {
        RATE_D_UP = 0,      // rate D is being tuned up
        RATE_D_DOWN = 1,    // rate D is being tuned down
        RATE_P_UP = 2,      // rate P is being tuned up
        RATE_FF_UP = 3,     // rate FF is being tuned up
        ANGLE_P_DOWN = 4,   // angle P is being tuned down
        ANGLE_P_UP = 5,     // angle P is being tuned up
        MAX_GAINS = 6,      // max allowable stable gains are determined
        TUNE_CHECK = 7,     // frequency sweep with tuned gains
        TUNE_COMPLETE = 8   // Reached end of tuning
    };
    TuneType tune_type;     // see TuneType
    TuneType tune_seq[6];   // holds sequence of tune_types to be performed
    uint8_t tune_seq_index; // current tune sequence step

    // get the next tune type
    void next_tune_type(TuneType &curr_tune_type, bool reset);

    // Sets customizable tune sequence for the vehicle
    virtual void set_tune_sequence() = 0;

    // get_axis_bitmask accessor
    virtual uint8_t get_axis_bitmask() const = 0;

    // get_testing_step_timeout_ms accessor
    virtual uint32_t get_testing_step_timeout_ms() const = 0;

    // get attitude for slow position hold in autotune mode
    void get_poshold_attitude_rad(float &roll_rad, float &pitch_rad, float &yaw_rad);

    // type of gains to load
    enum class GainType {
        ORIGINAL   = 0, // Gains as configured before autotune started
        TEST       = 1, // Gains applied during an active test
        INTRA_TEST = 2, // Gains applied between tests to maintain safe control while returning to level, with slower I-term buildup
        TUNED      = 3, // Gains discovered by the autotune process, used for flight testing or final use
    } loaded_gains;
    void load_gains(enum GainType gain_type);

    // TuneMode defines the high-level state of the autotune process.
    enum class TuneMode {
        UNINITIALISED = 0,  // Autotune has not yet been started.
        TUNING = 1,         // Autotune is actively running and tuning PID gains.
        FINISHED = 2,       // Tuning is complete, original (pre-tune) gains are restored.
        FAILED = 3,         // Tuning failed, vehicle is flying with original gains.
        VALIDATING = 4,     // Tuning complete, user is flight testing the newly tuned gains.
    };
    TuneMode mode;                       // see TuneMode for what modes are allowed

    // copies of object pointers to make code a bit clearer
    AC_AttitudeControl *attitude_control;
    AC_PosControl *pos_control;
    AP_AHRS_View *ahrs_view;
    AP_Motors *motors;

    AxisType axis;                  // current axis being tuned. see AxisType enum
    bool     positive_direction;    // false = tuning in negative direction (i.e. left for roll), true = positive direction (i.e. right for roll)
    bool     angle_step_commanded;  // true on first iteration of the tests (used to signal we must step the attitude or rate target)
    uint8_t  axes_completed;        // bitmask of completed axes
    uint32_t step_start_time_ms;    // start time of current tuning step (used for timeout checks)
    uint32_t step_timeout_ms;       // time limit of current autotune process
    uint32_t level_start_time_ms;   // start time of waiting for level
    int8_t   success_counter;       // counter for tuning gains
    float    start_angle;           // start angle
    float    start_rate;            // start rate - parent and multi
    float    test_accel_max_cdss;   // maximum acceleration variable
    float    step_scaler;           // scaler to reduce maximum target step - parent and multi

    LowPassFilterFloat  rotation_rate_filt; // filtered rotation rate in radians/second

    // backup of currently being tuned parameter values
    float    orig_roll_rp, orig_roll_ri, orig_roll_rd, orig_roll_rff, orig_roll_dff, orig_roll_fltt, orig_roll_smax, orig_roll_sp, orig_roll_accel_radss, orig_roll_rate_rads;
    float    orig_pitch_rp, orig_pitch_ri, orig_pitch_rd, orig_pitch_rff, orig_pitch_dff, orig_pitch_fltt, orig_pitch_smax, orig_pitch_sp, orig_pitch_accel_radss, orig_pitch_rate_rads;
    float    orig_yaw_rp, orig_yaw_ri, orig_yaw_rd, orig_yaw_rff, orig_yaw_dff, orig_yaw_fltt, orig_yaw_smax, orig_yaw_rLPF, orig_yaw_sp, orig_yaw_accel_radss, orig_yaw_rate_rads;
    bool     orig_bf_feedforward;

    // currently being tuned parameter values
    float    tune_roll_rp, tune_roll_rd, tune_roll_sp, tune_roll_accel_radss;
    float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel_radss;
    float    tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, tune_yaw_accel_radss;
    float    tune_roll_rff, tune_pitch_rff, tune_yaw_rd, tune_yaw_rff;

    uint32_t last_announce_ms;
    float   lean_angle;
    float   rotation_rate;
    float   desired_roll_rad, desired_pitch_rad, desired_yaw_rad;  // desired attitude target setpoints and test origins

    // heli specific variables
    float    start_freq;    //start freq for dwell test
    float    stop_freq;     //ending freq for dwell test

private:
    // return true if we have a good position estimate
    virtual bool position_ok();

    // methods subclasses must implement to specify max/min test angles:
    virtual float target_angle_max_rp_cd() const = 0;

    // methods subclasses must implement to specify max/min test angles:
    virtual float target_angle_max_y_cd() const = 0;

    // methods subclasses must implement to specify max/min test angles:
    virtual float target_angle_min_rp_cd() const = 0;

    // methods subclasses must implement to specify max/min test angles:
    virtual float target_angle_min_y_cd() const = 0;

    // methods subclasses must implement to specify max/min test angles:
    virtual float angle_lim_max_rp_cd() const = 0;

    // methods subclasses must implement to specify max/min test angles:
    virtual float angle_lim_neg_rpy_cd() const = 0;

    // initialise position controller
    bool init_position_controller();

    // Main tuning state machine. Handles WAITING_FOR_LEVEL, EXECUTING_TEST, UPDATE_GAINS, ABORT.
    // Updates attitude controller targets and evaluates test responses to tune gains.
    void control_attitude();

    // returns true if vehicle is close to level
    bool currently_level();

    bool     pilot_override;        // true = pilot is overriding controls so we suspend tuning temporarily
    bool     use_poshold;           // true = enable position hold
    bool     have_position;         // true = start_position is value
    Vector3p start_position_ned_m;  // target when holding position as an offset from EKF origin in meters in NED frame

    // variables
    uint32_t override_time;         // the last time the pilot overrode the controls

    // time in ms of last pilot override warning
    uint32_t last_pilot_override_warning;

    // True if we ever got a pilot testing command of tuned gains.
    // If true then disarming will save if the tuned gains are currently active.
    bool testing_switch_used;

};

#endif  // AC_AUTOTUNE_ENABLED
