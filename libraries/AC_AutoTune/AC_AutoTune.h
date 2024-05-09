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

#define AUTOTUNE_ANNOUNCE_INTERVAL_MS 2000

#define AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD 1000    // minimum target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_ANGLE_RLLPIT_CD     2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_ANGLE_YAW_CD        3000    // target angle during TESTING_RATE step that will cause us to move to next step

class AC_AutoTune
{
public:
    // constructor
    AC_AutoTune();

    // main run loop
    virtual void run();

    // save gained, called on disarm
    virtual void save_tuning_gains() = 0;

    // stop tune, reverting gains
    void stop();

    // reset Autotune so that gains are not saved again and autotune can be run again.
    void reset() {
        mode = UNINITIALISED;
        axes_completed = 0;
    }

protected:
    // axis that can be tuned
    enum AxisType {
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
    virtual float get_pilot_desired_climb_rate_cms(void) const = 0;

    // get pilot input for designed roll and pitch, and yaw rate
    virtual void get_pilot_desired_rp_yrate_cd(float &roll_cd, float &pitch_cd, float &yaw_rate_cds) = 0;

    // init pos controller Z velocity and accel limits
    virtual void init_z_limits() = 0;

#if HAL_LOGGING_ENABLED
    // log PIDs at full rate for during twitch
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

    // reset the test vaariables for each vehicle
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
    virtual void set_gains_post_tune(AxisType test_axis)=0;

    // reverse direction for twitch test
    virtual bool twitch_reverse_direction() = 0;


#if HAL_LOGGING_ENABLED
    virtual void Log_AutoTune() = 0;
    virtual void Log_AutoTuneDetails() = 0;
    virtual void Log_AutoTuneSweep() = 0;
#endif

    // internal init function, should be called from init()
    bool init_internals(bool use_poshold,
                        AC_AttitudeControl *attitude_control,
                        AC_PosControl *pos_control,
                        AP_AHRS_View *ahrs_view,
                        AP_InertialNav *inertial_nav);

    // send intermittent updates to user on status of tune
    virtual void do_gcs_announcements() = 0;

    // send post test updates to user
    virtual void do_post_test_gcs_announcements() = 0;

    // send message with high level status (e.g. Started, Stopped)
    void update_gcs(uint8_t message_id) const;

    // send lower level step status (e.g. Pilot overrides Active)
    void send_step_string();

    // convert tune type to string for reporting
    const char *type_string() const;

    // return current axis string
    const char *axis_string() const;

    // report final gains for a given axis to GCS
    virtual void report_final_gains(AxisType test_axis) const = 0;

    // Functions added for heli autotune

    // Add additional updating gain functions specific to heli
    // generic method used by subclasses to update gains for the rate ff up tune type
    virtual void updating_rate_ff_up_all(AxisType test_axis)=0;

    // generic method used by subclasses to update gains for the max gain tune type
    virtual void updating_max_gains_all(AxisType test_axis)=0;

    // steps performed while in the tuning mode
    enum StepType {
        WAITING_FOR_LEVEL = 0,    // autotune is waiting for vehicle to return to level before beginning the next twitch
        TESTING           = 1,    // autotune has begun a test and is watching the resulting vehicle movement
        UPDATE_GAINS      = 2     // autotune has completed a test and is updating the gains based on the results
    };

    // mini steps performed while in Tuning mode, Testing step
    enum TuneType {
        RD_UP = 0,                // rate D is being tuned up
        RD_DOWN = 1,              // rate D is being tuned down
        RP_UP = 2,                // rate P is being tuned up
        RFF_UP = 3,               // rate FF is being tuned up
        SP_UP = 4,                // angle P is being tuned up
        SP_DOWN = 5,              // angle P is being tuned down
        MAX_GAINS = 6,            // max allowable stable gains are determined
        TUNE_CHECK = 7,           // frequency sweep with tuned gains
        TUNE_COMPLETE = 8         // Reached end of tuning
    };
    TuneType tune_seq[6];         // holds sequence of tune_types to be performed
    uint8_t tune_seq_curr;        // current tune sequence step

    // get the next tune type
    void next_tune_type(TuneType &curr_tune_type, bool reset);

    // Sets customizable tune sequence for the vehicle
    virtual void set_tune_sequence() = 0;

    // get_axis_bitmask accessor
    virtual uint8_t get_axis_bitmask() const = 0;

    // get_testing_step_timeout_ms accessor
    virtual uint32_t get_testing_step_timeout_ms() const = 0;

    // get attitude for slow position hold in autotune mode
    void get_poshold_attitude(float &roll_cd, float &pitch_cd, float &yaw_cd);

    // type of gains to load
    enum GainType {
        GAIN_ORIGINAL   = 0,
        GAIN_TEST       = 1,
        GAIN_INTRA_TEST = 2,
        GAIN_TUNED      = 3,
    };
    void load_gains(enum GainType gain_type);

    // autotune modes (high level states)
    enum TuneMode {
        UNINITIALISED = 0,        // autotune has never been run
        TUNING = 1,               // autotune is testing gains
        SUCCESS = 2,              // tuning has completed, user is flight testing the new gains
        FAILED = 3,               // tuning has failed, user is flying on original gains
    };
    TuneMode mode;                       // see TuneMode for what modes are allowed

    // copies of object pointers to make code a bit clearer
    AC_AttitudeControl *attitude_control;
    AC_PosControl *pos_control;
    AP_AHRS_View *ahrs_view;
    AP_InertialNav *inertial_nav;
    AP_Motors *motors;

    AxisType axis;                       // current axis being tuned. see AxisType enum
    bool     positive_direction;         // false = tuning in negative direction (i.e. left for roll), true = positive direction (i.e. right for roll)
    StepType step;                       // see StepType for what steps are performed
    TuneType tune_type;                  // see TuneType
    bool     ignore_next;                // true = ignore the next test
    bool     twitch_first_iter;          // true on first iteration of a twitch (used to signal we must step the attitude or rate target)
    uint8_t  axes_completed;             // bitmask of completed axes
    float    test_rate_min;                         // the minimum angular rate achieved during TESTING_RATE step-multi only
    float    test_rate_max;                         // the maximum angular rate achieved during TESTING_RATE step-multi only
    float    test_angle_min;                        // the minimum angle achieved during TESTING_ANGLE step-multi only
    float    test_angle_max;                        // the maximum angle achieved during TESTING_ANGLE step-multi only
    uint32_t step_start_time_ms;                    // start time of current tuning step (used for timeout checks)
    uint32_t step_time_limit_ms;                    // time limit of current autotune process
    int8_t   counter;                               // counter for tuning gains
    float    target_rate;                           // target rate-multi only
    float    target_angle;                          // target angle-multi only
    float    start_rate;                            // start rate - parent and multi
    float    start_angle;                           // start angle
    float    rate_max;                              // maximum rate variable - parent and multi
    float    test_accel_max;                        // maximum acceleration variable
    float    step_scaler;                           // scaler to reduce maximum target step - parent and multi
    float    abort_angle;                           // Angle that test is aborted- parent and multi
    float    desired_yaw_cd;                        // yaw heading during tune - parent and Tradheli

    LowPassFilterFloat  rotation_rate_filt;         // filtered rotation rate in radians/second

    // backup of currently being tuned parameter values
    float    orig_roll_rp, orig_roll_ri, orig_roll_rd, orig_roll_rff, orig_roll_dff, orig_roll_fltt, orig_roll_smax, orig_roll_sp, orig_roll_accel;
    float    orig_pitch_rp, orig_pitch_ri, orig_pitch_rd, orig_pitch_rff, orig_pitch_dff, orig_pitch_fltt, orig_pitch_smax, orig_pitch_sp, orig_pitch_accel;
    float    orig_yaw_rp, orig_yaw_ri, orig_yaw_rd, orig_yaw_rff, orig_yaw_dff, orig_yaw_fltt, orig_yaw_smax, orig_yaw_rLPF, orig_yaw_sp, orig_yaw_accel;
    bool     orig_bf_feedforward;

    // currently being tuned parameter values
    float    tune_roll_rp, tune_roll_rd, tune_roll_sp, tune_roll_accel;
    float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel;
    float    tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, tune_yaw_accel;
    float    tune_roll_rff, tune_pitch_rff, tune_yaw_rd, tune_yaw_rff;

    uint32_t announce_time;
    float lean_angle;
    float rotation_rate;
    float roll_cd, pitch_cd;

    // heli specific variables
    uint8_t  freq_cnt;                              // dwell test iteration counter
    float    start_freq;                            //start freq for dwell test
    float    stop_freq;                             //ending freq for dwell test
    bool     ff_up_first_iter;                      // true on first iteration of ff up testing

private:
    // return true if we have a good position estimate
    virtual bool position_ok();

    // initialise position controller
    bool init_position_controller();

    // main state machine to level vehicle, perform a test and update gains
    // directly updates attitude controller with targets
    void control_attitude();

    // returns true if vehicle is close to level
    bool currently_level();

    bool     pilot_override;             // true = pilot is overriding controls so we suspend tuning temporarily
    bool     use_poshold;                // true = enable position hold
    bool     have_position;              // true = start_position is value
    Vector3f start_position;             // target when holding position as an offset from EKF origin in cm in NEU frame

    // variables
    uint32_t override_time;                         // the last time the pilot overrode the controls
    uint32_t level_start_time_ms;                   // start time of waiting for level
    uint32_t level_fail_warning_time_ms;            // last time level failure warning message was sent to GCS

    // time in ms of last pilot override warning
    uint32_t last_pilot_override_warning;

};

#endif  // AC_AUTOTUNE_ENABLED
