#include "Copter.h"

#if AUTOTUNE_ENABLED == ENABLED

/*
 * Init and run calls for autotune flight mode
 *
 * Instructions:
 *      1) Set up one flight mode switch position to be AltHold.
 *      2) Set the Ch7 Opt or Ch8 Opt to AutoTune to allow you to turn the auto tuning on/off with the ch7 or ch8 switch.
 *      3) Ensure the ch7 or ch8 switch is in the LOW position.
 *      4) Wait for a calm day and go to a large open area.
 *      5) Take off and put the vehicle into AltHold mode at a comfortable altitude.
 *      6) Set the ch7/ch8 switch to the HIGH position to engage auto tuning:
 *          a) You will see it twitch about 20 degrees left and right for a few minutes, then it will repeat forward and back.
 *          b) Use the roll and pitch stick at any time to reposition the copter if it drifts away (it will use the original PID gains during repositioning and between tests).
 *             When you release the sticks it will continue auto tuning where it left off.
 *          c) Move the ch7/ch8 switch into the LOW position at any time to abandon the autotuning and return to the origin PIDs.
 *          d) Make sure that you do not have any trim set on your transmitter or the autotune may not get the signal that the sticks are centered.
 *      7) When the tune completes the vehicle will change back to the original PID gains.
 *      8) Put the ch7/ch8 switch into the LOW position then back to the HIGH position to test the tuned PID gains.
 *      9) Put the ch7/ch8 switch into the LOW position to fly using the original PID gains.
 *      10) If you are happy with the autotuned PID gains, leave the ch7/ch8 switch in the HIGH position, land and disarm to save the PIDs permanently.
 *          If you DO NOT like the new PIDS, switch ch7/ch8 LOW to return to the original PIDs. The gains will not be saved when you disarm
 *
 * What it's doing during each "twitch":
 *      a) invokes 90 deg/sec rate request
 *      b) records maximum "forward" roll rate and bounce back rate
 *      c) when copter reaches 20 degrees or 1 second has passed, it commands level
 *      d) tries to keep max rotation rate between 80% ~ 100% of requested rate (90deg/sec) by adjusting rate P
 *      e) increases rate D until the bounce back becomes greater than 10% of requested rate (90deg/sec)
 *      f) decreases rate D until the bounce back becomes less than 10% of requested rate (90deg/sec)
 *      g) increases rate P until the max rotate rate becomes greater than the request rate (90deg/sec)
 *      h) invokes a 20deg angle request on roll or pitch
 *      i) increases stab P until the maximum angle becomes greater than 110% of the requested angle (20deg)
 *      j) decreases stab P by 25%
 *
 */

#define AUTOTUNE_AXIS_BITMASK_ROLL            1
#define AUTOTUNE_AXIS_BITMASK_PITCH           2
#define AUTOTUNE_AXIS_BITMASK_YAW             4

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500     // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   1000     // timeout for tuning mode's testing step
#define AUTOTUNE_LEVEL_ANGLE_CD             500     // angle which qualifies as level
#define AUTOTUNE_LEVEL_RATE_RP_CD          1000     // rate which qualifies as level for roll and pitch
#define AUTOTUNE_LEVEL_RATE_Y_CD            750     // rate which qualifies as level for yaw
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     500     // time we require the copter to be level
#define AUTOTUNE_RD_STEP                  0.05f     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.05f     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05f     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f     // I is set 10x smaller than P during testing
#define AUTOTUNE_PI_RATIO_FINAL            1.0f     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1f     // I is set 1x P after testing
#define AUTOTUNE_RD_MAX                  0.200f     // maximum Rate D value
#define AUTOTUNE_RLPF_MIN                  1.0f     // minimum Rate Yaw filter value
#define AUTOTUNE_RLPF_MAX                  5.0f     // maximum Rate Yaw filter value
#define AUTOTUNE_RP_MIN                   0.01f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                    2.0f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                   20.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    0.5f     // maximum Stab P value
#define AUTOTUNE_RP_ACCEL_MIN           4000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN            1000.0f     // Minimum acceleration for Yaw
#define AUTOTUNE_Y_FILT_FREQ              10.0f     // Autotune filter frequency when testing Yaw
#define AUTOTUNE_SUCCESS_COUNT                4     // The number of successful iterations we need to freeze at current gains
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in
#define AUTOTUNE_RD_BACKOFF                1.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_SP_BACKOFF                0.9f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0f     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF           1.0f     // back off from maximum acceleration

// roll and pitch axes
#define AUTOTUNE_TARGET_ANGLE_RLLPIT_CD     2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_RLLPIT_CDS     9000    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD 1000    // minimum target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS 4500    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step

// yaw axis
#define AUTOTUNE_TARGET_ANGLE_YAW_CD        3000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_YAW_CDS        9000    // target yaw rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD     500    // minimum target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_YAW_CDS    1500    // minimum target yaw rate during AUTOTUNE_STEP_TWITCHING step

// Auto Tune message ids for ground station
#define AUTOTUNE_MESSAGE_STARTED 0
#define AUTOTUNE_MESSAGE_STOPPED 1
#define AUTOTUNE_MESSAGE_SUCCESS 2
#define AUTOTUNE_MESSAGE_FAILED 3
#define AUTOTUNE_MESSAGE_SAVED_GAINS 4

#define AUTOTUNE_ANNOUNCE_INTERVAL_MS 2000

// autotune modes (high level states)
enum AutoTuneTuneMode {
    AUTOTUNE_MODE_UNINITIALISED = 0,        // autotune has never been run
    AUTOTUNE_MODE_TUNING = 1,               // autotune is testing gains
    AUTOTUNE_MODE_SUCCESS = 2,              // tuning has completed, user is flight testing the new gains
    AUTOTUNE_MODE_FAILED = 3,               // tuning has failed, user is flying on original gains
};

// steps performed while in the tuning mode
enum AutoTuneStepType {
    AUTOTUNE_STEP_WAITING_FOR_LEVEL = 0,    // autotune is waiting for vehicle to return to level before beginning the next twitch
    AUTOTUNE_STEP_TWITCHING = 1,            // autotune has begun a twitch and is watching the resulting vehicle movement
    AUTOTUNE_STEP_UPDATE_GAINS = 2          // autotune has completed a twitch and is updating the gains based on the results
};

// things that can be tuned
enum AutoTuneAxisType {
    AUTOTUNE_AXIS_ROLL = 0,                 // roll axis is being tuned (either angle or rate)
    AUTOTUNE_AXIS_PITCH = 1,                // pitch axis is being tuned (either angle or rate)
    AUTOTUNE_AXIS_YAW = 2,                  // pitch axis is being tuned (either angle or rate)
};

// mini steps performed while in Tuning mode, Testing step
enum AutoTuneTuneType {
    AUTOTUNE_TYPE_RD_UP = 0,                // rate D is being tuned up
    AUTOTUNE_TYPE_RD_DOWN = 1,              // rate D is being tuned down
    AUTOTUNE_TYPE_RP_UP = 2,                // rate P is being tuned up
    AUTOTUNE_TYPE_SP_DOWN = 3,              // angle P is being tuned down
    AUTOTUNE_TYPE_SP_UP = 4                 // angle P is being tuned up
};

// autotune_state_struct - hold state flags
static struct autotune_state_struct {
    AutoTuneTuneMode    mode                : 2;    // see AutoTuneTuneMode for what modes are allowed
    uint8_t             pilot_override      : 1;    // true = pilot is overriding controls so we suspend tuning temporarily
    AutoTuneAxisType    axis                : 2;    // see AutoTuneAxisType for which things can be tuned
    uint8_t             positive_direction  : 1;    // false = tuning in negative direction (i.e. left for roll), true = positive direction (i.e. right for roll)
    AutoTuneStepType    step                : 2;    // see AutoTuneStepType for what steps are performed
    AutoTuneTuneType    tune_type           : 3;    // see AutoTuneTuneType
    uint8_t             ignore_next         : 1;    // true = ignore the next test
    uint8_t             twitch_first_iter   : 1;    // true on first iteration of a twitch (used to signal we must step the attitude or rate target)
    bool                use_poshold         : 1;    // true = enable position hold
    bool                have_position       : 1;    // true = start_position is value
    Vector3f            start_position;
} autotune_state;

// variables
static uint32_t autotune_override_time;                         // the last time the pilot overrode the controls
static float    autotune_test_min;                              // the minimum angular rate achieved during TESTING_RATE step
static float    autotune_test_max;                              // the maximum angular rate achieved during TESTING_RATE step
static uint32_t autotune_step_start_time;                       // start time of current tuning step (used for timeout checks)
static uint32_t autotune_step_stop_time;                        // start time of current tuning step (used for timeout checks)
static int8_t   autotune_counter;                               // counter for tuning gains
static float    autotune_target_rate, autotune_start_rate;      // target and start rate
static float    autotune_target_angle, autotune_start_angle;    // target and start angles
static float    autotune_desired_yaw;                           // yaw heading during tune
static float    rate_max, autotune_test_accel_max;              // maximum acceleration variables

LowPassFilterFloat  rotation_rate_filt;                         // filtered rotation rate in radians/second

// backup of currently being tuned parameter values
static float    orig_roll_rp = 0, orig_roll_ri, orig_roll_rd, orig_roll_sp, orig_roll_accel;
static float    orig_pitch_rp = 0, orig_pitch_ri, orig_pitch_rd, orig_pitch_sp, orig_pitch_accel;
static float    orig_yaw_rp = 0, orig_yaw_ri, orig_yaw_rd, orig_yaw_rLPF, orig_yaw_sp, orig_yaw_accel;
static bool     orig_bf_feedforward;

// currently being tuned parameter values
static float    tune_roll_rp, tune_roll_rd, tune_roll_sp, tune_roll_accel;
static float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel;
static float    tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, tune_yaw_accel;

static uint32_t autotune_announce_time;
static float lean_angle;
static float rotation_rate;
static float autotune_roll_cd, autotune_pitch_cd;

static struct {
    Copter::AUTOTUNE_LEVEL_ISSUE issue{Copter::AUTOTUNE_LEVEL_ISSUE_NONE};
    float maximum;
    float current;
} autotune_level_problem;

// autotune_init - should be called when autotune mode is selected
bool Copter::autotune_init(bool ignore_checks)
{
    bool success = true;

    switch (autotune_state.mode) {
        case AUTOTUNE_MODE_FAILED:
            // autotune has been run but failed so reset state to uninitialized
            autotune_state.mode = AUTOTUNE_MODE_UNINITIALISED;
            // no break to allow fall through to restart the tuning

        case AUTOTUNE_MODE_UNINITIALISED:
            // autotune has never been run
            success = autotune_start(false);
            if (success) {
                // so store current gains as original gains
                autotune_backup_gains_and_initialise();
                // advance mode to tuning
                autotune_state.mode = AUTOTUNE_MODE_TUNING;
                // send message to ground station that we've started tuning
                autotune_update_gcs(AUTOTUNE_MESSAGE_STARTED);
            }
            break;

        case AUTOTUNE_MODE_TUNING:
            // we are restarting tuning after the user must have switched ch7/ch8 off so we restart tuning where we left off
            success = autotune_start(false);
            if (success) {
                // reset gains to tuning-start gains (i.e. low I term)
                autotune_load_intra_test_gains();
                // write dataflash log even and send message to ground station
                Log_Write_Event(DATA_AUTOTUNE_RESTART);
                autotune_update_gcs(AUTOTUNE_MESSAGE_STARTED);
            }
            break;

        case AUTOTUNE_MODE_SUCCESS:
            // we have completed a tune and the pilot wishes to test the new gains in the current flight mode
            // so simply apply tuning gains (i.e. do not change flight mode)
            autotune_load_tuned_gains();
            Log_Write_Event(DATA_AUTOTUNE_PILOT_TESTING);
            break;
    }

    // only do position hold if starting autotune from LOITER or POSHOLD
    autotune_state.use_poshold = (control_mode == LOITER || control_mode == POSHOLD);
    autotune_state.have_position = false;

    return success;
}

// autotune_stop - should be called when the ch7/ch8 switch is switched OFF
void Copter::autotune_stop()
{
    // set gains to their original values
    autotune_load_orig_gains();

    // re-enable angle-to-rate request limits
    attitude_control->use_ff_and_input_shaping(true);

    // log off event and send message to ground station
    autotune_update_gcs(AUTOTUNE_MESSAGE_STOPPED);
    Log_Write_Event(DATA_AUTOTUNE_OFF);

    // Note: we leave the autotune_state.mode as it was so that we know how the autotune ended
    // we expect the caller will change the flight mode back to the flight mode indicated by the flight mode switch
}

// autotune_start - Initialize autotune flight mode
bool Copter::autotune_start(bool ignore_checks)
{
    // only allow flip from Stabilize, AltHold,  PosHold or Loiter modes
    if (control_mode != STABILIZE && control_mode != ALT_HOLD &&
        control_mode != LOITER && control_mode != POSHOLD) {
        return false;
    }

    // ensure throttle is above zero
    if (ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    if (!motors->armed() || !ap.auto_armed || ap.land_complete) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

const char *Copter::autotune_level_issue_string() const
{
    switch (autotune_level_problem.issue) {
    case Copter::AUTOTUNE_LEVEL_ISSUE_NONE:
        return "None";
    case Copter::AUTOTUNE_LEVEL_ISSUE_ANGLE_ROLL:
        return "Angle(R)";
    case Copter::AUTOTUNE_LEVEL_ISSUE_ANGLE_PITCH:
        return "Angle(P)";
    case Copter::AUTOTUNE_LEVEL_ISSUE_ANGLE_YAW:
        return "Angle(Y)";
    case Copter::AUTOTUNE_LEVEL_ISSUE_RATE_ROLL:
        return "Rate(R)";
    case Copter::AUTOTUNE_LEVEL_ISSUE_RATE_PITCH:
        return "Rate(P)";
    case Copter::AUTOTUNE_LEVEL_ISSUE_RATE_YAW:
        return "Rate(Y)";
    }
    return "Bug";
}

void Copter::autotune_send_step_string()
{
    if (autotune_state.pilot_override) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: Paused: Pilot Override Active");
        return;
    }
    switch (autotune_state.step) {
    case AUTOTUNE_STEP_WAITING_FOR_LEVEL:
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: WFL (%s) (%f > %f)", autotune_level_issue_string(), (double)(autotune_level_problem.current*0.01f), (double)(autotune_level_problem.maximum*0.01f));
        return;
    case AUTOTUNE_STEP_UPDATE_GAINS:
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: UPDATING_GAINS");
        return;
    case AUTOTUNE_STEP_TWITCHING:
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: TWITCHING");
        return;
    }
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: unknown step");
}

const char *Copter::autotune_type_string() const
{
    switch (autotune_state.tune_type) {
    case AUTOTUNE_TYPE_RD_UP:
        return "Rate D Up";
    case AUTOTUNE_TYPE_RD_DOWN:
        return "Rate D Down";
    case AUTOTUNE_TYPE_RP_UP:
        return "Rate P Up";
    case AUTOTUNE_TYPE_SP_DOWN:
        return "Angle P Down";
    case AUTOTUNE_TYPE_SP_UP:
        return "Angle P Up";
    }
    return "Bug";
}

void Copter::autotune_do_gcs_announcements()
{
    const uint32_t now = millis();
    if (now - autotune_announce_time < AUTOTUNE_ANNOUNCE_INTERVAL_MS) {
        return;
    }
    float tune_rp = 0.0f;
    float tune_rd = 0.0f;
    float tune_sp = 0.0f;
    float tune_accel = 0.0f;
    char axis = '?';
    switch (autotune_state.axis) {
    case AUTOTUNE_AXIS_ROLL:
        tune_rp = tune_roll_rp;
        tune_rd = tune_roll_rd;
        tune_sp = tune_roll_sp;
        tune_accel = tune_roll_accel;
        axis = 'R';
        break;
    case AUTOTUNE_AXIS_PITCH:
        tune_rp = tune_pitch_rp;
        tune_rd = tune_pitch_rd;
        tune_sp = tune_pitch_sp;
        tune_accel = tune_pitch_accel;
        axis = 'P';
        break;
    case AUTOTUNE_AXIS_YAW:
        tune_rp = tune_yaw_rp;
        tune_rd = tune_yaw_rLPF;
        tune_sp = tune_yaw_sp;
        tune_accel = tune_yaw_accel;
        axis = 'Y';
        break;
    }

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: (%c) %s", axis, autotune_type_string());
    autotune_send_step_string();
    if (!is_zero(lean_angle)) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: lean=%f target=%f", (double)lean_angle, (double)autotune_target_angle);
    }
    if (!is_zero(rotation_rate)) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: rotation=%f target=%f", (double)(rotation_rate*0.01f), (double)(autotune_target_rate*0.01f));
    }
    switch (autotune_state.tune_type) {
    case AUTOTUNE_TYPE_RD_UP:
    case AUTOTUNE_TYPE_RD_DOWN:
    case AUTOTUNE_TYPE_RP_UP:
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: p=%f d=%f", (double)tune_rp, (double)tune_rd);
        break;
    case AUTOTUNE_TYPE_SP_DOWN:
    case AUTOTUNE_TYPE_SP_UP:
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: p=%f accel=%f", (double)tune_sp, (double)tune_accel);
        break;
    }
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: success %u/%u", autotune_counter, AUTOTUNE_SUCCESS_COUNT);

    autotune_announce_time = now;
}

// autotune_run - runs the autotune flight mode
// should be called at 100hz or more
void Copter::autotune_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t target_climb_rate;

    // tell the user what's going on
    autotune_do_gcs_announcements();

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    // this should not actually be possible because of the autotune_init() checks
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // check for pilot requested take-off - this should not actually be possible because of autotune_init() checks
    if (ap.land_complete && target_climb_rate > 0) {
        // indicate we are taking off
        set_land_complete(false);
        // clear i term when we're taking off
        set_throttle_takeoff();
    }

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);
        pos_control->update_z_controller();
    }else{
        // check if pilot is overriding the controls
        bool zero_rp_input = is_zero(target_roll) && is_zero(target_pitch);
        if (!zero_rp_input || !is_zero(target_yaw_rate) || target_climb_rate != 0) {
            if (!autotune_state.pilot_override) {
                autotune_state.pilot_override = true;
                // set gains to their original values
                autotune_load_orig_gains();
                attitude_control->use_ff_and_input_shaping(true);
            }
            // reset pilot override time
            autotune_override_time = millis();
            if (!zero_rp_input) {
                // only reset position on roll or pitch input
                autotune_state.have_position = false;
            }
        }else if (autotune_state.pilot_override) {
            // check if we should resume tuning after pilot's override
            if (millis() - autotune_override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
                autotune_state.pilot_override = false;             // turn off pilot override
                // set gains to their intra-test values (which are very close to the original gains)
                // autotune_load_intra_test_gains(); //I think we should be keeping the originals here to let the I term settle quickly
                autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL; // set tuning step back from beginning
                autotune_desired_yaw = ahrs.yaw_sensor;
            }
        }

        if (zero_rp_input) {
            // pilot input on throttle and yaw will still use position hold if enabled
            autotune_get_poshold_attitude(target_roll, target_pitch, autotune_desired_yaw);
        }

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // if pilot override call attitude controller
        if (autotune_state.pilot_override || autotune_state.mode != AUTOTUNE_MODE_TUNING) {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        }else{
            // somehow get attitude requests from autotuning
            autotune_attitude_control();
        }

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
    }
}

bool Copter::autotune_check_level(const Copter::AUTOTUNE_LEVEL_ISSUE issue, const float current, const float maximum) const
{
    if (current > maximum) {
        autotune_level_problem.current = current;
        autotune_level_problem.maximum = maximum;
        autotune_level_problem.issue = issue;
        return false;
    }
    return true;
}

bool Copter::autotune_currently_level()
{
    if (!autotune_check_level(Copter::AUTOTUNE_LEVEL_ISSUE_ANGLE_ROLL,
                              labs(ahrs.roll_sensor - autotune_roll_cd),
                              AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }

    if (!autotune_check_level(Copter::AUTOTUNE_LEVEL_ISSUE_ANGLE_PITCH,
                              labs(ahrs.pitch_sensor - autotune_pitch_cd),
                              AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }
    if (!autotune_check_level(Copter::AUTOTUNE_LEVEL_ISSUE_ANGLE_YAW,
                              labs(wrap_180_cd(ahrs.yaw_sensor-(int32_t)autotune_desired_yaw)),
                              AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }
    if (!autotune_check_level(Copter::AUTOTUNE_LEVEL_ISSUE_RATE_ROLL,
                              (ToDeg(ahrs.get_gyro().x) * 100.0f),
                              AUTOTUNE_LEVEL_RATE_RP_CD)) {
        return false;
    }
    if (!autotune_check_level(Copter::AUTOTUNE_LEVEL_ISSUE_RATE_PITCH,
                              (ToDeg(ahrs.get_gyro().y) * 100.0f),
                              AUTOTUNE_LEVEL_RATE_RP_CD)) {
        return false;
    }
    if (!autotune_check_level(Copter::AUTOTUNE_LEVEL_ISSUE_RATE_YAW,
                              (ToDeg(ahrs.get_gyro().z) * 100.0f),
                              AUTOTUNE_LEVEL_RATE_Y_CD)) {
        return false;
    }
    return true;
}

// autotune_attitude_controller - sets attitude control targets during tuning
void Copter::autotune_attitude_control()
{
    rotation_rate = 0.0f;        // rotation rate in radians/second
    lean_angle = 0.0f;
    const float direction_sign = autotune_state.positive_direction ? 1.0f : -1.0f;

    // check tuning step
    switch (autotune_state.step) {

    case AUTOTUNE_STEP_WAITING_FOR_LEVEL:
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)
        // re-enable rate limits
        attitude_control->use_ff_and_input_shaping(true);

        autotune_get_poshold_attitude(autotune_roll_cd, autotune_pitch_cd, autotune_desired_yaw);
        
        // hold level attitude
        attitude_control->input_euler_angle_roll_pitch_yaw(autotune_roll_cd, autotune_pitch_cd, autotune_desired_yaw, true, get_smoothing_gain());

        // hold the copter level for 0.5 seconds before we begin a twitch
        // reset counter if we are no longer level
        if (!autotune_currently_level()) {
            autotune_step_start_time = millis();
        }

        // if we have been level for a sufficient amount of time (0.5 seconds) move onto tuning step
        if (millis() - autotune_step_start_time >= AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "AutoTune: Twitch");
            // initiate variables for next step
            autotune_state.step = AUTOTUNE_STEP_TWITCHING;
            autotune_step_start_time = millis();
            autotune_step_stop_time = autotune_step_start_time + AUTOTUNE_TESTING_STEP_TIMEOUT_MS;
            autotune_state.twitch_first_iter = true;
            autotune_test_max = 0.0f;
            autotune_test_min = 0.0f;
            rotation_rate_filt.reset(0.0f);
            rate_max = 0.0f;
            // set gains to their to-be-tested values
            autotune_load_twitch_gains();
        }

        switch (autotune_state.axis) {
        case AUTOTUNE_AXIS_ROLL:
            autotune_target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_roll())*100.0f, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            autotune_target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_roll())*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            autotune_start_rate = ToDeg(ahrs.get_gyro().x) * 100.0f;
            autotune_start_angle = ahrs.roll_sensor;
            rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_roll_pid().filt_hz()*2.0f);
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate_filt.reset(autotune_start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
        break;
        case AUTOTUNE_AXIS_PITCH:
            autotune_target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_pitch())*100.0f, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            autotune_target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_pitch())*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            autotune_start_rate = ToDeg(ahrs.get_gyro().y) * 100.0f;
            autotune_start_angle = ahrs.pitch_sensor;
            rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_pitch_pid().filt_hz()*2.0f);
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate_filt.reset(autotune_start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
            break;
        case AUTOTUNE_AXIS_YAW:
            autotune_target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_yaw()*0.75f)*100.0f, AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, AUTOTUNE_TARGET_RATE_YAW_CDS);
            autotune_target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_yaw()*0.75f)*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD, AUTOTUNE_TARGET_ANGLE_YAW_CD);
            autotune_start_rate = ToDeg(ahrs.get_gyro().z) * 100.0f;
            autotune_start_angle = ahrs.yaw_sensor;
            rotation_rate_filt.set_cutoff_frequency(AUTOTUNE_Y_FILT_FREQ);
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate_filt.reset(autotune_start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
            break;
        }
        break;

    case AUTOTUNE_STEP_TWITCHING:
        // Run the twitching step
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)

        // disable rate limits
        attitude_control->use_ff_and_input_shaping(false);
        // hold current attitude
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);

        if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
            // step angle targets on first iteration
            if (autotune_state.twitch_first_iter) {
                autotune_state.twitch_first_iter = false;
                // Testing increasing stabilize P gain so will set lean angle target
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    // request roll to 20deg
                    attitude_control->input_angle_step_bf_roll_pitch_yaw(direction_sign * autotune_target_angle, 0.0f, 0.0f);
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    // request pitch to 20deg
                    attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f, direction_sign * autotune_target_angle, 0.0f);
                    break;
                case AUTOTUNE_AXIS_YAW:
                    // request pitch to 20deg
                    attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f, 0.0f, direction_sign * autotune_target_angle);
                    break;
                }
            }
        } else {
            // Testing rate P and D gains so will set body-frame rate targets.
            // Rate controller will use existing body-frame rates and convert to motor outputs
            // for all axes except the one we override here.
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                // override body-frame roll rate
                attitude_control->rate_bf_roll_target(direction_sign * autotune_target_rate + autotune_start_rate);
                break;
            case AUTOTUNE_AXIS_PITCH:
                // override body-frame pitch rate
                attitude_control->rate_bf_pitch_target(direction_sign * autotune_target_rate + autotune_start_rate);
                break;
            case AUTOTUNE_AXIS_YAW:
                // override body-frame yaw rate
                attitude_control->rate_bf_yaw_target(direction_sign * autotune_target_rate + autotune_start_rate);
                break;
            }
        }

        // capture this iterations rotation rate and lean angle
        // Add filter to measurements
        switch (autotune_state.axis) {
        case AUTOTUNE_AXIS_ROLL:
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().x) * 100.0f), MAIN_LOOP_SECONDS);
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().x) * 100.0f - autotune_start_rate), MAIN_LOOP_SECONDS);
            }
            lean_angle = direction_sign * (ahrs.roll_sensor - (int32_t)autotune_start_angle);
            break;
        case AUTOTUNE_AXIS_PITCH:
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().y) * 100.0f), MAIN_LOOP_SECONDS);
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().y) * 100.0f - autotune_start_rate), MAIN_LOOP_SECONDS);
            }
            lean_angle = direction_sign * (ahrs.pitch_sensor - (int32_t)autotune_start_angle);
            break;
        case AUTOTUNE_AXIS_YAW:
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().z) * 100.0f), MAIN_LOOP_SECONDS);
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().z) * 100.0f - autotune_start_rate), MAIN_LOOP_SECONDS);
            }
            lean_angle = direction_sign * wrap_180_cd(ahrs.yaw_sensor-(int32_t)autotune_start_angle);
            break;
        }

        switch (autotune_state.tune_type) {
        case AUTOTUNE_TYPE_RD_UP:
        case AUTOTUNE_TYPE_RD_DOWN:
            autotune_twitching_test(rotation_rate, autotune_target_rate, autotune_test_min, autotune_test_max);
            autotune_twitching_measure_acceleration(autotune_test_accel_max, rotation_rate, rate_max);
            if (lean_angle >= autotune_target_angle) {
                autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
            }
            break;
        case AUTOTUNE_TYPE_RP_UP:
            autotune_twitching_test(rotation_rate, autotune_target_rate*(1+0.5f*g.autotune_aggressiveness), autotune_test_min, autotune_test_max);
            autotune_twitching_measure_acceleration(autotune_test_accel_max, rotation_rate, rate_max);
            if (lean_angle >= autotune_target_angle) {
                autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
            }
            break;
        case AUTOTUNE_TYPE_SP_DOWN:
        case AUTOTUNE_TYPE_SP_UP:
            autotune_twitching_test(lean_angle, autotune_target_angle*(1+0.5f*g.autotune_aggressiveness), autotune_test_min, autotune_test_max);
            autotune_twitching_measure_acceleration(autotune_test_accel_max, rotation_rate - direction_sign * autotune_start_rate, rate_max);
            break;
        }

        // log this iterations lean angle and rotation rate
        Log_Write_AutoTuneDetails(lean_angle, rotation_rate);
        DataFlash.Log_Write_Rate(ahrs, *motors, *attitude_control, *pos_control);
        break;

    case AUTOTUNE_STEP_UPDATE_GAINS:

        // re-enable rate limits
        attitude_control->use_ff_and_input_shaping(true);

        // log the latest gains
        if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_angle, autotune_test_min, autotune_test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, autotune_test_accel_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_angle, autotune_test_min, autotune_test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, autotune_test_accel_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_angle, autotune_test_min, autotune_test_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, autotune_test_accel_max);
                break;
            }
        } else {
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_rate, autotune_test_min, autotune_test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, autotune_test_accel_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_rate, autotune_test_min, autotune_test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, autotune_test_accel_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_rate, autotune_test_min, autotune_test_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, autotune_test_accel_max);
                break;
            }
        }

        // Check results after mini-step to increase rate D gain
        switch (autotune_state.tune_type) {
        case AUTOTUNE_TYPE_RD_UP:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_d_up(tune_roll_rd, g.autotune_min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_d_up(tune_pitch_rd, g.autotune_min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_d_up(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RLPF_MAX, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to decrease rate D gain
        case AUTOTUNE_TYPE_RD_DOWN:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_d_down(tune_roll_rd, g.autotune_min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_d_down(tune_pitch_rd, g.autotune_min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to increase rate P gain
        case AUTOTUNE_TYPE_RP_UP:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_p_up_d_down(tune_roll_rd, g.autotune_min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_p_up_d_down(tune_pitch_rd, g.autotune_min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_p_up_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case AUTOTUNE_TYPE_SP_DOWN:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_p_down(tune_roll_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_p_down(tune_pitch_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_p_down(tune_yaw_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case AUTOTUNE_TYPE_SP_UP:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_p_up(tune_roll_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_p_up(tune_pitch_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_p_up(tune_yaw_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            }
            break;
        }

        // we've complete this step, finalize pids and move to next step
        if (autotune_counter >= AUTOTUNE_SUCCESS_COUNT) {

            // reset counter
            autotune_counter = 0;

            // move to the next tuning type
            switch (autotune_state.tune_type) {
            case AUTOTUNE_TYPE_RD_UP:
                autotune_state.tune_type = AutoTuneTuneType(autotune_state.tune_type + 1);
                break;
            case AUTOTUNE_TYPE_RD_DOWN:
                autotune_state.tune_type = AutoTuneTuneType(autotune_state.tune_type + 1);
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    tune_roll_rd = MAX(g.autotune_min_d, tune_roll_rd * AUTOTUNE_RD_BACKOFF);
                    tune_roll_rp = MAX(AUTOTUNE_RP_MIN, tune_roll_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    tune_pitch_rd = MAX(g.autotune_min_d, tune_pitch_rd * AUTOTUNE_RD_BACKOFF);
                    tune_pitch_rp = MAX(AUTOTUNE_RP_MIN, tune_pitch_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_YAW:
                    tune_yaw_rLPF = MAX(AUTOTUNE_RLPF_MIN, tune_yaw_rLPF * AUTOTUNE_RD_BACKOFF);
                    tune_yaw_rp = MAX(AUTOTUNE_RP_MIN, tune_yaw_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                }
                break;
            case AUTOTUNE_TYPE_RP_UP:
                autotune_state.tune_type = AutoTuneTuneType(autotune_state.tune_type + 1);
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    tune_roll_rp = MAX(AUTOTUNE_RP_MIN, tune_roll_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    tune_pitch_rp = MAX(AUTOTUNE_RP_MIN, tune_pitch_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_YAW:
                    tune_yaw_rp = MAX(AUTOTUNE_RP_MIN, tune_yaw_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                }
                break;
            case AUTOTUNE_TYPE_SP_DOWN:
                autotune_state.tune_type = AutoTuneTuneType(autotune_state.tune_type + 1);
                break;
            case AUTOTUNE_TYPE_SP_UP:
                // we've reached the end of a D-up-down PI-up-down tune type cycle
                autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

                // advance to the next axis
                bool autotune_complete = false;
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    tune_roll_sp = MAX(AUTOTUNE_SP_MIN, tune_roll_sp * AUTOTUNE_SP_BACKOFF);
                    tune_roll_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, autotune_test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    if (autotune_pitch_enabled()) {
                        autotune_state.axis = AUTOTUNE_AXIS_PITCH;
                    } else if (autotune_yaw_enabled()) {
                        autotune_state.axis = AUTOTUNE_AXIS_YAW;
                    } else {
                        autotune_complete = true;
                    }
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    tune_pitch_sp = MAX(AUTOTUNE_SP_MIN, tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
                    tune_pitch_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, autotune_test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    if (autotune_yaw_enabled()) {
                        autotune_state.axis = AUTOTUNE_AXIS_YAW;
                    } else {
                        autotune_complete = true;
                    }
                    break;
                case AUTOTUNE_AXIS_YAW:
                    tune_yaw_sp = MAX(AUTOTUNE_SP_MIN, tune_yaw_sp * AUTOTUNE_SP_BACKOFF);
                    tune_yaw_accel = MAX(AUTOTUNE_Y_ACCEL_MIN, autotune_test_accel_max * AUTOTUNE_ACCEL_Y_BACKOFF);
                    autotune_complete = true;
                    break;
                }

                // if we've just completed all axes we have successfully completed the autotune
                    // change to TESTING mode to allow user to fly with new gains
                if (autotune_complete) {
                    autotune_state.mode = AUTOTUNE_MODE_SUCCESS;
                    autotune_update_gcs(AUTOTUNE_MESSAGE_SUCCESS);
                    Log_Write_Event(DATA_AUTOTUNE_SUCCESS);
                    AP_Notify::events.autotune_complete = 1;
                } else {
                    AP_Notify::events.autotune_next_axis = 1;
                }
                break;
            }
        }

        // reverse direction
        autotune_state.positive_direction = !autotune_state.positive_direction;

        if (autotune_state.axis == AUTOTUNE_AXIS_YAW) {
            attitude_control->input_euler_angle_roll_pitch_yaw(0.0f, 0.0f, ahrs.yaw_sensor, false, get_smoothing_gain());
        }

        // set gains to their intra-test values (which are very close to the original gains)
        autotune_load_intra_test_gains();

        // reset testing step
        autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
        autotune_step_start_time = millis();
        break;
    }
}

// autotune_backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
void Copter::autotune_backup_gains_and_initialise()
{
    // initialise state because this is our first time
    if (autotune_roll_enabled()) {
        autotune_state.axis = AUTOTUNE_AXIS_ROLL;
    } else if (autotune_pitch_enabled()) {
        autotune_state.axis = AUTOTUNE_AXIS_PITCH;
    } else if (autotune_yaw_enabled()) {
        autotune_state.axis = AUTOTUNE_AXIS_YAW;
    }
    autotune_state.positive_direction = false;
    autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
    autotune_step_start_time = millis();
    autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

    autotune_desired_yaw = ahrs.yaw_sensor;

    g.autotune_aggressiveness = constrain_float(g.autotune_aggressiveness, 0.05f, 0.2f);

    orig_bf_feedforward = attitude_control->get_bf_feedforward();

    // backup original pids and initialise tuned pid values
    orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
    orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
    orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
    orig_roll_sp = attitude_control->get_angle_roll_p().kP();
    orig_roll_accel = attitude_control->get_accel_roll_max();
    tune_roll_rp = attitude_control->get_rate_roll_pid().kP();
    tune_roll_rd = attitude_control->get_rate_roll_pid().kD();
    tune_roll_sp = attitude_control->get_angle_roll_p().kP();
    tune_roll_accel = attitude_control->get_accel_roll_max();

    orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
    orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    orig_pitch_accel = attitude_control->get_accel_pitch_max();
    tune_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    tune_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    tune_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    tune_pitch_accel = attitude_control->get_accel_pitch_max();

    orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
    orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
    orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_hz();
    orig_yaw_accel = attitude_control->get_accel_yaw_max();
    orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    tune_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_hz();
    tune_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_accel = attitude_control->get_accel_yaw_max();

    Log_Write_Event(DATA_AUTOTUNE_INITIALISED);
}

// autotune_load_orig_gains - set gains to their original values
//  called by autotune_stop and autotune_failed functions
void Copter::autotune_load_orig_gains()
{
    attitude_control->bf_feedforward(orig_bf_feedforward);
    if (autotune_roll_enabled()) {
        if (!is_zero(orig_roll_rp)) {
            attitude_control->get_rate_roll_pid().kP(orig_roll_rp);
            attitude_control->get_rate_roll_pid().kI(orig_roll_ri);
            attitude_control->get_rate_roll_pid().kD(orig_roll_rd);
            attitude_control->get_angle_roll_p().kP(orig_roll_sp);
            attitude_control->set_accel_roll_max(orig_roll_accel);
        }
    }
    if (autotune_pitch_enabled()) {
        if (!is_zero(orig_pitch_rp)) {
            attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(orig_pitch_ri);
            attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
            attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
            attitude_control->set_accel_pitch_max(orig_pitch_accel);
        }
    }
    if (autotune_yaw_enabled()) {
        if (!is_zero(orig_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(orig_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(orig_yaw_ri);
            attitude_control->get_rate_yaw_pid().kD(orig_yaw_rd);
            attitude_control->get_rate_yaw_pid().filt_hz(orig_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(orig_yaw_sp);
            attitude_control->set_accel_yaw_max(orig_yaw_accel);
        }
    }
}

// autotune_load_tuned_gains - load tuned gains
void Copter::autotune_load_tuned_gains()
{
    if (!attitude_control->get_bf_feedforward()) {
        attitude_control->bf_feedforward(true);
        attitude_control->set_accel_roll_max(0.0f);
        attitude_control->set_accel_pitch_max(0.0f);
    }
    if (autotune_roll_enabled()) {
        if (!is_zero(tune_roll_rp)) {
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            attitude_control->set_accel_roll_max(tune_roll_accel);
        }
    }
    if (autotune_pitch_enabled()) {
        if (!is_zero(tune_pitch_rp)) {
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            attitude_control->set_accel_pitch_max(tune_pitch_accel);
        }
    }
    if (autotune_yaw_enabled()) {
        if (!is_zero(tune_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            attitude_control->get_rate_yaw_pid().kD(0.0f);
            attitude_control->get_rate_yaw_pid().filt_hz(tune_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
            attitude_control->set_accel_yaw_max(tune_yaw_accel);
        }
    }
}

// autotune_load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
void Copter::autotune_load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the gains
    attitude_control->bf_feedforward(true);
    if (autotune_roll_enabled()) {
        attitude_control->get_rate_roll_pid().kP(orig_roll_rp);
        attitude_control->get_rate_roll_pid().kI(orig_roll_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_roll_pid().kD(orig_roll_rd);
        attitude_control->get_angle_roll_p().kP(orig_roll_sp);
    }
    if (autotune_pitch_enabled()) {
        attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(orig_pitch_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
        attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
    }
    if (autotune_yaw_enabled()) {
        attitude_control->get_rate_yaw_pid().kP(orig_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_yaw_pid().kD(orig_yaw_rd);
        attitude_control->get_rate_yaw_pid().filt_hz(orig_yaw_rLPF);
        attitude_control->get_angle_yaw_p().kP(orig_yaw_sp);
    }
}

// autotune_load_twitch_gains - load the to-be-tested gains for a single axis
// called by autotune_attitude_control() just before it beings testing a gain (i.e. just before it twitches)
void Copter::autotune_load_twitch_gains()
{
    switch (autotune_state.axis) {
        case AUTOTUNE_AXIS_ROLL:
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kI(tune_roll_rp*0.01f);
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            break;
        case AUTOTUNE_AXIS_PITCH:
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*0.01f);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            break;
        case AUTOTUNE_AXIS_YAW:
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*0.01f);
            attitude_control->get_rate_yaw_pid().kD(0.0f);
            attitude_control->get_rate_yaw_pid().filt_hz(tune_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
            break;
    }
}

// autotune_save_tuning_gains - save the final tuned gains for each axis
// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
void Copter::autotune_save_tuning_gains()
{
    // if we successfully completed tuning
    if (autotune_state.mode == AUTOTUNE_MODE_SUCCESS) {

        if (!attitude_control->get_bf_feedforward()) {
            attitude_control->bf_feedforward_save(true);
            attitude_control->save_accel_roll_max(0.0f);
            attitude_control->save_accel_pitch_max(0.0f);
        }

        // sanity check the rate P values
        if (autotune_roll_enabled() && !is_zero(tune_roll_rp)) {
            // rate roll gains
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
            attitude_control->get_rate_roll_pid().save_gains();

            // stabilize roll
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            attitude_control->get_angle_roll_p().save_gains();

            // acceleration roll
            attitude_control->save_accel_roll_max(tune_roll_accel);

            // resave pids to originals in case the autotune is run again
            orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
            orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
            orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
            orig_roll_sp = attitude_control->get_angle_roll_p().kP();
            orig_roll_accel = attitude_control->get_accel_roll_max();
        }

        if (autotune_pitch_enabled() && !is_zero(tune_pitch_rp)) {
            // rate pitch gains
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_rate_pitch_pid().save_gains();

            // stabilize pitch
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            attitude_control->get_angle_pitch_p().save_gains();

            // acceleration pitch
            attitude_control->save_accel_pitch_max(tune_pitch_accel);

            // resave pids to originals in case the autotune is run again
            orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
            orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
            orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
            orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
            orig_pitch_accel = attitude_control->get_accel_pitch_max();
        }

        if (autotune_yaw_enabled() && !is_zero(tune_yaw_rp)) {
            // rate yaw gains
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            attitude_control->get_rate_yaw_pid().kD(0.0f);
            attitude_control->get_rate_yaw_pid().filt_hz(tune_yaw_rLPF);
            attitude_control->get_rate_yaw_pid().save_gains();

            // stabilize yaw
            attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
            attitude_control->get_angle_yaw_p().save_gains();

            // acceleration yaw
            attitude_control->save_accel_yaw_max(tune_yaw_accel);

            // resave pids to originals in case the autotune is run again
            orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
            orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
            orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
            orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_hz();
            orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
            orig_yaw_accel = attitude_control->get_accel_pitch_max();
        }
        // update GCS and log save gains event
        autotune_update_gcs(AUTOTUNE_MESSAGE_SAVED_GAINS);
        Log_Write_Event(DATA_AUTOTUNE_SAVEDGAINS);
        // reset Autotune so that gains are not saved again and autotune can be run again.
        autotune_state.mode = AUTOTUNE_MODE_UNINITIALISED;
    }
}

// autotune_update_gcs - send message to ground station
void Copter::autotune_update_gcs(uint8_t message_id)
{
    switch (message_id) {
        case AUTOTUNE_MESSAGE_STARTED:
            gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Started");
            break;
        case AUTOTUNE_MESSAGE_STOPPED:
            gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Stopped");
            break;
        case AUTOTUNE_MESSAGE_SUCCESS:
            gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Success");
            break;
        case AUTOTUNE_MESSAGE_FAILED:
            gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Failed");
            break;
        case AUTOTUNE_MESSAGE_SAVED_GAINS:
            gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Saved gains");
            break;
    }
}

// axis helper functions
inline bool Copter::autotune_roll_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_ROLL;
}

inline bool Copter::autotune_pitch_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_PITCH;
}

inline bool Copter::autotune_yaw_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_YAW;
}

// autotune_twitching_test - twitching tests
// update min and max and test for end conditions
void Copter::autotune_twitching_test(float measurement, float target, float &measurement_min, float &measurement_max)
{
    // capture maximum measurement
    if (measurement > measurement_max) {
        // the measurement is continuing to increase without stopping
        measurement_max = measurement;
        measurement_min = measurement;
    }

    // capture minimum measurement after the measurement has peaked (aka "bounce back")
    if ((measurement < measurement_min) && (measurement_max > target * 0.5f)) {
        // the measurement is bouncing back
        measurement_min = measurement;
    }

    // calculate early stopping time based on the time it takes to get to 90%
    if (measurement_max < target * 0.75f) {
        // the measurement not reached the 90% threshold yet
        autotune_step_stop_time = autotune_step_start_time + (millis() - autotune_step_start_time) * 3.0f;
        autotune_step_stop_time = MIN(autotune_step_stop_time, autotune_step_start_time + AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }

    if (measurement_max > target) {
        // the measurement has passed the target
        autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
    }

    if (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) {
        // the measurement has passed 50% of the target and bounce back is larger than the threshold
        autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
    }

    if (millis() >= autotune_step_stop_time) {
        // we have passed the maximum stop time
        autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
    }
}

// autotune_updating_d_up - increase D and adjust P to optimize the D term for a little bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void Copter::autotune_updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D gain so stop tuning
                tune_d = tune_d_min;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }else if ((measurement_max < target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }else{
        // we have a good measurement of bounce back
        if (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) {
            // ignore the next result unless it is the same as this one
            autotune_state.ignore_next = true;
            // bounce back is bigger than our threshold so increment the success counter
            autotune_counter++;
        }else{
            if (autotune_state.ignore_next == false) {
                // bounce back is smaller than our threshold so decrement the success counter
                if (autotune_counter > 0 ) {
                    autotune_counter--;
                }
                // increase D gain (which should increase bounce back)
                tune_d += tune_d*tune_d_step_ratio*2.0f;
                // stop tuning if we hit maximum D
                if (tune_d >= tune_d_max) {
                    tune_d = tune_d_max;
                    autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                }
            } else {
                autotune_state.ignore_next = false;
            }
        }
    }
}

// autotune_updating_d_down - decrease D and adjust P to optimize the D term for no bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void Copter::autotune_updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D gain
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D so stop tuning
                tune_d = tune_d_min;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }else if ((measurement_max < target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }else{
        // we have a good measurement of bounce back
        if (measurement_max-measurement_min < measurement_max*g.autotune_aggressiveness) {
            if (autotune_state.ignore_next == false) {
                // bounce back is less than our threshold so increment the success counter
                autotune_counter++;
            } else {
                autotune_state.ignore_next = false;
            }
        }else{
            // ignore the next result unless it is the same as this one
            autotune_state.ignore_next = true;
            // bounce back is larger than our threshold so decrement the success counter
            if (autotune_counter > 0 ) {
                autotune_counter--;
            }
            // decrease D gain (which should decrease bounce back)
            tune_d -= tune_d*tune_d_step_ratio;
            // stop tuning if we hit minimum D
            if (tune_d <= tune_d_min) {
                tune_d = tune_d_min;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }
}

// autotune_updating_p_down - decrease P until we don't reach the target before time out
// P is decreased to ensure we are not overshooting the target
void Copter::autotune_updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max)
{
    if (measurement_max < target*(1+0.5f*g.autotune_aggressiveness)) {
        if (autotune_state.ignore_next == false) {
            // if maximum measurement was lower than target so increment the success counter
            autotune_counter++;
        } else {
            autotune_state.ignore_next = false;
        }
    }else{
        // ignore the next result unless it is the same as this one
        autotune_state.ignore_next = true;
        // if maximum measurement was higher than target so decrement the success counter
        if (autotune_counter > 0 ) {
            autotune_counter--;
        }
        // decrease P gain (which should decrease the maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit maximum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            autotune_counter = AUTOTUNE_SUCCESS_COUNT;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }
}

// autotune_updating_p_up - increase P to ensure the target is reached
// P is increased until we achieve our target within a reasonable time
void Copter::autotune_updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max)
{
    if (measurement_max > target*(1+0.5f*g.autotune_aggressiveness)) {
        // ignore the next result unless it is the same as this one
        autotune_state.ignore_next = 1;
        // if maximum measurement was greater than target so increment the success counter
        autotune_counter++;
    }else{
        if (autotune_state.ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (autotune_counter > 0 ) {
                autotune_counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            autotune_state.ignore_next = false;
        }
    }
}

// autotune_updating_p_up - increase P to ensure the target is reached while checking bounce back isn't increasing
// P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
void Copter::autotune_updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target*(1+0.5f*g.autotune_aggressiveness)) {
        // ignore the next result unless it is the same as this one
        autotune_state.ignore_next = true;
        // if maximum measurement was greater than target so increment the success counter
        autotune_counter++;
    } else if ((measurement_max < target) && (measurement_max > target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) && (tune_d > tune_d_min)) {
        // if bounce back was larger than the threshold so decrement the success counter
        if (autotune_counter > 0 ) {
            autotune_counter--;
        }
        // decrease D gain (which should decrease bounce back)
        tune_d -= tune_d*tune_d_step_ratio;
        // stop tuning if we hit minimum D
        if (tune_d <= tune_d_min) {
            tune_d = tune_d_min;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
        // decrease P gain to match D gain reduction
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit minimum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
        // cancel change in direction
        autotune_state.positive_direction = !autotune_state.positive_direction;
    }else{
        if (autotune_state.ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (autotune_counter > 0 ) {
                autotune_counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            autotune_state.ignore_next = false;
        }
    }
}

// autotune_twitching_measure_acceleration - measure rate of change of measurement
void Copter::autotune_twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max)
{
    if (rate_measurement_max < rate_measurement) {
        rate_measurement_max = rate_measurement;
        rate_of_change = (1000.0f*rate_measurement_max)/(millis() - autotune_step_start_time);
    }
}

// get attitude for slow position hold in autotune mode
void Copter::autotune_get_poshold_attitude(float &roll_cd, float &pitch_cd, float &yaw_cd)
{
    roll_cd = pitch_cd = 0;

    if (!autotune_state.use_poshold) {
        // we are not trying to hold position
        return;
    }

    // do we know where we are?
    if (!position_ok()) {
        return;
    }

    if (!autotune_state.have_position) {
        autotune_state.have_position = true;
        autotune_state.start_position = inertial_nav.get_position();
    }

    // don't go past 10 degrees, as autotune result would deteriorate too much
    const float angle_max_cd = 1000;

    // hit the 10 degree limit at 20 meters position error
    const float dist_limit_cm = 2000;

    // we only start adjusting yaw if we are more than 5m from the
    // target position. That corresponds to a lean angle of 2.5 degrees
    const float yaw_dist_limit_cm = 500;
    
    Vector3f pdiff = inertial_nav.get_position() - autotune_state.start_position;
    pdiff.z = 0;
    float dist_cm = pdiff.length();
    if (dist_cm < 10) {
        // don't do anything within 10cm
        return;
    }

    /*
      very simple linear controller
     */
    float scaling = constrain_float(angle_max_cd * dist_cm / dist_limit_cm, 0, angle_max_cd);
    Vector2f angle_ne(pdiff.x, pdiff.y);
    angle_ne *= scaling / dist_cm;

    // rotate into body frame
    pitch_cd = angle_ne.x * ahrs.cos_yaw() + angle_ne.y * ahrs.sin_yaw();
    roll_cd  = angle_ne.x * ahrs.sin_yaw() - angle_ne.y * ahrs.cos_yaw();

    if (dist_cm < yaw_dist_limit_cm) {
        // no yaw adjustment
        return;
    }

    /*
      also point so that twitching occurs perpendicular to the wind,
      if we have drifted more than yaw_dist_limit_cm from the desired
      position. This ensures that autotune doesn't have to deal with
      more than 2.5 degrees of attitude on the axis it is tuning
     */
    float target_yaw_cd = degrees(atan2f(pdiff.y, pdiff.x)) * 100;
    if (autotune_state.axis == AUTOTUNE_AXIS_PITCH) {
        // for roll and yaw tuning we point along the wind, for pitch
        // we point across the wind
        target_yaw_cd += 9000;
    }
    // go to the nearest 180 degree mark, with 5 degree slop to prevent oscillation
    if (fabsf(yaw_cd - target_yaw_cd) > 9500) {
        target_yaw_cd += 18000;
    }

    yaw_cd = target_yaw_cd;
}

#endif  // AUTOTUNE_ENABLED == ENABLED
