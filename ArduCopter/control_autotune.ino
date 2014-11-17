/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if AUTOTUNE_ENABLED == ENABLED

/*
 * control_autotune.pde - init and run calls for autotune flight mode
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
 *      g) increases rate P until the max rotate rate becomes greater than the requeste rate (90deg/sec)
 *      h) invokes a 20deg angle request on roll or pitch
 *      i) increases stab P until the maximum angle becomes greater than 110% of the requested angle (20deg)
 *      j) decreases stab P by 25%
 *
 * Notes: AUTOTUNE should not be set-up as a flight mode, it should be invoked only from the ch7/ch8 switch.
 *
 */

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500    // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS    500    // timeout for tuning mode's testing step
#define AUTOTUNE_TARGET_ANGLE_CD           2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_CDS           9000    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_LEVEL_ANGLE_CD             300    // angle which qualifies as level
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     250    // time we require the copter to be level
#define AUTOTUNE_AGGRESSIVENESS            0.1f    // tuning for 10% overshoot
#define AUTOTUNE_RD_STEP                0.0005f    // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                 0.005f    // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                   0.5f    // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_SP_BACKOFF               0.75f    // Stab P gains are reduced to 75% of their maximum value discovered during tuning
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f    // I is set 10x smaller than P during testing
#define AUTOTUNE_RP_RATIO_FINAL            1.0f    // I is set 1x P after testing
#define AUTOTUNE_RD_MIN                  0.002f    // minimum Rate D value
#define AUTOTUNE_RD_MAX                  0.020f    // maximum Rate D value
#define AUTOTUNE_RP_MIN                   0.01f    // minimum Rate P value
#define AUTOTUNE_RP_MAX                   0.35f    // maximum Rate P value
#define AUTOTUNE_SP_MAX                   20.0f    // maximum Stab P value
#define AUTOTUNE_SUCCESS_COUNT                4    // how many successful iterations we need to freeze at current gains

// Auto Tune message ids for ground station
#define AUTOTUNE_MESSAGE_STARTED 0
#define AUTOTUNE_MESSAGE_STOPPED 1
#define AUTOTUNE_MESSAGE_SUCCESS 2
#define AUTOTUNE_MESSAGE_FAILED 3

// autotune modes (high level states)
enum AutoTuneTuneMode {
    AUTOTUNE_MODE_UNINITIALISED = 0,          // autotune has never been run
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
    AUTOTUNE_AXIS_PITCH = 1                 // pitch axis is being tuned (either angle or rate)
};

// mini steps performed while in Tuning mode, Testing step
enum AutoTuneTuneType {
    AUTOTUNE_TYPE_RD_UP = 0,                // rate D is being tuned up
    AUTOTUNE_TYPE_RD_DOWN = 1,              // rate D is being tuned down
    AUTOTUNE_TYPE_RP_UP = 2,                // rate P is being tuned up
    AUTOTUNE_TYPE_SP_UP = 3                 // angle P is being tuned up
};

// autotune_state_struct - hold state flags
struct autotune_state_struct {
    AutoTuneTuneMode    mode                : 2;    // see AutoTuneTuneMode for what modes are allowed
    uint8_t             pilot_override      : 1;    // 1 = pilot is overriding controls so we suspend tuning temporarily
    AutoTuneAxisType    axis                : 1;    // see AutoTuneAxisType for which things can be tuned
    uint8_t             positive_direction  : 1;    // 0 = tuning in negative direction (i.e. left for roll), 1 = positive direction (i.e. right for roll)
    AutoTuneStepType    step                : 2;    // see AutoTuneStepType for what steps are performed
    AutoTuneTuneType    tune_type           : 2;    // see AutoTuneTuneType
} autotune_state;

// variables
static uint32_t autotune_override_time;                                     // the last time the pilot overrode the controls
static float    autotune_test_min;                                          // the minimum angular rate achieved during TESTING_RATE step
static float    autotune_test_max;                                          // the maximum angular rate achieved during TESTING_RATE step
static uint32_t autotune_step_start_time;                                   // start time of current tuning step (used for timeout checks)
static int8_t   autotune_counter;                                           // counter for tuning gains
static float    orig_roll_rp = 0, orig_roll_ri, orig_roll_rd, orig_roll_sp;     // backup of currently being tuned parameter values
static float    orig_pitch_rp = 0, orig_pitch_ri, orig_pitch_rd, orig_pitch_sp; // backup of currently being tuned parameter values
static float    tune_roll_rp, tune_roll_rd, tune_roll_sp;                   // currently being tuned parameter values
static float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp;                // currently being tuned parameter values

// autotune_start - should be called when the ch7/ch8 switch is switched ON
static void autotune_start()
{
    switch (autotune_state.mode) {
        case AUTOTUNE_MODE_FAILED:
            // autotune has been run but failed so reset state to uninitialised
            autotune_state.mode = AUTOTUNE_MODE_UNINITIALISED;
            // no break to allow fall through to restart the tuning
        case AUTOTUNE_MODE_UNINITIALISED:
            // autotune has never been run
            // switch into the AUTOTUNE flight mode
            if (set_mode(AUTOTUNE)) {
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
            // set_mode to AUTOTUNE
            if (set_mode(AUTOTUNE)) {
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
}

// autotune_stop - should be called when the ch7/ch8 switch is switched OFF
static void autotune_stop()
{
    // set gains to their original values
    autotune_load_orig_gains();

    // re-enable angle-to-rate request limits
    attitude_control.limit_angle_to_rate_request(true);

    // log off event and send message to ground statoin
    autotune_update_gcs(AUTOTUNE_MESSAGE_STOPPED);
    Log_Write_Event(DATA_AUTOTUNE_OFF);

    // Note: we leave the autotune_state.mode as it was so that we know how the autotune ended
    // we expect the caller will change the flight mode back to the flight mode indicated by the flight mode switch
}

// autotune_init - initialise autotune flight mode
static bool autotune_init(bool ignore_checks)
{
    // only allow flip from Stabilize or AltHold flight modes
    if (control_mode != STABILIZE && control_mode != ALT_HOLD) {
        return false;
    }

    // ensure throttle is above zero
    if (g.rc_3.control_in <= 0) {
        return false;
    }

    // ensure we are flying
    if (!motors.armed() || !ap.auto_armed || ap.land_complete) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    return true;
}

// autotune_run - runs the autotune flight mode
// should be called at 100hz or more
static void autotune_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t target_climb_rate;

    // if not auto armed set throttle to zero and exit immediately
    // this should not actually be possible because of the autotune_init() checks
    if(!ap.auto_armed) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        pos_control.set_alt_target_to_current_alt();
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

    // check for pilot requested take-off - this should not actually be possible because of autotune_init() checks
    if (ap.land_complete && target_climb_rate > 0) {
        // indicate we are taking off
        set_land_complete(false);
        // clear i term when we're taking off
        set_throttle_takeoff();
    }

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(g.rc_3.control_in), false);
        pos_control.set_alt_target_to_current_alt();
    }else{
        // check if pilot is overriding the controls
        if (target_roll != 0 || target_pitch != 0 || target_yaw_rate != 0.0f || target_climb_rate != 0) {
            if (!autotune_state.pilot_override) {
                autotune_state.pilot_override = true;
                // set gains to their original values
                autotune_load_orig_gains();
            }
            // reset pilot override time
            autotune_override_time = millis();
        }else if (autotune_state.pilot_override) {
            // check if we should resume tuning after pilot's override
            if (millis() - autotune_override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
                autotune_state.pilot_override = false;             // turn off pilot override
                // set gains to their intra-test values (which are very close to the original gains)
                autotune_load_intra_test_gains();
                autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL; // set tuning step back from beginning
                autotune_step_start_time = millis();
            }
        }

        // if pilot override call attitude controller
        if (autotune_state.pilot_override || autotune_state.mode != AUTOTUNE_MODE_TUNING) {
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        }else{
            // somehow get attitude requests from autotuning
            autotune_attitude_control();
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// autotune_attitude_controller - sets attitude control targets during tuning
static void autotune_attitude_control()
{
    float rotation_rate;        // rotation rate in radians/second
    int32_t lean_angle;

    // check tuning step
    switch (autotune_state.step) {

    case AUTOTUNE_STEP_WAITING_FOR_LEVEL:
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)
        // re-enable rate limits
        attitude_control.limit_angle_to_rate_request(true);

        // hold level attitude
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0.0f, 0.0f, 0.0f);

        // hold the copter level for 0.25 seconds before we begin a twitch
        // reset counter if we are no longer level
        if ((labs(ahrs.roll_sensor) > AUTOTUNE_LEVEL_ANGLE_CD) || (labs(ahrs.pitch_sensor) > AUTOTUNE_LEVEL_ANGLE_CD)) {
            autotune_step_start_time = millis();
        }

        // if we have been level for a sufficient amount of time (0.25 seconds) move onto tuning step
        if (millis() - autotune_step_start_time >= AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
            // init variables for next step
            autotune_state.step = AUTOTUNE_STEP_TWITCHING;
            autotune_step_start_time = millis();
            autotune_test_max = 0;
            autotune_test_min = 0;
            rotation_rate = 0;
            // set gains to their to-be-tested values
            autotune_load_twitch_gains();
        }
        break;

    case AUTOTUNE_STEP_TWITCHING:
        // Run the twitching step
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)

        // disable rate limits
        attitude_control.limit_angle_to_rate_request(false);

        if(autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP){
            // Testing increasing stabilize P gain so will set lean angle target
            if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                // request roll to 20deg
                if (autotune_state.positive_direction) {
                    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(AUTOTUNE_TARGET_ANGLE_CD, 0.0f, 0.0f);
                }else{
                    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(-AUTOTUNE_TARGET_ANGLE_CD, 0.0f, 0.0f);
                }
            }else{
                // request pitch to 20deg
                if (autotune_state.positive_direction) {
                    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0.0f, AUTOTUNE_TARGET_ANGLE_CD, 0.0f);
                }else{
                    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0.0f, -AUTOTUNE_TARGET_ANGLE_CD, 0.0f);
                }
            }
        } else {
            // Testing rate P and D gains so will set body-frame rate targets
            if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                // override body-frame roll rate (rate controller will use existing pitch and yaw body-frame rates and convert to motor outputs)
                if (autotune_state.positive_direction) {
                    attitude_control.rate_bf_roll_target(AUTOTUNE_TARGET_RATE_CDS);
                }else{
                    attitude_control.rate_bf_roll_target(-AUTOTUNE_TARGET_RATE_CDS);
                }
            }else{
                // override body-frame pitch rate (rate controller will use existing roll and yaw body-frame rates and convert to motor outputs)
                if (autotune_state.positive_direction) {
                    attitude_control.rate_bf_pitch_target(AUTOTUNE_TARGET_RATE_CDS);
                }else{
                    attitude_control.rate_bf_pitch_target(-AUTOTUNE_TARGET_RATE_CDS);
                }
            }
        }

        // capture this iterations rotation rate and lean angle
        if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
            // 20 Hz filter on rate
            rotation_rate = ToDeg(fabs(ahrs.get_gyro().x)) * 100.0f;
            lean_angle = labs(ahrs.roll_sensor);
        }else{
            // 20 Hz filter on rate
            // rotation_rate = rotation_rate + 0.55686f*(ToDeg(fabs(ahrs.get_gyro().y))*100.0f-rotation_rate);
            rotation_rate = ToDeg(fabs(ahrs.get_gyro().y)) * 100.0f;
            lean_angle = labs(ahrs.pitch_sensor);
        }
        // log this iterations lean angle and rotation rate
        Log_Write_AutoTuneDetails((int16_t)lean_angle, rotation_rate);

        // compare rotation rate or lean angle to previous iterations of this testing step
        if(autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP){
            // when tuning stabilize P gain, capture the max lean angle
            if (lean_angle > autotune_test_max) {
                autotune_test_max = lean_angle;
                autotune_test_min = lean_angle;
            }

            // capture min lean angle
            if (lean_angle < autotune_test_min && autotune_test_max > AUTOTUNE_TARGET_ANGLE_CD*(1-AUTOTUNE_AGGRESSIVENESS)) {
                autotune_test_min = lean_angle;
            }
        }else{
            // when tuning rate P and D gain, capture max rotation rate
            if (rotation_rate > autotune_test_max) {
                autotune_test_max = rotation_rate;
                autotune_test_min = rotation_rate;
            }

            // capture min rotation rate after the rotation rate has peaked (aka "bounce back rate")
            if (rotation_rate < autotune_test_min && autotune_test_max > AUTOTUNE_TARGET_RATE_CDS*0.5) {
                autotune_test_min = rotation_rate;
            }
        }

        // check for end of test conditions
        // testing step time out after 0.5sec
        if(millis() - autotune_step_start_time >= AUTOTUNE_TESTING_STEP_TIMEOUT_MS) {
                autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
        }
        if(autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP){
            // stabilize P testing completes when the lean angle reaches 22deg or the vehicle has rotated 22deg
            if ((lean_angle >= AUTOTUNE_TARGET_ANGLE_CD*(1+AUTOTUNE_AGGRESSIVENESS)) ||
                    (autotune_test_max-autotune_test_min > AUTOTUNE_TARGET_ANGLE_CD*AUTOTUNE_AGGRESSIVENESS)) {
                autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
            }
        }else{
            // rate P and D testing completes when the vehicle reaches 20deg
            if (lean_angle >= AUTOTUNE_TARGET_ANGLE_CD) {
                autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
            }
            // rate P and D testing can also complete when the "bounce back rate" is at least 9deg less than the maximum rotation rate
            if (autotune_state.tune_type == AUTOTUNE_TYPE_RD_UP || autotune_state.tune_type == AUTOTUNE_TYPE_RD_DOWN) {
                if(autotune_test_max-autotune_test_min > AUTOTUNE_TARGET_RATE_CDS*AUTOTUNE_AGGRESSIVENESS) {
                    autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
                }
            }
        }
        break;

    case AUTOTUNE_STEP_UPDATE_GAINS:
        // set gains to their intra-test values (which are very close to the original gains)
        autotune_load_intra_test_gains();

        // re-enable rate limits
        attitude_control.limit_angle_to_rate_request(true);

        // log the latest gains
        if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
            Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_test_min, autotune_test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp);
        }else{
            Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_test_min, autotune_test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp);
        }

        // Check results after mini-step to increase rate D gain
        if (autotune_state.tune_type == AUTOTUNE_TYPE_RD_UP) {
            // when tuning the rate D gain
            if (autotune_test_max > AUTOTUNE_TARGET_RATE_CDS) {
                // if max rotation rate was higher than target, reduce rate P
                if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                    tune_roll_rp -= AUTOTUNE_RP_STEP;
                    // abandon tuning if rate P falls below 0.01
                    if(tune_roll_rp < AUTOTUNE_RP_MIN) {
                        tune_roll_rp = AUTOTUNE_RP_MIN;
                        autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                        Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                    }
                }else{
                    tune_pitch_rp -= AUTOTUNE_RP_STEP;
                    // abandon tuning if rate P falls below 0.01
                    if( tune_pitch_rp < AUTOTUNE_RP_MIN ) {
                        tune_pitch_rp = AUTOTUNE_RP_MIN;
                        autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                        Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                    }
                }
            // if maximum rotation rate was less than 80% of requested rate increase rate P
            }else if(autotune_test_max < AUTOTUNE_TARGET_RATE_CDS*(1.0f-AUTOTUNE_AGGRESSIVENESS*2.0f) &&
                    ((autotune_state.axis == AUTOTUNE_AXIS_ROLL && tune_roll_rp <= AUTOTUNE_RP_MAX) ||
                    (autotune_state.axis == AUTOTUNE_AXIS_PITCH && tune_pitch_rp <= AUTOTUNE_RP_MAX)) ) {
                if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                    tune_roll_rp += AUTOTUNE_RP_STEP*2.0f;
                }else{
                    tune_pitch_rp += AUTOTUNE_RP_STEP*2.0f;
                }
            }else{
                // if "bounce back rate" if greater than 10% of requested rate (i.e. >9deg/sec) this is a good tune
                if (autotune_test_max-autotune_test_min > AUTOTUNE_TARGET_RATE_CDS*AUTOTUNE_AGGRESSIVENESS) {
                    autotune_counter++;
                }else{
                    // bounce back was too small so reduce number of good tunes
                    if (autotune_counter > 0 ) {
                        autotune_counter--;
                    }
                    // increase rate D (which should increase "bounce back rate")
                    if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                        tune_roll_rd += AUTOTUNE_RD_STEP*2.0f;
                        // stop tuning if we hit max D
                        if (tune_roll_rd >= AUTOTUNE_RD_MAX) {
                            tune_roll_rd = AUTOTUNE_RD_MAX;
                            autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                        }
                    }else{
                        tune_pitch_rd += AUTOTUNE_RD_STEP*2.0f;
                        // stop tuning if we hit max D
                        if (tune_pitch_rd >= AUTOTUNE_RD_MAX) {
                            tune_pitch_rd = AUTOTUNE_RD_MAX;
                            autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                        }
                    }
                }
            }
        // Check results after mini-step to decrease rate D gain
        } else if (autotune_state.tune_type == AUTOTUNE_TYPE_RD_DOWN) {
            if (autotune_test_max > AUTOTUNE_TARGET_RATE_CDS) {
                // if max rotation rate was higher than target, reduce rate P
                if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                    tune_roll_rp -= AUTOTUNE_RP_STEP;
                    // reduce rate D if tuning if rate P falls below 0.01
                    if(tune_roll_rp < AUTOTUNE_RP_MIN) {
                        tune_roll_rp = AUTOTUNE_RP_MIN;
                        tune_roll_rd -= AUTOTUNE_RD_STEP;
                        // stop tuning if we hit min D
                        if (tune_roll_rd <= AUTOTUNE_RD_MIN) {
                            tune_roll_rd = AUTOTUNE_RD_MIN;
                            autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                        }
                    }
                }else{
                    tune_pitch_rp -= AUTOTUNE_RP_STEP;
                    // reduce rate D if tuning if rate P falls below 0.01
                    if( tune_pitch_rp < AUTOTUNE_RP_MIN ) {
                        tune_pitch_rp = AUTOTUNE_RP_MIN;
                        tune_pitch_rd -= AUTOTUNE_RD_STEP;
                        // stop tuning if we hit min D
                        if (tune_pitch_rd <= AUTOTUNE_RD_MIN) {
                            tune_pitch_rd = AUTOTUNE_RD_MIN;
                            autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                        }
                    }
                }
            // if maximum rotation rate was less than 80% of requested rate increase rate P
            }else if(autotune_test_max < AUTOTUNE_TARGET_RATE_CDS*(1-AUTOTUNE_AGGRESSIVENESS*2.0f) &&
                    ((autotune_state.axis == AUTOTUNE_AXIS_ROLL && tune_roll_rp <= AUTOTUNE_RP_MAX) ||
                    (autotune_state.axis == AUTOTUNE_AXIS_PITCH && tune_pitch_rp <= AUTOTUNE_RP_MAX)) ) {
                if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                    tune_roll_rp += AUTOTUNE_RP_STEP;
                }else{
                    tune_pitch_rp += AUTOTUNE_RP_STEP;
                }
            }else{
                // if "bounce back rate" if less than 10% of requested rate (i.e. >9deg/sec) this is a good tune
                if (autotune_test_max-autotune_test_min < AUTOTUNE_TARGET_RATE_CDS*AUTOTUNE_AGGRESSIVENESS) {
                    autotune_counter++;
                }else{
                    // bounce back was too large so reduce number of good tunes
                    if (autotune_counter > 0 ) {
                        autotune_counter--;
                    }
                    // decrease rate D (which should decrease "bounce back rate")
                    if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                        tune_roll_rd -= AUTOTUNE_RD_STEP;
                        // stop tuning if we hit min D
                        if (tune_roll_rd <= AUTOTUNE_RD_MIN) {
                            tune_roll_rd = AUTOTUNE_RD_MIN;
                            autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                        }
                    }else{
                        tune_pitch_rd -= AUTOTUNE_RD_STEP;
                        // stop tuning if we hit min D
                        if (tune_pitch_rd <= AUTOTUNE_RD_MIN) {
                            tune_pitch_rd = AUTOTUNE_RD_MIN;
                            autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                        }
                    }
                }
            }
        // Check results after mini-step to increase rate P gain
        } else if (autotune_state.tune_type == AUTOTUNE_TYPE_RP_UP) {
            // if max rotation rate greater than target, this is a good tune
            if (autotune_test_max > AUTOTUNE_TARGET_RATE_CDS) {
                autotune_counter++;
            }else{
                // rotation rate was too low so reduce number of good tunes
                if (autotune_counter > 0 ) {
                    autotune_counter--;
                }
                // increase rate P and I gains
                if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                    tune_roll_rp += AUTOTUNE_RP_STEP;
                    // stop tuning if we hit max P
                    if (tune_roll_rp >= AUTOTUNE_RP_MAX) {
                        tune_roll_rp = AUTOTUNE_RP_MAX;
                        autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                        Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                    }
                }else{
                    tune_pitch_rp += AUTOTUNE_RP_STEP;
                    // stop tuning if we hit max P
                    if (tune_pitch_rp >= AUTOTUNE_RP_MAX) {
                        tune_pitch_rp = AUTOTUNE_RP_MAX;
                        autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                        Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                    }
                }
            }
        // Check results after mini-step to increase stabilize P gain
        } else if (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP) {
            // if max angle reaches 22deg this is a successful tune
            if (autotune_test_max > AUTOTUNE_TARGET_ANGLE_CD*(1+AUTOTUNE_AGGRESSIVENESS) ||
                    (autotune_test_max-autotune_test_min > AUTOTUNE_TARGET_ANGLE_CD*AUTOTUNE_AGGRESSIVENESS)) {
                autotune_counter++;
            }else{
                // did not reach the target angle so this is a bad tune
                if (autotune_counter > 0 ) {
                    autotune_counter--;
                }
                // increase stabilize P and I gains
                if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                    tune_roll_sp += AUTOTUNE_SP_STEP;
                    // stop tuning if we hit max P
                    if (tune_roll_sp >= AUTOTUNE_SP_MAX) {
                        tune_roll_sp = AUTOTUNE_SP_MAX;
                        autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                        Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                    }
                }else{
                    tune_pitch_sp += AUTOTUNE_SP_STEP;
                    // stop tuning if we hit max P
                    if (tune_pitch_sp >= AUTOTUNE_SP_MAX) {
                        tune_pitch_sp = AUTOTUNE_SP_MAX;
                        autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                        Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                    }
                }
            }
        }

        // reverse direction
        autotune_state.positive_direction = !autotune_state.positive_direction;

        // we've complete this step, finalise pids and move to next step
        if (autotune_counter >= AUTOTUNE_SUCCESS_COUNT) {

            // reset counter
            autotune_counter = 0;

            // move to the next tuning type
            if (autotune_state.tune_type < AUTOTUNE_TYPE_SP_UP) {
                autotune_state.tune_type++;
            }else{
                // we've reached the end of a D-up-down PI-up-down tune type cycle
                autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

                // if we've just completed roll move onto pitch
                if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
                    tune_roll_sp = tune_roll_sp * AUTOTUNE_SP_BACKOFF;
                    autotune_state.axis = AUTOTUNE_AXIS_PITCH;
                }else{
                    tune_pitch_sp = tune_pitch_sp * AUTOTUNE_SP_BACKOFF;
                    tune_roll_sp = min(tune_roll_sp, tune_pitch_sp);
                    tune_pitch_sp = min(tune_roll_sp, tune_pitch_sp);
                    // if we've just completed pitch we have successfully completed the autotune
                    // change to TESTING mode to allow user to fly with new gains
                    autotune_state.mode = AUTOTUNE_MODE_SUCCESS;
                    autotune_update_gcs(AUTOTUNE_MESSAGE_SUCCESS);
                    Log_Write_Event(DATA_AUTOTUNE_SUCCESS);
                }
            }
        }

        // reset testing step
        autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
        autotune_step_start_time = millis();
        break;
    }
}

// autotune has failed, return to standard gains and log event
//  called when the autotune is unable to find good gains
static void autotune_failed()
{
    // set autotune mode to failed so that it cannot restart
    autotune_state.mode = AUTOTUNE_MODE_FAILED;
    // set gains to their original values
    autotune_load_orig_gains();
    // re-enable angle-to-rate request limits
    attitude_control.limit_angle_to_rate_request(true);
    // log failure
    Log_Write_Event(DATA_AUTOTUNE_FAILED);
}

// autotune_backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
static void autotune_backup_gains_and_initialise()
{
    // initialise state because this is our first time
    autotune_state.axis = AUTOTUNE_AXIS_ROLL;
    autotune_state.positive_direction = false;
    autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
    autotune_step_start_time = millis();
    autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

    // backup original pids
    orig_roll_rp = g.pid_rate_roll.kP();
    orig_roll_ri = g.pid_rate_roll.kI();
    orig_roll_rd = g.pid_rate_roll.kD();
    orig_roll_sp = g.p_stabilize_roll.kP();
    orig_pitch_rp = g.pid_rate_pitch.kP();
    orig_pitch_ri = g.pid_rate_pitch.kI();
    orig_pitch_rd = g.pid_rate_pitch.kD();
    orig_pitch_sp = g.p_stabilize_pitch.kP();

    // initialise tuned pid values
    tune_roll_rp = g.pid_rate_roll.kP();
    tune_roll_rd = g.pid_rate_roll.kD();
    tune_roll_sp = g.p_stabilize_roll.kP();
    tune_pitch_rp = g.pid_rate_pitch.kP();
    tune_pitch_rd = g.pid_rate_pitch.kD();
    tune_pitch_sp = g.p_stabilize_pitch.kP();

    Log_Write_Event(DATA_AUTOTUNE_INITIALISED);
}

// autotune_load_orig_gains - set gains to their original values
//  called by autotune_stop and autotune_failed functions
static void autotune_load_orig_gains()
{
    // sanity check the original gains
    if (orig_roll_rp != 0 && orig_pitch_rp != 0) {
        g.pid_rate_roll.kP(orig_roll_rp);
        g.pid_rate_roll.kI(orig_roll_ri);
        g.pid_rate_roll.kD(orig_roll_rd);
        g.p_stabilize_roll.kP(orig_roll_sp);
        g.pid_rate_pitch.kP(orig_pitch_rp);
        g.pid_rate_pitch.kI(orig_pitch_ri);
        g.pid_rate_pitch.kD(orig_pitch_rd);
        g.p_stabilize_pitch.kP(orig_pitch_sp);
    }
}

// autotune_load_tuned_gains - load tuned gains
static void autotune_load_tuned_gains()
{
    // sanity check the gains
    if (tune_roll_rp != 0 && tune_pitch_rp != 0) {
        g.pid_rate_roll.kP(tune_roll_rp);
        g.pid_rate_roll.kI(tune_roll_rp*AUTOTUNE_RP_RATIO_FINAL);
        g.pid_rate_roll.kD(tune_roll_rd);
        g.p_stabilize_roll.kP(tune_roll_sp);
        g.pid_rate_pitch.kP(tune_pitch_rp);
        g.pid_rate_pitch.kI(tune_pitch_rp*AUTOTUNE_RP_RATIO_FINAL);
        g.pid_rate_pitch.kD(tune_pitch_rd);
        g.p_stabilize_pitch.kP(tune_pitch_sp);
    }else{
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
static void autotune_load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the original gains
    if (orig_roll_rp != 0 && orig_pitch_rp != 0) {
        g.pid_rate_roll.kP(orig_roll_rp);
        g.pid_rate_roll.kI(orig_roll_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        g.pid_rate_roll.kD(orig_roll_rd);
        g.p_stabilize_roll.kP(orig_roll_sp);
        g.pid_rate_pitch.kP(orig_pitch_rp);
        g.pid_rate_pitch.kI(orig_pitch_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        g.pid_rate_pitch.kD(orig_pitch_rd);
        g.p_stabilize_pitch.kP(orig_pitch_sp);
    }else{
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_load_twitch_gains - load the to-be-tested gains for a single axis
//  called by autotune_attitude_control() just before it beings testing a gain (i.e. just before it twitches)
static void autotune_load_twitch_gains()
{
    if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
        if (tune_roll_rp != 0) {
            g.pid_rate_roll.kP(tune_roll_rp);
            g.pid_rate_roll.kI(tune_roll_rp*0.01f);
            g.pid_rate_roll.kD(tune_roll_rd);
            g.p_stabilize_roll.kP(tune_roll_sp);
        }else{
            // log an error message and fail the autotune
            Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
        }
    }else{
        if (tune_pitch_rp != 0) {
            g.pid_rate_pitch.kP(tune_pitch_rp);
            g.pid_rate_pitch.kI(tune_pitch_rp*0.01f);
            g.pid_rate_pitch.kD(tune_pitch_rd);
            g.p_stabilize_pitch.kP(tune_pitch_sp);
        }else{
            // log an error message and fail the autotune
            Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
        }
    }
}

// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
static void autotune_save_tuning_gains()
{
    // if we successfully completed tuning
    if (autotune_state.mode == AUTOTUNE_MODE_SUCCESS) {
        // sanity check the rate P values
        if (tune_roll_rp != 0 && tune_pitch_rp != 0) {

            // rate roll gains
            g.pid_rate_roll.kP(tune_roll_rp);
            g.pid_rate_roll.kI(tune_roll_rp*AUTOTUNE_RP_RATIO_FINAL);
            g.pid_rate_roll.kD(tune_roll_rd);
            g.pid_rate_roll.save_gains();

            // rate pitch gains
            g.pid_rate_pitch.kP(tune_pitch_rp);
            g.pid_rate_pitch.kI(tune_pitch_rp*AUTOTUNE_RP_RATIO_FINAL);
            g.pid_rate_pitch.kD(tune_pitch_rd);
            g.pid_rate_pitch.save_gains();

            // stabilize roll
            g.p_stabilize_roll.kP(tune_roll_sp);
            g.p_stabilize_roll.save_gains();

            // stabilize pitch
            g.p_stabilize_pitch.save_gains();
            g.p_stabilize_pitch.kP(tune_pitch_sp);

            // resave pids to originals in case the autotune is run again
            orig_roll_rp = g.pid_rate_roll.kP();
            orig_roll_ri = g.pid_rate_roll.kI();
            orig_roll_rd = g.pid_rate_roll.kD();
            orig_roll_sp = g.p_stabilize_roll.kP();
            orig_pitch_rp = g.pid_rate_pitch.kP();
            orig_pitch_ri = g.pid_rate_pitch.kI();
            orig_pitch_rd = g.pid_rate_pitch.kD();
            orig_pitch_sp = g.p_stabilize_pitch.kP();

            // log save gains event
            Log_Write_Event(DATA_AUTOTUNE_SAVEDGAINS);
        }else{
            // log an error message and fail the autotune
            Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
        }
    }
}

// send message to ground station
void autotune_update_gcs(uint8_t message_id)
{
    switch (message_id) {
        case AUTOTUNE_MESSAGE_STARTED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Started"));
            break;
        case AUTOTUNE_MESSAGE_STOPPED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Stopped"));
            break;
        case AUTOTUNE_MESSAGE_SUCCESS:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Success"));
            break;
        case AUTOTUNE_MESSAGE_FAILED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Failed"));
            break;
    }
}
#endif  // AUTOTUNE_ENABLED == ENABLED
