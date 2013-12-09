/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if AUTOTUNE == ENABLED
/*
    Auto tuning works in this way:
        i) set up 3-position ch7 or ch8 switch to "AutoTune"
        2) take-off in stabilize mode, put the copter into a level hover and switch ch7/ch8 to high position to start tuning
        3) auto tuner brings roll and pitch level
        4) the following procedure is run for roll and then pitch
            a) invokes 90 deg/sec rate request
            b) records maximum "forward" roll rate and bounce back rate
            c) when copter reaches 20 degrees or 1 second has passed, it commands level
            d) tries to keep max rotation rate between 80% ~ 100% of requested rate (90deg/sec) by adjusting rate P
            e) increases rate D until the bounce back becomes greater than 10% of requested rate (90deg/sec)
            f) decreases rate D until the bounce back becomes less than 10% of requested rate (90deg/sec)
            g) increases rate P until the max rotate rate becomes greater than the requeste rate (90deg/sec)
            h) invokes a 20deg angle request on roll or pitch
            i) increases stab P until the maximum angle becomes greater than 110% of the requested angle (20deg)
            j) decreases stab P by 25%
    If pilot inputs any stick inputs these becomes the desired roll, pitch angles sent to the stabilize controller and the tuner is disabled until the sticks are put back into the middle for 1 second
*/

#define AUTO_TUNE_PILOT_OVERRIDE_TIMEOUT_MS  500    // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTO_TUNE_TARGET_RATE_TIMEOUT_MS     500    // timeout for rate test step
#define AUTO_TUNE_TARGET_RATE_CDS           9000    // target roll/pitch rate during AUTO_TUNE_STEP_TESTING step
#define AUTO_TUNE_LEVEL_ANGLE_CD             300    // angle which qualifies as level
#define AUTO_TUNE_TARGET_ANGLE_CD           2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTO_TUNE_REQUIRED_LEVEL_TIME_MS     250    // time we require the copter to be level
#define AUTO_TUNE_AGGRESSIVENESS            0.1f    // tuning for 10% overshoot
#define AUTO_TUNE_RD_STEP                0.0005f    // minimum increment when increasing/decreasing Rate D term
#define AUTO_TUNE_RP_STEP                 0.005f    // minimum increment when increasing/decreasing Rate P term
#define AUTO_TUNE_SP_STEP                   0.5f    // minimum increment when increasing/decreasing Stab P term
#define AUTO_TUNE_SP_BACKOFF               0.75f    // back off on the Stab P tune
#define AUTO_TUNE_PI_RATIO_FOR_TESTING      0.1f    // I is set 10x smaller than P during testing
#define AUTO_TUNE_RP_RATIO_FINAL            1.0f    // I is set 1x P after testing
#define AUTO_TUNE_RD_MIN                  0.002f    // minimum Rate D value
#define AUTO_TUNE_RD_MAX                  0.015f    // maximum Rate D value
#define AUTO_TUNE_RP_MIN                   0.01f    // minimum Rate P value
#define AUTO_TUNE_RP_MAX                   0.25f    // maximum Rate P value
#define AUTO_TUNE_SP_MAX                   15.0f    // maximum Stab P value
#define AUTO_TUNE_SUCCESS_COUNT                4    // how many successful iterations we need to freeze at current gains

// Auto Tune message ids for ground station
#define AUTO_TUNE_MESSAGE_STARTED 0
#define AUTO_TUNE_MESSAGE_SUCCESS 1
#define AUTO_TUNE_MESSAGE_FAILED 2

enum AutoTuneTuneMode {
    AUTO_TUNE_MODE_UNINITIALISED = 0,
    AUTO_TUNE_MODE_TUNING = 1,
    AUTO_TUNE_MODE_TESTING = 2,
    AUTO_TUNE_MODE_FAILED = 3
};

// things that can be tuned
enum AutoTuneAxisType {
    AUTO_TUNE_AXIS_ROLL = 0,
    AUTO_TUNE_AXIS_PITCH = 1
};

// steps performed during tuning
enum AutoTuneStepType {
    AUTO_TUNE_STEP_WAITING_FOR_LEVEL = 0,
    AUTO_TUNE_STEP_TESTING = 1,
    AUTO_TUNE_STEP_UPDATE_GAINS = 2
};

// steps performed during tuning
enum AutoTuneTuneType {
    AUTO_TUNE_TYPE_RD_UP = 0,
    AUTO_TUNE_TYPE_RD_DOWN = 1,
    AUTO_TUNE_TYPE_RP_UP = 2,
    AUTO_TUNE_TYPE_SP_UP = 3
};

// state
struct auto_tune_state_struct {
    AutoTuneTuneMode mode       : 2;    // see AutoTuneTuneMode for what modes are allowed
    uint8_t pilot_override      : 1;    // 1 = pilot is overriding controls so we suspend tuning temporarily
    AutoTuneAxisType axis       : 1;    // see AutoTuneAxisType for which things can be tuned
    uint8_t positive_direction  : 1;    // 0 = tuning in negative direction (i.e. left for roll), 1 = positive direction (i.e. right for roll)
    AutoTuneStepType step       : 2;    // see AutoTuneStepType for what steps are performed
    AutoTuneTuneType tune_type  : 2;    // see AutoTuneTuneType
} auto_tune_state;

// variables
static uint32_t auto_tune_override_time;   // the last time the pilot overrode the controls
static float auto_tune_test_min;           // the minimum angular rate achieved during TESTING_RATE step
static float auto_tune_test_max;           // the maximum angular rate achieved during TESTING_RATE step
static uint32_t auto_tune_timer;           // generic timer variable
static int8_t auto_tune_counter;           // counter for tuning gains
static float orig_roll_rp, orig_roll_ri, orig_roll_rd, orig_roll_sp;     // backup of currently being tuned parameter values
static float orig_pitch_rp, orig_pitch_ri, orig_pitch_rd, orig_pitch_sp; // backup of currently being tuned parameter values
static float tune_roll_rp, tune_roll_rd, tune_roll_sp;     // currently being tuned parameter values
static float tune_pitch_rp, tune_pitch_rd, tune_pitch_sp;  // currently being tuned parameter values

// store current pids as originals
static void auto_tune_initialise()
{
    // initialise gains and axis because this is our first time
    auto_tune_state.axis = AUTO_TUNE_AXIS_ROLL;
    auto_tune_state.positive_direction = false;
    auto_tune_state.step = AUTO_TUNE_STEP_WAITING_FOR_LEVEL;
    auto_tune_timer = millis();
    auto_tune_state.tune_type = AUTO_TUNE_TYPE_RD_UP;

    // backup original pids
    orig_roll_rp = g.pid_rate_roll.kP();
    orig_roll_ri = g.pid_rate_roll.kI();
    orig_roll_rd = g.pid_rate_roll.kD();
    orig_roll_sp = g.pi_stabilize_roll.kP();
    orig_pitch_rp = g.pid_rate_pitch.kP();
    orig_pitch_ri = g.pid_rate_pitch.kI();
    orig_pitch_rd = g.pid_rate_pitch.kD();
    orig_pitch_sp = g.pi_stabilize_pitch.kP();

    // initialise tuned pid values
    tune_roll_rp = g.pid_rate_roll.kP();
    tune_roll_rd = g.pid_rate_roll.kD();
    tune_roll_sp = g.pi_stabilize_roll.kP();
    tune_pitch_rp = g.pid_rate_pitch.kP();
    tune_pitch_rd = g.pid_rate_pitch.kD();
    tune_pitch_sp = g.pi_stabilize_pitch.kP();

    Log_Write_Event(DATA_AUTOTUNE_INITIALISED);
}

// auto_tune_intra_test_gains - gains used between tests
static void auto_tune_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    g.pid_rate_roll.kP(orig_roll_rp);
    g.pid_rate_roll.kI(orig_roll_rp*AUTO_TUNE_PI_RATIO_FOR_TESTING);
    g.pid_rate_roll.kD(orig_roll_rd);
    g.pi_stabilize_roll.kP(orig_roll_sp);
    g.pid_rate_pitch.kP(orig_pitch_rp);
    g.pid_rate_pitch.kI(orig_pitch_rp*AUTO_TUNE_PI_RATIO_FOR_TESTING);
    g.pid_rate_pitch.kD(orig_pitch_rd);
    g.pi_stabilize_pitch.kP(orig_pitch_sp);

    // re-enable the rate limits
    ap.disable_stab_rate_limit = false;
}

// auto_tune_restore_orig_gains - restore pids to their original values
static void auto_tune_restore_orig_gains()
{
    g.pid_rate_roll.kP(orig_roll_rp);
    g.pid_rate_roll.kI(orig_roll_ri);
    g.pid_rate_roll.kD(orig_roll_rd);
    g.pi_stabilize_roll.kP(orig_roll_sp);
    g.pid_rate_pitch.kP(orig_pitch_rp);
    g.pid_rate_pitch.kI(orig_pitch_ri);
    g.pid_rate_pitch.kD(orig_pitch_rd);
    g.pi_stabilize_pitch.kP(orig_pitch_sp);
    ap.disable_stab_rate_limit = false;
}

// auto_tune_load_tuned_pids - restore pids to their tuned values
static void auto_tune_load_tuned_gains()
{
    if (tune_roll_rp != 0 && tune_pitch_rp != 0) {
        g.pid_rate_roll.kP(tune_roll_rp);
        g.pid_rate_roll.kI(tune_roll_rp*AUTO_TUNE_RP_RATIO_FINAL);
        g.pid_rate_roll.kD(tune_roll_rd);
        g.pi_stabilize_roll.kP(tune_roll_sp);
        g.pid_rate_pitch.kP(tune_pitch_rp);
        g.pid_rate_pitch.kI(tune_pitch_rp*AUTO_TUNE_RP_RATIO_FINAL);
        g.pid_rate_pitch.kD(tune_pitch_rd);
        g.pi_stabilize_pitch.kP(tune_pitch_sp);
        ap.disable_stab_rate_limit = false;
    }
}

// auto_tune_load_test_gains - load the to-be-tested gains for a single axis
static void auto_tune_load_test_gains()
{
    // restore pids to their tuning values
    if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
        g.pid_rate_roll.kP(tune_roll_rp);
        g.pid_rate_roll.kI(tune_roll_rp*0.01f);
        g.pid_rate_roll.kD(tune_roll_rd);
        g.pi_stabilize_roll.kP(tune_roll_sp);
    }else{
        g.pid_rate_pitch.kP(tune_pitch_rp);
        g.pid_rate_pitch.kI(tune_pitch_rp*0.01f);
        g.pid_rate_pitch.kD(tune_pitch_rd);
        g.pi_stabilize_pitch.kP(tune_pitch_sp);
    }
    ap.disable_stab_rate_limit = true;  // disable rate limits
}

// start an auto tuning session
// returns true if we should change the roll-pitch mode to ROLL_PITCH_AUTOTUNE
// To-Do: make autotune a flight mode so that this slightly non-intuitive returning of a flag is not required
static bool auto_tune_start()
{
    bool requires_autotune_rp_mode = false;

    switch (auto_tune_state.mode) {
        case AUTO_TUNE_MODE_UNINITIALISED:
            // initialise gains and axis because this is our first time
            auto_tune_initialise();
            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_STARTED);
            auto_tune_state.mode = AUTO_TUNE_MODE_TUNING;
            requires_autotune_rp_mode = true;
            break;
        case AUTO_TUNE_MODE_TUNING:
            // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
            auto_tune_intra_test_gains();
            Log_Write_Event(DATA_AUTOTUNE_RESTART);
            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_STARTED);
            requires_autotune_rp_mode = true;
            break;
        case AUTO_TUNE_MODE_TESTING:
            // we have completed a tune and are testing the new gains
            auto_tune_load_tuned_gains();
            Log_Write_Event(DATA_AUTOTUNE_TESTING);
            requires_autotune_rp_mode = false;
            break;
        case AUTO_TUNE_MODE_FAILED:
            Log_Write_Event(DATA_AUTOTUNE_ABANDONED);
            requires_autotune_rp_mode = false;
            break;
    }

    // tell caller we require roll-pitch mode to be changed to ROLL_PITCH_AUTOTUNE
    return requires_autotune_rp_mode;
}

// turn off tuning and return to standard pids
static void auto_tune_stop()
{
    ap.disable_stab_rate_limit = false;
    rate_targets_frame = EARTH_FRAME;   // regular stabilize mode frame
    // restore pids to their original values
    auto_tune_restore_orig_gains();
    Log_Write_Event(DATA_AUTOTUNE_OFF);
}

// save discovered gains to eeprom if auto tuner is enabled (i.e. switch is in middle or high position)
static void auto_tune_save_tuning_gains_and_reset()
{
    if (auto_tune_state.mode == AUTO_TUNE_MODE_TESTING) {
        auto_tune_load_tuned_gains();
        g.pid_rate_roll.save_gains();
        g.pid_rate_pitch.save_gains();
        g.pi_stabilize_roll.save_gains();
        g.pi_stabilize_pitch.save_gains();
        orig_roll_rp = g.pid_rate_roll.kP();
        orig_roll_ri = g.pid_rate_roll.kI();
        orig_roll_rd = g.pid_rate_roll.kD();
        orig_roll_sp = g.pi_stabilize_roll.kP();
        orig_pitch_rp = g.pid_rate_pitch.kP();
        orig_pitch_ri = g.pid_rate_pitch.kI();
        orig_pitch_rd = g.pid_rate_pitch.kD();
        orig_pitch_sp = g.pi_stabilize_pitch.kP();
        Log_Write_Event(DATA_AUTOTUNE_SAVEDGAINS);
    }
    // reset state of autotune
    auto_tune_state.mode = AUTO_TUNE_MODE_UNINITIALISED;
}

// send message to ground station
void auto_tune_update_gcs(uint8_t message_id)
{
    switch (message_id) {
        case AUTO_TUNE_MESSAGE_STARTED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Started"));
            break;
        case AUTO_TUNE_MESSAGE_SUCCESS:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Success"));
            break;
        case AUTO_TUNE_MESSAGE_FAILED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Failed"));
            break;
    }
}

// Auto tuning roll-pitch controller
static void
get_autotune_roll_pitch_controller(int16_t pilot_roll_angle, int16_t pilot_pitch_angle, int16_t pilot_yaw_command)
{
    int32_t target_roll_rate, target_pitch_rate;
    float rotation_rate;        // rotation rate in radians/second
    int32_t lean_angle;

    // exit immediately if not actively tuning
    if (!auto_tune_state.mode == AUTO_TUNE_MODE_TUNING) {
        return;
    }

    // check for pilot override
    if (pilot_roll_angle != 0 || pilot_pitch_angle != 0 || pilot_yaw_command != 0) {
        if (!auto_tune_state.pilot_override) {
            // restore pids to their original values
            auto_tune_restore_orig_gains();
        }
        auto_tune_state.pilot_override = true;
        auto_tune_override_time = millis();
    }else if (auto_tune_state.pilot_override) {
        // check if we should resume tuning after pilot's override
        if (millis() - auto_tune_override_time > AUTO_TUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
            auto_tune_state.pilot_override = false;             // turn off pilot override
            auto_tune_state.step = AUTO_TUNE_STEP_WAITING_FOR_LEVEL; // set tuning step back from beginning
            auto_tune_timer = millis();
        }
    }

    // check tuning step
    if (!auto_tune_state.pilot_override) {
        switch (auto_tune_state.step) {
            case AUTO_TUNE_STEP_WAITING_FOR_LEVEL:
                // reset counter if we are no longer level
                if ((labs(ahrs.roll_sensor) > AUTO_TUNE_LEVEL_ANGLE_CD) || (labs(ahrs.pitch_sensor) > AUTO_TUNE_LEVEL_ANGLE_CD)) {
                    auto_tune_timer = millis();
                }

                // if we have been level for a sufficient amount of time (0.5 seconds) move onto next step
                if (millis() - auto_tune_timer >= AUTO_TUNE_REQUIRED_LEVEL_TIME_MS) {
                    auto_tune_state.step = AUTO_TUNE_STEP_TESTING;
                    // init variables for next step
                    auto_tune_test_max = 0;
                    auto_tune_test_min = 0;
                    rotation_rate = 0;
                    auto_tune_timer = millis();
                    // initialise rate controller targets
                    acro_roll_rate = roll_rate_target_bf;
                    acro_pitch_rate = pitch_rate_target_bf;
                    acro_yaw_rate = yaw_rate_target_bf;
                    // restore pids to their to-be-tested values
                    auto_tune_load_test_gains();
                    ap.disable_stab_rate_limit = true;  // disable rate limits
                }
                break;

            case AUTO_TUNE_STEP_TESTING:

                // Run the test
                // update rotation targets in body-earth frame
                if(auto_tune_state.tune_type == AUTO_TUNE_TYPE_SP_UP){
                    if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                        // override roll angle
                        if (auto_tune_state.positive_direction) {
                            control_roll = AUTO_TUNE_TARGET_ANGLE_CD;
                        }else{
                            control_roll = -AUTO_TUNE_TARGET_ANGLE_CD;
                        }
                        get_stabilize_roll(control_roll);
                    }else{
                        // override pitch angle
                        if (auto_tune_state.positive_direction) {
                            control_pitch = AUTO_TUNE_TARGET_ANGLE_CD;
                        }else{
                            control_pitch = -AUTO_TUNE_TARGET_ANGLE_CD;
                        }
                        get_stabilize_pitch(control_pitch);
                    }
                } else {
                    if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                        // override roll rate
                        if (auto_tune_state.positive_direction) {
                            target_roll_rate = AUTO_TUNE_TARGET_RATE_CDS;
                        }else{
                            target_roll_rate = -AUTO_TUNE_TARGET_RATE_CDS;
                        }
                        // set body frame targets for rate controller
                        set_roll_rate_target(target_roll_rate, BODY_FRAME);
                        set_pitch_rate_target(acro_pitch_rate, BODY_FRAME);
                        set_yaw_rate_target(acro_yaw_rate, BODY_FRAME);
                    }else{
                        // override pitch rate
                        if (auto_tune_state.positive_direction) {
                            target_pitch_rate = AUTO_TUNE_TARGET_RATE_CDS;
                        }else{
                            target_pitch_rate = -AUTO_TUNE_TARGET_RATE_CDS;
                        }
                        // set body frame targets for rate controller
                        set_pitch_rate_target(target_pitch_rate, BODY_FRAME);
                        set_roll_rate_target(acro_roll_rate, BODY_FRAME);
                        set_yaw_rate_target(acro_yaw_rate, BODY_FRAME);
                    }
                    rate_targets_frame = BODY_EARTH_FRAME;
                }
                
                
                // Get Rate and Angle
                if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                    // 20 Hz filter on rate
                    rotation_rate = ToDeg(fabs(ahrs.get_gyro().x)) * 100.0f;
                    lean_angle = labs(ahrs.roll_sensor);
                }else{
                    // 20 Hz filter on rate
                    // rotation_rate = rotation_rate + 0.55686f*(ToDeg(fabs(ahrs.get_gyro().y))*100.0f-rotation_rate);
                    rotation_rate = ToDeg(fabs(ahrs.get_gyro().y)) * 100.0f;
                    lean_angle = labs(ahrs.pitch_sensor);
                }

                // Make measurements
                if(auto_tune_state.tune_type == AUTO_TUNE_TYPE_SP_UP){
                    // capture max angle
                    if (lean_angle > auto_tune_test_max) {
                        auto_tune_test_max = lean_angle;
                        auto_tune_test_min = lean_angle;
                    }
                    
                    // capture min rotation rate
                    if (lean_angle < auto_tune_test_min && auto_tune_test_max > AUTO_TUNE_TARGET_ANGLE_CD*(1-AUTO_TUNE_AGGRESSIVENESS)) {
                        auto_tune_test_min = lean_angle;
                    }
                }else{
                    // capture max rotation rate
                    if (rotation_rate > auto_tune_test_max) {
                        auto_tune_test_max = rotation_rate;
                        auto_tune_test_min = rotation_rate;
                    }
                    
                    // capture min rotation rate
                    if (rotation_rate < auto_tune_test_min && auto_tune_test_max > AUTO_TUNE_TARGET_RATE_CDS*(1-2*AUTO_TUNE_AGGRESSIVENESS)) {
                        auto_tune_test_min = rotation_rate;
                    }
                }
        
                // check for end of test conditions
                if(millis() - auto_tune_timer >= AUTO_TUNE_TARGET_RATE_TIMEOUT_MS) {
                        auto_tune_state.step = AUTO_TUNE_STEP_UPDATE_GAINS;
                }
                if(auto_tune_state.tune_type == AUTO_TUNE_TYPE_SP_UP){
                    if ((lean_angle >= AUTO_TUNE_TARGET_ANGLE_CD*(1+AUTO_TUNE_AGGRESSIVENESS)) || 
                            (auto_tune_test_max-auto_tune_test_min > AUTO_TUNE_TARGET_ANGLE_CD*AUTO_TUNE_AGGRESSIVENESS)) {
                        auto_tune_state.step = AUTO_TUNE_STEP_UPDATE_GAINS;
                    }
                }else{
                    if (lean_angle >= AUTO_TUNE_TARGET_ANGLE_CD) {
                        auto_tune_state.step = AUTO_TUNE_STEP_UPDATE_GAINS;
                    }
                    if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_RD_UP || auto_tune_state.tune_type == AUTO_TUNE_TYPE_RD_DOWN) {
                        if(auto_tune_test_max-auto_tune_test_min > AUTO_TUNE_TARGET_RATE_CDS*AUTO_TUNE_AGGRESSIVENESS) {
                            auto_tune_state.step = AUTO_TUNE_STEP_UPDATE_GAINS;
                        }
                    }
                }

                // logging
                Log_Write_AutoTuneDetails((int16_t)lean_angle, rotation_rate);
                break;

            case AUTO_TUNE_STEP_UPDATE_GAINS:
                // restore gains to their intra-test values
                auto_tune_intra_test_gains();

                // logging
                if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                    Log_Write_AutoTune(auto_tune_state.axis, auto_tune_state.tune_type, auto_tune_test_min, auto_tune_test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp);
                }else{
                    Log_Write_AutoTune(auto_tune_state.axis, auto_tune_state.tune_type, auto_tune_test_min, auto_tune_test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp);
                }

                // do gain updates
                if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_RD_UP) {
                    if (auto_tune_test_max > AUTO_TUNE_TARGET_RATE_CDS) {
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp -= AUTO_TUNE_RP_STEP;
                        }else{
                            tune_pitch_rp -= AUTO_TUNE_RP_STEP;
                        }
                        // stop the auto tune if we have hit the minimum roll or pitch rate P
                        if(((auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL && tune_roll_rp < AUTO_TUNE_RP_MIN) ||
                            (auto_tune_state.axis == AUTO_TUNE_AXIS_PITCH && tune_pitch_rp < AUTO_TUNE_RP_MIN)) ) {
                            auto_tune_state.mode = AUTO_TUNE_MODE_FAILED;
                            Log_Write_Event(DATA_AUTOTUNE_ABANDONED);
                            set_roll_pitch_mode(ROLL_PITCH_STABLE);
                            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_FAILED);
                            return;
                        }
                    }else if(auto_tune_test_max < AUTO_TUNE_TARGET_RATE_CDS*(1.0f-AUTO_TUNE_AGGRESSIVENESS*2.0f) && 
                            ((auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL && tune_roll_rp <= AUTO_TUNE_RP_MAX) ||
                            (auto_tune_state.axis == AUTO_TUNE_AXIS_PITCH && tune_pitch_rp <= AUTO_TUNE_RP_MAX)) ) {
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp += AUTO_TUNE_RP_STEP*2.0f;
                        }else{
                            tune_pitch_rp += AUTO_TUNE_RP_STEP*2.0f;
                        }
                    }else{
                        if (auto_tune_test_max-auto_tune_test_min > AUTO_TUNE_TARGET_RATE_CDS*AUTO_TUNE_AGGRESSIVENESS) {
                            auto_tune_counter++;
                        }else{
                            if (auto_tune_counter > 0 ) {
                                auto_tune_counter--;
                            }
                            if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                                tune_roll_rd += AUTO_TUNE_RD_STEP*2.0f;
                                // stop tuning if we hit max D
                                if (tune_roll_rd >= AUTO_TUNE_RD_MAX) {
                                    tune_roll_rd = AUTO_TUNE_RD_MAX;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                                }
                            }else{
                                tune_pitch_rd += AUTO_TUNE_RD_STEP*2.0f;
                                // stop tuning if we hit max D
                                if (tune_pitch_rd >= AUTO_TUNE_RD_MAX) {
                                    tune_pitch_rd = AUTO_TUNE_RD_MAX;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                                }
                            }
                        }
                    }
                } else if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_RD_DOWN) {
                    if (auto_tune_test_max > AUTO_TUNE_TARGET_RATE_CDS) {
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp -= AUTO_TUNE_RP_STEP;
                        }else{
                            tune_pitch_rp -= AUTO_TUNE_RP_STEP;
                        }
                        // stop the auto tune if we have hit the minimum roll or pitch rate P
                        if(((auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL && tune_roll_rp < AUTO_TUNE_RP_MIN) ||
                            (auto_tune_state.axis == AUTO_TUNE_AXIS_PITCH && tune_pitch_rp < AUTO_TUNE_RP_MIN)) ) {
                            auto_tune_state.mode = AUTO_TUNE_MODE_FAILED;
                            Log_Write_Event(DATA_AUTOTUNE_ABANDONED);
                            set_roll_pitch_mode(ROLL_PITCH_STABLE);
                            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_FAILED);
                            return;
                        }
                    }else if(auto_tune_test_max < AUTO_TUNE_TARGET_RATE_CDS*(1-AUTO_TUNE_AGGRESSIVENESS*2.0f) && 
                            ((auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL && tune_roll_rp <= AUTO_TUNE_RP_MAX) ||
                            (auto_tune_state.axis == AUTO_TUNE_AXIS_PITCH && tune_pitch_rp <= AUTO_TUNE_RP_MAX)) ) {
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp += AUTO_TUNE_RP_STEP;
                        }else{
                            tune_pitch_rp += AUTO_TUNE_RP_STEP;
                        }
                    }else{
                        if (auto_tune_test_max-auto_tune_test_min < AUTO_TUNE_TARGET_RATE_CDS*AUTO_TUNE_AGGRESSIVENESS) {
                            auto_tune_counter++;
                        }else{
                            if (auto_tune_counter > 0 ) {
                                auto_tune_counter--;
                            }
                            if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                                tune_roll_rd -= AUTO_TUNE_RD_STEP;
                                // stop tuning if we hit max D
                                if (tune_roll_rd <= AUTO_TUNE_RD_MIN) {
                                    tune_roll_rd = AUTO_TUNE_RD_MIN;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                                }
                            }else{
                                tune_pitch_rd -= AUTO_TUNE_RD_STEP;
                                // stop tuning if we hit max D
                                if (tune_pitch_rd <= AUTO_TUNE_RD_MIN) {
                                    tune_pitch_rd = AUTO_TUNE_RD_MIN;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                                }
                            }
                        }
                    }
                } else if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_RP_UP) {
                    if (auto_tune_test_max > AUTO_TUNE_TARGET_RATE_CDS) {
                        auto_tune_counter++;
                    }else{
                        if (auto_tune_counter > 0 ) {
                            auto_tune_counter--;
                        }
                        // increase P & I or D term
                        // update PI term
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp += AUTO_TUNE_RP_STEP;
                            // stop tuning if we hit max P
                            if (tune_roll_rp >= AUTO_TUNE_RP_MAX) {
                                tune_roll_rp = AUTO_TUNE_RP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                            }
                        }else{
                            tune_pitch_rp += AUTO_TUNE_RP_STEP;
                            // stop tuning if we hit max P
                            if (tune_pitch_rp >= AUTO_TUNE_RP_MAX) {
                                tune_pitch_rp = AUTO_TUNE_RP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                            }
                        }
                    }
                } else if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_SP_UP) {
                    if (auto_tune_test_max > AUTO_TUNE_TARGET_ANGLE_CD*(1+AUTO_TUNE_AGGRESSIVENESS) ||
                            (auto_tune_test_max-auto_tune_test_min > AUTO_TUNE_TARGET_ANGLE_CD*AUTO_TUNE_AGGRESSIVENESS)) {
                        auto_tune_counter++;
                    }else{
                        if (auto_tune_counter > 0 ) {
                            auto_tune_counter--;
                        }
                        // increase P & I or D term
                        // update PI term
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_sp += AUTO_TUNE_SP_STEP;
                            // stop tuning if we hit max P
                            if (tune_roll_sp >= AUTO_TUNE_SP_MAX) {
                                tune_roll_sp = AUTO_TUNE_SP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                            }
                        }else{
                            tune_pitch_sp += AUTO_TUNE_SP_STEP;
                            // stop tuning if we hit max P
                            if (tune_pitch_sp >= AUTO_TUNE_SP_MAX) {
                                tune_pitch_sp = AUTO_TUNE_SP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                            }
                        }
                    }
                }

                // reverse direction
                auto_tune_state.positive_direction = !auto_tune_state.positive_direction;

                // we've complete this step, finalise pids and move to next step
                if (auto_tune_counter >= AUTO_TUNE_SUCCESS_COUNT) {

                    // reset counter
                    auto_tune_counter = 0;

                    // move to the next tuning type
                    if (auto_tune_state.tune_type < AUTO_TUNE_TYPE_SP_UP) {
                        auto_tune_state.tune_type++;
                    }else{
                        // we've reached the end of a D-up-down PI-up-down tune type cycle
                        auto_tune_state.tune_type = AUTO_TUNE_TYPE_RD_UP;
                        
                        // if we've just completed roll move onto pitch
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_sp = tune_roll_sp * AUTO_TUNE_SP_BACKOFF;
                            auto_tune_state.axis = AUTO_TUNE_AXIS_PITCH;
                        }else{
                            tune_pitch_sp = tune_pitch_sp * AUTO_TUNE_SP_BACKOFF;
                            tune_roll_sp = min(tune_roll_sp, tune_pitch_sp);
                            tune_pitch_sp = min(tune_roll_sp, tune_pitch_sp);
                            // if we've just completed pitch we are done tuning and are moving onto testing
                            auto_tune_state.mode = AUTO_TUNE_MODE_TESTING;
                            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_SUCCESS);
                            Log_Write_Event(DATA_AUTOTUNE_COMPLETE);
                            set_roll_pitch_mode(ROLL_PITCH_STABLE);
                        }
                    }
                }

                // reset testing step
                auto_tune_state.step = AUTO_TUNE_STEP_WAITING_FOR_LEVEL;
                auto_tune_timer = millis();
                break;
        }
    }
}
#endif  // AUTOTUNE == ENABLED