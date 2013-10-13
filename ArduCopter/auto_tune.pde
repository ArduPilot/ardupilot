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
#define AUTO_TUNE_RD_MIN                    0.0f    // minimum Rate D value
#define AUTO_TUNE_RD_MAX                    0.1f    // maximum Rate D value
#define AUTO_TUNE_RP_MIN                   0.02f    // minimum Rate P value
#define AUTO_TUNE_RP_MAX                    1.0f    // maximum Rate P value
#define AUTO_TUNE_SP_MIN                    1.0f    // minimum Stab P value
#define AUTO_TUNE_SP_MAX                   15.0f    // maximum Stab P value
#define AUTO_TUNE_SUCCESS_COUNT                4    // how many successful iterations we need to freeze at current gains

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
    uint8_t enabled             : 1;    // 0 = disabled, 1 = enabled
    uint8_t active              : 1;    // 0 = inactive (temporarily suspended), 1 = actively tuning
    uint8_t pilot_override      : 1;    // 1 = pilot is overriding controls so we suspend tuning temporarily
    AutoTuneAxisType axis       : 1;    // see AutoTuneAxisType for which things can be tuned
    uint8_t positive_direction  : 1;    // 0 = tuning in negative direction (i.e. left for roll), 1 = positive direction (i.e. right for roll)
    AutoTuneStepType step       : 2;    // see AutoTuneStepType for what steps are performed
    AutoTuneTuneType tune_type  : 2;    // see AutoTuneTuneType
} auto_tune_state;

// variables
uint32_t auto_tune_override_time;   // the last time the pilot overrode the controls
float auto_tune_test_min;           // the minimum angular rate achieved during TESTING_RATE step
float auto_tune_test_max;           // the maximum angular rate achieved during TESTING_RATE step
uint32_t auto_tune_timer;           // generic timer variable
int8_t auto_tune_counter;                       // counter for tuning gains
float orig_roll_rp, orig_roll_ri, orig_roll_rd, orig_roll_sp;    // backup of currently being tuned parameter values
float orig_pitch_rp, orig_pitch_ri, orig_pitch_rd, orig_pitch_sp; // backup of currently being tuned parameter values
float tune_roll_rp, tune_roll_rd, tune_roll_sp;    // currently being tuned parameter values
float tune_pitch_rp, tune_pitch_rd, tune_pitch_sp;  // currently being tuned parameter values

// store current pids as originals
void auto_tune_save_orig_pids()
{
    orig_roll_rp = g.pid_rate_roll.kP();
    orig_roll_ri = g.pid_rate_roll.kI();
    orig_roll_rd = g.pid_rate_roll.kD();
    orig_roll_sp = g.pi_stabilize_roll.kP();
    orig_pitch_rp = g.pid_rate_pitch.kP();
    orig_pitch_ri = g.pid_rate_pitch.kI();
    orig_pitch_rd = g.pid_rate_pitch.kD();
    orig_pitch_sp = g.pi_stabilize_pitch.kP();
}

// auto_tune_restore_orig_pids - restore pids to their original values
void auto_tune_restore_orig_pids()
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

// auto_tune_load_tuned_pids - restore pids to their original values
void auto_tune_load_tuned_pids()
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

// start an auto tuning session
void auto_tune_start()
{
    // check we are in stabilize mode
    if (control_mode == STABILIZE || control_mode == ALT_HOLD) {
        // reset axis if this is our first time (i.e. we were not just suspended)
        if (!auto_tune_state.active) {
            auto_tune_state.active = true;
            auto_tune_state.axis = AUTO_TUNE_AXIS_ROLL;
            auto_tune_state.positive_direction = false;
            auto_tune_state.step = AUTO_TUNE_STEP_WAITING_FOR_LEVEL;
            auto_tune_timer = millis();
            auto_tune_state.tune_type = AUTO_TUNE_TYPE_RD_UP;
            auto_tune_save_orig_pids();
            // initialise tuned pid values
            tune_roll_rp = g.pid_rate_roll.kP();
            tune_roll_rd = g.pid_rate_roll.kD();
            tune_roll_sp = g.pi_stabilize_roll.kP();
            tune_pitch_rp = g.pid_rate_pitch.kP();
            tune_pitch_rd = g.pid_rate_pitch.kD();
            tune_pitch_sp = g.pi_stabilize_pitch.kP();
        }else{
            // restarting from suspended state
            auto_tune_restore_orig_pids();
        }
        // set roll-pitch mode to our special auto tuning stabilize roll-pitch mode
        if (set_roll_pitch_mode(ROLL_PITCH_AUTOTUNE)) {
            auto_tune_state.enabled = true;
            Log_Write_Event(DATA_AUTOTUNE_ON);
        }
    }
}

// turn off tuning and return to standard pids
void auto_tune_stop()
{
    if (auto_tune_state.enabled) {
        auto_tune_state.enabled = false;
        auto_tune_state.active = false;
        ap.disable_stab_rate_limit = false;
        if (roll_pitch_mode == ROLL_PITCH_AUTOTUNE) {
            set_roll_pitch_mode(ROLL_PITCH_STABLE); // restore roll-pitch mode
            rate_targets_frame = EARTH_FRAME;   // regular stabilize mode frame
        }
        // restore pids to their original values
        auto_tune_restore_orig_pids();
        Log_Write_Event(DATA_AUTOTUNE_OFF);
    }
}

// stop tuning but remain with tuned pids
void auto_tune_suspend()
{
    auto_tune_load_tuned_pids();
    if (auto_tune_state.active) {
        auto_tune_state.active = false;
        Log_Write_Event(DATA_AUTOTUNE_SUSPENDED);
    }
}

// save discovered gains to eeprom if auto tuner is enabled (i.e. switch is in middle or high position)
void auto_tune_save_tuning_gains()
{
    if (auto_tune_state.enabled) {
        auto_tune_load_tuned_pids();
        g.pid_rate_roll.save_gains();
        g.pid_rate_pitch.save_gains();
        g.pi_stabilize_roll.save_gains();
        g.pi_stabilize_pitch.save_gains();
        Log_Write_Event(DATA_AUTOTUNE_SAVEDGAINS);
    }
}

// Auto tuning roll-pitch controller
static void
get_autotune_roll_pitch_controller(int32_t pilot_roll_angle, int32_t pilot_pitch_angle)
{
    int32_t target_roll_rate, target_pitch_rate;
    float rotation_rate;        // rotation rate in radians/second
    int32_t lean_angle;

    // exit immediately if not enabled or not actively tuning
    if (!auto_tune_state.enabled || !auto_tune_state.active) {
        return;
    }

    // check for pilot override
    if (pilot_roll_angle != 0 || pilot_pitch_angle != 0 ) {
        if (!auto_tune_state.pilot_override) {
            // restore pids to their original values
            auto_tune_restore_orig_pids();
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
                // restore pids to their original values
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
                            Log_Write_Event(DATA_AUTOTUNE_ABANDONED);
                            auto_tune_stop();
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
                                }
                            }else{
                                tune_pitch_rd += AUTO_TUNE_RD_STEP*2.0f;
                                // stop tuning if we hit max D
                                if (tune_pitch_rd >= AUTO_TUNE_RD_MAX) {
                                    tune_pitch_rd = AUTO_TUNE_RD_MAX;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
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
                            Log_Write_Event(DATA_AUTOTUNE_ABANDONED);
                            auto_tune_stop();
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
                                }
                            }else{
                                tune_pitch_rd -= AUTO_TUNE_RD_STEP;
                                // stop tuning if we hit max D
                                if (tune_pitch_rd <= AUTO_TUNE_RD_MIN) {
                                    tune_pitch_rd = AUTO_TUNE_RD_MIN;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
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
                            }
                        }else{
                            tune_pitch_rp += AUTO_TUNE_RP_STEP;
                            // stop tuning if we hit max P
                            if (tune_pitch_rp >= AUTO_TUNE_RP_MAX) {
                                tune_pitch_rp = AUTO_TUNE_RP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
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
                            }
                        }else{
                            tune_pitch_sp += AUTO_TUNE_SP_STEP;
                            // stop tuning if we hit max P
                            if (tune_pitch_sp >= AUTO_TUNE_SP_MAX) {
                                tune_pitch_sp = AUTO_TUNE_SP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
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
                            // if we've just completed pitch we are done so suspend tuning
                            auto_tune_suspend();
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