/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_flip.pde - init and run calls for flip flight mode
 *      original implementation in 2010 by Jose Julio
 *      Adapted and updated for AC2 in 2011 by Jason Short
 *
 *      Controls:
 *          CH7_OPT or CH8_OPT parameter must be set to "Flip" (AUX_SWITCH_FLIP)
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Roll will be to the left is roll stick is held slightly left, otherwise it will roll right
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 *
 *      State machine approach:
 *          Flip_Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          Flip_Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          Flip_Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 *               Note: this final stage relies upon the AttitudeControl libraries angle_ef_targets having been saved by the attitude_control.init_targets() call and not modified afterwards
 */

#define FLIP_THR_INC        170     // throttle increase during Flip_Start stage (under 45deg lean angle)
#define FLIP_THR_DEC        120     // throttle decrease during Flip_Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROLL_RATE      40000   // roll rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define FLIP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define FLIP_RECOVERY_ANGLE 500     // consider successful recovery when roll is back within 5 degrees of original

#define FLIP_ROLL_RIGHT      1      // used to set flip_dir
#define FLIP_ROLL_LEFT      -1      // used to set flip_dir

FlipState flip_state;               // current state of flip
uint8_t   flip_orig_control_mode;   // flight mode when flip was initated
uint32_t  flip_start_time;          // time since flip began
int8_t    flip_dir;                 // roll direction (-1 = roll left, 1 = roll right)

// flip_init - initialise flip controller
static bool flip_init(bool ignore_checks)
{
    // only allow flip from ACRO, Stabilize, AltHold or Drift flight modes
    if (control_mode != ACRO && control_mode != STABILIZE && control_mode != ALT_HOLD) {
        return false;
    }

    // if in acro or stabilize ensure throttle is above zero
    if ((g.rc_3.control_in <= 0) && (control_mode == ACRO || control_mode == STABILIZE)) {
        return false;
    }

    // ensure roll input is less than 40deg
    if (abs(g.rc_1.control_in) >= 4000) {
        return false;
    }

    // only allow flip when flying
    if (!motors.armed() || ap.land_complete) {
        return false;
    }

    // capture original flight mode so that we can return to it after completion
    flip_orig_control_mode = control_mode;

    // initialise state
    flip_state = Flip_Start;
    flip_start_time = millis();

    // choose direction based on pilot's roll stick
    if (g.rc_1.control_in >= 0) {
        flip_dir = FLIP_ROLL_RIGHT;
    }else{
        flip_dir = FLIP_ROLL_LEFT;
    }

    // log start of flip
    Log_Write_Event(DATA_FLIP_START);

    // capture current attitude in angle_ef_targets while will be used as attitude targets during the Flip_Recovery stage
    // clear stabilized rate errors which will be used during the Flip_Start and Flip_Roll stages
    attitude_control.init_targets();

    return true;
}

// flip_abandon - pilot request to abandon flip
static void flip_stop()
{
    // exit immediatley if not in flip mode
    if (control_mode != FLIP) {
        return;
    }

    // return to original flip mode
    if (!set_mode(flip_orig_control_mode)) {
        // this should never happen but just in case
        set_mode(STABILIZE);
    }

    // log completion
    Log_Write_Event(DATA_FLIP_END);
}

// flip_run - runs the flip controller
// should be called at 100hz or more
static void flip_run()
{
    const Vector3f curr_ef_targets = attitude_control.angle_ef_targets();   // original earth-frame angle targets to recover
    int16_t throttle_out;

    // if pilot inputs roll > 40deg or timeout occurs abandon flip
    if (!motors.armed() || (abs(g.rc_1.control_in) >= 4000) || ((millis() - flip_start_time) > FLIP_TIMEOUT_MS)) {
        flip_state = Flip_Abandon;
    }

    // get pilot's desired throttle
    throttle_out = get_pilot_desired_throttle(g.rc_3.control_in);

    // get roll rate
    int32_t roll_angle = ahrs.roll_sensor * flip_dir;

    // state machine
    switch (flip_state) {

    case Flip_Start:
        // under 45 degrees request 400deg/sec roll
        attitude_control.rate_bf_roll_pitch_yaw(FLIP_ROLL_RATE * flip_dir, 0.0, 0.0);
        // increase throttle
        throttle_out += FLIP_THR_INC;
        // beyond 45deg lean angle move to next stage
        if (roll_angle >= 4500) {
            flip_state = Flip_Roll;
        }
        break;

    case Flip_Roll:
        // between 45deg ~ -90deg request 400deg/sec roll
        attitude_control.rate_bf_roll_pitch_yaw(FLIP_ROLL_RATE * flip_dir, 0.0, 0.0);
        // decrease throttle
        throttle_out -= FLIP_THR_DEC;
        // beyond -90deg move on to recovery
        if((roll_angle < 4500) && (roll_angle > -9000)) {
            flip_state = Flip_Recover;
        }
        break;

    case Flip_Recover:
        // use originally captured earth-frame angle targets to recover
        attitude_control.angle_ef_roll_pitch_yaw(curr_ef_targets.x, curr_ef_targets.y, curr_ef_targets.z,false);

        // increase throttle to gain any lost alitude
        throttle_out += FLIP_THR_INC;

        // check for successful recovery
        if (fabs(curr_ef_targets.x - (float)ahrs.roll_sensor) <= FLIP_RECOVERY_ANGLE) {
            // restore original flight mode
            if (!set_mode(flip_orig_control_mode)) {
                // this should never happen but just in case
                set_mode(STABILIZE);
            }
            // log successful completion
            Log_Write_Event(DATA_FLIP_END);
        }
        break;

    case Flip_Abandon:
        // restore original flight mode
        if (!set_mode(flip_orig_control_mode)) {
            // this should never happen but just in case
            set_mode(STABILIZE);
        }
        // log abandoning flip
        Log_Write_Error(ERROR_SUBSYSTEM_FLIP,ERROR_CODE_FLIP_ABANDONED);
        break;
    }

    // output pilot's throttle without angle boost
    attitude_control.set_throttle_out(throttle_out, false);
}
