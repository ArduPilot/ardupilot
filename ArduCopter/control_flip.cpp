/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_flip.pde - init and run calls for flip flight mode
 *      original implementation in 2010 by Jose Julio
 *      Adapted and updated for AC2 in 2011 by Jason Short
 *
 *      Controls:
 *          CH7_OPT - CH12_OPT parameter must be set to "Flip" (AUXSW_FLIP) which is "2"
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Vehicle will Roll right by default but if roll or pitch stick is held slightly left, forward or back it will flip in that direction
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 *
 *      State machine approach:
 *          Flip_Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          Flip_Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          Flip_Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 */

#define FLIP_THR_INC        200     // throttle increase during Flip_Start stage (under 45deg lean angle)
#define FLIP_THR_DEC        240     // throttle decrease during Flip_Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROTATION_RATE  40000   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define FLIP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define FLIP_RECOVERY_ANGLE 500     // consider successful recovery when roll is back within 5 degrees of original

#define FLIP_ROLL_RIGHT      1      // used to set flip_dir
#define FLIP_ROLL_LEFT      -1      // used to set flip_dir

#define FLIP_PITCH_BACK      1      // used to set flip_dir
#define FLIP_PITCH_FORWARD  -1      // used to set flip_dir

FlipState flip_state;               // current state of flip
uint8_t   flip_orig_control_mode;   // flight mode when flip was initated
uint32_t  flip_start_time;          // time since flip began
int8_t    flip_roll_dir;            // roll direction (-1 = roll left, 1 = roll right)
int8_t    flip_pitch_dir;           // pitch direction (-1 = pitch forward, 1 = pitch back)

// flip_init - initialise flip controller
bool Copter::flip_init(bool ignore_checks)
{
    // only allow flip from ACRO, Stabilize, AltHold or Drift flight modes
    if (control_mode != ACRO && control_mode != STABILIZE && control_mode != ALT_HOLD) {
        return false;
    }

    // if in acro or stabilize ensure throttle is above zero
    if (ap.throttle_zero && (control_mode == ACRO || control_mode == STABILIZE)) {
        return false;
    }

    // ensure roll input is less than 40deg
    if (abs(channel_roll->control_in) >= 4000) {
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

    flip_roll_dir = flip_pitch_dir = 0;

    // choose direction based on pilot's roll and pitch sticks
    if (channel_pitch->control_in > 300) {
        flip_pitch_dir = FLIP_PITCH_BACK;
    }else if(channel_pitch->control_in < -300) {
        flip_pitch_dir = FLIP_PITCH_FORWARD;
    }else if (channel_roll->control_in >= 0) {
        flip_roll_dir = FLIP_ROLL_RIGHT;
    }else{
        flip_roll_dir = FLIP_ROLL_LEFT;
    }

    // log start of flip
    Log_Write_Event(DATA_FLIP_START);

    // capture current attitude which will be used during the Flip_Recovery stage
    flip_orig_attitude.x = constrain_float(ahrs.roll_sensor, -aparm.angle_max, aparm.angle_max);
    flip_orig_attitude.y = constrain_float(ahrs.pitch_sensor, -aparm.angle_max, aparm.angle_max);
    flip_orig_attitude.z = ahrs.yaw_sensor;

    return true;
}

// flip_run - runs the flip controller
// should be called at 100hz or more
void Copter::flip_run()
{
    int16_t throttle_out;
    float recovery_angle;

    // if pilot inputs roll > 40deg or timeout occurs abandon flip
    if (!motors.armed() || (abs(channel_roll->control_in) >= 4000) || (abs(channel_pitch->control_in) >= 4000) || ((millis() - flip_start_time) > FLIP_TIMEOUT_MS)) {
        flip_state = Flip_Abandon;
    }

    // get pilot's desired throttle
    throttle_out = get_pilot_desired_throttle(channel_throttle->control_in);

    // get corrected angle based on direction and axis of rotation
    // we flip the sign of flip_angle to minimize the code repetition
    int32_t flip_angle;

    if (flip_roll_dir != 0) {
        flip_angle = ahrs.roll_sensor * flip_roll_dir;
    } else {
        flip_angle = ahrs.pitch_sensor * flip_pitch_dir;
    }

    // state machine
    switch (flip_state) {

    case Flip_Start:
        // under 45 degrees request 400deg/sec roll or pitch
        attitude_control.input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * flip_roll_dir, FLIP_ROTATION_RATE * flip_pitch_dir, 0.0);

        // increase throttle
        throttle_out += FLIP_THR_INC;

        // beyond 45deg lean angle move to next stage
        if (flip_angle >= 4500) {
            if (flip_roll_dir != 0) {
                // we are rolling
            flip_state = Flip_Roll;
            } else {
                // we are pitching
                flip_state = Flip_Pitch_A;
        }
        }
        break;

    case Flip_Roll:
        // between 45deg ~ -90deg request 400deg/sec roll
        attitude_control.input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * flip_roll_dir, 0.0, 0.0);
        // decrease throttle
        if (throttle_out >= g.throttle_min) {
            throttle_out = MAX(throttle_out - FLIP_THR_DEC, g.throttle_min);
        }

        // beyond -90deg move on to recovery
        if ((flip_angle < 4500) && (flip_angle > -9000)) {
            flip_state = Flip_Recover;
        }
        break;

    case Flip_Pitch_A:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control.input_rate_bf_roll_pitch_yaw(0.0, FLIP_ROTATION_RATE * flip_pitch_dir, 0.0);
        // decrease throttle
        if (throttle_out >= g.throttle_min) {
            throttle_out = MAX(throttle_out - FLIP_THR_DEC, g.throttle_min);
        }

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) > 9000) && (flip_angle > 4500)) {
            flip_state = Flip_Pitch_B;
        }
        break;

    case Flip_Pitch_B:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control.input_rate_bf_roll_pitch_yaw(0.0, FLIP_ROTATION_RATE * flip_pitch_dir, 0.0);
        // decrease throttle
        if (throttle_out >= g.throttle_min) {
            throttle_out = MAX(throttle_out - FLIP_THR_DEC, g.throttle_min);
        }

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) < 9000) && (flip_angle > -4500)) {
            flip_state = Flip_Recover;
        }
        break;

    case Flip_Recover:
        // use originally captured earth-frame angle targets to recover
        attitude_control.input_euler_angle_roll_pitch_yaw(flip_orig_attitude.x, flip_orig_attitude.y, flip_orig_attitude.z, false);

        // increase throttle to gain any lost altitude
        throttle_out += FLIP_THR_INC;

        if (flip_roll_dir != 0) {
            // we are rolling
            recovery_angle = fabsf(flip_orig_attitude.x - (float)ahrs.roll_sensor);
        } else {
            // we are pitching
            recovery_angle = fabsf(flip_orig_attitude.y - (float)ahrs.pitch_sensor);
        }

        // check for successful recovery
        if (fabsf(recovery_angle) <= FLIP_RECOVERY_ANGLE) {
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
    if (throttle_out == 0) {
        attitude_control.set_throttle_out_unstabilized(0,false,g.throttle_filt);
    } else {
        attitude_control.set_throttle_out(throttle_out, false, g.throttle_filt);
    }
}
