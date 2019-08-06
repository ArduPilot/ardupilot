#include "Copter.h"

#if MODE_FLIP_ENABLED == ENABLED

/*
 * Init and run calls for flip flight mode
 *      original implementation in 2010 by Jose Julio
 *      Adapted and updated for AC2 in 2011 by Jason Short
 *
 *      Controls:
 *          RC7_OPTION - RC12_OPTION parameter must be set to "Flip" (AUXSW_FLIP) which is "2"
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Vehicle will Roll right by default but if roll or pitch stick is held slightly left, forward or back it will flip in that direction
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 *
 *      State machine approach:
 *          FlipState::Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          FlipState::Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          FlipState::Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 */

#define FLIP_THR_INC        0.20f   // throttle increase during FlipState::Start stage (under 45deg lean angle)
#define FLIP_THR_DEC        0.24f   // throttle decrease during FlipState::Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROTATION_RATE  40000   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define FLIP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define FLIP_RECOVERY_ANGLE 500     // consider successful recovery when roll is back within 5 degrees of original

#define FLIP_ROLL_RIGHT      1      // used to set flip_dir
#define FLIP_ROLL_LEFT      -1      // used to set flip_dir

#define FLIP_PITCH_BACK      1      // used to set flip_dir
#define FLIP_PITCH_FORWARD  -1      // used to set flip_dir

// flip_init - initialise flip controller
bool Copter::ModeFlip::init(bool ignore_checks)
{
    // only allow flip from ACRO, Stabilize, AltHold or Drift flight modes
    if (copter.control_mode != ACRO &&
        copter.control_mode != STABILIZE &&
        copter.control_mode != ALT_HOLD &&
        copter.control_mode != FLOWHOLD) {
        return false;
    }

    // if in acro or stabilize ensure throttle is above zero
    if (ap.throttle_zero && (copter.control_mode == ACRO || copter.control_mode == STABILIZE)) {
        return false;
    }

    // ensure roll input is less than 40deg
    if (abs(channel_roll->get_control_in()) >= 4000) {
        return false;
    }

    // only allow flip when flying
    if (!motors->armed() || ap.land_complete) {
        return false;
    }

    // capture original flight mode so that we can return to it after completion
    orig_control_mode = copter.control_mode;

    // initialise state
    _state = FlipState::Start;
    start_time_ms = millis();

    roll_dir = pitch_dir = 0;

    // choose direction based on pilot's roll and pitch sticks
    if (channel_pitch->get_control_in() > 300) {
        pitch_dir = FLIP_PITCH_BACK;
    } else if (channel_pitch->get_control_in() < -300) {
        pitch_dir = FLIP_PITCH_FORWARD;
    } else if (channel_roll->get_control_in() >= 0) {
        roll_dir = FLIP_ROLL_RIGHT;
    } else {
        roll_dir = FLIP_ROLL_LEFT;
    }

    // log start of flip
    Log_Write_Event(DATA_FLIP_START);

    // capture current attitude which will be used during the FlipState::Recovery stage
    const float angle_max = copter.aparm.angle_max;
    orig_attitude.x = constrain_float(ahrs.roll_sensor, -angle_max, angle_max);
    orig_attitude.y = constrain_float(ahrs.pitch_sensor, -angle_max, angle_max);
    orig_attitude.z = ahrs.yaw_sensor;

    return true;
}

// run - runs the flip controller
// should be called at 100hz or more
void Copter::ModeFlip::run()
{
    // if pilot inputs roll > 40deg or timeout occurs abandon flip
    if (!motors->armed() || (abs(channel_roll->get_control_in()) >= 4000) || (abs(channel_pitch->get_control_in()) >= 4000) || ((millis() - start_time_ms) > FLIP_TIMEOUT_MS)) {
        _state = FlipState::Abandon;
    }

    // get pilot's desired throttle
    float throttle_out = get_pilot_desired_throttle();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get corrected angle based on direction and axis of rotation
    // we flip the sign of flip_angle to minimize the code repetition
    int32_t flip_angle;

    if (roll_dir != 0) {
        flip_angle = ahrs.roll_sensor * roll_dir;
    } else {
        flip_angle = ahrs.pitch_sensor * pitch_dir;
    }

    // state machine
    switch (_state) {

    case FlipState::Start:
        // under 45 degrees request 400deg/sec roll or pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * roll_dir, FLIP_ROTATION_RATE * pitch_dir, 0.0);

        // increase throttle
        throttle_out += FLIP_THR_INC;

        // beyond 45deg lean angle move to next stage
        if (flip_angle >= 4500) {
            if (roll_dir != 0) {
                // we are rolling
            _state = FlipState::Roll;
            } else {
                // we are pitching
                _state = FlipState::Pitch_A;
        }
        }
        break;

    case FlipState::Roll:
        // between 45deg ~ -90deg request 400deg/sec roll
        attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * roll_dir, 0.0, 0.0);
        // decrease throttle
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // beyond -90deg move on to recovery
        if ((flip_angle < 4500) && (flip_angle > -9000)) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Pitch_A:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, FLIP_ROTATION_RATE * pitch_dir, 0.0);
        // decrease throttle
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) > 9000) && (flip_angle > 4500)) {
            _state = FlipState::Pitch_B;
        }
        break;

    case FlipState::Pitch_B:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0, FLIP_ROTATION_RATE * pitch_dir, 0.0);
        // decrease throttle
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) < 9000) && (flip_angle > -4500)) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Recover: {
        // use originally captured earth-frame angle targets to recover
        attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

        // increase throttle to gain any lost altitude
        throttle_out += FLIP_THR_INC;

        float recovery_angle;
        if (roll_dir != 0) {
            // we are rolling
            recovery_angle = fabsf(orig_attitude.x - (float)ahrs.roll_sensor);
        } else {
            // we are pitching
            recovery_angle = fabsf(orig_attitude.y - (float)ahrs.pitch_sensor);
        }

        // check for successful recovery
        if (fabsf(recovery_angle) <= FLIP_RECOVERY_ANGLE) {
            // restore original flight mode
            if (!copter.set_mode(orig_control_mode, MODE_REASON_FLIP_COMPLETE)) {
                // this should never happen but just in case
                copter.set_mode(STABILIZE, MODE_REASON_UNKNOWN);
            }
            // log successful completion
            Log_Write_Event(DATA_FLIP_END);
        }
        break;
    }
    case FlipState::Abandon:
        // restore original flight mode
        if (!copter.set_mode(orig_control_mode, MODE_REASON_FLIP_COMPLETE)) {
            // this should never happen but just in case
            copter.set_mode(STABILIZE, MODE_REASON_UNKNOWN);
        }
        // log abandoning flip
        AP::logger().Write_Error(LogErrorSubsystem::FLIP, LogErrorCode::FLIP_ABANDONED);
        break;
    }

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);
}

#endif
