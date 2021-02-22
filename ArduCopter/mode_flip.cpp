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

#define FLIP_THR_INC        0.20f   // throttle increase during Flip_Start stage (under 45deg lean angle)
#define FLIP_THR_DEC        0.24f   // throttle decrease during Flip_Roll stage (between 45deg ~ -90deg roll)
#define FLIP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define FLIP_RECOVERY_ANGLE 1000     // consider successful recovery when roll is back within 10 degrees of original

#define FLIP_ROLL_RIGHT      1      // used to set flip_dir
#define FLIP_ROLL_LEFT      -1      // used to set flip_dir

#define FLIP_PITCH_BACK      1      // used to set flip_dir
#define FLIP_PITCH_FORWARD  -1      // used to set flip_dir

/*
  flip parameters
 */    
const AP_Param::GroupInfo ModeFlip::var_info[] = {
    // @Param: RAMP_MS
    // @DisplayName: Flip ramp time
    // @Description: Ramp time in milliseconds at the start of a flip. This allows you to adjust for height loss in flip
    // @Units: ms
    // @Range: 100 2000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("RAMP_MS", 1, ModeFlip, ramp_ms, 1000),

    // @Param: RAMP_CD
    // @DisplayName: Flip ramp angle in centidegrees
    // @Description: The ramp angle is how far in the opposite direction of the flip that we start rotating to give a flip that doesn't change position
    // @Units: cdeg
    // @Range: 100 2000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("RAMP_CD", 2, ModeFlip, ramp_cd, 500),

    // @Param: ROT_RATE
    // @DisplayName: Flip rotation rate
    // @Description: The rotation rate in degrees/second for a flip
    // @Units: deg/s
    // @Range: 100 3000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ROT_RATE", 3, ModeFlip, rot_rate_dps, 720),

    // @Param: ACCEL_MAX
    // @DisplayName: Flip maximum roll acceleration rate
    // @Description: The rotation acceleration rate in degrees/second/second for a flip. This overrides the ATC_ACCEL_R_MAX and ATC_ACCEL_P_MAX during a flip. A value of zero means not to override existing values
    // @Units: deg/s/s
    // @Range: 0 10000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("ACCEL_MAX", 4, ModeFlip, rot_accel_max, 0),

    // 5 was STOP_CD

    AP_GROUPEND
};

ModeFlip::ModeFlip(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// flip_init - initialise flip controller
bool ModeFlip::init(bool ignore_checks)
{
    // only allow flip from ACRO, Stabilize, AltHold or Drift flight modes
    if (copter.control_mode != Mode::Number::ACRO &&
        copter.control_mode != Mode::Number::STABILIZE &&
        copter.control_mode != Mode::Number::ALT_HOLD &&
        copter.control_mode != Mode::Number::FLOWHOLD &&
        copter.control_mode != Mode::Number::LOITER) {
        return false;
    }

    if (AP_Notify::flags.failsafe_battery) {
        gcs().send_text(MAV_SEVERITY_INFO, "Flip refused (voltage)");
        return false;
    }
    
    // if in acro or stabilize ensure throttle is above zero
    if (copter.ap.throttle_zero && (copter.control_mode == Mode::Number::ACRO || copter.control_mode == Mode::Number::STABILIZE)) {
        return false;
    }

    // capture current attitude which will be used during the Flip_Recovery stage
    flip_orig_attitude = attitude_control->get_att_target_euler_cd();

    // ensure roll input is less than 40deg
    if ((fabsf(flip_orig_attitude.x) >= 4000.0f)||(fabsf(flip_orig_attitude.y) >= 4000.0)) {
        return false;
    }

    // only allow flip when flying
    if (!motors->armed() || copter.ap.land_complete) {
        return false;
    }

    // capture original flight mode so that we can return to it after completion
    flip_orig_control_mode = copter.control_mode;

    // initialise state
    flip_state = FlipState::Start;
    flip_start_time = millis();

    flip_roll_dir = flip_pitch_dir = 0;

    // choose direction based on pilot's roll and pitch sticks
    if (channel_pitch->get_control_in() > 300) {
        flip_pitch_dir = FLIP_PITCH_BACK;
    }else if(channel_pitch->get_control_in() < -300) {
        flip_pitch_dir = FLIP_PITCH_FORWARD;
    }else if (channel_roll->get_control_in() >= 0) {
        flip_roll_dir = FLIP_ROLL_RIGHT;
    }else{
        flip_roll_dir = FLIP_ROLL_LEFT;
    }

    // log start of flip
    AP::logger().Write_Event(LogEvent::FLIP_START);

    orig_pitch_accel = attitude_control->get_accel_pitch_max();
    orig_roll_accel = attitude_control->get_accel_roll_max();

    if (rot_accel_max > 0) {
        attitude_control->set_accel_pitch_max(rot_accel_max*100);
        attitude_control->set_accel_roll_max(rot_accel_max*100);
    }
    
    return true;
}

// flip_run - runs the flip controller
// should be called at 100hz or more
void ModeFlip::run()
{
    float recovery_angle;

    // if pilot inputs roll > 40deg or timeout occurs abandon flip
    if (!motors->armed() || (millis() - flip_start_time) > FLIP_TIMEOUT_MS) {
        flip_state = FlipState::Abandon;
    }

    /*
      also abandon if users moves stick to 80% other than in direction
      of the flip This allows for user to use full stick movement in
      the direction of the flip and not cause the flip to be abandoned
    */
    if (flip_roll_dir != 0) {
        if (labs(channel_pitch->get_control_in()) >= 4000 ||
            channel_roll->get_control_in() * (-flip_roll_dir) >= 4000) {
            flip_state = FlipState::Abandon;
        }
    } else {
        if ((labs(channel_roll->get_control_in()) >= 4000 ||
             channel_pitch->get_control_in() * (-flip_pitch_dir) >= 4000)) {
            flip_state = FlipState::Abandon;
        }
    }

    // ensure Z controller is active when returning to old mode
    copter.pos_control->set_active_z();
    
    // try to zero vertical velocity
    pos_control->accel_to_throttle(- pos_control->get_vel_z_p().kP() * inertial_nav.get_velocity_z());
    
    // get corrected angle based on direction and axis of rotation
    // we flip the sign of flip_angle to minimize the code repetition
    int32_t flip_angle;
    int32_t flip_orig_angle;
    float rotation_rate_cd;
    float smoothing_gain = 4.0f;
    Vector3f flip_attitude = attitude_control->get_att_target_euler_cd();
    if (flip_roll_dir != 0) {
        flip_angle = flip_attitude.x * flip_roll_dir;
        // move flip angle to -45 degrees to 315 degrees.
        flip_angle = wrap_360_cd(flip_angle+4500.0f)-4500.0f;
        flip_orig_angle = flip_orig_attitude.x * flip_roll_dir;
        rotation_rate_cd = sqrt_controller(36000.0f-flip_angle+flip_orig_angle, smoothing_gain, attitude_control->get_accel_roll_max(), 0.0f);
    } else {
        flip_angle = flip_attitude.y * flip_pitch_dir;
        if (flip_attitude.x < -9000 || flip_attitude.x > 9000) {
            flip_angle = 18000 - flip_attitude.y * flip_pitch_dir;
        } else {
            flip_angle = flip_attitude.y * flip_pitch_dir;
        }
        // move flip angle to -45 degrees to 315 degrees.
        flip_angle = wrap_360_cd(flip_angle+4500.0f)-4500.0f;
        flip_orig_angle = flip_orig_attitude.y * flip_pitch_dir;
        rotation_rate_cd = sqrt_controller(36000.0f-flip_angle+flip_orig_angle, smoothing_gain, attitude_control->get_accel_pitch_max(), 0.0f);
    }
    rotation_rate_cd = MIN(rotation_rate_cd, uint32_t(rot_rate_dps.get())*100U);

    AP::logger().Write("FLIP", "TimeUS,Tms,State,Ang", "QIBi",
                       AP_HAL::micros64(),
                       millis() - flip_start_time,
                       uint8_t(flip_state),
                       flip_angle);
    
    // state machine
    switch (flip_state) {

    case FlipState::Start: {
        uint16_t flip_ramp_ms = constrain_int16(ramp_ms.get(), 100, 3000);
        int16_t flip_ramp_angle_cd = constrain_int16(ramp_cd.get(), -2000, 2000);
        
        // calculate rotation rate and send to attitude controller
        rotation_rate_cd = 1000.0f*(-flip_ramp_angle_cd-flip_orig_angle)/uint16_t(flip_ramp_ms);
        attitude_control->input_rate_bf_roll_pitch_yaw(rotation_rate_cd * flip_roll_dir, rotation_rate_cd * flip_pitch_dir, 0.0);

        // force full throttle
        attitude_control->set_throttle_out(1.0, false, g.throttle_filt);

        // beyond 45deg lean angle move to next stage
        if ((millis() - flip_start_time) > flip_ramp_ms) {
            if (flip_roll_dir != 0) {
                // we are rolling
                flip_state = FlipState::Roll;
            } else {
                // we are pitching
                flip_state = FlipState::Pitch;
            }
        }
        break;
    }

    case FlipState::Roll:
        // between 45deg ~ -90deg request 400deg/sec roll
        attitude_control->input_rate_bf_roll_pitch_yaw(rotation_rate_cd * flip_roll_dir, 0.0, 0.0);
        // decrease throttle
        attitude_control->set_throttle_mix_value(2.0f);

        // set zero throttle
        attitude_control->set_throttle_out(0, false, g.throttle_filt);
        
        // beyond -90deg move on to recovery
        if (flip_angle > 31000) {
            flip_state = FlipState::Recover;
        }
        break;

    case FlipState::Pitch:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, rotation_rate_cd * flip_pitch_dir, 0.0);
        // decrease throttle
        attitude_control->set_throttle_mix_value(2.0f);

        // set zero throttle
        attitude_control->set_throttle_out(0, false, g.throttle_filt);
        
        // beyond -90deg move on to recovery
        if (flip_angle > 31000) {
            flip_state = FlipState::Recover;
        }
        break;

    case FlipState::Recover:
        if (flip_roll_dir != 0) {
            // we are rolling
            recovery_angle = fabsf(flip_orig_attitude.x - (float)ahrs.roll_sensor);
        } else {
            // we are pitching
            recovery_angle = fabsf(flip_orig_attitude.y - (float)ahrs.pitch_sensor);
        }

        // use originally captured earth-frame angle targets to recover
        attitude_control->input_euler_angle_roll_pitch_yaw(flip_orig_attitude.x, flip_orig_attitude.y, flip_orig_attitude.z, false);

        // check for successful recovery
        if (fabsf(recovery_angle) <= FLIP_RECOVERY_ANGLE) {
            // restore original flight mode
            if (!copter.set_mode(flip_orig_control_mode, ModeReason::FLIP_COMPLETE)) {
                // this should never happen but just in case
                copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
            }
            // log successful completion
            AP::logger().Write_Event(LogEvent::FLIP_END);
        }
        break;

    case FlipState::Abandon:
        // restore original flight mode
        if (!copter.set_mode(flip_orig_control_mode, ModeReason::FLIP_COMPLETE)) {
            // this should never happen but just in case
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
        }
        // log abandoning flip
        AP::logger().Write_Error(LogErrorSubsystem::FLIP, LogErrorCode::FLIP_ABANDONED);
        break;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
}

/*
  restore accelerations on mode exit
 */
void ModeFlip::stop(void)
{
    attitude_control->set_accel_pitch_max_slew(orig_pitch_accel);
    attitude_control->set_accel_roll_max_slew(orig_roll_accel);
}

#endif // MODE_FLIP_ENABLED
