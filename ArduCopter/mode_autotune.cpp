#include "Copter.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library
 */

#if AUTOTUNE_ENABLED == ENABLED

bool AutoTune::init()
{
    // use position hold while tuning if we were in QLOITER
    bool position_hold = (copter.control_mode == Mode::Number::LOITER || copter.control_mode == Mode::Number::POSHOLD);

    return init_internals(position_hold,
                          copter.attitude_control,
                          copter.pos_control,
                          copter.ahrs_view,
                          &copter.inertial_nav);
}

/*
  start autotune mode
 */
bool AutoTune::start()
{
    // only allow flip from Stabilize, AltHold,  PosHold or Loiter modes
    if (copter.control_mode != Mode::Number::STABILIZE &&
        copter.control_mode != Mode::Number::ALT_HOLD &&
        copter.control_mode != Mode::Number::LOITER &&
        copter.control_mode != Mode::Number::POSHOLD) {
        return false;
    }

    // ensure throttle is above zero
    if (copter.ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    if (!copter.motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return false;
    }

    return AC_AutoTune::start();
}

void AutoTune::run()
{
    // apply SIMPLE mode transform to pilot inputs
    copter.update_simple_mode();

    // reset target lean angles and heading while landed
    if (copter.ap.land_complete) {
        // we are landed, shut down
        float target_climb_rate = get_pilot_desired_climb_rate_cms();

        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }
        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->set_yaw_target_to_current_heading();

        float target_roll, target_pitch, target_yaw_rate;
        get_pilot_desired_rp_yrate_cd(target_roll, target_pitch, target_yaw_rate);

        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        copter.pos_control->relax_alt_hold_controllers(0.0f);
        copter.pos_control->update_z_controller();
    } else {
        // run autotune mode
        AC_AutoTune::run();
    }
}


/*
  get stick input climb rate
 */
float AutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    target_climb_rate = copter.get_avoidance_adjusted_climbrate(target_climb_rate);

    return target_climb_rate;
}

/*
  get stick roll, pitch and yaw rate
 */
void AutoTune::get_pilot_desired_rp_yrate_cd(float &des_roll_cd, float &des_pitch_cd, float &yaw_rate_cds)
{
    copter.mode_autotune.get_pilot_desired_lean_angles(des_roll_cd, des_pitch_cd, copter.aparm.angle_max,
                                                       copter.attitude_control->get_althold_lean_angle_max());
    yaw_rate_cds = copter.mode_autotune.get_pilot_desired_yaw_rate(copter.channel_yaw->get_control_in());
}

/*
  setup z controller velocity and accel limits
 */
void AutoTune::init_z_limits()
{
    copter.pos_control->set_max_speed_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up);
    copter.pos_control->set_max_accel_z(copter.g.pilot_accel_z);
}

void AutoTune::log_pids()
{
    copter.logger.Write_PID(LOG_PIDR_MSG, copter.attitude_control->get_rate_roll_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDP_MSG, copter.attitude_control->get_rate_pitch_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDY_MSG, copter.attitude_control->get_rate_yaw_pid().get_pid_info());
}

/*
  check if we have a good position estimate
 */
bool AutoTune::position_ok()
{
    return copter.position_ok();
}

/*
  initialise autotune mode
*/
bool ModeAutoTune::init(bool ignore_checks)
{
    return copter.autotune.init();
}


void ModeAutoTune::run()
{
    copter.autotune.run();
}

void ModeAutoTune::save_tuning_gains()
{
    copter.autotune.save_tuning_gains();
}

void ModeAutoTune::stop()
{
    copter.autotune.stop();
}

void ModeAutoTune::reset()
{
    copter.autotune.reset();
}

#endif  // AUTOTUNE_ENABLED == ENABLED
