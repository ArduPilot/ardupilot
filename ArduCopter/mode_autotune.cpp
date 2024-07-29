#include "Copter.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library
 */

#if AUTOTUNE_ENABLED == ENABLED

bool Copter_AutoTune::init()
{
    if (!enabled()) {
        return false;
    }

    // only allow AutoTune from some flight modes, for example Stabilize, AltHold,  PosHold or Loiter modes
    if (!copter.flightmode->allows_autotune()) {
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

    // use position hold while tuning if we were in QLOITER
    bool position_hold = (copter.flightmode->mode_number() == Mode::Number::LOITER || copter.flightmode->mode_number() == Mode::Number::POSHOLD);

    return init_internals(position_hold);
}

void Copter_AutoTune::run()
{
    if (run_previous_mode()) {
        // Autotune wants us to behave like the previous mode, we don't need to do anything except call the base class run function
        AutoTune::run();
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    copter.update_simple_mode();

    // disarm when the landing detector says we've landed and spool state is ground idle
    if (copter.ap.land_complete && copter.motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // if not armed set throttle to zero and exit immediately
    if (copter.ap.land_complete) {
        copter.flightmode->make_safe_ground_handling();
        return;
    }

    // run autotune mode
    AutoTune::run();

}


/*
  get stick input climb rate
 */
float Copter_AutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    target_climb_rate = copter.mode_autotune.get_avoidance_adjusted_climbrate(target_climb_rate);

    return target_climb_rate;
}

/*
  get stick roll, pitch and yaw rate
 */
void Copter_AutoTune::get_pilot_desired_rp_yrate_cd(float &des_roll_cd, float &des_pitch_cd, float &yaw_rate_cds)
{
    copter.mode_autotune.get_pilot_desired_lean_angles(des_roll_cd, des_pitch_cd, copter.aparm.angle_max,
                                                       copter.attitude_control->get_althold_lean_angle_max_cd());
    yaw_rate_cds = copter.mode_autotune.get_pilot_desired_yaw_rate(copter.channel_yaw->norm_input_dz());
}

/*
  setup z controller velocity and accel limits
 */
void Copter_AutoTune::init_z_limits()
{
    // set vertical speed and acceleration limits
    copter.pos_control->set_max_speed_accel_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
    copter.pos_control->set_correction_speed_accel_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
}

#if HAL_LOGGING_ENABLED
void Copter_AutoTune::log_pids()
{
    copter.logger.Write_PID(LOG_PIDR_MSG, copter.attitude_control->get_rate_roll_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDP_MSG, copter.attitude_control->get_rate_pitch_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDY_MSG, copter.attitude_control->get_rate_yaw_pid().get_pid_info());
}
#endif

/*
  check if we have a good position estimate
 */
bool Copter_AutoTune::position_ok()
{
    return copter.position_ok();
}

/*
  initialise autotune mode
*/
bool ModeAutoTune::init(bool ignore_checks)
{
    // Keep pointer to the previous mode
    last_mode = copter.flightmode;

    return autotune.init();
}

void ModeAutoTune::run()
{
    if (autotune.run_previous_mode()) {
        // Autotune wants us to behave like the previous mode
        // This is a bit of a hack, which I dont really like.
        last_mode->run();
    }

    autotune.run();
}

void ModeAutoTune::save_tuning_gains()
{
    autotune.save_tuning_gains();
}

void ModeAutoTune::exit()
{
    autotune.stop();
}

void ModeAutoTune::reset()
{
    autotune.reset();
}

#endif  // AUTOTUNE_ENABLED == ENABLED
