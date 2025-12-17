#include "Copter.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library
 */

#if AUTOTUNE_ENABLED

bool AutoTune::init()
{
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

    return init_internals(position_hold,
                          copter.attitude_control,
                          copter.pos_control,
                          copter.ahrs_view);
}

void AutoTune::run()
{
    // apply SIMPLE mode transform to pilot inputs
    copter.update_simple_mode();

    // disarm when the landing detector says we've landed and spool state is ground idle
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // if not armed set throttle to zero and exit immediately
    if (copter.ap.land_complete) {
        copter.flightmode->make_safe_ground_handling();
        return;
    }

    // run autotune mode
    AC_AutoTune::run();
}


/*
  get stick input climb rate
 */
float AutoTune::get_desired_climb_rate_ms(void) const
{
    float target_climb_rate_ms = copter.get_pilot_desired_climb_rate_ms();

    // get avoidance adjusted climb rate
    target_climb_rate_ms = copter.mode_autotune.get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

    return target_climb_rate_ms;
}

/*
  get stick roll, pitch and yaw rate
 */
void AutoTune::get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads)
{
    copter.mode_autotune.get_pilot_desired_lean_angles_rad(des_roll_rad, des_pitch_rad, attitude_control->lean_angle_max_rad(),
                                                       copter.attitude_control->get_althold_lean_angle_max_rad());
    des_yaw_rate_rads = copter.mode_autotune.get_pilot_desired_yaw_rate_rads();
}

/*
  setup z controller velocity and accel limits
 */
void AutoTune::init_z_limits()
{
    // set vertical speed and acceleration limits
    copter.pos_control->D_set_max_speed_accel_m(copter.flightmode->get_pilot_speed_dn_ms(), copter.flightmode->get_pilot_speed_up_ms(), copter.flightmode->get_pilot_accel_D_mss());
    copter.pos_control->D_set_correction_speed_accel_m(copter.flightmode->get_pilot_speed_dn_ms(), copter.flightmode->get_pilot_speed_up_ms(), copter.flightmode->get_pilot_accel_D_mss());
}

#if HAL_LOGGING_ENABLED
void AutoTune::log_pids()
{
    copter.logger.Write_PID(LOG_PIDR_MSG, copter.attitude_control->get_rate_roll_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDP_MSG, copter.attitude_control->get_rate_pitch_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDY_MSG, copter.attitude_control->get_rate_yaw_pid().get_pid_info());
}
#endif

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
    return autotune.init();
}

void ModeAutoTune::run()
{
    autotune.run();
}

void ModeAutoTune::exit()
{
    autotune.stop();
}

#endif  // AUTOTUNE_ENABLED
