#include "Copter.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library
 */

#if AUTOTUNE_ENABLED == ENABLED

bool Copter::AutoTune::init()
{
    // use position hold while tuning if we were in QLOITER
    bool position_hold = (copter.control_mode == LOITER || copter.control_mode == POSHOLD);

    return init_internals(position_hold,
                          copter.attitude_control,
                          copter.pos_control,
                          copter.ahrs_view,
                          &copter.inertial_nav);
}

/*
  start autotune mode
 */
bool Copter::AutoTune::start()
{
    // only allow flip from Stabilize, AltHold,  PosHold or Loiter modes
    if (copter.control_mode != STABILIZE && copter.control_mode != ALT_HOLD &&
        copter.control_mode != LOITER && copter.control_mode != POSHOLD) {
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

void Copter::AutoTune::run()
{
    // apply SIMPLE mode transform to pilot inputs
    copter.update_simple_mode();

    // reset target lean angles and heading while landed
    if (copter.ap.land_complete) {
        // we are landed, shut down
        float target_climb_rate = get_pilot_desired_climb_rate_cms();

        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            copter.motors->set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
        } else {
            copter.motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->set_yaw_target_to_current_heading();

        int32_t target_roll, target_pitch, target_yaw_rate;
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
float Copter::AutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    target_climb_rate = copter.get_avoidance_adjusted_climbrate(target_climb_rate);

    return target_climb_rate;
}

/*
  get stick roll, pitch and yaw rate
 */
void Copter::AutoTune::get_pilot_desired_rp_yrate_cd(int32_t &des_roll_cd, int32_t &des_pitch_cd, int32_t &yaw_rate_cds)
{
    // map from int32_t to float
    float des_roll, des_pitch;

    copter.mode_autotune.get_pilot_desired_lean_angles(des_roll, des_pitch, copter.aparm.angle_max,
                                                       copter.attitude_control->get_althold_lean_angle_max());
    yaw_rate_cds = copter.mode_autotune.get_pilot_desired_yaw_rate(copter.channel_yaw->get_control_in());

    des_roll_cd = des_roll;
    des_pitch_cd = des_pitch;
}

/*
  setup z controller velocity and accel limits
 */
void Copter::AutoTune::init_z_limits()
{
    copter.pos_control->set_max_speed_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up);
    copter.pos_control->set_max_accel_z(copter.g.pilot_accel_z);
}

void Copter::AutoTune::log_pids()
{
    copter.logger.Log_Write_PID(LOG_PIDR_MSG, copter.attitude_control->get_rate_roll_pid().get_pid_info());
    copter.logger.Log_Write_PID(LOG_PIDP_MSG, copter.attitude_control->get_rate_pitch_pid().get_pid_info());
    copter.logger.Log_Write_PID(LOG_PIDY_MSG, copter.attitude_control->get_rate_yaw_pid().get_pid_info());
}


/*
  Write an event packet. This maps from AC_AutoTune event IDs to
  copter event IDs
*/
void Copter::AutoTune::Log_Write_Event(enum at_event id)
{
    const struct {
        enum at_event eid;
        uint8_t id;
    } map[] = {
        { EVENT_AUTOTUNE_INITIALISED, DATA_AUTOTUNE_INITIALISED },
        { EVENT_AUTOTUNE_OFF, DATA_AUTOTUNE_OFF },
        { EVENT_AUTOTUNE_RESTART, DATA_AUTOTUNE_RESTART },
        { EVENT_AUTOTUNE_SUCCESS, DATA_AUTOTUNE_SUCCESS },
        { EVENT_AUTOTUNE_FAILED, DATA_AUTOTUNE_FAILED },
        { EVENT_AUTOTUNE_REACHED_LIMIT, DATA_AUTOTUNE_REACHED_LIMIT },
        { EVENT_AUTOTUNE_PILOT_TESTING, DATA_AUTOTUNE_PILOT_TESTING },
        { EVENT_AUTOTUNE_SAVEDGAINS, DATA_AUTOTUNE_SAVEDGAINS },
    };
    for (uint8_t i=0; i<ARRAY_SIZE(map); i++) {
        if (id == map[i].eid) {
            copter.Log_Write_Event(map[i].id);
            break;
        }
    }
}

/*
  check if we have a good position estimate
 */
bool Copter::AutoTune::position_ok()
{
    return copter.position_ok();
}

/*
  initialise autotune mode
*/
bool Copter::ModeAutoTune::init(bool ignore_checks)
{
    return copter.autotune.init();
}


void Copter::ModeAutoTune::run()
{
    copter.autotune.run();
}

void Copter::ModeAutoTune::save_tuning_gains()
{
    copter.autotune.save_tuning_gains();
}

void Copter::ModeAutoTune::stop()
{
    copter.autotune.stop();
}

#endif  // AUTOTUNE_ENABLED == ENABLED
