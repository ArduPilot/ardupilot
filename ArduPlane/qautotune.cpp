#include "Plane.h"
#include "qautotune.h"

#if QAUTOTUNE_ENABLED

/*
  initialise QAUTOTUNE mode
 */
bool QAutoTune::init()
{
    if (!plane.quadplane.available()) {
        return false;
    }

    // use position hold while tuning if we were in QLOITER
    bool position_hold = (plane.previous_mode == &plane.mode_qloiter);

    return init_internals(position_hold,
                          plane.quadplane.attitude_control,
                          plane.quadplane.pos_control,
                          plane.quadplane.ahrs_view);
}

float QAutoTune::get_desired_climb_rate_ms(void) const
{
    return plane.quadplane.get_pilot_desired_climb_rate_cms() * 0.01;
}

void QAutoTune::get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads)
{
    if (plane.channel_roll->get_control_in() == 0 && plane.channel_pitch->get_control_in() == 0) {
        des_roll_rad = 0.0;
        des_pitch_rad = 0.0;
    } else {
        des_roll_rad = cd_to_rad(plane.nav_roll_cd);
        des_pitch_rad = cd_to_rad(plane.nav_pitch_cd);
    }
    des_yaw_rate_rads = cd_to_rad(plane.quadplane.get_desired_yaw_rate_cds());
}

void QAutoTune::init_z_limits()
{
    // set vertical speed and acceleration limits
    // All limits must be positive
    plane.quadplane.pos_control->D_set_max_speed_accel_m(plane.quadplane.get_pilot_velocity_z_max_dn_m(),
                                                       plane.quadplane.pilot_speed_z_max_up_ms,
                                                       plane.quadplane.pilot_accel_z_mss);
    plane.quadplane.pos_control->D_set_correction_speed_accel_m(plane.quadplane.get_pilot_velocity_z_max_dn_m(),
                                                              plane.quadplane.pilot_speed_z_max_up_ms,
                                                              plane.quadplane.pilot_accel_z_mss);
}


#if HAL_LOGGING_ENABLED
// log VTOL PIDs for during twitch
void QAutoTune::log_pids(void)
{
    AP::logger().Write_PID(LOG_PIQR_MSG, plane.quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIQP_MSG, plane.quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIQY_MSG, plane.quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
}
#endif

#endif // QAUTOTUNE_ENABLED

