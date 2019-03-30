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
    bool position_hold = (plane.previous_mode == QLOITER);

    return init_internals(position_hold,
                          plane.quadplane.attitude_control,
                          plane.quadplane.pos_control,
                          plane.quadplane.ahrs_view,
                          &plane.quadplane.inertial_nav);
}

float QAutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    return plane.quadplane.get_pilot_desired_climb_rate_cms();
}

void QAutoTune::get_pilot_desired_rp_yrate_cd(float &des_roll_cd, float &des_pitch_cd, float &yaw_rate_cds)
{
    if (plane.channel_roll->get_control_in() == 0 && plane.channel_pitch->get_control_in() == 0) {
        des_roll_cd = 0;
        des_pitch_cd = 0;
    } else {
        des_roll_cd = plane.nav_roll_cd;
        des_pitch_cd = plane.nav_pitch_cd;
    }
    yaw_rate_cds = plane.quadplane.get_desired_yaw_rate_cds();
}

void QAutoTune::init_z_limits()
{
    plane.quadplane.pos_control->set_max_speed_z(-plane.quadplane.pilot_velocity_z_max, plane.quadplane.pilot_velocity_z_max);
    plane.quadplane.pos_control->set_max_accel_z(plane.quadplane.pilot_accel_z);
}


// Wrote an event packet
void QAutoTune::Log_Write_Event(enum at_event id)
{
    // offset of 30 aligned with ArduCopter autotune events
    uint8_t ev_id = 30 + (uint8_t)id;
    AP::logger().Write(
        "EVT",
        "TimeUS,Id",
        "s-",
        "F-",
        "QB",
        AP_HAL::micros64(),
        ev_id);
}

// log VTOL PIDs for during twitch
void QAutoTune::log_pids(void)
{
    AP::logger().Write_PID(LOG_PIQR_MSG, plane.quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIQP_MSG, plane.quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIQY_MSG, plane.quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
}

#endif // QAUTOTUNE_ENABLED

