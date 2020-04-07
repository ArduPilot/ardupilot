#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

/*
  code to monitor and report on the rate controllers, allowing for
  notification of controller oscillation
 */


/*
  update a RMS estimate of controller state
 */
void AC_AttitudeControl::control_monitor_filter_pid(float value, float &rms)
{
    const float filter_constant = 0.99f;
    // we don't do the sqrt() here as it is quite expensive. That is
    // done when reporting a result
    rms = filter_constant * rms + (1.0f - filter_constant) * sq(value);
}

/*
  update state in _control_monitor
 */
void AC_AttitudeControl::control_monitor_update(void)
{
    const AP_Logger::PID_Info &iroll  = get_rate_roll_pid().get_pid_info();
    control_monitor_filter_pid(iroll.P + iroll.FF,  _control_monitor.rms_roll_P);
    control_monitor_filter_pid(iroll.D,             _control_monitor.rms_roll_D);

    const AP_Logger::PID_Info &ipitch = get_rate_pitch_pid().get_pid_info();
    control_monitor_filter_pid(ipitch.P + iroll.FF,  _control_monitor.rms_pitch_P);
    control_monitor_filter_pid(ipitch.D,             _control_monitor.rms_pitch_D);

    const AP_Logger::PID_Info &iyaw   = get_rate_yaw_pid().get_pid_info();
    control_monitor_filter_pid(iyaw.P + iyaw.D + iyaw.FF,  _control_monitor.rms_yaw);
}

/*
  log a CRTL message
 */
void AC_AttitudeControl::control_monitor_log(void)
{
// @LoggerMessage: CTRL
// @Description: Attitude Control oscillation monitor diagnostics
// @Field: TimeUS: Time since system startup
// @Field: RMSRollP: LPF Root-Mean-Squared Roll Rate controller P gain
// @Field: RMSRollD: LPF Root-Mean-Squared Roll rate controller D gain
// @Field: RMSPitchP: LPF Root-Mean-Squared Pitch Rate controller P gain
// @Field: RMSPitchD: LPF Root-Mean-Squared Pitch Rate controller D gain
// @Field: RMSYaw: LPF Root-Mean-Squared Yaw Rate controller P+D gain
    AP::logger().Write("CTRL", "TimeUS,RMSRollP,RMSRollD,RMSPitchP,RMSPitchD,RMSYaw", "Qfffff",
                                           AP_HAL::micros64(),
                                           (double)safe_sqrt(_control_monitor.rms_roll_P),
                                           (double)safe_sqrt(_control_monitor.rms_roll_D),
                                           (double)safe_sqrt(_control_monitor.rms_pitch_P),
                                           (double)safe_sqrt(_control_monitor.rms_pitch_D),
                                           (double)safe_sqrt(_control_monitor.rms_yaw));

}

/*
  return current controller RMS filter value for roll
 */
float AC_AttitudeControl::control_monitor_rms_output_roll(void) const
{
    return safe_sqrt(_control_monitor.rms_roll_P + _control_monitor.rms_roll_D);
}

/*
  return current controller RMS filter value for roll_P
 */
float AC_AttitudeControl::control_monitor_rms_output_roll_P(void) const
{
    return safe_sqrt(_control_monitor.rms_roll_P);
}

/*
  return current controller RMS filter value for roll_D
 */
float AC_AttitudeControl::control_monitor_rms_output_roll_D(void) const
{
    return safe_sqrt(_control_monitor.rms_roll_D);
}

/*
  return current controller RMS filter value for pitch
 */
float AC_AttitudeControl::control_monitor_rms_output_pitch(void) const
{
    return safe_sqrt(_control_monitor.rms_pitch_P + _control_monitor.rms_pitch_D);
}

/*
  return current controller RMS filter value for pitch_P
 */
float AC_AttitudeControl::control_monitor_rms_output_pitch_P(void) const
{
    return safe_sqrt(_control_monitor.rms_pitch_P);
}

/*
  return current controller RMS filter value for pitch_D
 */
float AC_AttitudeControl::control_monitor_rms_output_pitch_D(void) const
{
    return safe_sqrt(_control_monitor.rms_pitch_D);
}

/*
  return current controller RMS filter value for yaw
 */
float AC_AttitudeControl::control_monitor_rms_output_yaw(void) const
{
    return safe_sqrt(_control_monitor.rms_yaw);
}
