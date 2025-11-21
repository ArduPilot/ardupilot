#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

#include "AC_AttitudeControl.h"
#include "AC_PosControl.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "LogStructure.h"

// Write an ANG packet
void AC_AttitudeControl::Write_ANG() const
{
    Vector3f targets = get_att_target_euler_rad() * RAD_TO_DEG;

    const struct log_ANG pkt{
        LOG_PACKET_HEADER_INIT(LOG_ANG_MSG),
        time_us         : AP::scheduler().get_loop_start_time_us(),
        control_roll    : targets.x,
        roll            : degrees(_ahrs.roll),
        control_pitch   : targets.y,
        pitch           : degrees(_ahrs.pitch),
        control_yaw     : wrap_360(targets.z),
        yaw             : wrap_360(degrees(_ahrs.yaw)),
        sensor_dt       : AP::scheduler().get_last_loop_time_s()
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a rate packet
void AC_AttitudeControl::Write_Rate(const AC_PosControl &pos_control) const
{
    const Vector3f rate_targets_degs = rate_bf_targets() * RAD_TO_DEG;
    const Vector3f &accel_target_ned_mss = pos_control.get_accel_target_NED_mss();
    const Vector3f gyro_rate = _rate_gyro_rads * RAD_TO_DEG;
    const struct log_Rate pkt_rate{
        LOG_PACKET_HEADER_INIT(LOG_RATE_MSG),
        time_us         : _rate_gyro_time_us,
        control_roll    : rate_targets_degs.x,
        roll            : gyro_rate.x,
        roll_out        : _motors.get_roll()+_motors.get_roll_ff(),
        control_pitch   : rate_targets_degs.y,
        pitch           : gyro_rate.y,
        pitch_out       : _motors.get_pitch()+_motors.get_pitch_ff(),
        control_yaw     : rate_targets_degs.z,
        yaw             : gyro_rate.z,
        yaw_out         : _motors.get_yaw()+_motors.get_yaw_ff(),
        control_accel   : (float)(-accel_target_ned_mss.z),
        accel           : (float)(-(_ahrs.get_accel_ef().z + GRAVITY_MSS)),
        accel_out       : _motors.get_throttle(),
        throttle_slew   : _motors.get_throttle_slew_rate()
    };
    AP::logger().WriteBlock(&pkt_rate, sizeof(pkt_rate));

    /*
      log P/PD gain scale if not == 1.0
     */
    const Vector3f &scale = get_last_angle_P_scale();
    const Vector3f &pd_scale = _pd_scale_used;
    const Vector3f &i_scale = _i_scale_used;
    if (scale != AC_AttitudeControl::VECTORF_111 
        || pd_scale != AC_AttitudeControl::VECTORF_111
        || i_scale != AC_AttitudeControl::VECTORF_111) {
        const struct log_ATSC pkt_ATSC {
            LOG_PACKET_HEADER_INIT(LOG_ATSC_MSG),
            time_us  : _rate_gyro_time_us,
            scaleP_x : scale.x,
            scaleP_y : scale.y,
            scaleP_z : scale.z,
            scalePD_x : pd_scale.x,
            scalePD_y : pd_scale.y,
            scalePD_z : pd_scale.z,
            scaleI_x : i_scale.x,
            scaleI_y : i_scale.y,
            scaleI_z : i_scale.z,
        };
        AP::logger().WriteBlock(&pkt_ATSC, sizeof(pkt_ATSC));
    }
}

#endif // HAL_LOGGING_ENABLED
