#include "AP_AHRS.h"
#include <AP_Logger/AP_Logger.h>

#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>


// Write an AHRS2 packet
void AP_AHRS::Write_AHRS2() const
{
    Vector3f euler;
    Location loc;
    Quaternion quat;
    if (!get_secondary_attitude(euler) || !get_secondary_position(loc) || !get_secondary_quaternion(quat)) {
        return;
    }
    const struct log_AHRS pkt{
        LOG_PACKET_HEADER_INIT(LOG_AHR2_MSG),
        time_us : AP_HAL::micros64(),
        roll  : (int16_t)(degrees(euler.x)*100),
        pitch : (int16_t)(degrees(euler.y)*100),
        yaw   : (uint16_t)(wrap_360_cd(degrees(euler.z)*100)),
        alt   : loc.alt*1.0e-2f,
        lat   : loc.lat,
        lng   : loc.lng,
        q1    : quat.q1,
        q2    : quat.q2,
        q3    : quat.q3,
        q4    : quat.q4,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write AOA and SSA
void AP_AHRS::Write_AOA_SSA(void) const
{
    const struct log_AOA_SSA aoa_ssa{
        LOG_PACKET_HEADER_INIT(LOG_AOA_SSA_MSG),
        time_us         : AP_HAL::micros64(),
        AOA             : getAOA(),
        SSA             : getSSA()
    };

    AP::logger().WriteBlock(&aoa_ssa, sizeof(aoa_ssa));
}

// Write an attitude packet
void AP_AHRS::Write_Attitude(const Vector3f &targets) const
{
    const struct log_Attitude pkt{
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : (int16_t)targets.x,
        roll            : (int16_t)roll_sensor,
        control_pitch   : (int16_t)targets.y,
        pitch           : (int16_t)pitch_sensor,
        control_yaw     : (uint16_t)wrap_360_cd(targets.z),
        yaw             : (uint16_t)wrap_360_cd(yaw_sensor),
        error_rp        : (uint16_t)(get_error_rp() * 100),
        error_yaw       : (uint16_t)(get_error_yaw() * 100),
        active          : uint8_t(active_EKF_type()),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void AP_AHRS::Write_Origin(LogOriginType origin_type, const Location &loc) const
{
    const struct log_ORGN pkt{
        LOG_PACKET_HEADER_INIT(LOG_ORGN_MSG),
        time_us     : AP_HAL::micros64(),
        origin_type : (uint8_t)origin_type,
        latitude    : loc.lat,
        longitude   : loc.lng,
        altitude    : loc.alt
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a POS packet
void AP_AHRS::Write_POS() const
{
    Location loc;
    if (!get_location(loc)) {
        return;
    }
    float home, origin;
    AP::ahrs().get_relative_position_D_home(home);
    const struct log_POS pkt{
        LOG_PACKET_HEADER_INIT(LOG_POS_MSG),
        time_us        : AP_HAL::micros64(),
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt*1.0e-2f,
        rel_home_alt   : -home,
        rel_origin_alt : get_relative_position_D_origin(origin) ? -origin : AP::logger().quiet_nanf(),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a packet for video stabilisation
void AP_AHRS::write_video_stabilisation() const
{
    Quaternion current_attitude;
    get_quat_body_to_ned(current_attitude);
    Vector3f accel = get_accel() - get_accel_bias();
    const struct log_Video_Stabilisation pkt {
        LOG_PACKET_HEADER_INIT(LOG_VIDEO_STABILISATION_MSG),
        time_us         : AP_HAL::micros64(),
        gyro_x          : _gyro_estimate.x,
        gyro_y          : _gyro_estimate.y,
        gyro_z          : _gyro_estimate.z,
        accel_x         : accel.x,
        accel_y         : accel.y,
        accel_z         : accel.z,
        Q1              : current_attitude.q1,
        Q2              : current_attitude.q2,
        Q3              : current_attitude.q3,
        Q4              : current_attitude.q4,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude view packet
void AP_AHRS_View::Write_AttitudeView(const Vector3f &targets) const
{
    const struct log_Attitude pkt{
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : (int16_t)targets.x,
        roll            : (int16_t)roll_sensor,
        control_pitch   : (int16_t)targets.y,
        pitch           : (int16_t)pitch_sensor,
        control_yaw     : (uint16_t)wrap_360_cd(targets.z),
        yaw             : (uint16_t)wrap_360_cd(yaw_sensor),
        error_rp        : (uint16_t)(get_error_rp() * 100),
        error_yaw       : (uint16_t)(get_error_yaw() * 100),
        active          : uint8_t(AP::ahrs().active_EKF_type()),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a rate packet
void AP_AHRS_View::Write_Rate(const AP_Motors &motors, const AC_AttitudeControl &attitude_control,
                                const AC_PosControl &pos_control) const
{
    const Vector3f &rate_targets = attitude_control.rate_bf_targets();
    const Vector3f &accel_target = pos_control.get_accel_target_cmss();
    const auto timeus = AP_HAL::micros64();
    const struct log_Rate pkt_rate{
        LOG_PACKET_HEADER_INIT(LOG_RATE_MSG),
        time_us         : timeus,
        control_roll    : degrees(rate_targets.x),
        roll            : degrees(get_gyro().x),
        roll_out        : motors.get_roll()+motors.get_roll_ff(),
        control_pitch   : degrees(rate_targets.y),
        pitch           : degrees(get_gyro().y),
        pitch_out       : motors.get_pitch()+motors.get_pitch_ff(),
        control_yaw     : degrees(rate_targets.z),
        yaw             : degrees(get_gyro().z),
        yaw_out         : motors.get_yaw()+motors.get_yaw_ff(),
        control_accel   : (float)accel_target.z,
        accel           : (float)(-(get_accel_ef().z + GRAVITY_MSS) * 100.0f),
        accel_out       : motors.get_throttle(),
        throttle_slew   : motors.get_throttle_slew_rate()
    };
    AP::logger().WriteBlock(&pkt_rate, sizeof(pkt_rate));

    /*
      log P/PD gain scale if not == 1.0
     */
    const Vector3f &scale = attitude_control.get_angle_P_scale_logging();
    const Vector3f &pd_scale = attitude_control.get_PD_scale_logging();
    if (scale != AC_AttitudeControl::VECTORF_111 || pd_scale != AC_AttitudeControl::VECTORF_111) {
        const struct log_ATSC pkt_ATSC {
            LOG_PACKET_HEADER_INIT(LOG_ATSC_MSG),
            time_us  : timeus,
            scaleP_x : scale.x,
            scaleP_y : scale.y,
            scaleP_z : scale.z,
            scalePD_x : pd_scale.x,
            scalePD_y : pd_scale.y,
            scalePD_z : pd_scale.z,
        };
        AP::logger().WriteBlock(&pkt_ATSC, sizeof(pkt_ATSC));
    }
}
