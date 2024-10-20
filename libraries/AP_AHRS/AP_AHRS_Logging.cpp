#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

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

// Write an attitude packet, targets in degrees
void AP_AHRS::Write_Attitude(const Vector3f &targets) const
{
    const struct log_Attitude pkt{
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : targets.x,
        roll            : degrees(roll),
        control_pitch   : targets.y,
        pitch           : degrees(pitch),
        control_yaw     : wrap_360(targets.z),
        yaw             : wrap_360(degrees(yaw)),
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
        gyro_x          : state.gyro_estimate.x,
        gyro_y          : state.gyro_estimate.y,
        gyro_z          : state.gyro_estimate.z,
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

// Write an attitude view packet, targets in degrees
void AP_AHRS_View::Write_AttitudeView(const Vector3f &targets) const
{
    const struct log_Attitude pkt{
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : targets.x,
        roll            : degrees(roll),
        control_pitch   : targets.y,
        pitch           : degrees(pitch),
        control_yaw     : wrap_360(targets.z),
        yaw             : wrap_360(degrees(yaw)),
        active          : uint8_t(AP::ahrs().active_EKF_type()),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

#endif  // HAL_LOGGING_ENABLED
