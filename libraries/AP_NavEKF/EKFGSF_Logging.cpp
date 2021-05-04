#include "EKFGSF_yaw.h"

#include <AP_Logger/AP_Logger.h>

#pragma GCC diagnostic ignored "-Wnarrowing"

void EKFGSF_yaw::Log_Write(uint64_t time_us, LogMessages id0, LogMessages id1, uint8_t core_index)
{
    if (!vel_fuse_running) {
        return;
    }

    static_assert(N_MODELS_EKFGSF >= 5, "Logging will break on <5 EKFGSF models");

    const struct log_KY0 ky0{
        LOG_PACKET_HEADER_INIT(id0),
        time_us                 : time_us,
        core                    : core_index,
        yaw_composite           : GSF.yaw,
        yaw_composite_variance  : sqrtF(MAX(GSF.yaw_variance, 0.0f)),
        yaw0                    : EKF[0].X[2],
        yaw1                    : EKF[1].X[2],
        yaw2                    : EKF[2].X[2],
        yaw3                    : EKF[3].X[2],
        yaw4                    : EKF[4].X[2],
        wgt0                    : GSF.weights[0],
        wgt1                    : GSF.weights[1],
        wgt2                    : GSF.weights[2],
        wgt3                    : GSF.weights[3],
        wgt4                    : GSF.weights[4],
    };
    AP::logger().WriteBlock(&ky0, sizeof(ky0));

    const struct log_KY1 ky1{
        LOG_PACKET_HEADER_INIT(id1),
        time_us                 : time_us,
        core                    : core_index,
        ivn0                    : EKF[0].innov[0],
        ivn1                    : EKF[1].innov[0],
        ivn2                    : EKF[2].innov[0],
        ivn3                    : EKF[3].innov[0],
        ivn4                    : EKF[4].innov[0],
        ive0                    : EKF[0].innov[1],
        ive1                    : EKF[1].innov[1],
        ive2                    : EKF[2].innov[1],
        ive3                    : EKF[3].innov[1],
        ive4                    : EKF[4].innov[1],
    };
    AP::logger().WriteBlock(&ky1, sizeof(ky1));
}
