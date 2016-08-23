#include "AP_Arming_Sub.h"
#include "Sub.h"

void AP_Arming_Sub::rc_calibration_checks()
{
    const RC_Channel *channels[] = {
        sub.channel_roll,
        sub.channel_pitch,
        sub.channel_throttle,
        sub.channel_yaw
    };
    rc_checks_copter_sub(channels);
}

void AP_Arming_Sub::pre_arm_checks(bool report)
{
    if (armed) {
        return;
    }
    AP_Arming::pre_arm_checks(report);
}

void AP_Arming_Sub::ins_checks()
{
    // call parent class checks
    AP_Arming::ins_checks();

    // additional sub-specific checks
        if (!AP::ahrs().healthy()) {
            const char *reason = AP::ahrs().prearm_failure_reason();
            if (reason == nullptr) {
                reason = "AHRS not healthy";
            }
            check_failed(ARMING_CHECK_INS, "%s", reason);
        }
}
