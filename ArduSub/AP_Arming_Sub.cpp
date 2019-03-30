#include "AP_Arming_Sub.h"
#include "Sub.h"

bool AP_Arming_Sub::rc_calibration_checks(bool display_failure)
{
    const RC_Channel *channels[] = {
        sub.channel_roll,
        sub.channel_pitch,
        sub.channel_throttle,
        sub.channel_yaw
    };
    return rc_checks_copter_sub(display_failure, channels);
}

bool AP_Arming_Sub::pre_arm_checks(bool display_failure)
{
    if (armed) {
        return true;
    }

    return AP_Arming::pre_arm_checks(display_failure);
}

bool AP_Arming_Sub::ins_checks(bool display_failure)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(display_failure)) {
        return false;
    }

    // additional sub-specific checks
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        if (!AP::ahrs().healthy()) {
            const char *reason = AP::ahrs().prearm_failure_reason();
            if (reason == nullptr) {
                reason = "AHRS not healthy";
            }
            check_failed(ARMING_CHECK_INS, display_failure, "%s", reason);
            return false;
        }
    }

    return true;
}
