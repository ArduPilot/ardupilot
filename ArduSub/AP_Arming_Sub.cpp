#include "AP_Arming_Sub.h"
#include "Sub.h"

enum HomeState AP_Arming_Sub::home_status() const
{
    return sub.ap.home_state;
}

bool AP_Arming_Sub::rc_check(bool display_failure)
{
    const RC_Channel *channels[] = {
        sub.channel_roll,
        sub.channel_pitch,
        sub.channel_throttle,
        sub.channel_yaw
    };
    return rc_checks_copter_sub(display_failure, channels, false /* check_min_max */);
}

bool AP_Arming_Sub::pre_arm_checks(bool report)
{
    if (armed) {
        return true;
    }

    return AP_Arming::pre_arm_checks(report) & rc_check(report) & ins_checks(report);
}

bool AP_Arming_Sub::ins_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(report)) {
        return false;
    }

    // additional plane specific checks
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        if (!ahrs.healthy()) {
            if (report) {
                const char *reason = ahrs.prearm_failure_reason();
                if (reason) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
                } else {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: AHRS not healthy");
                }
            }
            return false;
        }
    }

    return true;
}
