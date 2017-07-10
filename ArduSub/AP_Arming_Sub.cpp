#include "AP_Arming_Sub.h"
#include "Sub.h"

enum HomeState AP_Arming_Sub::home_status() const
{
    return sub.ap.home_state;
}

bool AP_Arming_Sub::rc_check(bool report)
{
    // set rc-checks to success if RC checks are disabled
    if ((checks_to_perform != ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_RC)) {
        return true;
    }

    static const char* message_fail = "PreArm: Check RC min/max parameters";
    bool ret = true;

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (sub.channel_roll->get_radio_min() > 1300 || sub.channel_roll->get_radio_max() < 1700 || sub.channel_pitch->get_radio_min() > 1300 || sub.channel_pitch->get_radio_max() < 1700) {
        ret = false;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (sub.channel_throttle->get_radio_min() > 1300 || sub.channel_throttle->get_radio_max() < 1700 || sub.channel_yaw->get_radio_min() > 1300 || sub.channel_yaw->get_radio_max() < 1700) {
        ret = false;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (sub.channel_roll->get_radio_trim() < 1300 || sub.channel_roll->get_radio_trim() > 1700 || sub.channel_pitch->get_radio_trim() < 1300 || sub.channel_pitch->get_radio_trim() > 1700) {
        ret = false;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (sub.channel_yaw->get_radio_trim() < 1300 || sub.channel_yaw->get_radio_trim() > 1700) {
        ret = false;
    }

    if (report && !ret) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, message_fail);
    }

    return ret;
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
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
                } else {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: AHRS not healthy");
                }
            }
            return false;
        }
    }

    return true;
}
