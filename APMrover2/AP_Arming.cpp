#include "AP_Arming.h"
#include "Rover.h"

// perform pre_arm_rc_checks checks
bool AP_Arming_Rover::pre_arm_rc_checks(const bool display_failure)
{
    // set rc-checks to success if RC checks are disabled
    if ((checks_to_perform != ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_RC)) {
        return true;
    }

    const RC_Channel *channels[] = {
            rover.channel_steer,
            rover.channel_throttle,
    };
    const char *channel_names[] = {"Steer", "Throttle"};

    for (uint8_t i= 0 ; i < ARRAY_SIZE(channels); i++) {
        const RC_Channel *channel = channels[i];
        const char *channel_name = channel_names[i];
        // check if radio has been calibrated
        if (channel->get_radio_min() > 1300) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio min too high", channel_name);
            return false;
        }
        if (channel->get_radio_max() < 1700) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio max too low", channel_name);
            return false;
        }
        if (channel->get_radio_trim() < channel->get_radio_min()) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio trim below min", channel_name);
            return false;
        }
        if (channel->get_radio_trim() > channel->get_radio_max()) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio trim above max", channel_name);
            return false;
        }
    }
    return true;
}

// performs pre_arm gps related checks and returns true if passed
bool AP_Arming_Rover::gps_checks(bool display_failure)
{
    if (!rover.control_mode->requires_position() && !rover.control_mode->requires_velocity()) {
        // we don't care!
        return true;
    }

    // check for ekf failsafe
    if (rover.failsafe.ekf) {
        check_failed(ARMING_CHECK_NONE, display_failure, "EKF failsafe");
        return false;
    }

    // ensure position esetimate is ok
    if (!rover.ekf_position_ok()) {
        const char *reason = AP::ahrs().prearm_failure_reason();
        if (reason == nullptr) {
            reason = "Need Position Estimate";
        }
        check_failed(ARMING_CHECK_NONE, display_failure, "%s", reason);
        return false;
    }

    // call parent gps checks
    return AP_Arming::gps_checks(display_failure);
}

bool AP_Arming_Rover::pre_arm_checks(bool report)
{
    if (SRV_Channels::get_emergency_stop()) {
        check_failed(ARMING_CHECK_NONE, report, "Motors Emergency Stopped");
        return false;
    }

    return (AP_Arming::pre_arm_checks(report)
            & rover.g2.motors.pre_arm_check(report)
            & fence_checks(report)
            & proximity_check(report));
}

bool AP_Arming_Rover::fence_checks(bool report)
{
    // check fence is initialised
    const char *fail_msg = nullptr;
    if (!rover.g2.fence.pre_arm_check(fail_msg)) {
        if (report && fail_msg != nullptr) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Fence : %s", fail_msg);
        }
        return false;
    }
    return true;
}

// check nothing is too close to vehicle
bool AP_Arming_Rover::proximity_check(bool report)
{
    // return true immediately if no sensor present
    if (rover.g2.proximity.get_status() == AP_Proximity::Proximity_NotConnected) {
        return true;
    }

    // return false if proximity sensor unhealthy
    if (rover.g2.proximity.get_status() < AP_Proximity::Proximity_Good) {
        check_failed(ARMING_CHECK_NONE, report, "check proximity sensor");
        return false;
    }

    return true;
}
