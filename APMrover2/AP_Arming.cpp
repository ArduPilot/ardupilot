#include "AP_Arming.h"
#include "Rover.h"

enum HomeState AP_Arming_Rover::home_status() const
{
    return rover.home_is_set;
}

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
        if (!channel->min_max_configured()) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: RC %s not configured", channel_name);
            }
            return false;
        }
        if (channel->get_radio_min() > 1300) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: %s radio min too high", channel_name);
            }
            return false;
        }
        if (channel->get_radio_max() < 1700) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: %s radio max too low", channel_name);
            }
            return false;
        }
        if (channel->get_radio_trim() < channel->get_radio_min()) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: %s radio trim below min", channel_name);
            }
            return false;
        }
        if (channel->get_radio_trim() > channel->get_radio_max()) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: %s radio trim above max", channel_name);
            }
            return false;
        }
    }
    return true;
}

// performs pre_arm gps related checks and returns true if passed
bool AP_Arming_Rover::position_checks(bool report) {
    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Waiting for Nav Checks");
        }
        return false;
    }

    // check if flight mode requires GPS
    const bool mode_requires_gps = rover.control_mode->is_autopilot_mode();

    // check if fence requires GPS
    bool fence_requires_gps = false;
#if AC_FENCE == ENABLED
    // if circular or polygon fence is enabled we need GPS
    fence_requires_gps = (rover.g2.fence.get_enabled_fences() & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) > 0;
#endif

    // return true if GPS is not required
    if (!mode_requires_gps && !fence_requires_gps) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // ensure GPS is ok
    if (!rover.position_ok()) {
        if (report) {
            const char *reason = ahrs.prearm_failure_reason();
            if (reason) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
            } else {
                if (!mode_requires_gps && fence_requires_gps) {
                    // clarify to user why they need GPS in non-GPS flight mode
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Fence enabled, need 3D Fix");
                } else {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Need 3D Fix");
                }
            }
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }
    // if we got here all must be ok
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}
