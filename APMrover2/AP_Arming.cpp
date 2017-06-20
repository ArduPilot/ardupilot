#include "AP_Arming.h"
#include "Rover.h"

enum HomeState AP_Arming_Rover::home_status() const
{
    return rover.home_is_set;
}


bool AP_Arming_Rover::pre_arm_checks(bool report) {
    if (rover.motor_type_class == Rover::UGV_TYPE_UNDEFINED) {
        if (report) {
            rover.gcs_send_text(MAV_SEVERITY_WARNING, "PreArm: FRAME_TYPE unset");
        }
        return false;
    }
    return hardware_safety_check(report)
           & barometer_checks(report)
           & ins_checks(report)
           & compass_checks(report)
           & gps_checks(report)
           & battery_checks(report)
           & logging_checks(report)
           & manual_transmitter_checks(report)
           & board_voltage_checks(report);
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
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
                rover.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "PreArm: RC %s not configured", channel_name);
            }
            return false;
        }
        if (channel->get_radio_min() > 1300) {
            if (display_failure) {
                rover.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "PreArm: %s radio min too high", channel_name);
            }
            return false;
        }
        if (channel->get_radio_max() < 1700) {
            if (display_failure) {
                rover.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "PreArm: %s radio max too low", channel_name);
            }
            return false;
        }
        if (channel->get_radio_trim() < channel->get_radio_min()) {
            if (display_failure) {
                rover.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "PreArm: %s radio trim below min", channel_name);
            }
            return false;
        }
        if (channel->get_radio_trim() > channel->get_radio_max()) {
            if (display_failure) {
                rover.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "PreArm: %s radio trim above max", channel_name);
            }
            return false;
        }
    }
    return true;
}
