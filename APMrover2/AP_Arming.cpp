#include "AP_Arming.h"
#include "Rover.h"

// perform pre_arm_rc_checks checks
void AP_Arming_Rover::pre_arm_rc_checks()
{
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
            check_failed(ARMING_CHECK_RC, "%s radio min too high", channel_name);
            return;
        }
        if (channel->get_radio_max() < 1700) {
            check_failed(ARMING_CHECK_RC, "%s radio max too low", channel_name);
            return;
        }
        if (channel->get_radio_trim() < channel->get_radio_min()) {
            check_failed(ARMING_CHECK_RC, "%s radio trim below min", channel_name);
            return;
        }
        if (channel->get_radio_trim() > channel->get_radio_max()) {
            check_failed(ARMING_CHECK_RC, "%s radio trim above max", channel_name);
            return;
        }
    }
}

// performs pre_arm gps related checks and returns true if passed
void AP_Arming_Rover::gps_checks()
{
    if (!rover.control_mode->requires_position() && !rover.control_mode->requires_velocity()) {
        // we don't care!
        return;
    }

    // call parent gps checks
    return AP_Arming::gps_checks();
}

void AP_Arming_Rover::_pre_arm_checks()
{
    AP_Arming::_pre_arm_checks();
    rover.g2.motors.pre_arm_check();
    fence_checks();
    proximity_check();
}

void AP_Arming_Rover::fence_checks()
{
    // check fence is initialised
    const char *fail_msg = nullptr;
    if (!rover.g2.fence.pre_arm_check(fail_msg)) {
        if (fail_msg == nullptr) {
            check_failed(ARMING_CHECK_NONE, "Check fence");
        } else {
            check_failed(ARMING_CHECK_NONE, "Fence: %s", fail_msg);
        }
    }
}

// check nothing is too close to vehicle
void AP_Arming_Rover::proximity_check()
{
    // return true immediately if no sensor present
    if (rover.g2.proximity.get_status() == AP_Proximity::Proximity_NotConnected) {
        return;
    }

    // return false if proximity sensor unhealthy
    if (rover.g2.proximity.get_status() < AP_Proximity::Proximity_Good) {
        check_failed(ARMING_CHECK_PARAMETERS, "check proximity sensor");
        return;
    }
}
