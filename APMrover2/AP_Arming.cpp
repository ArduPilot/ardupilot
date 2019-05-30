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

    const AP_AHRS &ahrs = AP::ahrs();

    // always check if inertial nav has started and is ready
    if (!ahrs.prearm_healthy()) {
        const char *reason = ahrs.prearm_failure_reason();
        if (reason == nullptr) {
            reason = "AHRS not healthy";
        }
        check_failed(ARMING_CHECK_NONE, display_failure, "%s", reason);
        return false;
    }

    // check for ekf failsafe
    if (rover.failsafe.ekf) {
        check_failed(ARMING_CHECK_NONE, display_failure, "EKF failsafe");
        return false;
    }

    // ensure position esetimate is ok
    if (!rover.ekf_position_ok()) {
        const char *reason = ahrs.prearm_failure_reason();
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
    //are arming checks disabled?
    if (checks_to_perform == ARMING_CHECK_NONE) {
        return true;
    }
    if (SRV_Channels::get_emergency_stop()) {
        check_failed(ARMING_CHECK_NONE, report, "Motors Emergency Stopped");
        return false;
    }

    return (AP_Arming::pre_arm_checks(report)
            & rover.g2.motors.pre_arm_check(report)
            & fence_checks(report));
}

bool AP_Arming_Rover::arm_checks(AP_Arming::Method method)
{
    //are arming checks disabled?
    if (checks_to_perform == ARMING_CHECK_NONE) {
        return true;
    }
    return AP_Arming::arm_checks(method);
}

void AP_Arming_Rover::update_soft_armed()
{
    hal.util->set_soft_armed(is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    AP::logger().set_vehicle_armed(hal.util->get_soft_armed());
}

/*
  update AHRS soft arm state and log as needed
 */
void AP_Arming_Rover::change_arm_state(void)
{
    Log_Write_Arm_Disarm();
    update_soft_armed();
}

/*
  arm motors
 */
bool AP_Arming_Rover::arm(AP_Arming::Method method, const bool do_arming_checks)
{
    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        return false;
    }

    // Set the SmartRTL home location. If activated, SmartRTL will ultimately try to land at this point
    rover.g2.smart_rtl.set_home(true);

    // initialize simple mode heading
    rover.mode_simple.init_heading();

    // save home heading for use in sail vehicles
    rover.g2.windvane.record_home_heading();

    change_arm_state();

    gcs().send_text(MAV_SEVERITY_INFO, "Throttle armed");

    return true;
}

/*
  disarm motors
 */
bool AP_Arming_Rover::disarm(void)
{
    if (!AP_Arming::disarm()) {
        return false;
    }
    if (rover.control_mode != &rover.mode_auto) {
        // reset the mission on disarm if we are not in auto
        rover.mode_auto.mission.reset();
    }

    // only log if disarming was successful
    change_arm_state();

    gcs().send_text(MAV_SEVERITY_INFO, "Throttle disarmed");

    return true;
}
