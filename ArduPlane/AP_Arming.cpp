/*
  additional arming checks for plane
 */
#include "AP_Arming.h"
#include "Plane.h"

const AP_Param::GroupInfo AP_Arming_Plane::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    // index 3 was RUDDER and should not be used

    AP_GROUPEND
};

/*
  additional arming checks for plane

 */
bool AP_Arming_Plane::pre_arm_checks(bool display_failure)
{
    //are arming checks disabled?
    if (checks_to_perform == 0) {
        return true;
    }
    if (hal.util->was_watchdog_armed()) {
        // on watchdog reset bypass arming checks to allow for
        // in-flight arming if we were armed before the reset. This
        // allows a reset on a BVLOS flight to return home if the
        // operator can command arming over telemetry
        return true;
    }

    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(display_failure);

    // Check airspeed sensor
    ret &= AP_Arming::airspeed_checks(display_failure);

    if (plane.g.fs_timeout_long < plane.g.fs_timeout_short && plane.g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
        check_failed(display_failure, "FS_LONG_TIMEOUT < FS_SHORT_TIMEOUT");
        ret = false;
    }

    if (plane.aparm.roll_limit_cd < 300) {
        check_failed(display_failure, "LIM_ROLL_CD too small (%u)", (unsigned)plane.aparm.roll_limit_cd);
        ret = false;
    }

    if (plane.aparm.pitch_limit_max_cd < 300) {
        check_failed(display_failure, "LIM_PITCH_MAX too small (%u)", (unsigned)plane.aparm.pitch_limit_max_cd);
        ret = false;
    }

    if (plane.aparm.pitch_limit_min_cd > -300) {
        check_failed(display_failure, "LIM_PITCH_MIN too large (%u)", (unsigned)plane.aparm.pitch_limit_min_cd);
        ret = false;
    }

    if (plane.channel_throttle->get_reverse() && 
        plane.g.throttle_fs_enabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->get_radio_max()) {
        check_failed(display_failure, "Invalid THR_FS_VALUE for rev throttle");
        ret = false;
    }

    if (plane.quadplane.enabled() && !plane.quadplane.available()) {
        check_failed(display_failure, "Quadplane enabled but not running");
        ret = false;
    }

    if (plane.quadplane.available() && plane.scheduler.get_loop_rate_hz() < 100) {
        check_failed(display_failure, "quadplane needs SCHED_LOOP_RATE >= 100");
        ret = false;
    }

    if (plane.quadplane.enabled() && plane.quadplane.available()) {
        // ensure controllers are OK with us arming:
        char failure_msg[50];
        if (!plane.quadplane.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
            return false;
        }
        if (!plane.quadplane.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
            return false;
        }
    }

    if (plane.control_mode == &plane.mode_auto && plane.mission.num_commands() <= 1) {
        check_failed(display_failure, "No mission loaded");
        ret = false;
    }

    // check adsb avoidance failsafe
    if (plane.failsafe.adsb) {
        check_failed(display_failure, "ADSB threat detected");
        ret = false;
    }

    if (SRV_Channels::get_emergency_stop()) {
        check_failed(display_failure,"Motors Emergency Stopped");
        ret = false;
    }

    return ret;
}

bool AP_Arming_Plane::ins_checks(bool display_failure)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(display_failure)) {
        return false;
    }

    // additional plane specific checks
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        if (!AP::ahrs().prearm_healthy()) {
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

bool AP_Arming_Plane::arm_checks(AP_Arming::Method method)
{
    //are arming checks disabled?
    if (checks_to_perform == 0) {
        return true;
    }

    if (hal.util->was_watchdog_armed()) {
        // on watchdog reset bypass arming checks to allow for
        // in-flight arming if we were armed before the reset. This
        // allows a reset on a BVLOS flight to return home if the
        // operator can command arming over telemetry
        gcs().send_text(MAV_SEVERITY_WARNING, "watchdog: Bypassing arming checks");
        return true;
    }

#if GEOFENCE_ENABLED == ENABLED
    if (plane.g.fence_autoenable == 3) {
        if (!plane.geofence_set_enabled(true)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Fence: cannot enable for arming");
            return false;
        } else if (!plane.geofence_prearm_check()) {
            plane.geofence_set_enabled(false);
            return false;
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Fence: auto-enabled for arming");
        }
    }
#endif
    
    // call parent class checks
    return AP_Arming::arm_checks(method);
}

/*
  update HAL soft arm state and log as needed
*/
void AP_Arming_Plane::change_arm_state(void)
{
    Log_Write_Arm_Disarm();
    update_soft_armed();
    plane.quadplane.set_armed(hal.util->get_soft_armed());
}

bool AP_Arming_Plane::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    if (!AP_Arming::arm(method, do_arming_checks)) {
        return false;
    }

    change_arm_state();

    gcs().send_text(MAV_SEVERITY_INFO, "Throttle armed");

    return true;
}

/*
  disarm motors
 */
bool AP_Arming_Plane::disarm(void)
{
    if (!AP_Arming::disarm()) {
        return false;
    }
    if (plane.control_mode != &plane.mode_auto) {
        // reset the mission on disarm if we are not in auto
        plane.mission.reset();
    }

    // suppress the throttle in auto-throttle modes
    plane.throttle_suppressed = plane.auto_throttle_mode;

    //only log if disarming was successful
    change_arm_state();

    // reload target airspeed which could have been modified by a mission
    plane.aparm.airspeed_cruise_cm.load();

#if QAUTOTUNE_ENABLED
    //save qautotune gains if enabled and success
    if (plane.control_mode == &plane.mode_qautotune) {
        plane.quadplane.qautotune.save_tuning_gains();
    } else {
        plane.quadplane.qautotune.reset();
    }
#endif

    gcs().send_text(MAV_SEVERITY_INFO, "Throttle disarmed");

#if GEOFENCE_ENABLED == ENABLED
    if (plane.g.fence_autoenable == 3) {
        plane.geofence_set_enabled(false);
    }
#endif
    
    return true;
}

void AP_Arming_Plane::update_soft_armed()
{
    hal.util->set_soft_armed(is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    AP::logger().set_vehicle_armed(hal.util->get_soft_armed());
}

