/*
  additional arming checks for plane
 */
#include "AP_Arming.h"
#include "Plane.h"

constexpr uint32_t AP_ARMING_DELAY_MS = 2000; // delay from arming to start of motor spoolup

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
    if (armed || require == (uint8_t)Required::NO) {
        // if we are already armed or don't need any arming checks
        // then skip the checks
        return true;
    }
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
        Plane::ThrFailsafe(plane.g.throttle_fs_enabled.get()) != Plane::ThrFailsafe::Disabled &&
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

    if (plane.quadplane.available() && !plane.quadplane.motors->initialised_ok()) {
        check_failed(display_failure, "Quadplane: check motor setup");
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
        char failure_msg[50] = {};
        if (!AP::ahrs().pre_arm_check(true, failure_msg, sizeof(failure_msg))) {
            check_failed(ARMING_CHECK_INS, display_failure, "AHRS: %s", failure_msg);
            return false;
        }
    }

    return true;
}

bool AP_Arming_Plane::arm_checks(AP_Arming::Method method)
{
    if (method == AP_Arming::Method::RUDDER) {
        const AP_Arming::RudderArming arming_rudder = get_rudder_arming_type();

        if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
            //parameter disallows rudder arming/disabling

            // if we emit a message here then someone doing surface
            // checks may be bothered by the message being emitted.
            // check_failed(true, "Rudder arming disabled");
            return false;
        }

        // if throttle is not down, then pilot cannot rudder arm/disarm
        if (plane.get_throttle_input() != 0){
            check_failed(true, "Non-zero throttle");
            return false;
        }
    }

    if (!plane.control_mode->allows_arming()) {
        check_failed(true, "Mode does not allow arming");
        return false;
    }

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

    // call parent class checks
    return AP_Arming::arm_checks(method);
}

/*
  update HAL soft arm state
*/
void AP_Arming_Plane::change_arm_state(void)
{
    update_soft_armed();
    plane.quadplane.set_armed(hal.util->get_soft_armed());
}

bool AP_Arming_Plane::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    if (!AP_Arming::arm(method, do_arming_checks)) {
        return false;
    }

    if ((method == Method::AUXSWITCH) && (plane.quadplane.options & QuadPlane::OPTION_AIRMODE)) {
        // if no airmode switch assigned, honour the QuadPlane option bit:
        if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRMODE) == nullptr) {
            plane.quadplane.air_mode = AirMode::ON;
        }
    }

    change_arm_state();

    // rising edge of delay_arming oneshot
    delay_arming = true;

    gcs().send_text(MAV_SEVERITY_INFO, "Throttle armed");

    return true;
}

/*
  disarm motors
 */
bool AP_Arming_Plane::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    if (do_disarm_checks &&
        method == AP_Arming::Method::RUDDER) {
        // don't allow rudder-disarming in flight:
        if (plane.is_flying()) {
            // obviously this could happen in-flight so we can't warn about it
            return false;
        }
        // option must be enabled:
        if (get_rudder_arming_type() != AP_Arming::RudderArming::ARMDISARM) {
            gcs().send_text(MAV_SEVERITY_INFO, "Rudder disarm: disabled");
            return false;
        }
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }
    if (plane.control_mode != &plane.mode_auto) {
        // reset the mission on disarm if we are not in auto
        plane.mission.reset();
    }

    // suppress the throttle in auto-throttle modes
    plane.throttle_suppressed = plane.control_mode->does_auto_throttle();

    // if no airmode switch assigned, ensure airmode is off:
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRMODE) == nullptr) {
        plane.quadplane.air_mode = AirMode::OFF;
    }

    //only log if disarming was successful
    change_arm_state();

#if QAUTOTUNE_ENABLED
    //save qautotune gains if enabled and success
    if (plane.control_mode == &plane.mode_qautotune) {
        plane.quadplane.qautotune.save_tuning_gains();
    } else {
        plane.quadplane.qautotune.reset();
    }
#endif

    gcs().send_text(MAV_SEVERITY_INFO, "Throttle disarmed");

    return true;
}

void AP_Arming_Plane::update_soft_armed()
{
    hal.util->set_soft_armed((plane.quadplane.motor_test.running || is_armed()) &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    AP::logger().set_vehicle_armed(hal.util->get_soft_armed());

    // update delay_arming oneshot
    if (delay_arming &&
        (AP_HAL::millis() - hal.util->get_last_armed_change() >= AP_ARMING_DELAY_MS)) {

        delay_arming = false;
    }
}

