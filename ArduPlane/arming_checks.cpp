// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for plane
 */
#include "Plane.h"

const AP_Param::GroupInfo AP_Arming_Plane::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    // @Param: RUDDER
    // @DisplayName: Rudder Arming
    // @Description: Control arm/disarm by rudder input. When enabled arming is done with right rudder, disarming with left rudder. Rudder arming only works in manual throttle modes with throttle at zero +- deadzone (RCx_DZ)
    // @Values: 0:Disabled,1:ArmingOnly,2:ArmOrDisarm
    // @User: Advanced
    AP_GROUPINFO("RUDDER",       3,     AP_Arming_Plane,  rudder_arming_value,     ARMING_RUDDER_ARMONLY),

    AP_GROUPEND
};


/*
  additional arming checks for plane
 */
bool AP_Arming_Plane::pre_arm_checks(bool report)
{
    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(report);

    // Check airspeed sensor
    uint64_t enabled_checks = 0;
    uint64_t passed_checks = 0;
    ret &= AP_Arming::airspeed_checks(report, enabled_checks, passed_checks);

    if (plane.aparm.roll_limit_cd < 300) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: LIM_ROLL_CD too small (%u)", plane.aparm.roll_limit_cd);
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_max_cd < 300) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: LIM_PITCH_MAX too small (%u)", plane.aparm.pitch_limit_max_cd);
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_min_cd > -300) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: LIM_PITCH_MIN too large (%u)", plane.aparm.pitch_limit_min_cd);
        }
        ret = false;        
    }

    if (plane.channel_throttle->get_reverse() && 
        plane.g.throttle_fs_enabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->get_radio_max()) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Invalid THR_FS_VALUE for rev throttle");
        }
        ret = false;
    }

    if (plane.quadplane.available() && plane.scheduler.get_loop_rate_hz() < 100) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: quadplane needs SCHED_LOOP_RATE > 100");
        }
        ret = false;
    }

    if (plane.control_mode == AUTO && plane.mission.num_commands() <= 1) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: No mission loaded");
        }
        ret = false;
    }

    return ret;
}

AP_Arming::ArmingCheckResult AP_Arming_Plane::ins_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    // call parent class checks
    ArmingCheckResult ret = AP_Arming::ins_checks(report, enabled_checks, passed_checks);
    bool group_contains_failure = false;
    bool group_is_disabled = true;

    // additional plane specific checks
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        enabled_checks |= ARMING_CHECK_INS;
        passed_checks |= ARMING_CHECK_INS;
        group_is_disabled = false;
        if (!ahrs.healthy()) {
            if (report) {
                const char *reason = ahrs.prearm_failure_reason();
                if (reason) {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
                } else {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: AHRS not healthy");
                }
            }
            passed_checks &= ~ARMING_CHECK_INS;
            group_contains_failure = true;
        }
    }

    if (ret==ARMING_CHECK_DISABLED && group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }
    
    if (ret==ARMING_CHECK_FAILED || group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }

    // if we got here checks have passed
    return ARMING_CHECK_PASSED;
}
