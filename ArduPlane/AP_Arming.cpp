/*
  additional arming checks for plane
 */
#include "AP_Arming.h"
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

enum HomeState AP_Arming_Plane::home_status() const
{
    return plane.home_is_set;
}

bool AP_Arming_Plane::arm(uint8_t method)
{
    // start logging here so we can check success or failure in
    // arm_checks
    if (plane.g.log_bitmask != NONE &&
        !plane.DataFlash.logging_started()) {
        plane.start_logging();
    }

    return AP_Arming::arm(method);
}

/*
  additional arming checks for plane

 */
bool AP_Arming_Plane::pre_arm_checks(bool report)
{
    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(report);

    // Check airspeed sensor
    ret &= AP_Arming::airspeed_checks(report);

    if (plane.aparm.roll_limit_cd < 300) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: LIM_ROLL_CD too small (%u)", plane.aparm.roll_limit_cd);
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_max_cd < 300) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: LIM_PITCH_MAX too small (%u)", plane.aparm.pitch_limit_max_cd);
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_min_cd > -300) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: LIM_PITCH_MIN too large (%u)", plane.aparm.pitch_limit_min_cd);
        }
        ret = false;        
    }

    if (plane.channel_throttle->get_reverse() && 
        plane.g.throttle_fs_enabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->get_radio_max()) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Invalid THR_FS_VALUE for rev throttle");
        }
        ret = false;
    }

    if (plane.quadplane.available() && plane.scheduler.get_loop_rate_hz() < 100) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: quadplane needs SCHED_LOOP_RATE > 100");
        }
        ret = false;
    }

    if (plane.control_mode == AUTO && plane.mission.num_commands() <= 1) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: No mission loaded");
        }
        ret = false;
    }

    // check adsb avoidance failsafe
    if (plane.failsafe.adsb) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: ADSB threat detected");
        }
        ret = false;
    }

#if HAVE_PX4_MIXER
    if (plane.last_mixer_crc == -1) {
        if (report) {
            // if you ever get this error, a reboot is recommended.
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Mixer error");
        }
        ret = false;
    }
#endif // CONFIG_HAL_BOARD

    return ret;
}

bool AP_Arming_Plane::ins_checks(bool report)
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
