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
void AP_Arming_Plane::_pre_arm_checks()
{
    // call parent class checks
    AP_Arming::_pre_arm_checks();

    // Check airspeed sensor
    AP_Arming::airspeed_checks();

    if (plane.g.fs_timeout_long < plane.g.fs_timeout_short && plane.g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
        check_failed(ARMING_CHECK_NONE, "FS_LONG_TIMEOUT < FS_SHORT_TIMEOUT");
    }

    if (plane.aparm.roll_limit_cd < 300) {
        check_failed(ARMING_CHECK_NONE, "LIM_ROLL_CD too small (%u)", plane.aparm.roll_limit_cd);
    }

    if (plane.aparm.pitch_limit_max_cd < 300) {
        check_failed(ARMING_CHECK_NONE, "LIM_PITCH_MAX too small (%u)", plane.aparm.pitch_limit_max_cd);
    }

    if (plane.aparm.pitch_limit_min_cd > -300) {
        check_failed(ARMING_CHECK_NONE, "LIM_PITCH_MIN too large (%u)", plane.aparm.pitch_limit_min_cd);
    }

    if (plane.channel_throttle->get_reverse() && 
        plane.g.throttle_fs_enabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->get_radio_max()) {
        check_failed(ARMING_CHECK_NONE, "Invalid THR_FS_VALUE for rev throttle");
    }

    if (plane.quadplane.available() && plane.scheduler.get_loop_rate_hz() < 100) {
        check_failed(ARMING_CHECK_NONE, "quadplane needs SCHED_LOOP_RATE >= 100");
    }

    if (plane.control_mode == AUTO && plane.mission.num_commands() <= 1) {
        check_failed(ARMING_CHECK_NONE, "No mission loaded");
    }

    // check adsb avoidance failsafe
    if (plane.failsafe.adsb) {
        check_failed(ARMING_CHECK_NONE, "ADSB threat detected");
    }

#if HAVE_PX4_MIXER
    if (plane.last_mixer_crc == -1) {
        check_failed(ARMING_CHECK_NONE, "Mixer error");
    }
#endif
}

void AP_Arming_Plane::ins_checks()
{
    // call parent class checks
    AP_Arming::ins_checks();

    // additional plane specific checks
        if (!AP::ahrs().healthy()) {
            const char *reason = AP::ahrs().prearm_failure_reason();
            if (reason == nullptr) {
                reason = "AHRS not healthy";
            }
            check_failed(ARMING_CHECK_INS, "%s", reason);
        }
}
