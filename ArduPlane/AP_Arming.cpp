/*
  additional arming checks for plane
 */
#include "AP_Arming.h"
#include "Plane.h"

#include "qautotune.h"

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

#if HAL_QUADPLANE_ENABLED
    ret &= quadplane_checks(display_failure);
#endif

#if AP_TERRAIN_AVAILABLE
    if (plane.g.terrain_follow || plane.mission.contains_terrain_relative()) {
        // check terrain data is loaded and healthy
        uint16_t terr_pending=0, terr_loaded=0;
        plane.terrain.get_statistics(terr_pending, terr_loaded);
        if (plane.terrain.status() != AP_Terrain::TerrainStatusOK) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "terrain data unhealthy");
            ret = false;
        } else if (terr_pending != 0) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "waiting for terrain data");
            ret = false;
        }
    }
#endif
    
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

    if (plane.g2.flight_options & FlightOptions::CENTER_THROTTLE_TRIM){
       int16_t trim = plane.channel_throttle->get_radio_trim();
       if (trim < 1250 || trim > 1750) {
           check_failed(display_failure, "Throttle trim not near center stick(%u)",trim );
           ret = false;
       }
    }

    if (plane.mission.get_in_landing_sequence_flag() &&
        !plane.mission.starts_with_takeoff_cmd()) {
        check_failed(display_failure,"In landing sequence");
        ret = false;
    }
    
    return ret;
}

#if HAL_QUADPLANE_ENABLED
bool AP_Arming_Plane::quadplane_checks(bool display_failure)
{
    if (!plane.quadplane.enabled()) {
        return true;
    }

    if (!plane.quadplane.available()) {
        check_failed(display_failure, "Quadplane enabled but not running");
        return false;
    }

    bool ret = true;

    if (plane.scheduler.get_loop_rate_hz() < 100) {
        check_failed(display_failure, "quadplane needs SCHED_LOOP_RATE >= 100");
        ret = false;
    }

    if (!plane.quadplane.motors->initialised_ok()) {
        check_failed(display_failure, "Quadplane: check motor setup");
        ret = false;
    }

    // lean angle parameter check
    if (plane.quadplane.aparm.angle_max < 1000 || plane.quadplane.aparm.angle_max > 8000) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check Q_ANGLE_MAX");
        ret = false;
    }

    if ((plane.quadplane.tailsitter.enable > 0) && (plane.quadplane.tiltrotor.enable > 0)) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "set TAILSIT_ENABLE 0 or TILT_ENABLE 0");
        ret = false;

    } else {

        if ((plane.quadplane.tailsitter.enable > 0) && !plane.quadplane.tailsitter.enabled()) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "tailsitter setup not complete, reboot");
            ret = false;
        }

        if ((plane.quadplane.tiltrotor.enable > 0) && !plane.quadplane.tiltrotor.enabled()) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "tiltrotor setup not complete, reboot");
            ret = false;
        }
    }

    // ensure controllers are OK with us arming:
    char failure_msg[50] = {};
    if (!plane.quadplane.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
        ret = false;
    }
    if (!plane.quadplane.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
        ret = false;
    }
    if (!plane.quadplane.motors->check_mot_pwm_params()) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check Q_M_PWM_MIN/MAX");
        ret = false;
    }

    if (plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO)) {
        if (!plane.control_mode->is_vtol_mode() && (plane.control_mode != &plane.mode_auto) && (plane.control_mode != &plane.mode_guided)) {
            check_failed(display_failure,"not in Q mode");
            ret = false;
        }
        if ((plane.control_mode == &plane.mode_auto) && !plane.quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
            check_failed(display_failure,"not in VTOL takeoff");
            ret = false;
        }
    }

    if ((plane.control_mode == &plane.mode_auto) && !plane.mission.starts_with_takeoff_cmd()) {
        check_failed(display_failure,"missing takeoff waypoint");
        ret = false;
    }

    if (plane.control_mode == &plane.mode_rtl) {
        check_failed(display_failure,"in RTL mode");
        ret = false;
    }
    
    /*
      Q_ASSIST_SPEED really should be enabled for all quadplanes except tailsitters
     */
    if (check_enabled(ARMING_CHECK_PARAMETERS) &&
        is_zero(plane.quadplane.assist_speed) &&
        !plane.quadplane.tailsitter.enabled()) {
        check_failed(display_failure,"Q_ASSIST_SPEED is not set");
        ret = false;
    }

    return ret;
}
#endif // HAL_QUADPLANE_ENABLED

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
        if (!is_zero(plane.get_throttle_input())){
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
#if HAL_QUADPLANE_ENABLED
    plane.quadplane.set_armed(hal.util->get_soft_armed());
#endif
}

bool AP_Arming_Plane::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    if (!AP_Arming::arm(method, do_arming_checks)) {
        return false;
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
        (method == AP_Arming::Method::MAVLINK ||
         method == AP_Arming::Method::RUDDER)) {
        if (plane.is_flying()) {
            // don't allow mavlink or rudder disarm while flying
            return false;
        }
    }
    
    if (do_disarm_checks && method == AP_Arming::Method::RUDDER) {
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
#if HAL_QUADPLANE_ENABLED
    if ((plane.quadplane.air_mode == AirMode::ON) && (rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRMODE) == nullptr)) {
        plane.quadplane.air_mode = AirMode::OFF;
    }
#endif

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

    // re-initialize speed variable used in AUTO and GUIDED for
    // DO_CHANGE_SPEED commands
    plane.new_airspeed_cm = -1;
    
    gcs().send_text(MAV_SEVERITY_INFO, "Throttle disarmed");

    return true;
}

void AP_Arming_Plane::update_soft_armed()
{
    bool _armed = is_armed();
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.motor_test.running){
        _armed = true;
    }
#endif
    _armed = _armed && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;

    hal.util->set_soft_armed(_armed);
    AP::logger().set_vehicle_armed(hal.util->get_soft_armed());

    // update delay_arming oneshot
    if (delay_arming &&
        (AP_HAL::millis() - hal.util->get_last_armed_change() >= AP_ARMING_DELAY_MS)) {

        delay_arming = false;
    }
}

/*
  extra plane mission checks
 */
bool AP_Arming_Plane::mission_checks(bool report)
{
    // base checks
    bool ret = AP_Arming::mission_checks(report);
    if (plane.mission.get_landing_sequence_start() > 0 && plane.g.rtl_autoland == RtlAutoland::RTL_DISABLE) {
        ret = false;
        check_failed(ARMING_CHECK_MISSION, report, "DO_LAND_START set and RTL_AUTOLAND disabled");
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available()) {
        const uint16_t num_commands = plane.mission.num_commands();
        AP_Mission::Mission_Command prev_cmd {};
        for (uint16_t i=1; i<num_commands; i++) {
            AP_Mission::Mission_Command cmd;
            if (!plane.mission.read_cmd_from_storage(i, cmd)) {
                break;
            }
            if ((cmd.id == MAV_CMD_NAV_VTOL_LAND || cmd.id == MAV_CMD_NAV_LAND) &&
                prev_cmd.id == MAV_CMD_NAV_WAYPOINT) {
                const float dist = cmd.content.location.get_distance(prev_cmd.content.location);
                const float tecs_land_speed = plane.TECS_controller.get_land_airspeed();
                const float landing_speed = is_positive(tecs_land_speed)?tecs_land_speed:plane.aparm.airspeed_cruise_cm*0.01;
                const float min_dist = 0.75 * plane.quadplane.stopping_distance(sq(landing_speed));
                if (dist < min_dist) {
                    ret = false;
                    check_failed(ARMING_CHECK_MISSION, report, "VTOL land too short, min %.0fm", min_dist);
                }
            }
            prev_cmd = cmd;
        }
    }
#endif
    return ret;
}
