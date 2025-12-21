/*
  additional arming checks for plane
 */
#include "AP_Arming_Plane.h"
#include "Plane.h"

#include "qautotune.h"

constexpr uint32_t AP_ARMING_DELAY_MS = 2000; // delay from arming to start of motor spoolup

const AP_Param::GroupInfo AP_Arming_Plane::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    // index 3 was RUDDER and should not be used

#if AP_PLANE_BLACKBOX_LOGGING
    // @Param: BBOX_SPD
    // @DisplayName: Blackbox speed
    // @Description: This is a 3D GPS speed threshold above which we will force arm the vehicle to start logging. WARNING: This should only be used on a vehicle with no propellers attached to the flight controller and when the flight controller is not in control of the vehicle.
    // @Units: m/s
    // @Increment: 1
    // @Range: 1 20
    // @User: Advanced
    AP_GROUPINFO("BBOX_SPD", 4, AP_Arming_Plane, blackbox_speed, 5),
#endif // AP_PLANE_BLACKBOX_LOGGING
    
    AP_GROUPEND
};

// expected to return true if the terrain database is required to have
// all data loaded
bool AP_Arming_Plane::terrain_database_required() const
{
#if AP_TERRAIN_AVAILABLE
    if (plane.g.terrain_follow) {
        return true;
    }
#endif
    return AP_Arming::terrain_database_required();
}

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
    if (!hal.scheduler->is_system_initialized()) {
        check_failed(display_failure, "System not initialised");
        return false;
    }
    // are arming checks disabled?
    if (should_skip_all_checks()) {
        return mandatory_checks(display_failure);
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

#if AP_AIRSPEED_ENABLED
    // Check airspeed sensor
    ret &= AP_Arming::airspeed_checks(display_failure);
#endif

    if (plane.aparm.roll_limit < 3) {
        check_failed(display_failure, "ROLL_LIMIT_DEG too small (%.1f)", plane.aparm.roll_limit.get());
        ret = false;
    }

    if (plane.aparm.pitch_limit_max < 3) {
        check_failed(display_failure, "PTCH_LIM_MAX_DEG too small (%.1f)", plane.aparm.pitch_limit_max.get());
        ret = false;
    }

    if (plane.aparm.pitch_limit_min > -3) {
        check_failed(display_failure, "PTCH_LIM_MIN_DEG too large (%.1f)", plane.aparm.pitch_limit_min.get());
        ret = false;
    }

    if (plane.aparm.airspeed_min < MIN_AIRSPEED_MIN) {
        check_failed(display_failure, "AIRSPEED_MIN too low (%i < %i)", plane.aparm.airspeed_min.get(), MIN_AIRSPEED_MIN);
        ret = false;
    }

    if (plane.channel_throttle->get_reverse() && 
        plane.g.throttle_fs_enabled != Plane::ThrFailsafe::Disabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->get_radio_max()) {
        check_failed(display_failure, "Invalid THR_FS_VALUE for rev throttle");
        ret = false;
    }

    ret &= rc_received_if_enabled_check(display_failure);

#if HAL_QUADPLANE_ENABLED
    ret &= quadplane_checks(display_failure);
#endif

    // check adsb avoidance failsafe
    if (plane.failsafe.adsb) {
        check_failed(display_failure, "ADSB threat detected");
        ret = false;
    }

    if (plane.flight_option_enabled(FlightOptions::CENTER_THROTTLE_TRIM)){
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

    char failure_msg[50] {};
    if (!plane.control_mode->pre_arm_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        check_failed(display_failure, "%s %s", plane.control_mode->name(), failure_msg);
        return false;
    }

    return ret;
}

bool AP_Arming_Plane::mandatory_checks(bool display_failure)
{
    bool ret = true;

    ret &= rc_received_if_enabled_check(display_failure);

    // Call parent class checks
    ret &= AP_Arming::mandatory_checks(display_failure);

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

    char failure_msg[50] {};
    if (!plane.quadplane.motors->arming_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        check_failed(display_failure, "Motors: %s", failure_msg);
        ret = false;
    }

    // lean angle parameter check
    if (plane.quadplane.aparm.angle_max < 1000 || plane.quadplane.aparm.angle_max > 8000) {
        check_failed(Check::PARAMETERS, display_failure, "Check Q_ANGLE_MAX");
        ret = false;
    }

    if ((plane.quadplane.tailsitter.enable > 0) && (plane.quadplane.tiltrotor.enable > 0)) {
        check_failed(Check::PARAMETERS, display_failure, "set TAILSIT_ENABLE 0 or TILT_ENABLE 0");
        ret = false;

    } else {

        if ((plane.quadplane.tailsitter.enable > 0) && !plane.quadplane.tailsitter.enabled()) {
            check_failed(Check::PARAMETERS, display_failure, "tailsitter setup not complete, reboot");
            ret = false;
        }

        if ((plane.quadplane.tiltrotor.enable > 0) && !plane.quadplane.tiltrotor.enabled()) {
            check_failed(Check::PARAMETERS, display_failure, "tiltrotor setup not complete, reboot");
            ret = false;
        }
    }

    // ensure controllers are OK with us arming:
    if (!plane.quadplane.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(Check::PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
        ret = false;
    }
    if (!plane.quadplane.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(Check::PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
        ret = false;
    }

    /*
      Q_ASSIST_SPEED really should be enabled for all quadplanes except tailsitters
     */
    if (check_enabled(Check::PARAMETERS) &&
        is_zero(plane.quadplane.assist.speed) &&
        !plane.quadplane.tailsitter.enabled()) {
        check_failed(display_failure,"Q_ASSIST_SPEED is not set");
        ret = false;
    }

    if ((plane.quadplane.tailsitter.enable > 0) && (plane.quadplane.q_fwd_thr_use != QuadPlane::FwdThrUse::OFF)) {
        check_failed(Check::PARAMETERS, display_failure, "set Q_FWD_THR_USE to 0");
        ret = false;
    }

    // combining Q_RTL_MODE with either of the RTL_AUTOLAND options
    // leads to precedence questions, so just don't allow it:
    if (plane.g.rtl_autoland != RtlAutoland::RTL_DISABLE &&
        plane.quadplane.rtl_mode != QuadPlane::RTL_MODE::NONE) {
        check_failed(Check::PARAMETERS, display_failure, "unset one of RTL_AUTOLAND or Q_RTL_MODE");
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
    if (check_enabled(Check::INS)) {
        char failure_msg[50] = {};
        if (!AP::ahrs().pre_arm_check(true, failure_msg, sizeof(failure_msg))) {
            check_failed(Check::INS, display_failure, "AHRS: %s", failure_msg);
            return false;
        }
    }

    return true;
}

bool AP_Arming_Plane::arm_checks(AP_Arming::Method method)
{

    // are arming checks disabled?
    if (should_skip_all_checks()) {
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

    if (plane.update_home()) {
        // after update_home the home position could still be
        // different from the current_loc if the EKF refused the
        // resetHeightDatum call. If we are updating home then we want
        // to force the home to be the current_loc so relative alt
        // takeoffs work correctly
        if (plane.ahrs.set_home(plane.current_loc)) {
            // update current_loc
            plane.update_current_loc();
        }
    }

    change_arm_state();

    // rising edge of delay_arming oneshot
    delay_arming = true;

#if MODE_AUTOLAND_ENABLED
    plane.mode_autoland.arm_check();
#endif

    if (method == AP_Arming::Method::RUDDER) {
        // initialise the timer used to warn the user they're holding
        // their stick over:
        plane.takeoff_state.rudder_takeoff_warn_ms = AP_HAL::millis();
    }

    send_arm_disarm_statustext("Throttle armed");

    return true;
}

/*
  disarm motors
 */
bool AP_Arming_Plane::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    if (do_disarm_checks &&
        (AP_Arming::method_is_GCS(method) ||
         method == AP_Arming::Method::RUDDER)) {
        if (plane.is_flying()) {
            // don't allow mavlink or rudder disarm while flying
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
    // Possibly save auto tuned parameters
    plane.quadplane.qautotune.disarmed(plane.control_mode == &plane.mode_qautotune);
#endif

    // re-initialize speed variable used in AUTO and GUIDED for
    // DO_CHANGE_SPEED commands
    plane.new_airspeed_cm = -1;

#if MODE_AUTOLAND_ENABLED
    // takeoff direction always cleared on disarm
    plane.takeoff_state.initial_direction.initialized = false;
#endif
    send_arm_disarm_statustext("Throttle disarmed");
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

    hal.util->set_soft_armed(_armed);
#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(hal.util->get_soft_armed());
#endif

    // update delay_arming oneshot
    if (delay_arming &&
        (AP_HAL::millis() - hal.util->get_last_armed_change() >= AP_ARMING_DELAY_MS)) {

        delay_arming = false;
    }

#if AP_PLANE_BLACKBOX_LOGGING
    if (blackbox_speed > 0) {
        const float speed3d = plane.gps.status() >= AP_GPS::GPS_OK_FIX_3D?plane.gps.velocity().length():0;
        const uint32_t now = AP_HAL::millis();
        if (speed3d > blackbox_speed) {
            last_over_3dspeed_ms = now;
        }
        if (!_armed && speed3d > blackbox_speed) {
            // force safety on so we don't run motors
            hal.rcout->force_safety_on();
            AP_Param::set_by_name("RC_PROTOCOLS", 0);
            arm(Method::BLACKBOX, false);
            gcs().send_text(MAV_SEVERITY_WARNING, "BlackBox: arming at %.1f m/s", speed3d);
        }
        if (_armed && now - last_over_3dspeed_ms > 20000U) {
            gcs().send_text(MAV_SEVERITY_WARNING, "BlackBox: disarming at %.1f m/s", speed3d);
            disarm(Method::BLACKBOX, false);
        }
    }
#endif
}

/*
  extra plane mission checks
 */
bool AP_Arming_Plane::mission_checks(bool report)
{
    // base checks
    bool ret = AP_Arming::mission_checks(report);
    if (plane.g.rtl_autoland == RtlAutoland::RTL_DISABLE) {
        if (plane.mission.contains_item(MAV_CMD_DO_LAND_START)) {
            ret = false;
            check_failed(Check::MISSION, report, "DO_LAND_START set and RTL_AUTOLAND disabled");
        }
        if (plane.mission.contains_item(MAV_CMD_DO_RETURN_PATH_START)) {
            ret = false;
            check_failed(Check::MISSION, report, "DO_RETURN_PATH_START set and RTL_AUTOLAND disabled");
        }
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
            if (plane.is_land_command(cmd.id) &&
                prev_cmd.id == MAV_CMD_NAV_WAYPOINT) {
                const float dist = cmd.content.location.get_distance(prev_cmd.content.location);
                const float tecs_land_speed = plane.TECS_controller.get_land_airspeed();
                const float landing_speed = is_positive(tecs_land_speed)?tecs_land_speed:plane.aparm.airspeed_cruise;
                const float min_dist = 0.75 * plane.quadplane.stopping_distance_m(sq(landing_speed));
                if (dist < min_dist) {
                    ret = false;
                    check_failed(Check::MISSION, report, "VTOL land too short, min %.0fm", min_dist);
                }
            }
            prev_cmd = cmd;
        }
    }
#endif
    return ret;
}

// Checks rc has been received if it is configured to be used
bool AP_Arming_Plane::rc_received_if_enabled_check(bool display_failure)
{
    if (rc().enabled_protocols() == 0) {
        // No protocols enabled, will never get RC, don't block arming
        return true;
    }

    // If RC failsafe is enabled we must receive RC before arming
    if ((plane.g.throttle_fs_enabled == Plane::ThrFailsafe::Enabled) &&
        !(rc().has_had_rc_receiver() || rc().has_had_rc_override())) {
        check_failed(display_failure, "Waiting for RC");
        return false;
    }

    return true;
}
