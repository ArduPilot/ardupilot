#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

/*
  mode takeoff parameters
 */
const AP_Param::GroupInfo ModeTakeoff::var_info[] = {
    // @Param: ALT
    // @DisplayName: Takeoff mode altitude
    // @Description: This is the target altitude for TAKEOFF mode
    // @Range: 0 200
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("ALT", 1, ModeTakeoff, target_alt, 50),

    // @Param: LVL_ALT
    // @DisplayName: Takeoff mode altitude level altitude
    // @Description: This is the altitude below which the wings are held level for TAKEOFF and AUTO modes. Below this altitude, roll demand is restricted to LEVEL_ROLL_LIMIT. Normal-flight roll restriction resumes above TKOFF_LVL_ALT*3 or TKOFF_ALT, whichever is lower. Roll limits are scaled while between TKOFF_LVL_ALT and those altitudes for a smooth transition.
    // @Range: 0 50
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("LVL_ALT", 2, ModeTakeoff, level_alt, 10),

    // @Param: LVL_PITCH
    // @DisplayName: Takeoff mode altitude initial pitch
    // @Description: This is the target pitch during the takeoff.
    // @Range: 0 30
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("LVL_PITCH", 3, ModeTakeoff, level_pitch, 15),

    // @Param: DIST
    // @DisplayName: Takeoff mode distance
    // @Description: This is the distance from the takeoff location where the plane will loiter. The loiter point will be in the direction of takeoff (the direction the plane is facing when the plane begins takeoff)
    // @Range: 0 500
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("DIST", 4, ModeTakeoff, target_dist, 200),

    // @Param: GND_PITCH
    // @DisplayName: Takeoff run pitch demand
    // @Description: Degrees of pitch angle demanded during the takeoff run before speed reaches TKOFF_ROTATE_SPD. For taildraggers set to 3-point ground pitch angle and use TKOFF_TDRAG_ELEV to prevent nose tipover. For nose-wheel steer aircraft set to the ground pitch angle and if a reduction in nose-wheel load is required as speed rises, use a positive offset in TKOFF_GND_PITCH of up to 5 degrees above the angle on ground, starting at the mesured pitch angle and incrementing in 1 degree steps whilst checking for premature rotation and takeoff with each increment. To increase nose-wheel load, use a negative TKOFF_TDRAG_ELEV and refer to notes on TKOFF_TDRAG_ELEV before making adjustments.
    // @Units: deg
    // @Range: -5.0 10.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GND_PITCH", 5, ModeTakeoff, ground_pitch, 5),

    AP_GROUPEND
};

ModeTakeoff::ModeTakeoff() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeTakeoff::_enter()
{
    takeoff_mode_setup = false;
    have_autoenabled_fences = false;

    return true;
}

void ModeTakeoff::update()
{
    // don't setup waypoints if we dont have a valid position and home!
    if (!(plane.current_loc.initialised() && AP::ahrs().home_is_set())) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }

    const float alt = target_alt;
    const float dist = target_dist;
    if (!takeoff_mode_setup) {
        plane.auto_state.takeoff_altitude_rel_cm = alt * 100;
        const uint16_t altitude = plane.relative_ground_altitude(false,true);
        const float direction = degrees(ahrs.get_yaw());
        // see if we will skip takeoff as already flying
        if (plane.is_flying() && (millis() - plane.started_flying_ms > 10000U) && ahrs.groundspeed() > 3) {
            if (altitude >= alt) {
                gcs().send_text(MAV_SEVERITY_INFO, "Above TKOFF alt - loitering");
                plane.next_WP_loc = plane.current_loc;
                takeoff_mode_setup = true;
                plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Climbing to TKOFF alt then loitering");
                start_loc = plane.current_loc;
                plane.next_WP_loc = plane.current_loc;
                plane.next_WP_loc.alt += ((alt - altitude) *100);
                plane.next_WP_loc.offset_bearing(direction, dist);
                takeoff_mode_setup = true;
                plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
            }
            // not flying so do a full takeoff sequence
        } else {
            // setup target waypoint and alt for loiter at dist and alt from start
            start_loc = plane.current_loc;
            plane.prev_WP_loc = plane.current_loc;
            plane.next_WP_loc = plane.current_loc;
            plane.next_WP_loc.alt += alt*100.0;
            plane.next_WP_loc.offset_bearing(direction, dist);

            plane.crash_state.is_crashed = false;

            plane.auto_state.takeoff_pitch_cd = level_pitch * 100;

            plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);

            if (!plane.throttle_suppressed) {
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff to %.0fm for %.1fm heading %.1f deg",
                                alt, dist, direction);
                plane.takeoff_state.start_time_ms = millis();
                plane.takeoff_state.throttle_max_timer_ms = millis();
                takeoff_mode_setup = true;
                plane.steer_state.hold_course_cd = wrap_360_cd(direction*100); // Necessary to allow Plane::takeoff_calc_roll() to function.
            }
        }
    }
    // check for optional takeoff timeout
    if (plane.check_takeoff_timeout()) {
        plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        takeoff_mode_setup = false;
    }

    // We update the waypoint to follow once we're past TKOFF_LVL_ALT or we
    // pass the target location. The check for target location prevents us
    // flying towards a wrong location if we can't climb.
    // This will correct any yaw estimation errors (caused by EKF reset
    // or compass interference from max throttle), since it's using position bearing.
    const float altitude_cm = plane.current_loc.alt - start_loc.alt;
    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF
        && plane.steer_state.hold_course_cd == -1 // This is the enter-once flag.
        && (altitude_cm >= (level_alt * 100.0f) || start_loc.get_distance(plane.current_loc) >= dist)
    ) {
        const float direction = start_loc.get_bearing_to(plane.current_loc) * 0.01;
        plane.next_WP_loc = start_loc;
        plane.next_WP_loc.offset_bearing(direction, dist);
        plane.next_WP_loc.alt += alt*100.0;
        plane.steer_state.hold_course_cd = wrap_360_cd(direction*100); // Necessary to allow Plane::takeoff_calc_roll() to function.
    }
        
    // We finish the initial level takeoff if we climb past
    // TKOFF_ALT or we pass the target location. The check for
    // target location prevents us flying forever if we can't climb.
    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF &&
        (altitude_cm >= (alt*100 - 200) ||
        start_loc.get_distance(plane.current_loc) >= dist)) {
        plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
    }

    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF) {
        //below TKOFF_ALT
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.takeoff_calc_throttle();
    } else {
#if AP_FENCE_ENABLED
        if (!have_autoenabled_fences) {
            plane.fence.auto_enable_fence_after_takeoff();
            have_autoenabled_fences = true;
        }
#endif
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
        
        //check if in long failsafe due to being in initial TAKEOFF stage; if it is, recall long failsafe now to get fs action via events call
        if (plane.long_failsafe_pending) {
            plane.long_failsafe_pending = false;
            plane.failsafe_long_on_event(FAILSAFE_LONG, ModeReason::MODE_TAKEOFF_FAILSAFE);
        }
    }
}

void ModeTakeoff::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

