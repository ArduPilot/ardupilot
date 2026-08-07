#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

#if MODE_AUTOLAND_ENABLED

// This is added to the target altitude to make sure the true target is exceeded.
// This should be larger than the expected steady state error.
constexpr float fast_climb_extra_alt = 10;

/*
  mode AutoLand parameters
 */
const AP_Param::GroupInfo ModeAutoLand::var_info[] = {
    // @Param: WP_ALT
    // @DisplayName: Final approach WP altitude
    // @Description: This is the target altitude above HOME for final approach waypoint
    // @Range: 0 200
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WP_ALT", 1, ModeAutoLand, final_wp_alt, 55),

    // @Param: WP_DIST
    // @DisplayName: Final approach WP distance
    // @Description: This is the distance from Home that the final approach waypoint is set. The waypoint point will be in the opposite direction of takeoff (the direction the plane is facing when the plane sets its takeoff heading)
    // @Range: 0 700
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WP_DIST", 2, ModeAutoLand, final_wp_dist, 400),

    // @Param: DIR_OFF
    // @DisplayName: Landing direction offset from takeoff
    // @Description: The captured takeoff direction after ground course is established in autotakeoffsis offset by this amount to create a different landing direction and approach.However,if TKOFF_OPTION bit1 is set, the takeoff(landing) direction is captured immediately via compass heading upon arming, then this offset is NOT applied.
    // @Range: -360 360
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("DIR_OFF", 3, ModeAutoLand, landing_dir_off, 0),

    // @Param: OPTIONS
    // @DisplayName: Autoland mode options
    // @Description: Enables optional autoland mode behaviors
    // @Bitmask: 0: When set if there is a healthy compass in use the compass heading will be captured at arming and used for the AUTOLAND mode's initial takeoff direction instead of capturing ground course in NAV_TAKEOFF or Mode TAKEOFF or other modes.
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 4, ModeAutoLand, options, 0),

    // @Param: CLIMB
    // @DisplayName: Minimum altitude above terrain before turning upon entry
    // @Description: Vehicle will climb with limited turn ability (LEVEL_ROLL_LIMIT) until it is at least this altitude above the terrain at the point of entry, before proceeding to loiter-to-alt and landing legs. 0 Disables.
    // @Range: 0 100
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("CLIMB", 5, ModeAutoLand, terrain_alt_min, 0),


    AP_GROUPEND
};

ModeAutoLand::ModeAutoLand() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeAutoLand::_enter()
{
    //must be flying to enter
    if (!plane.is_flying()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Must already be flying!");
        return false;
    }

    // autoland not available for quadplanes as capture of takeoff direction
    // doesn't make sense
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "autoland not available");
        return false;
    }
#endif

    if (!plane.takeoff_state.initial_direction.initialized) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff initial direction not set");
        return false;
    }

    plane.set_target_altitude_current();
    plane.auto_state.next_wp_crosstrack = true;

    plane.prev_WP_loc = plane.current_loc;

    // In flight stage normal for approach
    plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);

    const Location &home = ahrs.get_home();

#ifndef HAL_LANDING_DEEPSTALL_ENABLED
    if (plane.landing.get_type() == AP_Landing::TYPE_DEEPSTALL) {
        // Deep stall landings require only a landing location, they do there own loiter to alt and approach
        cmd_land.id = MAV_CMD_NAV_LAND;
        cmd_land.content.location = home;

        // p1 gives the altitude from which to start the deep stall above the location alt
        cmd_land.p1 = final_wp_alt;
        plane.start_command(cmd_land);

        stage = AutoLandStage::LANDING;
        return true;
    }
#endif // HAL_LANDING_DEEPSTALL_ENABLED

    /*
      Glide slope landing is in 3 steps:
        1) a loitering to alt waypoint centered on base leg
        2) exiting and proceeeing to a final approach land start WP, with crosstrack
        3) the landing WP at home, with crosstrack

      the base leg point is 90 degrees off from the landing leg
     */

    /*
      first calculate the starting waypoint we will use when doing the
      NAV_LAND. This is offset by final_wp_dist from home, in a
      direction 180 degrees from takeoff direction
     */
    land_start = home;
    land_start.offset_bearing(plane.takeoff_state.initial_direction.heading, -final_wp_dist);
    land_start.set_alt_m(final_wp_alt, Location::AltFrame::ABOVE_HOME);
    land_start.change_alt_frame(Location::AltFrame::ABSOLUTE);

    /*
      now create the initial target waypoint for the loitering to alt centered on base leg waypoint. We
      choose if we will do a right or left turn onto the landing based
      on where we are when we enter the landing mode
     */
    const float bearing_to_current_deg = degrees(land_start.get_bearing(plane.current_loc));
    const float bearing_err_deg = wrap_180(plane.takeoff_state.initial_direction.heading - bearing_to_current_deg);
    const float bearing_offset_deg = (bearing_err_deg > 0) ? -90 : 90;

    // Try and minimize loiter radius by using the smaller of the waypoint loiter radius or 1/3 of the final WP distance
    const float loiter_radius = MIN(final_wp_dist * 0.333, abs(plane.aparm.loiter_radius));

    // corrected_loiter_radius is the radius the vehicle will actually fly, this gets larger as altitude increases.
    // Strictly this gets the loiter radius at the current altitude, really we want the loiter radius at final_wp_alt.
    const float corrected_loiter_radius = plane.nav_controller->loiter_radius(loiter_radius);

    cmd_loiter.id = MAV_CMD_NAV_LOITER_TO_ALT;
    cmd_loiter.p1 = loiter_radius;
    cmd_loiter.content.location = land_start;
    cmd_loiter.content.location.offset_bearing(plane.takeoff_state.initial_direction.heading + bearing_offset_deg, corrected_loiter_radius);
    cmd_loiter.content.location.loiter_ccw = bearing_err_deg>0? 1 :0;

    // May need to climb first
    bool climb_first = false;
    if (terrain_alt_min > 0) {
        // Work out the distance needed to climb above terrain
#if AP_TERRAIN_AVAILABLE
        const bool use_terrain = plane.terrain_enabled_in_current_mode();
#else
        const bool use_terrain = false;
#endif
        const float dist_to_climb = terrain_alt_min - plane.relative_ground_altitude(RangeFinderUse::CLIMB, use_terrain);
        if (is_positive(dist_to_climb)) {
            // Copy loiter and update target altitude to current altitude plus climb altitude
            cmd_climb = cmd_loiter;
            float abs_alt;
            if (plane.current_loc.get_alt_m(Location::AltFrame::ABSOLUTE, abs_alt)) {
                cmd_climb.content.location.set_alt_m(abs_alt + dist_to_climb + fast_climb_extra_alt, Location::AltFrame::ABSOLUTE);
                climb_first = true;
            }
        }
    }

#if AP_TERRAIN_AVAILABLE
    // Update loiter location to be relative terrain if enabled
    if (plane.terrain_enabled_in_current_mode()) {
        cmd_loiter.content.location.set_alt_m(final_wp_alt, Location::AltFrame::ABOVE_TERRAIN);
    };
#endif
    // land WP at home
    cmd_land.id = MAV_CMD_NAV_LAND;
    cmd_land.content.location = home;

    // start first leg toward the base leg loiter to alt point
    if (climb_first) {
        stage = AutoLandStage::CLIMB;
        plane.start_command(cmd_climb);

    } else {
        stage = AutoLandStage::LOITER;
        plane.start_command(cmd_loiter);
    }

    return true;
}

void ModeAutoLand::update()
{
    plane.calc_nav_roll();

    // Apply level roll limit in climb stage
    if (stage == AutoLandStage::CLIMB) {
        plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
        plane.nav_roll_cd = constrain_int16(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
    }

    plane.calc_nav_pitch();
    if (plane.landing.is_throttle_suppressed()) {
        // if landing is considered complete throttle is never allowed, regardless of landing type
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    } else {
        plane.calc_throttle();
    }
}

void ModeAutoLand::navigate()
{
    switch (stage) {
    case AutoLandStage::CLIMB:
        // Update loiter, although roll limit is applied the vehicle will still navigate (slowly)
        plane.update_loiter(cmd_climb.p1);

        ftype dist;
        if (plane.reached_loiter_target() || !cmd_climb.content.location.get_height_above(plane.current_loc, dist) || (dist < fast_climb_extra_alt)) {
            // Reached destination or Climb is done, move onto loiter
            plane.auto_state.next_wp_crosstrack = true;
            stage = AutoLandStage::LOITER;
            plane.start_command(cmd_loiter);
            plane.prev_WP_loc = plane.current_loc;
        }
        break;

    case AutoLandStage::LOITER:
        // check if we have arrived and completed loiter at base leg waypoint
        if (plane.verify_loiter_to_alt(cmd_loiter)) {
            stage = AutoLandStage::LANDING;
            plane.start_command(cmd_land);
            // Crosstrack from the land start location
            plane.prev_WP_loc = land_start;

        }
        break;

    case AutoLandStage::LANDING:
        plane.set_flight_stage(AP_FixedWing::FlightStage::LAND);
        plane.verify_command(cmd_land);
        // make sure we line up
        plane.auto_state.crosstrack = true;
        break;
    }
}

/*
  Takeoff direction is initialized after arm when sufficient altitude
  and ground speed is obtained, then captured takeoff direction +
  offset used as landing direction in AUTOLAND
*/
void ModeAutoLand::check_takeoff_direction()
{
#if HAL_QUADPLANE_ENABLED
    // we don't allow fixed wing autoland for quadplanes
    if (quadplane.available()) {
        return;
    }
#endif

    if (plane.takeoff_state.initial_direction.initialized) {
        return;
    }
    //set autoland direction to GPS course over ground
    if (plane.control_mode->allows_autoland_direction_capture() &&
        plane.is_flying() &&
        hal.util->get_soft_armed() &&
        plane.gps.ground_speed() > GPS_GND_CRS_MIN_SPD) {
        set_autoland_direction(plane.gps.ground_course() + landing_dir_off);
    }
}

// Sets autoland direction using ground course + offset parameter
void ModeAutoLand::set_autoland_direction(const float heading)
{
    plane.takeoff_state.initial_direction.heading = wrap_360(heading);
    plane.takeoff_state.initial_direction.initialized = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Autoland direction= %u",int(plane.takeoff_state.initial_direction.heading));
}

/*
  return true when the LOITER_TO_ALT is lined up ready for the landing, used in commands_logic verify loiter to alt
 */
bool ModeAutoLand::landing_lined_up(void)
{
    // use the line between the center of the LOITER_TO_ALT on the base leg and the
    // start of the landing leg (land_start_WP)
    return plane.mode_loiter.isHeadingLinedUp(cmd_loiter.content.location, cmd_land.content.location);
}

// possibly capture heading on arming for autoland mode if option is set and using a compass
void ModeAutoLand::arm_check(void)
{
    if (plane.ahrs.use_compass() && autoland_option_is_set(ModeAutoLand::AutoLandOption::AUTOLAND_DIR_ON_ARM)) {
        set_autoland_direction(plane.ahrs.get_yaw_deg());
    }
}

bool ModeAutoLand::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}


#endif // MODE_AUTOLAND_ENABLED

