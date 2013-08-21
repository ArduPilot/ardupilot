// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Parameters.h"

/*
 *  logic for dealing with the current command in the mission and home location
 */

/*
 * Called from
 * change_command
 * reload_commands_airstart
 */
static void init_commands()
{
	// The persistent command index.
    g.command_index.set_and_save(0);
    nav_command_ID  = NO_COMMAND;
    non_nav_command_ID      = NO_COMMAND;
    next_nav_command.id     = CMD_BLANK;
    // The transient command index.
    nav_command_index = 0;
}

/*
 * Apparently called only when switching into auto from another mode.
 */
static void update_auto()
{
    if (g.command_index >= g.command_total) {
        handle_no_commands();
        if(g.command_total == 0) {
            next_WP.lat             = home.lat + 1000;                  // so we don't have bad calcs
            next_WP.lng             = home.lng + 1000;                  // so we don't have bad calcs
        }
    } else {
        if(g.command_index != 0) {
            g.command_index = nav_command_index;						// Update persistent command index
            nav_command_index--;						// ????
        }
        nav_command_ID  = NO_COMMAND;
        non_nav_command_ID      = NO_COMMAND;
        next_nav_command.id     = CMD_BLANK;
        process_next_command();
    }
}

/*
  fetch a mission item from EEPROM
*/
static struct Location get_cmd_with_index_raw(int16_t i)
{
    struct Location temp;
    uint16_t mem;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (i > g.command_total) {
        memset(&temp, 0, sizeof(temp));
        temp.id = CMD_BLANK;
    }else{
        // read WP position
        mem = (WP_START_BYTE) + (i * WP_SIZE);
        temp.id = hal.storage->read_byte(mem);

        mem++;
        temp.options = hal.storage->read_byte(mem);

        mem++;
        temp.p1 = hal.storage->read_byte(mem);

        mem++;
        temp.alt = hal.storage->read_dword(mem);

        mem += 4;
        temp.lat = hal.storage->read_dword(mem);

        mem += 4;
        temp.lng = hal.storage->read_dword(mem);
    }

    return temp;
}

/*
  fetch a mission item from EEPROM. Adjust altitude to be absolute
*/
static struct Location get_cmd_with_index(int16_t i)
{
    struct Location temp;

    temp = get_cmd_with_index_raw(i);

    // Add on home altitude if we are a nav command (or other command with altitude) 
    // and stored alt is relative
    if ((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) &&
        (temp.options & MASK_OPTIONS_RELATIVE_ALT) &&
        (temp.lat != 0 || temp.lng != 0 || temp.alt != 0)) {
        temp.alt += home.alt;
    }

    return temp;
}

// Setters
// -------
static void set_cmd_with_index(struct Location temp, int16_t i)
{
    i = constrain_int16(i, 0, g.command_total.get());
    uint16_t mem = WP_START_BYTE + (i * WP_SIZE);

    // force home wp to absolute height
    if (i == 0) {
        temp.options &= ~(MASK_OPTIONS_RELATIVE_ALT);
    }
    // zero unused bits
    temp.options &= (MASK_OPTIONS_RELATIVE_ALT | MASK_OPTIONS_LOITER_DIRECTION);

    hal.storage->write_byte(mem, temp.id);

    mem++;
    hal.storage->write_byte(mem, temp.options);

    mem++;
    hal.storage->write_byte(mem, temp.p1);

    mem++;
    hal.storage->write_dword(mem, temp.alt);

    mem += 4;
    hal.storage->write_dword(mem, temp.lat);

    mem += 4;
    hal.storage->write_dword(mem, temp.lng);
}

static int32_t read_alt_to_hold()
{
    if (g.RTL_altitude_cm < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude_cm + home.alt;
}

/*
 * This function stores waypoint commands
 * It looks to see what the next command type is and finds the last command.
 * Called from  
 * do_takeoff
 * do_nav_wp
 * do_land
 * do_loiter_unlimited
 * do_loiter_turns
 * do_loiter_time
 */
static void set_next_WP(const struct Location *wp)
{
    // copy the current WP into the OldWP slot
    // ---------------------------------------
    prev_WP = next_WP;

    // Load the next_WP slot
    // ---------------------
    next_WP = *wp;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP.lat == 0 && next_WP.lng == 0) {
        next_WP.lat = current_loc.lat;
        next_WP.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP.alt == 0) {
            next_WP.alt = current_loc.alt;
        }
    }

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Resetting prev_WP"));
        prev_WP = current_loc;
    }

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;

    // zero out our loiter vals to watch for missed waypoints
    loiter_angle_reset();

    setup_glide_slope();

    // This was done already.
    loiter_angle_reset();
}

static void set_guided_WP(void)
{
    if (g.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP = guided_WP;

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;

    setup_glide_slope();

    loiter_angle_reset();
}

void init_home_from_gps() {
    // block until we get a good fix
    // -----------------------------
    while (!g_gps->new_data || !g_gps->fix) {
        g_gps->update();
#if HIL_MODE != HIL_MODE_DISABLED
        // update hil gps so we have new_data
        gcs_update();
#endif
    }

    home.id         = MAV_CMD_NAV_WAYPOINT;
    home.lng        = g_gps->longitude;                                 // Lon * 10**7
    home.lat        = g_gps->latitude;                                  // Lat * 10**7
    // This is not so good if you fly in a depression where alt < 0.
    home.alt        = max(g_gps->altitude_cm, 0);

    home_is_set = true;
}

// run this at setup on the ground
// -------------------------------
void init_home()
{
	init_home_from_gps();
	
    struct Location waypointZero= get_cmd_with_index_raw(0);
    bool isWP0Set = waypointZero.lng != 0 || waypointZero.lat != 0; 
    float horizontal_diff_m = get_distance(&home, &waypointZero);
    
    // If WP0 was set, use that as home if it is not too far from current GPS location.
    // If WP0 was never set, it will be read as at lat=0, lon=0. This is in the ocean. 
    // Any reasonable max. distance will then be exceeded for any startup location on land,
    // and the home location will remain the current location.
    if (isWP0Set) {
    	if (horizontal_diff_m <= g.stickyhome_rad_m) {
    		home.lng = waypointZero.lng;
    		home.lat = waypointZero.lat;
    		
    	    gcs_send_text_P(SEVERITY_LOW, PSTR("Using WP0 pos as home pos"));

    		int32_t alt_diff_cm = abs(home.alt - waypointZero.alt);
    		if (abs(alt_diff_cm) <= (int16_t)(g.stickyhome_mad_m.get())*100) {
    			home.alt = waypointZero.alt;
        	    gcs_send_text_P(SEVERITY_LOW, PSTR("Using WP0 alt as home alt"));
    		} else {
    			// Five jerks means: Position is within expected bounds but altitude is off.
    			// You can ignore this if you use relative altitude but it is a sloppy practise.
    			// If you use absolute altitude missions, an incorrectly initialized alt. can
    			// be dangerous. If getting this signal from the plane, better reset APM (just 
    			// reset, do NOT cycle power) and try again until GPS gets a better fix.
        	    gcs_send_text_fmt(PSTR("Altitude is off from WP0 by %ld cm"), alt_diff_cm);
    			demo_servos(5);
    		}
    	} else {
    		// Three jerks means: Position is beyond expected bounds.
    		gcs_send_text_fmt(PSTR("Position is off from WP0 by %.1f m"), horizontal_diff_m);
    		demo_servos(3);
    	}
    } else {
        // Save Home to EEPROM - Command 0
       	set_cmd_with_index(home, 0);
    }

    // Save prev loc
    // -------------
    next_WP = prev_WP = home;

    // Load home for a default guided_WP
    // -------------
    guided_WP = home;
    
    // Is this going to cause trouble if g.RTL_altitude_cm is negative? 
    guided_WP.alt += g.RTL_altitude_cm;
}
