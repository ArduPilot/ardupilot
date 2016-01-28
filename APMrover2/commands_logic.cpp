/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
bool Rover::start_command(const AP_Mission::Mission_Command& cmd)
{
    // log when new commands start
    if (should_log(MASK_LOG_CMD)) {
        DataFlash.Log_Write_Mission_Cmd(mission, cmd);
    }

    // exit immediately if not in AUTO mode
    if (control_mode != AUTO) {
        return false;
    }

    gcs_send_text_fmt(MAV_SEVERITY_INFO, "Executing command ID #%i",cmd.id);

    // remember the course of our next navigation leg
    next_navigation_leg_cd = mission.get_next_ground_course_cd(0);

	switch(cmd.id){
		case MAV_CMD_NAV_WAYPOINT:	// Navigate to Waypoint
			do_nav_wp(cmd);
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			do_RTL();
			break;

        // Conditional commands
		case MAV_CMD_CONDITION_DELAY:
			do_wait_delay(cmd);
			break;

		case MAV_CMD_CONDITION_DISTANCE:
			do_within_distance(cmd);
			break;

        // Do commands
		case MAV_CMD_DO_CHANGE_SPEED:
			do_change_speed(cmd);
			break;

		case MAV_CMD_DO_SET_HOME:
			do_set_home(cmd);
			break;

    	case MAV_CMD_DO_SET_SERVO:
            ServoRelayEvents.do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);
            break;

    	case MAV_CMD_DO_SET_RELAY:
            ServoRelayEvents.do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
            break;

    	case MAV_CMD_DO_REPEAT_SERVO:
            ServoRelayEvents.do_repeat_servo(cmd.content.repeat_servo.channel, cmd.content.repeat_servo.pwm,
                                             cmd.content.repeat_servo.repeat_count, cmd.content.repeat_servo.cycle_time * 1000.0f);
            break;

    	case MAV_CMD_DO_REPEAT_RELAY:
            ServoRelayEvents.do_repeat_relay(cmd.content.repeat_relay.num, cmd.content.repeat_relay.repeat_count,
                                             cmd.content.repeat_relay.cycle_time * 1000.0f);
            break;

#if CAMERA == ENABLED
        case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
            break;

        case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
            do_digicam_configure(cmd);
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
            do_digicam_control(cmd);
            break;

        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            camera.set_trigger_distance(cmd.content.cam_trigg_dist.meters);
            break;
#endif

#if MOUNT == ENABLED
		// Sets the region of interest (ROI) for a sensor set or the
		// vehicle itself. This can then be used by the vehicles control
		// system to control the vehicle attitude and the attitude of various
		// devices such as cameras.
		//    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
        case MAV_CMD_DO_SET_ROI:
            if (cmd.content.location.alt == 0 && cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                // switch off the camera tracking if enabled
                if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                    camera_mount.set_mode_to_default();
                }
            } else {
                // send the command to the camera mount
                camera_mount.set_roi_target(cmd.content.location);
            }
            break;
#endif

		default:
		    // return false for unhandled commands
		    return false;
	}

	// if we got this far we must have been successful
	return true;
}

// exit_mission - callback function called from ap-mission when the mission has completed
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
void Rover::exit_mission()
{
    if (control_mode == AUTO) {
        gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "No commands. Can't set AUTO. Setting HOLD");
        set_mode(HOLD);
    }
}

// verify_command_callback - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool Rover::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == AUTO) {
        bool cmd_complete = verify_command(cmd);

        // send message to GCS
        if (cmd_complete) {
            gcs_send_mission_item_reached_message(cmd.index);
        }

        return cmd_complete;
    }
    return false;
}
/********************************************************************************/
// Verify command Handlers
//      Returns true if command complete
/********************************************************************************/

bool Rover::verify_command(const AP_Mission::Mission_Command& cmd)
{
	switch(cmd.id) {

		case MAV_CMD_NAV_WAYPOINT:
			return verify_nav_wp(cmd);

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return verify_RTL();

        case MAV_CMD_CONDITION_DELAY:
            return verify_wait_delay();

        case MAV_CMD_CONDITION_DISTANCE:
            return verify_within_distance();

        default:
            if (cmd.id > MAV_CMD_CONDITION_LAST) {
                // this is a command that doesn't require verify
                return true;
            }
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Verify conditon. Unsupported command");
            return true;
	}
    return false;
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

void Rover::do_RTL(void)
{
    prev_WP = current_loc;
	control_mode 	= RTL;
	next_WP = home;
}

void Rover::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = abs(cmd.p1);

    // this is the distance we travel past the waypoint - not there yet so 0 initially
    distance_past_wp = 0;

	set_next_WP(cmd.content.location);
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
bool Rover::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
        // Check if we need to loiter at this waypoint
        if (loiter_time_max > 0) {
            if (loiter_time == 0) {  // check if we are just starting loiter
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Reached waypoint #%i. Loiter for %i seconds",
                                  (unsigned)cmd.index,
                                  (unsigned)loiter_time_max);
                // record the current time i.e. start timer
                loiter_time = millis();
            }
            // Check if we have loiter long enough
            if (((millis() - loiter_time) / 1000) < loiter_time_max) {
                return false;
            }
        } else {
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Reached waypoint #%i. Distance %um",
                              (unsigned)cmd.index,
                              (unsigned)get_distance(current_loc, next_WP));
        }
        return true;
    }

    // have we gone past the waypoint?
    // We should always go through the waypoint i.e. the above code
    // first before we go past it.
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        // check if we have gone futher past the wp then last time and output new message if we have
        if ((uint32_t)distance_past_wp != (uint32_t)get_distance(current_loc, next_WP)) {
            distance_past_wp = get_distance(current_loc, next_WP);
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Passed waypoint #%i. Distance %um",
                              (unsigned)cmd.index,
                              (unsigned)distance_past_wp);
        }
        // Check if we need to loiter at this waypoint
        if (loiter_time_max > 0) {
            if (((millis() - loiter_time) / 1000) < loiter_time_max) {
                return false;
            }
        }
        distance_past_wp = 0;
        return true;
    }

    return false;
}

bool Rover::verify_RTL()
{
	if (wp_distance <= g.waypoint_radius) {
		gcs_send_text(MAV_SEVERITY_INFO,"Reached destination");
                rtl_complete = true;
		return true;
	}

    // have we gone past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Reached destination. Distance away %um",
                          (unsigned)get_distance(current_loc, next_WP));
        rtl_complete = true;
        return true;
    }

    return false;
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

void Rover::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
	condition_start = millis();
	condition_value  = cmd.content.delay.seconds * 1000;    // convert seconds to milliseconds
}

void Rover::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
	condition_value  = cmd.content.distance.meters;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool Rover::verify_wait_delay()
{
	if ((uint32_t)(millis() - condition_start) > (uint32_t)condition_value){
		condition_value 	= 0;
		return true;
	}
	return false;
}

bool Rover::verify_within_distance()
{
	if (wp_distance < condition_value){
		condition_value = 0;
		return true;
	}
	return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

void Rover::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        g.speed_cruise.set(cmd.content.speed.target_ms);
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Cruise speed: %.1f m/s", (double)g.speed_cruise.get());
    }

	if (cmd.content.speed.throttle_pct > 0 && cmd.content.speed.throttle_pct <= 100) {
		g.throttle_cruise.set(cmd.content.speed.throttle_pct);
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Cruise throttle: %.1f", g.throttle_cruise.get());
    }
}

void Rover::do_set_home(const AP_Mission::Mission_Command& cmd)
{
	if(cmd.p1 == 1 && have_position) {
		init_home();
	} else {
        ahrs.set_home(cmd.content.location);
		home_is_set = HOME_SET_NOT_LOCKED;
		Log_Write_Home_And_Origin();
        GCS_MAVLINK::send_home_all(cmd.content.location);
	}
}

// do_digicam_configure Send Digicam Configure message with the camera library
void Rover::do_digicam_configure(const AP_Mission::Mission_Command& cmd)
{
#if CAMERA == ENABLED
    camera.configure(cmd.content.digicam_configure.shooting_mode,
                     cmd.content.digicam_configure.shutter_speed,
                     cmd.content.digicam_configure.aperture,
                     cmd.content.digicam_configure.ISO,
                     cmd.content.digicam_configure.exposure_type,
                     cmd.content.digicam_configure.cmd_id,
                     cmd.content.digicam_configure.engine_cutoff_time);
#endif
}

// do_digicam_control Send Digicam Control message with the camera library
void Rover::do_digicam_control(const AP_Mission::Mission_Command& cmd)
{
#if CAMERA == ENABLED
    if (camera.control(cmd.content.digicam_control.session,
                       cmd.content.digicam_control.zoom_pos,
                       cmd.content.digicam_control.zoom_step,
                       cmd.content.digicam_control.focus_lock,
                       cmd.content.digicam_control.shooting_cmd,
                       cmd.content.digicam_control.cmd_id)) {
            log_picture();
    }
#endif
}

// do_take_picture - take a picture with the camera library
void Rover::do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic(true);
    log_picture();
#endif
}

// log_picture - log picture taken and send feedback to GCS
void Rover::log_picture()
{
    if (!camera.using_feedback_pin()) {
        gcs_send_message(MSG_CAMERA_FEEDBACK);
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
        }
    } else {
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Trigger(ahrs, gps, current_loc);
        }      
    }
}
