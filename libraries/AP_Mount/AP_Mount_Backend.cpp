#include "AP_Mount_Backend.h"
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

// set_angle_targets - sets angle targets in degrees
void AP_Mount_Backend::set_angle_targets(float roll, float tilt, float pan)
{
    // set angle targets
    _angle_ef_target_rad.x = radians(roll);
    _angle_ef_target_rad.y = radians(tilt);
    _angle_ef_target_rad.z = radians(pan);

    // set the mode to mavlink targeting
    _frontend.set_mode(_instance, MAV_MOUNT_MODE_MAVLINK_TARGETING);
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount_Backend::set_roi_target(const struct Location &target_loc)
{
    // set the target gps location
    _state._roi_target = target_loc;
/*
	hal.console->print("\n");
	hal.console->print("\n");
	hal.console->printf("lat:%ld", _state._roi_target.lat)  ;
	hal.console->print("   ");
	hal.console->printf("long:%ld", _state._roi_target.lng)  ;
	hal.console->print("\n");
	hal.console->print("\n");
*/



    // set the mode to GPS tracking mode
    _frontend.set_mode(_instance, MAV_MOUNT_MODE_GPS_POINT);
}

// process MOUNT_CONFIGURE messages received from GCS.  deprecated.
void AP_Mount_Backend::handle_mount_configure(const mavlink_mount_configure_t &packet)
{
    set_mode((MAV_MOUNT_MODE)packet.mount_mode);
    _state._stab_roll = packet.stab_roll;
    _state._stab_tilt = packet.stab_pitch;
    _state._stab_pan = packet.stab_yaw;
}

// process MOUNT_CONTROL messages received from GCS. deprecated.
void AP_Mount_Backend::handle_mount_control(const mavlink_mount_control_t &packet)
{
    control((int32_t)packet.input_a, (int32_t)packet.input_b, (int32_t)packet.input_c, _state._mode);
}

void AP_Mount_Backend::control(int32_t pitch_or_lat, int32_t roll_or_lon, int32_t yaw_or_alt, MAV_MOUNT_MODE mount_mode)
{
    _frontend.set_mode(_instance, mount_mode);

    // interpret message fields based on mode
    switch (_frontend.get_mode(_instance)) {
        case MAV_MOUNT_MODE_RETRACT:
        case MAV_MOUNT_MODE_NEUTRAL:
            // do nothing with request if mount is retracted or in neutral position
            break;

        // set earth frame target angles from mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            set_angle_targets(roll_or_lon*0.01f, pitch_or_lat*0.01f, yaw_or_alt*0.01f);
            break;

        // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
        case MAV_MOUNT_MODE_RC_TARGETING:
            // do nothing if pilot is controlling the roll, pitch and yaw
            break;

        // set lat, lon, alt position targets from mavlink message

        case MAV_MOUNT_MODE_GPS_POINT: {
            const Location target_location{
                pitch_or_lat,
                roll_or_lon,
                yaw_or_alt,
                Location::AltFrame::ABOVE_HOME
            };
            set_roi_target(target_location);
            break;
        }

        default:
            // do nothing
            break;
    }
}

void AP_Mount_Backend::rate_input_rad(float &out, const RC_Channel *chan, float min, float max) const
{
    if ((chan == nullptr) || (chan->get_radio_in() == 0)) {
        return;
    }
    out += chan->norm_input_dz() * 0.0001f * _frontend._joystick_speed;
    out = constrain_float(out, radians(min*0.01f), radians(max*0.01f));
}

// update_targets_from_rc - updates angle targets using input from receiver
void AP_Mount_Backend::update_targets_from_rc()
{
    const RC_Channel *roll_ch = rc().channel(_state._roll_rc_in - 1);
    const RC_Channel *tilt_ch = rc().channel(_state._tilt_rc_in - 1);
    const RC_Channel *pan_ch = rc().channel(_state._pan_rc_in - 1);

    // if joystick_speed is defined then pilot input defines a rate of change of the angle
    if (_frontend._joystick_speed) {
        // allow pilot position input to come directly from an RC_Channel
        rate_input_rad(_angle_ef_target_rad.x,
                       roll_ch,
                       _state._roll_angle_min,
                       _state._roll_angle_max);
        rate_input_rad(_angle_ef_target_rad.y,
                       tilt_ch,
                       _state._tilt_angle_min,
                       _state._tilt_angle_max);
        rate_input_rad(_angle_ef_target_rad.z,
                       pan_ch,
                       _state._pan_angle_min,
                       _state._pan_angle_max);
    } else {
        // allow pilot rate input to come directly from an RC_Channel
        if ((roll_ch != nullptr) && (roll_ch->get_radio_in() != 0)) {
            _angle_ef_target_rad.x = angle_input_rad(roll_ch, _state._roll_angle_min, _state._roll_angle_max);
        }
        if ((tilt_ch != nullptr) && (tilt_ch->get_radio_in() != 0)) {
            _angle_ef_target_rad.y = angle_input_rad(tilt_ch, _state._tilt_angle_min, _state._tilt_angle_max);
        }
        if ((pan_ch != nullptr) && (pan_ch->get_radio_in() != 0)) {
            _angle_ef_target_rad.z = angle_input_rad(pan_ch, _state._pan_angle_min, _state._pan_angle_max);
        }
    }
}




// returns the angle (radians) that the RC_Channel input is receiving
float AP_Mount_Backend::angle_input_rad(const RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return radians(((rc->norm_input() + 1.0f) * 0.5f * (angle_max - angle_min) + angle_min)*0.01f);
}

// calc_angle_to_location - calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
void AP_Mount_Backend::calc_angle_to_location(const struct Location &target, Vector3f& angles_to_target_deg, bool calc_tilt, bool calc_pan, bool relative_pan)
{

	//For logging
	_lat = target.lat;
	_long = target.lng;

/*
	hal.console->print("\n");
	hal.console->printf("pan:%ld", _frontend._current_loc.alt)  ;
	hal.console->print("\n");
*/


    float GPS_vector_x = (target.lng-_frontend._current_loc.lng)*cosf(ToRad((_frontend._current_loc.lat+target.lat)*0.00000005f))*0.01113195f;
    float GPS_vector_y = (target.lat-_frontend._current_loc.lat)*0.01113195f;
    //float GPS_vector_z = (target.alt-_frontend._current_loc.alt);                 // baro altitude(IN CM) should be adjusted to known home elevation before take off (Set altimeter).
    //float GPS_vector_z = (_frontend._current_loc.alt - target.alt);

    float GPS_vector_z = (_frontend._current_loc.alt);
    float target_distance = 100.0f*norm(GPS_vector_x, GPS_vector_y);      // Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.

    // initialise all angles to zero
    angles_to_target_deg.zero();

    // tilt calcs
    if (calc_tilt) {
       // angles_to_target_deg.y = ToDeg(atan2f(GPS_vector_z, target_distance));
    	//angles_to_target_deg.y = ToDeg(atan2f(target_distance, GPS_vector_z));
    	float tan_ratio = target_distance / GPS_vector_z;
    	angles_to_target_deg.y = 90.0f - ToDeg(atanf(tan_ratio));


    	if(command_flags.flip_image){

    		//angles_to_target_deg.y = 180.0f - angles_to_target_deg.y;

    	}

    }

/*
	hal.console->print("\n");
	hal.console->printf("Dis:%f", target_distance)  ;
	hal.console->printf("   ")  ;
	hal.console->printf("alt:%f", GPS_vector_z)  ;
	hal.console->printf("   ")  ;
	hal.console->printf("tilt:%f", angles_to_target_deg.y)  ;
	hal.console->print("\n");
*/

    // pan calcs
    if (calc_pan) {
        // calc absolute heading and then convert to vehicle relative yaw
        angles_to_target_deg.z = (atan2f(GPS_vector_x, GPS_vector_y));  //still in radians
        if (relative_pan) {
            angles_to_target_deg.z = wrap_PI(angles_to_target_deg.z - AP::ahrs().yaw);
        }

        angles_to_target_deg.z = ToDeg(angles_to_target_deg.z);

        _roi_pan = angles_to_target_deg.z;
    }
}






// set_angle_targets - sets angle targets in degrees
void AP_Mount_Backend::enable_RC_control(bool en)
{

_RC_control_enable = en;

}



// set_angle_targets - sets angle targets in degrees
void AP_Mount_Backend::enable_follow(bool en)
{

 if(en){

	 //enable_follow_yaw();

	 _enable_follow = true;

 }else{


	 _enable_follow = false;

 }

}

// set_angle_targets - sets angle targets in degrees
void AP_Mount_Backend::set_camera_point_ROI(float yaw)
{

	roi_gps_target.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
	roi_gps_target.alt = 0;
	roi_gps_target.lng = _frontend._current_loc.lng;
	roi_gps_target.lat = _frontend._current_loc.lat;


	//2D distance from target
	float tilt_angle;
	float pan_angle;


	if(!command_flags.flip_image){

		tilt_angle = constrain_float(_camera_tilt_angle, 0.1f, 89.9);
		tilt_angle = 90.0f - tilt_angle;

	}else{

		tilt_angle = constrain_float(_camera_tilt_angle, 90.1f, 179.9f);
		tilt_angle = tilt_angle - 90.1f;

		tilt_angle = constrain_float(_camera_tilt_angle, 0.1f, 89.9);

	}



	float distance = tanf(radians(tilt_angle))*((float)_frontend._current_loc.alt / 100.0f);

	if(_camera_pan_angle >= 0){

		pan_angle = _camera_pan_angle;

	}else{

		pan_angle = 360 + _camera_pan_angle;

	}


	float bearing = degrees(yaw) + pan_angle;   //// <- pass yaw to this funciton from usercode

	roi_gps_target.offset_bearing(bearing, distance);

/*
	hal.console->print("\n");
	hal.console->print("\n");
	hal.console->printf("yaw:%f", degrees(yaw))  ;
	hal.console->print("\n");
	hal.console->printf("tilt:%f", tilt_angle)  ;
	hal.console->print(	"\n");
	hal.console->printf("pan:%f", pan_angle)  ;
	hal.console->print("\n");



	hal.console->print("\n");
	hal.console->printf("camera:%f", tilt_angle)  ;
	hal.console->print("\n");

*/
	set_roi_target(roi_gps_target);
}



void AP_Mount_Backend::toggle_record(bool type){

	if(type){

		command_flags.toggle_video_tracking = true;

	}else{
		command_flags.toggle_video = true;
	}

}


void AP_Mount_Backend::toggle_tracking(){

	command_flags.toggle_tracking = true;

}

void AP_Mount_Backend::toggle_PIP(){

	command_flags.toggle_pip = true;

}





void AP_Mount_Backend::toggle_camera_state(bool type){


	if(type){

		command_flags.toggle_tracking_video_state = true;

	}else{

		command_flags.change_state = true;
	}

}



void AP_Mount_Backend::flip_image(bool flip){

	order_flip = true;

	if(flip){

		command_flags.flip_image = true;

	}else{

		command_flags.flip_image = false;

	}

}



void AP_Mount_Backend::center_yaw(){


	command_flags.center_yaw = true;


}


void AP_Mount_Backend::turn_camera_off(){


	command_flags.turn_camera_off = true;


}

void AP_Mount_Backend::take_picture(){


	command_flags.take_picture = true;
}
