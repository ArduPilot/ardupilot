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

    float GPS_vector_x = (target.lng-_frontend._current_loc.lng)*cosf(ToRad((_frontend._current_loc.lat+target.lat)*0.00000005f))*0.01113195f;
    float GPS_vector_y = (target.lat-_frontend._current_loc.lat)*0.01113195f;
    float target_distance = 100.0f*norm(GPS_vector_x, GPS_vector_y);      // Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.

    float GPS_vector_z = (_frontend._current_loc.alt); // Always assumes target is at the altitude of takeoff.  Should add consideration for 'target.alt'

    // initialise all angles to zero
    angles_to_target_deg.zero();

    // tilt calcs
    if (calc_tilt) {
    	float tan_ratio = target_distance / GPS_vector_z;
    	angles_to_target_deg.y = 90.0f - ToDeg(atanf(tan_ratio));
    }

/*
    //Debugging
	hal.console->print("\n");
	hal.console->print("\n");
	hal.console->printf("CMD Tilt: %4f", angles_to_target_deg.y);
	hal.console->print("\n");
	hal.console->printf("Alt:  %ld", _frontend._current_loc.alt);
	hal.console->print("\n");
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
    }
}


// set_angle_targets - sets angle targets in degrees
void AP_Mount_Backend::enable_RC_control(bool en)  { _RC_control_enable = en; }



void AP_Mount_Backend::cam_button_pressed(bool en)  { _cam_button_pressed = en; }



void AP_Mount_Backend::cam_button_output(int8_t output_type)  { _cam_button_output = output_type;  }


// set_angle_targets - sets angle targets in degrees
void AP_Mount_Backend::enable_follow(bool en)
{
	 if(en){
		 _enable_follow = true;
	 }else{
		 _enable_follow = false;
	 }

}

// set_angle_targets - sets angle targets in degrees
void AP_Mount_Backend::set_camera_point_ROI(float yaw)
{

	roi_gps_target.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
	roi_gps_target.alt = 0; // shouldn't be zero all the time.  Need terrian data
	roi_gps_target.lng = _frontend._current_loc.lng;
	roi_gps_target.lat = _frontend._current_loc.lat;

	//2D distance from target
	float tilt_angle;
	float pan_angle;

	tilt_angle = constrain_float(_camera_tilt_angle, 0.1f, 89.9);//Range of viewpro camera (hanging on bottom) from straightout (0) to straight down (90)


	//Covert angle for our distance calc
	tilt_angle = 90.0f - tilt_angle;
	tilt_angle = constrain_float(tilt_angle, 0.1f, 89.9);

	float distance = tanf(radians(tilt_angle))*((float)_frontend._current_loc.alt / 100.0f);

	//Camera pan angle can be negative.  Don't want negatives for our calc
	if(_camera_pan_angle >= 0){
		pan_angle = _camera_pan_angle;
	}else{
		pan_angle = 360 + _camera_pan_angle;
	}

	/* Debugging
	hal.console->print("\n");
	hal.console->print("\n");
	hal.console->printf("Camera Tilt: %4f", _camera_tilt_angle);
	hal.console->print("\n");
	hal.console->printf("Distance:  %4f", distance);
	hal.console->print("\n");
	hal.console->print("\n");
	 */

	float bearing = degrees(yaw) + pan_angle;   //// <- pass yaw to this funciton from usercode
	roi_gps_target.offset_bearing(bearing, distance);

	set_roi_target(roi_gps_target);
}


void AP_Mount_Backend::set_camera_zoom(bool zoom){

	if(!zoom){
		command_flags.zero_zoom = true;
	}else{
		command_flags.full_zoom = true;
	}

}


//Functions to control gimbal/camera from higher levels

void AP_Mount_Backend::toggle_record(){	command_flags.toggle_rec = true; }

void AP_Mount_Backend::toggle_camera_state(){command_flags.change_state = true;}

void AP_Mount_Backend::center_yaw(){ command_flags.center_yaw = true; }

void AP_Mount_Backend::look_down(){ command_flags.look_down = true; }

void AP_Mount_Backend::flip_image(){ command_flags.flip_image_IR = true; }


void AP_Mount_Backend::turn_camera_off(){command_flags.turn_camera_off = true; }

void AP_Mount_Backend::turn_camera_on(){command_flags.turn_camera_on = true; }



