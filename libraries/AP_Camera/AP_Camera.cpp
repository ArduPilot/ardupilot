// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Camera.h>
#include <AP_Relay.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_HAL.h>

// ------------------------------
#define CAM_DEBUG DISABLED

const AP_Param::GroupInfo AP_Camera::var_info[] PROGMEM = {
    // @Param: DURATION
    // @DisplayName: Duration that shutter is held open
    // @Description: How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("DURATION",    0, AP_Camera, _trigger_duration, AP_CAMERA_TRIGGER_DEFAULT_DURATION),

    // @Param: TRIGG_DIST
    // @DisplayName: Camera trigger distance
    // @Description: Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
    // @User: Standard
    // @Range: 0 1000
    AP_GROUPINFO("TRIGG_DIST",  1, AP_Camera, _trigg_dist, 0),

    AP_GROUPEND
};

void
AP_Camera::init()
{
    //setting up output channel
    _servo_out.set_range(0,1000);
    _servo_out.enable_out();

    //set the initial output to low
    output_to_channel(_servo_out,-4500);
}

/// basic relay activation
void
AP_Camera::relay_pic()
{
    _apm_relay->on();

    // leave a message that it should be active for this many loops (assumes 50hz loops)
    _trigger_counter = constrain_int16(_trigger_duration*5,0,255);
}

/// single entry point to take pictures
void
AP_Camera::trigger_pic()
{
    relay_pic();
    output_to_channel(_servo_out,4500);
}

/// de-activate the trigger after some delay, but without using a delay() function
/// should be called at 50hz
void
AP_Camera::trigger_pic_cleanup()
{
    if (_trigger_counter) {
        _trigger_counter--;
    } else {
        _apm_relay->off();
        output_to_channel(_servo_out,-4500);
    }
}

/*  update location, for triggering by GPS distance moved
    This function returns true if a picture should be taken
    The caller is responsible for taking the picture based on the return value of this function.
    The caller is also responsible for logging the details about the photo
*/
bool AP_Camera::update_location(const struct Location &loc)
{
    if (_trigg_dist == 0.0f) {
        return false;
    }
    if (_last_location.lat == 0 && _last_location.lng == 0) {
        _last_location = loc;
        return false;
    }
    if (_last_location.lat == loc.lat && _last_location.lng == loc.lng) {
        // we haven't moved - this can happen as update_location() may
        // be called without a new GPS fix
        return false;
    }
    if (get_distance(loc, _last_location) < _trigg_dist) {
        return false;
    }
    _last_location = loc;
    return true;
}

void AP_Camera::output_to_channel(RC_Channel &rc_out, int16_t angle)
{
    //deal with reversing the servo
    if(rc_out.get_reverse()) {
        angle = -angle;
    }

    //scale the desired output to meet the min and max set by the parameters
    if(angle>0) {
        rc_out.radio_out = constrain_int16( (float(angle)/4500)*(rc_out.radio_max-rc_out.radio_trim)+rc_out.radio_trim, rc_out.radio_min, rc_out.radio_max);
    } else {
        rc_out.radio_out = constrain_int16( (float(angle)/4500)*(rc_out.radio_trim-rc_out.radio_min)+rc_out.radio_trim, rc_out.radio_min, rc_out.radio_max);
    }

    //actually write to the output pin
    rc_out.output();
}
