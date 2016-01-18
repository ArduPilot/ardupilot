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

/// @file	AP_Camera.h
/// @brief	Photo or video camera manager, with EEPROM-backed storage of constants.

#ifndef AP_CAMERA_H
#define AP_CAMERA_H

#include <AP_Param.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include <AP_Relay.h>
#include <RC_Channel.h>

#define AP_CAMERA_TRIGGER_DEFAULT_DURATION  10      // default duration servo or relay is held open in 10ths of a second (i.e. 10 = 1 second)

/// @class	Camera
/// @brief	Object managing a Photo or video camera
class AP_Camera {

public:
    /// Constructor
    ///
    AP_Camera(AP_Relay *obj_relay, RC_Channel& servo_out) :
        _trigger_counter(0),             // count of number of cycles shutter has been held open
        _servo_out(servo_out)            // servo to open when camera is triggered (for package delivery)
    {
		AP_Param::setup_object_defaults(this, var_info);
        _apm_relay = obj_relay;
    }

    // BEV turns on servo output
    void            init();

    // single entry point to take pictures
    void            trigger_pic();

    // de-activate the trigger after some delay, but without using a delay() function
    // should be called at 50hz from main program
    void            trigger_pic_cleanup();

    // set camera trigger distance in a mission
    void            set_trigger_distance(uint32_t distance_m) { _trigg_dist.set(distance_m); }

    // Update location of vehicle and return true if a picture should be taken
    bool update_location(const struct Location &loc);

    //for writing to servo
    static void output_to_channel(RC_Channel& rc_out, int16_t angle);

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Int8         _trigger_duration;  // duration in 10ths of a second that the camera shutter is held open
    uint8_t         _trigger_counter;   // count of number of cycles shutter has been held open
    AP_Relay       *_apm_relay;         // pointer to relay object from the base class Relay. The subclasses could be AP_Relay_APM1 or AP_Relay_APM2
    RC_Channel&     _servo_out;

    void            relay_pic();        // basic relay activation

    AP_Float        _trigg_dist;     // distance between trigger points (meters)
    struct Location _last_location;

};

#endif /* AP_CAMERA_H */
