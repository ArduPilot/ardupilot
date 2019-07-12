/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       RC_Channels.cpp - class containing an array of RC_Channel objects
 *
 */

#include <stdlib.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

#include "RC_Channel.h"

<<<<<<< HEAD
=======
RC_Channel *RC_Channels::channels;
bool RC_Channels::has_new_overrides;
AP_Float *RC_Channels::override_timeout;

const AP_Param::GroupInfo RC_Channels::var_info[] = {
    // @Group: 1_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[0], "1_",  1, RC_Channels, RC_Channel),

    // @Group: 2_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[1], "2_",  2, RC_Channels, RC_Channel),

    // @Group: 3_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[2], "3_",  3, RC_Channels, RC_Channel),

    // @Group: 4_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[3], "4_",  4, RC_Channels, RC_Channel),

    // @Group: 5_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[4], "5_",  5, RC_Channels, RC_Channel),

    // @Group: 6_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[5], "6_",  6, RC_Channels, RC_Channel),

    // @Group: 7_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[6], "7_",  7, RC_Channels, RC_Channel),

    // @Group: 8_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[7], "8_",  8, RC_Channels, RC_Channel),

    // @Group: 9_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[8], "9_",  9, RC_Channels, RC_Channel),

    // @Group: 10_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[9], "10_", 10, RC_Channels, RC_Channel),

    // @Group: 11_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[10], "11_", 11, RC_Channels, RC_Channel),

    // @Group: 12_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[11], "12_", 12, RC_Channels, RC_Channel),

    // @Group: 13_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[12], "13_", 13, RC_Channels, RC_Channel),

    // @Group: 14_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[13], "14_", 14, RC_Channels, RC_Channel),

    // @Group: 15_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[14], "15_", 15, RC_Channels, RC_Channel),

    // @Group: 16_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[15], "16_", 16, RC_Channels, RC_Channel),

    // @Param: _OVERRIDE_TIME
    // @DisplayName: RC override timeout
    // @Description: Timeout after which RC overrides will no longer be used, and RC input will resume, 0 will disable RC overrides, -1 will never timeout, and continue using overrides until they are disabled
    // @User: Advanced
    // @Range: 0.0 120.0
    // @Units: s
    AP_GROUPINFO("_OVERRIDE_TIME", 32, RC_Channels, _override_timeout, 3.0),
    
    AP_GROUPEND
};


>>>>>>> b6638ba0750049a637f33b1929a3135351beaff0
/*
  channels group object constructor
 */
RC_Channels::RC_Channels(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("RC_Channels must be singleton");
    }
    _singleton = this;
}

void RC_Channels::init(void)
{
    // setup ch_in on channels
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        channel(i)->ch_in = i;
    }

    init_aux_all();
}

uint8_t RC_Channels::get_radio_in(uint16_t *chans, const uint8_t num_channels)
{
    memset(chans, 0, num_channels*sizeof(*chans));
<<<<<<< HEAD

    const uint8_t read_channels = MIN(num_channels, NUM_RC_CHANNELS);
    for (uint8_t i = 0; i < read_channels; i++) {
        chans[i] = channel(i)->get_radio_in();
    }

=======

    const uint8_t read_channels = MIN(num_channels, NUM_RC_CHANNELS);
    for (uint8_t i = 0; i < read_channels; i++) {
        chans[i] = channels[i].get_radio_in();
    }

>>>>>>> b6638ba0750049a637f33b1929a3135351beaff0
    return read_channels;
}

// update all the input channels
bool RC_Channels::read_input(void)
{
    if (!hal.rcin->new_input() && !has_new_overrides) {
        return false;
    }

    has_new_overrides = false;

    bool success = false;
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        success |= channel(i)->update();
    }

    return success;
}

uint8_t RC_Channels::get_valid_channel_count(void)
{
    return MIN(NUM_RC_CHANNELS, hal.rcin->num_channels());
}

int16_t RC_Channels::get_receiver_rssi(void)
{
    return hal.rcin->get_rssi();
}

void RC_Channels::clear_overrides(void)
{
    RC_Channels &_rc = rc();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        _rc.channel(i)->clear_override();
    }
    // we really should set has_new_overrides to true, and rerun read_input from
    // the vehicle code however doing so currently breaks the failsafe system on
    // copter and plane, RC_Channels needs to control failsafes to resolve this
}

void RC_Channels::set_override(const uint8_t chan, const int16_t value, const uint32_t timestamp_ms)
{
    RC_Channels &_rc = rc();
    if (chan < NUM_RC_CHANNELS) {
        _rc.channel(chan)->set_override(value, timestamp_ms);
    }
}

bool RC_Channels::has_active_overrides()
{
    RC_Channels &_rc = rc();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        if (_rc.channel(i)->has_override()) {
            return true;
        }
    }

    return false;
}

bool RC_Channels::receiver_bind(const int dsmMode)
{
    return hal.rcin->rc_bind(dsmMode);
}


// support for auxillary switches:
// read_aux_switches - checks aux switch positions and invokes configured actions
void RC_Channels::read_aux_all()
{
    if (!has_valid_input()) {
        // exit immediately when no RC input
        return;
    }
    bool need_log = false;

    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            continue;
        }
        need_log |= c->read_aux();
    }
    if (need_log) {
        // guarantee that we log when a switch changes
        AP::logger().Write_RCIN();
    }
}

void RC_Channels::init_aux_all()
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            continue;
        }
        c->init_aux();
    }
    reset_mode_switch();
}

//
// Support for mode switches
//
RC_Channel *RC_Channels::flight_mode_channel()
{
    const int8_t num = flight_mode_channel_number();
    if (num <= 0) {
        return nullptr;
    }
    if (num >= NUM_RC_CHANNELS) {
        return nullptr;
    }
    return channel(num-1);
}

void RC_Channels::reset_mode_switch()
{
    RC_Channel *c = flight_mode_channel();
    if (c == nullptr) {
        return;
    }
    c->reset_mode_switch();
}

void RC_Channels::read_mode_switch()
{
    if (!has_valid_input()) {
        // exit immediately when no RC input
        return;
    }
    RC_Channel *c = flight_mode_channel();
    if (c == nullptr) {
        return;
    }
    c->read_mode_switch();
}


// singleton instance
RC_Channels *RC_Channels::_singleton;


RC_Channels &rc()
{
    return *RC_Channels::get_singleton();
}
