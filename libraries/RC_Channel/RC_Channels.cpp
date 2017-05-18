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

#include "RC_Channel.h"

RC_Channel *RC_Channels::channels;

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
    
    AP_GROUPEND
};


/*
  channels group object constructor
 */
RC_Channels::RC_Channels(void)
{
    channels = obj_channels;
    
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    // setup ch_in on channels
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        channels[i].ch_in = i;
    }
}

/*
  call read() and set_pwm() on all channels
 */
void
RC_Channels::set_pwm_all(void)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        channels[i].set_pwm(channels[i].read());
    }
}
