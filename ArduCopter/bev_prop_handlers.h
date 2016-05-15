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
 *
 */


#ifndef __BEV_CUSTOM_H__
#define __BEV_CUSTOM_H__

//BEV our own includes
#include <BEV_SpdHgt.h>
#include <BEV_TransitionState.h>
#include <BEV_Effectors.h>
//#include <BEV_Gimbal.h>

static BEV_Effectors motors(to_copter_callback, to_plane_callback,
        to_copternav_callback, channel_roll_out, channel_pitch_out,
        channel_throttle_out, channel_rudder_out, //plane controller outputs
        g.rc_elevon_left, g.rc_elevon_right, //plane elevons
        g.rc_1, g.rc_2, g.rc_3, g.rc_4, //copter controller outputs, radio input
        g.rc_6, g.rc_9, //transition switch, transition channel out
        g.rc_7, g.rc_10); //gear switch and output

#define BEV_RTL_TRANSITION_DISTANCE (15000) //cm

#endif //__BEV_CUSTOM_H__
