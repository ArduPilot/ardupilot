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

#ifndef __BEV_GEAR_H__
#define __BEV_GEAR_H__

#include <RC_Channel.h>     // RC Channel Library

#define ENABLED                 1
#define DISABLED                0

//debugging options on a uart
#define BEV_GEAR_DEBUGGING DISABLED


/// @class      BEV_GEAR
class BEV_Gear {
public:
    BEV_Gear(RC_Channel &rc_gear_switch, RC_Channel &rc_gear_out_1);

    //initializer
    void Init();

    //do-ers
    void update(bool radio_failsafe = false);
    void raise();
    void lower();
    void toggle();

    //void accessors
    bool is_raised();
    bool is_lowered();


protected:
    RC_Channel& _rc_gear_out_1;
    RC_Channel& _rc_gear_switch;

private:
    //working variables
    int16_t _gear_position; //goes from 0 (down) to 1000 (up)
    bool _switch_position; //high >1500us = true, low <=1500us = false
    bool _old_switch_position; //same as above
};

#endif  // BEV_GEAR
