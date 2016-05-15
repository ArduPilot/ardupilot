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

#ifndef __BEV_SERVOS_H__
#define __BEV_SERVOS_H__

//for the params
#include <RC_Channel.h>
//for constrain
#include <AP_Math.h>

class BEV_Servos {
public:
    //constructor
    BEV_Servos();
    void init();
	void output(uint8_t servo, int16_t pwmout);
	void toggle();
};

#endif  // __BEV_SERVOS_H__
