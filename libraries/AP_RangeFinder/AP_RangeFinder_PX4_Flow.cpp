// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_RangeFinder_PX4_Flow.cpp - Arduino Library for using sonar on PX4Flow
 *       Code by krriel (info@krriel.de)
 */

#include "AP_RangeFinder_PX4_Flow.h"
#include "AP_OpticalFlow_PX4.h"

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PX4_Flow::AP_RangeFinder_PX4_Flow(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
}

/* 
   return that PX4Flow init is ok, real detect will not work because PX4_Flow will be init after RangeFinder functions
*/
bool AP_RangeFinder_PX4_Flow::detect(RangeFinder &_ranger, uint8_t instance)
{
    return true;
}


/* 
   update the state of the sensor
*/
void AP_RangeFinder_PX4_Flow::update(void)
{
    float dist_m;
    dist_m = AP_OpticalFlow_PX4_ground_distance();
    if (dist_m > -0.5) { //PX4Flow init ok
        if (dist_m < 0.0f) dist_m = 0.0f;
        state.distance_cm = dist_m * 100.0f;
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
