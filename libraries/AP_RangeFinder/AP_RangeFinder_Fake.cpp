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
 *   AP_RangeFinder_Fake.cpp - Fake rangefinder that reports a fixed distance to the ground
 *
 */
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "RangeFinder.h"
#include "AP_RangeFinder_Fake.h"

/* 
   Constructor that just sets the status as ready
*/
AP_RangeFinder_Fake::AP_RangeFinder_Fake(RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_state)
{
    state.max_distance_cm = state.ground_clearance_cm + 1;
    state.min_distance_cm = state.ground_clearance_cm - 1;
    set_status(RangeFinder::RangeFinder_NoData);
}

/* 
   If we instantiate a fake rangefinder, it should always be detected
*/
bool AP_RangeFinder_Fake::detect(RangeFinder::RangeFinder_State &_state)
{
    return true;
}

/*
  update distance_cm 
 */
void AP_RangeFinder_Fake::update(void)
{
    state.distance_cm = state.ground_clearance_cm;
    state.last_reading_ms = AP_HAL::millis();

    // update range_valid state based on distance measured
    update_status();
}

