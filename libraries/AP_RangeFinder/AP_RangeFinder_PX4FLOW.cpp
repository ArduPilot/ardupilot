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

#include "AP_RangeFinder_PX4FLOW.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_OpticalFlow/AP_OpticalFlow_PX4Flow.h>
#include <atomic>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PX4FLOW::AP_RangeFinder_PX4FLOW(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
}

bool AP_RangeFinder_PX4FLOW::detect(RangeFinder &_ranger, uint8_t instance)
{
    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_PX4FLOW::update(void)
{
    state.distance_cm = static_cast<float>(AP_OpticalFlow_PX4Flow::ground_distance.load(std::memory_order_relaxed)) * 0.1f;
    update_status();
}
