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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_RangeFinder_Backend::AP_RangeFinder_Backend(RangeFinder::RangeFinder_State &_state) :
        state(_state)
{
    _sem = hal.util->new_semaphore();    
}

// update status based on distance measurement
void AP_RangeFinder_Backend::update_status()
{
    // check distance
    if ((int16_t)state.distance_cm > state.max_distance_cm) {
        set_status(RangeFinder::RangeFinder_OutOfRangeHigh);
    } else if ((int16_t)state.distance_cm < state.min_distance_cm) {
        set_status(RangeFinder::RangeFinder_OutOfRangeLow);
    } else {
        set_status(RangeFinder::RangeFinder_Good);
    }
}

// set status and update valid count
void AP_RangeFinder_Backend::set_status(RangeFinder::RangeFinder_Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == RangeFinder::RangeFinder_Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}

/*
  set pre-arm checks to passed if the range finder has been exercised through a reasonable range of movement
      max distance sensed is at least 50cm > min distance sensed
      max distance < 200cm
      min distance sensed is within 10cm of ground clearance or sensor's minimum distance
 */
void AP_RangeFinder_Backend::update_pre_arm_check()
{
    // return immediately if already passed or no sensor data
    if (state.pre_arm_check || state.status == RangeFinder::RangeFinder_NotConnected || state.status == RangeFinder::RangeFinder_NoData) {
        return;
    }

    // update min, max captured distances
    state.pre_arm_distance_min = MIN(state.distance_cm, state.pre_arm_distance_min);
    state.pre_arm_distance_max = MAX(state.distance_cm, state.pre_arm_distance_max);

    // Check that the range finder has been exercised through a realistic range of movement
    if (((state.pre_arm_distance_max - state.pre_arm_distance_min) > RANGEFINDER_PREARM_REQUIRED_CHANGE_CM) &&
         (state.pre_arm_distance_max < RANGEFINDER_PREARM_ALT_MAX_CM) &&
         ((int16_t)state.pre_arm_distance_min < (MAX(state.ground_clearance_cm,state.min_distance_cm) + 10)) &&
         ((int16_t)state.pre_arm_distance_min > (MIN(state.ground_clearance_cm,state.min_distance_cm) - 10))) {
        state.pre_arm_check = true;
    }
}
