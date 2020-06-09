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
#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_RangeFinder_Backend::AP_RangeFinder_Backend(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        state(_state),
		params(_params)
{
    _backend_type = type();
}

MAV_DISTANCE_SENSOR AP_RangeFinder_Backend::get_mav_distance_sensor_type() const {
    if (type() == RangeFinder::Type::NONE) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return _get_mav_distance_sensor_type();
}

RangeFinder::Status AP_RangeFinder_Backend::status() const {
    if (type() == RangeFinder::Type::NONE) {
        // turned off at runtime?
        return RangeFinder::Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool AP_RangeFinder_Backend::has_data() const {
    return ((state.status != RangeFinder::Status::NotConnected) &&
            (state.status != RangeFinder::Status::NoData));
}

// update status based on distance measurement
void AP_RangeFinder_Backend::update_status()
{
    const float koeff = 1 - MIN(params.flt_coeff,.95); 
    int glitchcount = params.glitchcount; 
    if ( koeff > .95){
       glitchcount = 0;   //disable glitch filter if filter coeff param is 0, which also disables running average
    }
    // glitch remover: if measurement is greater than 25% from running average, use running average instead unless it has been greater for more than gltch_cnt measurements,then use it
    if (labs(state.distance_cm - avrgd_distance_cm) > (0.25 * avrgd_distance_cm) && glitch_count++ < glitchcount) { 
        state.distance_cm = avrgd_distance_cm;            
        } else {
       glitch_count = 0;
    }
    //running average of valid values
    avrgd_distance_cm = avrgd_distance_cm * (1-koeff) + (koeff) * state.distance_cm;
    state.distance_cm = avrgd_distance_cm;

    // check distance within range of finder
    if ((int16_t)state.distance_cm > params.max_distance_cm) {
        set_status(RangeFinder::Status::OutOfRangeHigh);
    } else if ((int16_t)state.distance_cm < params.min_distance_cm) {
        set_status(RangeFinder::Status::OutOfRangeLow);
    } else {
        set_status(RangeFinder::Status::Good);
    }
}

// set status and update valid count
void AP_RangeFinder_Backend::set_status(RangeFinder::Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == RangeFinder::Status::Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}

