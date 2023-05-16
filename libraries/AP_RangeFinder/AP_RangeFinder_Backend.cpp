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
    // check distance
    if (state.distance_m > max_distance_cm() * 0.01f) {
        set_status(RangeFinder::Status::OutOfRangeHigh);
    } else if (state.distance_m < min_distance_cm() * 0.01f) {
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

void AP_RangeFinder_Backend::update_filter()
{
    const uint32_t now = AP_HAL::millis();
    float h0 = 0.001f * (now - _filter_state.last_filter_ms);
    float v0 = state.distance_m;

    // Do not enable filted!
    if(is_zero(params.filt_r0.get())){
        _filter_state.vp = v0;
        return;
    }

    // reset h0 for timeout 1000ms
    if(_filter_state.last_filter_ms == 0 || (now - _filter_state.last_filter_ms) >= 1000){
        memset(&_filter_state,0,sizeof(_filter_state));
        _filter_state.v1 = v0;
        h0 = 0.02f;
    }
    _filter_state.last_filter_ms = now;

    float h1 = params.filt_n1 * h0;
    float h2 = params.filt_n2 * h0;

    float fh = fhan(_filter_state.v1 - v0,_filter_state.v2,params.filt_r0,h1);
    _filter_state.v1 += h0 * _filter_state.v2;
    _filter_state.v2 += h0 * fh;
    _filter_state.vp = _filter_state.v1 + h2 * _filter_state.v2;
}