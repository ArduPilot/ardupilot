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

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include <AP_AHRS/AP_AHRS.h>
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

RangeFinder::Status AP_RangeFinder_Backend::status(float distance_m) const {
    return _get_status_for_distance(distance_m);
}

// true if sensor is returning data
bool AP_RangeFinder_Backend::has_data() const {
    return ((state.status != RangeFinder::Status::NotConnected) &&
            (state.status != RangeFinder::Status::NoData));
}

// update status based on distance measurement
void AP_RangeFinder_Backend::update_status(RangeFinder::RangeFinder_State &state_arg) const
{
    set_status(state_arg, _get_status_for_distance(state_arg.distance_m));
}

void AP_RangeFinder_Backend::update_history(RangeFinder::RangeFinder_State &state_arg) const
{
    uint16_t index =
        (state.last_sample_history_index + 1) % RANGEFINDER_SAMPLE_HISTORY_SIZE;

    state.sample_history[index].distance_m = state.distance_m;
    state.sample_history[index].signal_quality_pct = state.signal_quality_pct;
    state.sample_history[index].timestamp_ms = state.last_reading_ms;

#if AP_AHRS_ENABLED
    // store the rangefinder deviation accounting for the vehicle attitude at
    // the time of measurement
    _calc_rngfnd_attitude_deviation_rad(
        state.sample_history[index].attitude_deviation_rad);

    // store horizontal distance traveled since last sample in decimeters as
    // uint8 for memory efficiency (range 0-25.5m in 0.1m increments, enough
    // for any reasonable speed at 10Hz sampling rate)
    Location current_loc;
    AP::ahrs().get_location(current_loc);
    float hor_loc_delta = current_loc.get_distance(state.last_reading_loc);
    state.sample_history[index].hor_loc_delta_dm = uint8_t(constrain_float(
        hor_loc_delta * 10 + 0.5, 0, UINT8_MAX));
    state.last_reading_loc = current_loc;
#endif // AP_AHRS_ENABLED

    state.last_sample_history_index = index;
    state.sample_history_size =
        MIN(state.sample_history_size + 1, RANGEFINDER_SAMPLE_HISTORY_SIZE);
}

// set status and update valid count
void AP_RangeFinder_Backend::set_status(RangeFinder::RangeFinder_State &state_arg, RangeFinder::Status _status)
{
    state_arg.status = _status;

    // update valid count
    if (_status == RangeFinder::Status::Good) {
        if (state_arg.range_valid_count < 10) {
            state_arg.range_valid_count++;
        }
    } else {
        state_arg.range_valid_count = 0;
    }
}

#if AP_SCRIPTING_ENABLED
// get a copy of state structure
void AP_RangeFinder_Backend::get_state(RangeFinder::RangeFinder_State &state_arg)
{
    WITH_SEMAPHORE(_sem);
    state_arg = state;
}
#endif

RangeFinder::Status
AP_RangeFinder_Backend::_get_status_for_distance(float distance_m) const
{
    // check distance
    if (distance_m > max_distance()) {
        return RangeFinder::Status::OutOfRangeHigh;
    } else if (distance_m < min_distance()) {
        return RangeFinder::Status::OutOfRangeLow;
    } else {
        return RangeFinder::Status::Good;
    }
}

#if AP_AHRS_ENABLED
// calculate the deviation of the rangefinder from its nominal direction,
// considering the current attitude of the vehicle in the pitch and roll axes
void AP_RangeFinder_Backend::_calc_rngfnd_attitude_deviation_rad(
    float &attitude_deviation_rad) const
{
    Quaternion rngfnd_rotation_quat;
    rngfnd_rotation_quat.from_rotation(orientation());

    Quaternion vehicle_attitude_quat;
    if (!AP::ahrs().get_quaternion(vehicle_attitude_quat)) {
        attitude_deviation_rad = 0; // assume no deviation
        return;
    }

    Quaternion rngfnd_orientation_quat =
        vehicle_attitude_quat * rngfnd_rotation_quat;

    // only account for roll and pitch deviation (assume no yaw deviation)
    attitude_deviation_rad =
        rngfnd_rotation_quat.roll_pitch_difference(rngfnd_orientation_quat);
}
#endif // AP_AHRS_ENABLED

#endif  // AP_RANGEFINDER_ENABLED
