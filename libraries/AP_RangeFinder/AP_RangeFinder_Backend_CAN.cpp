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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Backend_CAN.h"

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

const AP_Param::GroupInfo AP_RangeFinder_Backend_CAN::var_info[] = {

    // @Param: RECV_ID
    // @DisplayName: RangeFinder CAN receive ID
    // @Description: The receive ID of the CAN frames. A value of zero means all IDs are accepted.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("RECV_ID", 10, AP_RangeFinder_Backend_CAN, receive_id, 0),

    // @Param: SNR_MIN
    // @DisplayName: RangeFinder Minimum signal strength
    // @Description: RangeFinder Minimum signal strength (SNR) to accept distance
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("SNR_MIN", 11, AP_RangeFinder_Backend_CAN, snr_min, 0),

    AP_GROUPEND
};

// constructor
AP_RangeFinder_Backend_CAN::AP_RangeFinder_Backend_CAN(
    RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_CAN::Protocol can_type,
                                    const char *driver_name) :
    AP_RangeFinder_Backend(_state, _params)
{
    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
    multican_rangefinder = new MultiCAN{FUNCTOR_BIND_MEMBER(&AP_RangeFinder_Backend_CAN::handle_frame, bool, AP_HAL::CANFrame &), can_type, driver_name};
    if (multican_rangefinder == nullptr) {
        AP_BoardConfig::allocation_error("Failed to create rangefinder multican");
    }
}

// update the state of the sensor
void AP_RangeFinder_Backend_CAN::update(void)
{
    if (get_reading(state.distance_m)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms >= read_timeout_ms()) {
        set_status(RangeFinder::Status::NoData);
    }
}

// get distance measurement
bool AP_RangeFinder_Backend_CAN::get_reading(float &reading_m)
{
    WITH_SEMAPHORE(_sem);
    if (_distance_count != 0) {
        reading_m = _distance_sum / _distance_count;
        _distance_sum = 0;
        _distance_count = 0;
        return true;
    }

    return false;
}

// return true if the CAN ID is correct
bool AP_RangeFinder_Backend_CAN::is_correct_id(uint32_t id) const
{
    if (receive_id != 0 && id != uint32_t(receive_id.get())) {
        // incorrect receive ID
        return false;
    }
    return true;
}

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
