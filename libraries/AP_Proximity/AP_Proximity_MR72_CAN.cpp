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
#include "AP_Proximity_config.h"

#if AP_PROXIMITY_MR72_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_Proximity_MR72_CAN.h"

const AP_Param::GroupInfo AP_Proximity_MR72_CAN::var_info[] = {

    // @Param: RECV_ID
    // @DisplayName: CAN receive ID
    // @Description: The receive ID of the CAN frames. A value of zero means all IDs are accepted.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("RECV_ID", 1, AP_Proximity_MR72_CAN, receive_id, 0),

    AP_GROUPEND
};

AP_Proximity_MR72_CAN::AP_Proximity_MR72_CAN(AP_Proximity &_frontend,
                                     AP_Proximity::Proximity_State &_state,
                                     AP_Proximity_Params& _params):
    AP_Proximity_Backend(_frontend, _state, _params)
{
    multican_MR72 = new MultiCAN{FUNCTOR_BIND_MEMBER(&AP_Proximity_MR72_CAN::handle_frame, bool, AP_HAL::CANFrame &), AP_CAN::Protocol::NanoRadar, "MR72 MultiCAN"};
    if (multican_MR72 == nullptr) {
        AP_BoardConfig::allocation_error("Failed to create proximity multican");
    }

    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

// update state
void AP_Proximity_MR72_CAN::update(void)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t now = AP_HAL::millis();
    if (now - last_update_ms > 500) {
        // no new data.
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// handler for incoming frames. These come in at 100Hz
bool AP_Proximity_MR72_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);


    // check if message is coming from the right sensor ID
    const uint16_t id = frame.id;

    if (receive_id > 0 && (get_radar_id(frame.id) != uint32_t(receive_id))) {
        return false;
    }

    switch (id & 0xFU) {
    case 0xAU:
        // number of objects
        _object_count = frame.data[0];
        _current_object_index = 0;
        _temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
        _temp_boundary.reset();
        last_update_ms = AP_HAL::millis();
        break;
    case 0xBU:
        // obstacle data
        parse_distance_message(frame);
        break;
    default:
        break;
    }

    return true;

}

// parse a distance message from CAN frame
bool AP_Proximity_MR72_CAN::parse_distance_message(AP_HAL::CANFrame &frame)
{
    if (_current_object_index >= _object_count) {
        // should never happen
        return false;
    }
    _current_object_index++;

    Vector2f obstacle_fr;
    // This parsing comes from the NanoRadar MR72 datasheet
    obstacle_fr.x = ((frame.data[2] & 0x07U) * 256 + frame.data[3]) * 0.2 - 204.6;
    obstacle_fr.y = (frame.data[1] * 32 + (frame.data[2] >> 3)) * 0.2 - 500;
    const float yaw = correct_angle_for_orientation(wrap_360(degrees(atan2f(obstacle_fr.x, obstacle_fr.y))));

    const float objects_dist = obstacle_fr.length();

    if (ignore_reading(yaw, objects_dist)) {
        // obstacle is probably near ground or out of range
        return false;
    }

    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(yaw);
    _temp_boundary.add_distance(face, yaw, objects_dist);
    return true;
}

#endif // HAL_PROXIMITY_ENABLED
