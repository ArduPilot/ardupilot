#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_Proximity_MR72_CAN.h"
#include <AP_HAL/utility/sparse-endian.h>

#if (HAL_PROXIMITY_ENABLED && AP_PROXIMITY_MR72_ENABLED)

const AP_Param::GroupInfo AP_Proximity_MR72_CAN::var_info[] = {

    // @Param: RECV_ID
    // @DisplayName: CAN receive ID
    // @Description: The receive ID of the CAN frames.
    // @Range: 0 8
    // @User: Advanced
    AP_GROUPINFO("RECV_ID", 1, AP_Proximity_MR72_CAN, receive_id, 0),

    AP_GROUPEND
};

MR72_MultiCAN *AP_Proximity_MR72_CAN::multican;

AP_Proximity_MR72_CAN::AP_Proximity_MR72_CAN(AP_Proximity &_frontend,
                                     AP_Proximity::Proximity_State &_state,
                                     AP_Proximity_Params& _params):
    AP_Proximity_Backend(_frontend, _state, _params)
{
    if (multican == nullptr) {
        multican = new MR72_MultiCAN();
        if (multican == nullptr) {
            AP_BoardConfig::allocation_error("MR72_CAN");
        }
    }

    {
        // add to linked list of drivers
        WITH_SEMAPHORE(multican->sem);
        auto *prev = multican->drivers;
        next = prev;
        multican->drivers = this;
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

    if ((id & 0xF0) != (receive_id * 0x10)) {
        return false;
    }

    switch (id & 0xF) {
    case 0xA:
        // number of objects
        _object_count = frame.data[0];
        _current_object_index = 0;
        _temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
        _temp_boundary.reset();
        last_update_ms = AP_HAL::millis();
        break;
    case 0xB:
        // obstacle data
        parse_distance_message(frame);
        break;
    default:
        break;
    }

    return true;

}

bool AP_Proximity_MR72_CAN::parse_distance_message(AP_HAL::CANFrame &frame)
{
    if (_current_object_index >= _object_count) {
        // should never happen
        return false;
    }
    _current_object_index++;

    Vector2f obstacle_fr;
    obstacle_fr.x = ((frame.data[2]&0x07)*256+frame.data[3])*0.2-204.6;
    obstacle_fr.y = (frame.data[1]*32+(frame.data[2] >> 3))*0.2-500;
    float yaw = wrap_360(degrees(atan2f(-obstacle_fr.x, obstacle_fr.y)));
    yaw = correct_angle_for_orientation(yaw);

    const float objects_dist = obstacle_fr.length();

    if (ignore_reading(yaw, objects_dist)) {
        // obstacle is probably near ground or out of range
        return false;
    }

    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(yaw);
    _temp_boundary.add_distance(face, yaw, objects_dist);
    return true;
}

// handle frames from CANSensor, passing to the drivers
void MR72_MultiCAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(sem);
    for (auto *d = drivers; d; d=d->next) {
        if (d->handle_frame(frame)) {
            break;
        }
    }
}

#endif // HAL_PROXIMITY_ENABLED
