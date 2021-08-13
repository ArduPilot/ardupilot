#include "MissionItemProtocol_Fence.h"

#include <AC_Fence/AC_Fence.h>
#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>

#if AP_FENCE_ENABLED

/*
  public function to format mission item as mavlink_mission_item_int_t
 */
bool MissionItemProtocol_Fence::get_item_as_mission_item(uint16_t seq,
                                                         mavlink_mission_item_int_t &ret_packet)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return false;
    }
    const uint8_t num_stored_items = fence->polyfence().num_stored_items();
    if (seq > num_stored_items) {
        return false;
    }

    AC_PolyFenceItem fenceitem;

    if (!fence->polyfence().get_item(seq, fenceitem)) {
        return false;
    }

    MAV_CMD ret_cmd = MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION; // initialised to avoid compiler warning
    float p1 = 0;
    switch (fenceitem.type) {
    case AC_PolyFenceType::POLYGON_INCLUSION:
        ret_cmd = MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        p1 = fenceitem.vertex_count;
        break;
    case AC_PolyFenceType::POLYGON_EXCLUSION:
        ret_cmd = MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        p1 = fenceitem.vertex_count;
        break;
    case AC_PolyFenceType::RETURN_POINT:
        ret_cmd = MAV_CMD_NAV_FENCE_RETURN_POINT;
        break;
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
        ret_cmd = MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        p1 = fenceitem.radius;
        break;
    case AC_PolyFenceType::CIRCLE_INCLUSION:
        ret_cmd = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        p1 = fenceitem.radius;
        break;
    case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        // should never have an AC_PolyFenceItem with these types
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        FALLTHROUGH;
    case AC_PolyFenceType::END_OF_STORAGE:
        return false;
    }

    ret_packet.command = ret_cmd;
    ret_packet.param1 = p1;
    ret_packet.x = fenceitem.loc.x;
    ret_packet.y = fenceitem.loc.y;
    ret_packet.z = 0;

    return true;
}

MAV_MISSION_RESULT MissionItemProtocol_Fence::get_item(const GCS_MAVLINK &_link,
                                                       const mavlink_message_t &msg,
                                                       const mavlink_mission_request_int_t &packet,
                                                       mavlink_mission_item_int_t &ret_packet)
{
    const uint8_t num_stored_items = _fence.polyfence().num_stored_items();
    if (packet.seq > num_stored_items) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }

    if (!get_item_as_mission_item(packet.seq, ret_packet)) {
        return MAV_MISSION_ERROR;
    }

    return MAV_MISSION_ACCEPTED;
}

uint16_t MissionItemProtocol_Fence::item_count() const
{
    if (receiving) {
        return _new_items_count;
    }
    return _fence.polyfence().num_stored_items();
}

static MAV_MISSION_RESULT convert_MISSION_ITEM_INT_to_AC_PolyFenceItem(const mavlink_mission_item_int_t &mission_item_int, AC_PolyFenceItem &ret)
{
    if (mission_item_int.frame != MAV_FRAME_GLOBAL &&
        mission_item_int.frame != MAV_FRAME_GLOBAL_INT &&
        mission_item_int.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT &&
        mission_item_int.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
        mission_item_int.frame != MAV_FRAME_GLOBAL_TERRAIN_ALT &&
        mission_item_int.frame != MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
        return MAV_MISSION_UNSUPPORTED_FRAME;
    }

    switch (mission_item_int.command) {
    case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
        ret.type = AC_PolyFenceType::POLYGON_INCLUSION;
        ret.vertex_count = mission_item_int.param1;
        break;
    case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
        ret.type = AC_PolyFenceType::POLYGON_EXCLUSION;
        ret.vertex_count = mission_item_int.param1;
        break;
    case MAV_CMD_NAV_FENCE_RETURN_POINT:
        ret.type = AC_PolyFenceType::RETURN_POINT;
        break;
    case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
        ret.type = AC_PolyFenceType::CIRCLE_EXCLUSION;
        ret.radius = mission_item_int.param1;
        break;
    case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
        ret.type = AC_PolyFenceType::CIRCLE_INCLUSION;
        ret.radius = mission_item_int.param1;
        break;
    default:
        return MAV_MISSION_UNSUPPORTED;
    }
    ret.loc.x = mission_item_int.x;
    ret.loc.y = mission_item_int.y;
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Fence::replace_item(const mavlink_mission_item_int_t &mission_item_int)
{
    if (_new_items == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return MAV_MISSION_ERROR;
    }
    if (mission_item_int.seq >= _new_items_count) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }

    const MAV_MISSION_RESULT ret = convert_MISSION_ITEM_INT_to_AC_PolyFenceItem(mission_item_int, _new_items[mission_item_int.seq]);
    if (ret != MAV_MISSION_ACCEPTED) {
        return ret;
    }
    if (_updated_mask != nullptr) {
        _updated_mask[mission_item_int.seq/8] |= (1U<<(mission_item_int.seq%8));
    }
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Fence::append_item(const mavlink_mission_item_int_t &mission_item_int)
{
    return replace_item(mission_item_int);
}

void MissionItemProtocol_Fence::free_upload_resources()
{
    free(_new_items);
    _new_items = nullptr;
    delete[] _updated_mask;
    _updated_mask = nullptr;
}

MAV_MISSION_RESULT MissionItemProtocol_Fence::complete(const GCS_MAVLINK &_link)
{
    if (_updated_mask != nullptr) {
        // get any points that weren't filled in
        for (uint16_t i=0; i<_new_items_count; i++) {
            if (!(_updated_mask[i/8] & (1U<<(i%8)))) {
                if (!_fence.polyfence().get_item(i, _new_items[i])) {
                    _link.send_text(MAV_SEVERITY_INFO, "Error replacing item (%u)", i);
                    return MAV_MISSION_ERROR;
                }
            }
        }
    }

    bool success = _fence.polyfence().write_fence(_new_items, _new_items_count);
    if (!success) {
        return MAV_MISSION_ERROR;
    }

    // AP::logger().Write_Fence();
    return MAV_MISSION_ACCEPTED;
}
void MissionItemProtocol_Fence::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Fence upload timeout");
}

uint16_t MissionItemProtocol_Fence::max_items() const
{
    return _fence.polyfence().max_items();
}

void MissionItemProtocol_Fence::truncate(const mavlink_mission_count_t &packet)
{
    // FIXME: validate packet.count is same as allocated number of items
}

bool MissionItemProtocol_Fence::clear_all_items()
{
    return _fence.polyfence().write_fence(nullptr, 0);
}

MAV_MISSION_RESULT MissionItemProtocol_Fence::allocate_receive_resources(const uint16_t count)
{
    if (_new_items != nullptr) {
        // this is an error - the base class should have called
        // free_upload_resources first
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return MAV_MISSION_ERROR;
    }

    const uint16_t allocation_size = count * sizeof(AC_PolyFenceItem);
    if (allocation_size != 0) {
        _new_items = (AC_PolyFenceItem*)malloc(allocation_size);
        if (_new_items == nullptr) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Out of memory for upload");
            return MAV_MISSION_ERROR;
        }
    }
    _new_items_count = count;
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Fence::allocate_update_resources()
{
    const uint16_t _item_count = _fence.polyfence().num_stored_items();
    _updated_mask = new uint8_t[(_item_count+7)/8];
    if (_updated_mask == nullptr) {
        return MAV_MISSION_ERROR;
    }
    MAV_MISSION_RESULT ret = allocate_receive_resources(_item_count);
    if (ret != MAV_MISSION_ACCEPTED) {
        delete[] _updated_mask;
        _updated_mask = nullptr;
        return ret;
    }
    _new_items_count = _item_count;
    return MAV_MISSION_ACCEPTED;
}

#endif // AP_FENCE_ENABLED
