/*
  Implementation details for transfering rally point information using
  the MISSION_ITEM protocol to and from a GCS.

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

#include "MissionItemProtocol_PathPlanning.h"

#include <AC_Avoidance/AP_OAPathPlanner.h>

#include <GCS_MAVLink/GCS.h>

ap_message MissionItemProtocol_PathPlanning::next_item_ap_message_id() const {
    // this should never get called as this link is read only
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return MSG_LAST;
}

MAV_MISSION_RESULT MissionItemProtocol_PathPlanning::get_item(const GCS_MAVLINK &_link,
                                                       const mavlink_message_t &msg,
                                                       const mavlink_mission_request_int_t &packet,
                                                       mavlink_mission_item_int_t &ret_packet)
{
    if (!get_item_as_mission_item(packet.seq, ret_packet)) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }
    return MAV_MISSION_ACCEPTED;
}

// function to get rally item as mavlink_mission_item_int_t
bool MissionItemProtocol_PathPlanning::get_item_as_mission_item(uint16_t seq,
                                                         mavlink_mission_item_int_t &ret_packet)
{
    const AP_OAPathPlanner * oa = AP::ap_oapathplanner()->get_singleton();
    if (oa == nullptr || !oa->active()) {
        return false;
    }

    Location loc;
    if (!oa->get_path_point(seq, loc)) {
        return false;
    }

    // ToDo: deal with alt frame correctly, path planning does not do alt so it does not make a difference... yet
    ret_packet.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; 

    ret_packet.command = MAV_CMD_NAV_WAYPOINT;
    ret_packet.x = loc.lat;
    ret_packet.y = loc.lng;
    ret_packet.z = loc.alt;

    return true;
}

uint16_t MissionItemProtocol_PathPlanning::item_count() const {
    const AP_OAPathPlanner * oa = AP::ap_oapathplanner()->get_singleton();
    if (oa == nullptr || !oa->active()) {
        return 0;
    }
    return oa->get_path_count();
}

uint16_t MissionItemProtocol_PathPlanning::max_items() const {
    // this should never get called as this link is read only
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return 0;
}

MAV_MISSION_RESULT MissionItemProtocol_PathPlanning::replace_item(const mavlink_mission_item_int_t &cmd)
{
    // this should never get called as this link is read only
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return MAV_MISSION_UNSUPPORTED;
}

MAV_MISSION_RESULT MissionItemProtocol_PathPlanning::append_item(const mavlink_mission_item_int_t &cmd)
{
    // this should never get called as this link is read only
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return MAV_MISSION_UNSUPPORTED;
}

MAV_MISSION_RESULT MissionItemProtocol_PathPlanning::complete(const GCS_MAVLINK &_link)
{
    // this should never get called as this link is read only
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return MAV_MISSION_UNSUPPORTED;
}
