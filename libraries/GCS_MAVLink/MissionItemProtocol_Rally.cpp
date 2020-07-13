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

#include "MissionItemProtocol_Rally.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Rally/AP_Rally.h>
#include <GCS_MAVLink/GCS.h>


MAV_MISSION_RESULT MissionItemProtocol_Rally::append_item(const mavlink_mission_item_int_t &cmd)
{
    RallyLocation rallyloc;
    const MAV_MISSION_RESULT ret = convert_MISSION_ITEM_INT_to_RallyLocation(cmd, rallyloc);
    if (ret != MAV_MISSION_ACCEPTED) {
        return ret;
    }
    if (!rally.append(rallyloc)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Rally::complete(const GCS_MAVLINK &_link)
{
    AP::logger().Write_Rally();
    return MAV_MISSION_ACCEPTED;
}

bool MissionItemProtocol_Rally::clear_all_items()
{
    rally.truncate(0);
    return true;
}

MAV_MISSION_RESULT MissionItemProtocol_Rally::convert_MISSION_ITEM_INT_to_RallyLocation(const mavlink_mission_item_int_t &cmd, RallyLocation &ret)
{
    if (cmd.command != MAV_CMD_NAV_RALLY_POINT) {
        return MAV_MISSION_UNSUPPORTED;
    }
    if (cmd.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
        cmd.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT) {
        return MAV_MISSION_UNSUPPORTED_FRAME;
    }
    if (!check_lat(cmd.x)) {
        return MAV_MISSION_INVALID_PARAM5_X;
    }
    if (!check_lng(cmd.y)) {
        return MAV_MISSION_INVALID_PARAM6_Y;
    }
    if (cmd.z < INT16_MIN || cmd.z > INT16_MAX) {
        return MAV_MISSION_INVALID_PARAM7;
    }
    ret.lat = cmd.x;
    ret.lng = cmd.y;
    ret.alt = cmd.z;
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Rally::get_item(const GCS_MAVLINK &_link,
                                                       const mavlink_message_t &msg,
                                                       const mavlink_mission_request_int_t &packet,
                                                       mavlink_mission_item_int_t &ret_packet)
{
    RallyLocation rallypoint;

    if (!rally.get_rally_point_with_index(packet.seq, rallypoint)) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }

    ret_packet.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    ret_packet.command = MAV_CMD_NAV_RALLY_POINT;
    ret_packet.x = rallypoint.lat;
    ret_packet.y = rallypoint.lng;
    ret_packet.z = rallypoint.alt;

    return MAV_MISSION_ACCEPTED;
}

uint16_t MissionItemProtocol_Rally::item_count() const {
    return rally.get_rally_total();
}

uint16_t MissionItemProtocol_Rally::max_items() const {
    return rally.get_rally_max();
}

MAV_MISSION_RESULT MissionItemProtocol_Rally::replace_item(const mavlink_mission_item_int_t &cmd)
{
    RallyLocation rallyloc;
    const MAV_MISSION_RESULT ret = convert_MISSION_ITEM_INT_to_RallyLocation(cmd, rallyloc);
    if (ret != MAV_MISSION_ACCEPTED) {
        return ret;
    }
    if (!rally.set_rally_point_with_index(cmd.seq, rallyloc)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_Rally::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Rally upload timeout");
}

void MissionItemProtocol_Rally::truncate(const mavlink_mission_count_t &packet)
{
    rally.truncate(packet.count);
}
