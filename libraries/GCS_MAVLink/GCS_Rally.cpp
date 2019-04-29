/*
   GCS MAVLink functions related to rally points

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

#include "GCS.h"
#include <AP_Rally/AP_Rally.h>
#include <AP_Logger/AP_Logger.h>

void GCS_MAVLINK::handle_rally_point(mavlink_message_t *msg)
{
    AP_Rally *r = AP::rally();
    if (r == nullptr) {
        return;
    }

    mavlink_rally_point_t packet;
    mavlink_msg_rally_point_decode(msg, &packet);

    if (packet.idx >= r->get_rally_total() ||
        packet.idx >= r->get_rally_max()) {
        send_text(MAV_SEVERITY_WARNING,"Bad rally point ID");
        return;
    }

    if (packet.count != r->get_rally_total()) {
        send_text(MAV_SEVERITY_WARNING,"Bad rally point count");
        return;
    }

    // sanity check location
    if (!check_latlng(packet.lat, packet.lng)) {
        return;
    }

    RallyLocation rally_point;
    rally_point.lat = packet.lat;
    rally_point.lng = packet.lng;
    rally_point.alt = packet.alt;
    rally_point.break_alt = packet.break_alt;
    rally_point.land_dir = packet.land_dir;
    rally_point.flags = packet.flags;

    if (!r->set_rally_point_with_index(packet.idx, rally_point)) {
        send_text(MAV_SEVERITY_CRITICAL, "Error setting rally point");
    }
}

void GCS_MAVLINK::handle_rally_fetch_point(mavlink_message_t *msg)
{
    AP_Rally *r = AP::rally();
    if (r == nullptr) {
        return;
    }

    mavlink_rally_fetch_point_t packet;
    mavlink_msg_rally_fetch_point_decode(msg, &packet);

    if (packet.idx > r->get_rally_total()) {
        send_text(MAV_SEVERITY_WARNING, "Bad rally point ID");
        return;
    }

    RallyLocation rally_point;
    if (!r->get_rally_point_with_index(packet.idx, rally_point)) {
        send_text(MAV_SEVERITY_WARNING, "Failed to get rally point");
        return;
    }

    mavlink_msg_rally_point_send(chan, msg->sysid, msg->compid, packet.idx,
                                 r->get_rally_total(), rally_point.lat, rally_point.lng,
                                 rally_point.alt, rally_point.break_alt, rally_point.land_dir,
                                 rally_point.flags);
}

void GCS_MAVLINK::handle_common_rally_message(mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_RALLY_POINT:
        handle_rally_point(msg);
        break;
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT:
        handle_rally_fetch_point(msg);
        break;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled common rally message");
#endif
        break;
    }
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

    ret_packet.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    ret_packet.command = MAV_CMD_NAV_RALLY_POINT;
    ret_packet.x = rallypoint.lat;
    ret_packet.y = rallypoint.lng;
    ret_packet.z = rallypoint.alt;

    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_Rally::truncate(const mavlink_mission_count_t &packet)
{
    rally.truncate(packet.count);
}
void MissionItemProtocol_Rally::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Rally upload timeout");
}

MAV_MISSION_RESULT MissionItemProtocol_Rally::convert_MISSION_ITEM_INT_to_RallyLocation(const mavlink_mission_item_int_t &cmd, RallyLocation &ret)
{
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

void MissionItemProtocol_Rally::complete(const GCS_MAVLINK &_link)
{
    _link.send_text(MAV_SEVERITY_INFO, "Rally points received");
    AP::logger().Write_Rally();
}
