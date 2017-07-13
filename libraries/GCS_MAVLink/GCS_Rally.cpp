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

void GCS_MAVLINK::handle_rally_point(mavlink_message_t *msg)
{
    AP_Rally *r = get_rally();
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
    AP_Rally *r = get_rally();
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

    mavlink_msg_rally_point_send_buf(msg,
                                     chan, msg->sysid, msg->compid, packet.idx,
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
        break;
    }
}

