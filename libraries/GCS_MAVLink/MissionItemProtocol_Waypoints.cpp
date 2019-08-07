/*
  Implementation details for transfering waypoint information using
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

#include "MissionItemProtocol_Waypoints.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Mission/AP_Mission.h>

#include "GCS.h"

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::append_item(const mavlink_mission_item_int_t &mission_item_int)
{
    // sanity check for DO_JUMP command
    AP_Mission::Mission_Command cmd;

    const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    if (cmd.id == MAV_CMD_DO_JUMP) {
        if ((cmd.content.jump.target >= item_count() && cmd.content.jump.target > request_last) || cmd.content.jump.target == 0) {
            return MAV_MISSION_ERROR;
        }
    }

    if (!mission.add_cmd(cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

bool MissionItemProtocol_Waypoints::clear_all_items()
{
    return mission.clear();
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::complete(const GCS_MAVLINK &_link)
{
    _link.send_text(MAV_SEVERITY_INFO, "Flight plan received");
    AP::logger().Write_EntireMission();
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::get_item(const GCS_MAVLINK &_link,
                                                           const mavlink_message_t &msg,
                                                           const mavlink_mission_request_int_t &packet,
                                                           mavlink_mission_item_int_t &ret_packet)
{
    if (packet.seq != 0 && // always allow HOME to be read
        packet.seq >= mission.num_commands()) {
        // try to educate the GCS on the actual size of the mission:
        mavlink_msg_mission_count_send(_link.get_chan(),
                                       msg.sysid,
                                       msg.compid,
                                       mission.num_commands(),
                                       MAV_MISSION_TYPE_MISSION);
        return MAV_MISSION_ERROR;
    }

    AP_Mission::Mission_Command cmd;

    // retrieve mission from eeprom
    if (!mission.read_cmd_from_storage(packet.seq, cmd)) {
        return MAV_MISSION_ERROR;
    }

    if (!AP_Mission::mission_cmd_to_mavlink_int(cmd, ret_packet)) {
        return MAV_MISSION_ERROR;
    }

    // set packet's current field to 1 if this is the command being executed
    if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
        ret_packet.current = 1;
    } else {
        ret_packet.current = 0;
    }

    // set auto continue to 1
    ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

    ret_packet.command = cmd.id;

    return MAV_MISSION_ACCEPTED;
}

uint16_t MissionItemProtocol_Waypoints::item_count() const {
    return mission.num_commands();
}

uint16_t MissionItemProtocol_Waypoints::max_items() const {
    return mission.num_commands_max();
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::replace_item(const mavlink_mission_item_int_t &mission_item_int)
{
    AP_Mission::Mission_Command cmd;

    const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    // sanity check for DO_JUMP command
    if (cmd.id == MAV_CMD_DO_JUMP) {
        if ((cmd.content.jump.target >= item_count() && cmd.content.jump.target > request_last) || cmd.content.jump.target == 0) {
            return MAV_MISSION_ERROR;
        }
    }
    if (!mission.replace_cmd(cmd.index, cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_Waypoints::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Mission upload timeout");
}

void MissionItemProtocol_Waypoints::truncate(const mavlink_mission_count_t &packet)
{
    // new mission arriving, truncate mission to be the same length
    mission.truncate(packet.count);
}
