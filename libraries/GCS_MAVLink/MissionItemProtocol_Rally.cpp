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

#include "GCS_config.h"
#include <AP_Rally/AP_Rally_config.h>

#if HAL_GCS_ENABLED && HAL_RALLY_ENABLED

#include "MissionItemProtocol_Rally.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Rally/AP_Rally.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Terrain/AP_Terrain.h>

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
#if HAL_LOGGING_ENABLED
    AP::logger().Write_Rally();
#endif
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
    if (!check_lat(cmd.x)) {
        return MAV_MISSION_INVALID_PARAM5_X;
    }
    if (!check_lng(cmd.y)) {
        return MAV_MISSION_INVALID_PARAM6_Y;
    }
    if (cmd.z < INT16_MIN || cmd.z > INT16_MAX) {
        return MAV_MISSION_INVALID_PARAM7;
    }
    ret = {};

    switch (cmd.frame) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_INT:
            ret.alt_frame = uint8_t(Location::AltFrame::ABSOLUTE);
            break;

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            ret.alt_frame = uint8_t(Location::AltFrame::ABOVE_HOME);
            break;

#if AP_TERRAIN_AVAILABLE
        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
            ret.alt_frame = uint8_t(Location::AltFrame::ABOVE_TERRAIN);
            break;
#endif

        default:
            return MAV_MISSION_UNSUPPORTED_FRAME;
    }

    // Fresh points always use new alt frame format
    ret.alt_frame_valid = true;

    ret.lat = cmd.x;
    ret.lng = cmd.y;
    ret.alt = cmd.z;
    return MAV_MISSION_ACCEPTED;
}

/*
  static function to get rally item as mavlink_mission_item_int_t
 */
bool MissionItemProtocol_Rally::get_item_as_mission_item(uint16_t seq,
                                                         mavlink_mission_item_int_t &ret_packet)
{
    auto *rallyp = AP::rally();

    RallyLocation rallypoint;

    if (rallyp == nullptr || !rallyp->get_rally_point_with_index(seq, rallypoint)) {
        return false;
    }

    // Default to relative to home
    ret_packet = {
        x: rallypoint.lat,
        y: rallypoint.lng,
        z: float(rallypoint.alt),
        seq: seq,
        command: MAV_CMD_NAV_RALLY_POINT,
        frame: MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mission_type: MAV_MISSION_TYPE_RALLY,
    };

    if (rallypoint.alt_frame_valid == 1) {
        switch (Location::AltFrame(rallypoint.alt_frame)) {
            case Location::AltFrame::ABSOLUTE:
                ret_packet.frame = MAV_FRAME_GLOBAL;
                break;

            case Location::AltFrame::ABOVE_HOME:
                ret_packet.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
                break;

            case Location::AltFrame::ABOVE_ORIGIN:
                // Above origin alt frame is not supported
                return false;

            case Location::AltFrame::ABOVE_TERRAIN:
                ret_packet.frame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
                break;
        }
    }

    return true;
}

MAV_MISSION_RESULT MissionItemProtocol_Rally::get_item(uint16_t seq, mavlink_mission_item_int_t &ret_packet)
{
    if (seq >= item_count()) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }

    if (!get_item_as_mission_item(seq, ret_packet)) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }
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

#endif  // HAL_GCS_ENABLED && HAL_RALLY_ENABLED
