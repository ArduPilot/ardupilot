/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_CRSF_Protocol.h"

#if AP_CRSF_ENABLED

// get printable name for frame type (for debug)
const char* AP_CRSF_Protocol::get_frame_type(uint8_t byte, uint8_t subtype)
{
    switch (byte) {
    case CRSF_FRAMETYPE_GPS:
        return "GPS";
    case CRSF_FRAMETYPE_BATTERY_SENSOR:
        return "BATTERY";
    case CRSF_FRAMETYPE_HEARTBEAT:
        return "HEARTBEAT";
    case CRSF_FRAMETYPE_VTX:
        return "VTX";
    case CRSF_FRAMETYPE_VTX_TELEM:
        return "VTX_TELEM";
    case CRSF_FRAMETYPE_PARAM_DEVICE_PING:
        return "PING";
    case CRSF_FRAMETYPE_COMMAND:
        return "COMMAND";
    case CRSF_FRAMETYPE_ATTITUDE:
        return "ATTITUDE";
    case CRSF_FRAMETYPE_FLIGHT_MODE:
        return "FLIGHT_MODE";
    case CRSF_FRAMETYPE_PARAM_DEVICE_INFO:
        return "DEVICE_INFO";
    case CRSF_FRAMETYPE_PARAMETER_READ:
        return "PARAM_READ";
    case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
        return "SETTINGS_ENTRY";
    case CRSF_FRAMETYPE_LINK_STATISTICS:
        return "LINK_STATS";
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        return "RC";
    case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
        return "RCv3";
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED_11BIT:
        return "RCv3_11BIT";
    case CRSF_FRAMETYPE_LINK_STATISTICS_RX:
        return "LINK_STATSv3_RX";
    case CRSF_FRAMETYPE_LINK_STATISTICS_TX:
        return "LINK_STATSv3_TX";
    case CRSF_FRAMETYPE_PARAMETER_WRITE:
        return "PARAM_WRITE";
    case CRSF_FRAMETYPE_AP_CUSTOM_TELEM_LEGACY:
    case CRSF_FRAMETYPE_AP_CUSTOM_TELEM:
        switch (subtype) {
        case CRSF_AP_CUSTOM_TELEM_SINGLE_PACKET_PASSTHROUGH:
            return "AP_CUSTOM_SINGLE";
        case CRSF_AP_CUSTOM_TELEM_STATUS_TEXT:
            return "AP_CUSTOM_TEXT";
        case CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH:
            return "AP_CUSTOM_MULTI";
        }
        return "AP_CUSTOM";
    }
    return "UNKNOWN";
}

#endif  // AP_CRSF_ENABLED
