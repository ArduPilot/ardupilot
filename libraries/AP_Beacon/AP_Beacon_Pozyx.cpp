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

#include "AP_Beacon_Pozyx.h"

#if AP_BEACON_POZYX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <stdio.h>

#include "../GCS_MAVLink/GCS.h"

extern const AP_HAL::HAL& hal;

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_Pozyx::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_Pozyx::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // read any available characters
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();

        switch (parse_state) {

            default:
            case ParseState_WaitingForHeader:
                if (c == AP_BEACON_POZYX_HEADER) {
                    parse_state = ParseState_WaitingForMsgId;
                    linebuf_len = 0;
                }
                break;

            case ParseState_WaitingForMsgId:
                parse_msg_id = c;
                switch (parse_msg_id) {
                case AP_BEACON_POZYX_MSGID_BEACON_CONFIG:
                case AP_BEACON_POZYX_MSGID_BEACON_DIST:
                case AP_BEACON_POZYX_MSGID_POSITION:
                    parse_state = ParseState_WaitingForLen;
                    break;
                default:
                    // invalid message id
                    parse_state = ParseState_WaitingForHeader;
                    break;
                }
                break;

            case ParseState_WaitingForLen:
                parse_msg_len = c;
                if (parse_msg_len > AP_BEACON_POZYX_MSG_LEN_MAX) {
                    // invalid message length
                    parse_state = ParseState_WaitingForHeader;
                } else {
                    parse_state = ParseState_WaitingForContents;
                }
                break;

            case ParseState_WaitingForContents:
                // add to buffer
                linebuf[linebuf_len++] = c;
                if ((linebuf_len == parse_msg_len) || (linebuf_len == sizeof(linebuf))) {
                    // process buffer
                    parse_buffer();
                    // reset state for next message
                    parse_state = ParseState_WaitingForHeader;
                }
                break;
        }
    }
}

// structure for messages uploaded to ardupilot
union beacon_config_msg {
    struct __attribute__((__packed__)) {
        uint8_t beacon_id;
        uint8_t beacon_count;
        int32_t x;
        int32_t y;
        int32_t z;
    } info;
    uint8_t buf[14];
};

union beacon_distance_msg {
    struct __attribute__((__packed__)) {
        uint8_t beacon_id;
        uint32_t distance;
    } info;
    uint8_t buf[5];
};

union vehicle_position_msg {
    struct __attribute__((__packed__)) {
        int32_t x;
        int32_t y;
        int32_t z;
        int16_t position_error;
    } info;
    uint8_t buf[14];
};

// parse buffer
void AP_Beacon_Pozyx::parse_buffer()
{
    // check crc
    uint8_t checksum = 0;
    checksum ^= parse_msg_id;
    checksum ^= parse_msg_len;
    for (uint8_t i=0; i<linebuf_len; i++) {
        checksum ^= linebuf[i];
    }
    // return if failed checksum check
    if (checksum != 0) {
        return;
    }

    bool parsed = false;
    switch (parse_msg_id) {

        case AP_BEACON_POZYX_MSGID_BEACON_CONFIG:
            {
                beacon_config_msg msg;
                memcpy(msg.buf, linebuf, sizeof(msg.buf));
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BC %d %ld %ld %ld", msg.info.beacon_id, msg.info.x, msg.info.y, msg.info.z);

                Vector3f beacon_pos(msg.info.x * 0.001f, msg.info.y * 0.001f, -msg.info.z * 0.001f);

                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "P1 id=%d;len=%f;x=%ld;y=%ld;z=%ld", beacon_id, beacon_pos.length(), beacon_x, beacon_y, beacon_z);

                if (beacon_pos.length() <= AP_BEACON_DISTANCE_MAX) {
                    set_beacon_position(msg.info.beacon_id, beacon_pos);
                    parsed = true;
                }
            }
            break;

        case AP_BEACON_POZYX_MSGID_BEACON_DIST:
            {
                beacon_distance_msg msg;
                memcpy(msg.buf, linebuf, sizeof(msg.buf));
                
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BD %d %ld", msg.info.beacon_id, msg.info.distance);
                
                float beacon_dist = msg.info.distance * 0.001f;

                if (beacon_dist <= AP_BEACON_DISTANCE_MAX) {
                    set_beacon_distance(msg.info.beacon_id, beacon_dist);
                    parsed = true;
                }
            }
            break;

        case AP_BEACON_POZYX_MSGID_POSITION:
            {
                vehicle_position_msg msg;
                memcpy(msg.buf, linebuf, sizeof(msg.buf));

                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BP %ld %ld %ld %d", msg.info.x, msg.info.y, msg.info.z, msg.info.position_error);
                
                Vector3f veh_pos(Vector3f(msg.info.x * 0.001f, msg.info.y * 0.001f, -msg.info.z * 0.001f));
                
                if (veh_pos.length() <= AP_BEACON_DISTANCE_MAX) {
                    set_vehicle_position(veh_pos, msg.info.position_error * 0.001f);
                    parsed = true;
                }
            }
            break;

        default:
            // unrecognised message id
            break;
    }

    // record success
    if (parsed) {
        last_update_ms = AP_HAL::millis();
    }
}

#endif  // AP_BEACON_POZYX_ENABLED
