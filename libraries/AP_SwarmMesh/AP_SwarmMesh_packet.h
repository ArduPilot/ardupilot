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
#pragma once

#include "AP_SwarmMesh.h"

#if AP_SWARMMESH_ENABLED

#pragma pack(push, 1)
typedef struct {
    uint8_t  stx1;           // SYNC1 = 0xAD
    uint8_t  stx2;           // SYNC2 = 0xBC
    uint8_t  version;        // Protocol version, currently 1
    uint8_t  type;           // 0 = MAVLink payload, others reserved
    uint8_t  flags;          // Special behaviour (ex. no GPS lock so no synchronization so no freshness detection)
    uint8_t  origin_id;      // Origin node which created packet
    uint8_t  dest_id;        // Destination ID (targeted delivery)
    uint8_t  prev_id;        // Most recent forwarding node
    uint8_t  ttl;            // Time-to-live
    uint16_t seq;            // Packet sequence from origin
    uint64_t origin_time_us; // Timestamp when packet was created (unix)
    uint16_t deadline_ms;    // Relative freshness budget
    uint8_t  payload_len;    // Length of MAVLink frame (Max = 255)
    uint8_t  crc;            // CRC over all bytes above (stx through payload_len)
    // Payload bytes follow in the receive buffer
} p2p_header_t;
#pragma pack(pop)

static_assert(sizeof(p2p_header_t) == 23, "p2p header must be exactly 23 bytes");

#define SWARMMESH_SYNC1          0xAD
#define SWARMMESH_SYNC2          0xBC
#define SWARMMESH_VERSION_01     0x01
#define SWARMMESH_TYPE_MAVLINK   0x00
#define SWARMMESH_NORMAL         0x00
#define SWARMMESH_NO_RTC         0x01
#define SWARMMESH_HEADER_SIZE    (sizeof(p2p_header_t))
#define SWARMMESH_MAX_PAYLOAD    255
#define SWARMMESH_MSG_BUF_MAX    (sizeof(p2p_header_t) + SWARMMESH_MAX_PAYLOAD)
#define SWARMMESH_BROADCAST      0

#endif   // AP_SWARMMESH_ENABLED