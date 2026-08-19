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
/*
  Wire format of the coordination basket.

  Coordination state is carried as an ordinary MAVLink TUNNEL message, so it travels the
  same send/receive path as every other mesh message, is relayed like any other message,
  and reaches a companion computer unmodified through the _FWD_PORT forwarding that peer
  telemetry uses. A companion computer publishes its own state by sending the same TUNNEL
  message back over its normal telemetry link.
 */
#pragma once

#include "AP_SwarmMesh_config.h"

#if AP_SWARMMESH_COORD_ENABLED

#include <stdint.h>

// Identifies a TUNNEL as carrying a SwarmMesh coordination basket (MAV_TUNNEL_PAYLOAD_TYPE registered as local experiment).
// TODO: register a block in MAV_TUNNEL_PAYLOAD_TYPE and move off the experimental range.
#define SWARMMESH_COORD_PAYLOAD_TYPE  32768U

#define SWARMMESH_COORD_VERSION       1

#pragma pack(push, 1)
typedef struct {
    uint8_t  version;               // SWARMMESH_COORD_VERSION
    uint8_t  role;                  // script defined role
    uint8_t  task_id;               // script defined task
    uint8_t  formation_slot;        // script defined slot within a formation
    uint8_t  priority;              // script defined precedence, for tie breaking between peers
    uint8_t  user_len;              // number of valid bytes in user[]
    int32_t  target_pos[3];         // intended destination [lat (degE7), lon (degE7), alt (mm)]
    int16_t  target_velocity[3];    // intended velocity [x, y, z] NED, cm/s
    int16_t  target_accel[3];       // intended acceleration [x, y, z] NED, cm/s/s
    uint8_t  user[AP_SWARMMESH_COORD_USER_MAX];   // opaque to the library, meaning is the script's
} swarmmesh_coord_t;
#pragma pack(pop)

// bytes of a basket ahead of user[], so a short basket can still be validated
#define SWARMMESH_COORD_FIXED_LEN  (sizeof(swarmmesh_coord_t) - AP_SWARMMESH_COORD_USER_MAX)

static_assert(SWARMMESH_COORD_FIXED_LEN == 30, "coordination basket header must be exactly 30 bytes");
static_assert(sizeof(swarmmesh_coord_t) <= 128, "coordination basket must fit in a TUNNEL payload");

#endif  // AP_SWARMMESH_COORD_ENABLED
