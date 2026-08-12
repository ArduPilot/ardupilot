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
#include <AP_Filesystem/AP_Filesystem_config.h>

// requires SD card / filesystem write support (FATFS, LittleFS, ESP32, or POSIX on SITL/Linux)
#if AP_SWARMMESH_ENABLED && AP_FILESYSTEM_FILE_WRITING_ENABLED

// location of the peer table snapshot
#define AP_SWARMMESH_PEER_DIR  "/APM/PEERS"
#define AP_SWARMMESH_PEER_FILE "/APM/PEERS/peers.dat"

#define AP_SWARMMESH_PEER_FILE_MAGIC   0x53574D50UL  // 'SWMP'
#define AP_SWARMMESH_PEER_FILE_VERSION 2  // v2: global_pos/target_pos lat/lon/alt stored as int32_t, not float

#pragma pack(push, 1)

// fixed-size file header, followed immediately by snapshot_count records
typedef struct {
    uint32_t magic;           // AP_SWARMMESH_PEER_FILE_MAGIC
    uint16_t version;         // AP_SWARMMESH_PEER_FILE_VERSION
    uint16_t snapshot_size;   // sizeof(AP_SwarmMesh_PeerSnapshot_t) at write time
    uint16_t snapshot_count;  // number of snapshots following the header
    uint64_t saved_time_us;   // AP_HAL::micros64()
} AP_SwarmMesh_PeerFileHeader_t;

// one persisted peer state
typedef struct {
    uint8_t  sysid;
    uint8_t  vehicle_type;
    uint8_t  mode;
    uint8_t  armed_state;     // stored as 0/1
    uint8_t  landed_state;    // MAV_LANDED_STATE
    uint32_t failsafe_flags;
    uint16_t battery_voltage;
    float    local_pos_NED[3];
    int32_t  global_pos[3];   // [lat (degE7), lon (degE7), alt (mm)]
    float    attitude[3];
    uint8_t  role;
    uint8_t  task_id;
    uint8_t  formation_slot;
    int32_t  target_pos[3];   // [lat (degE7), lon (degE7), alt (mm)]
    uint8_t  priority;
} AP_SwarmMesh_PeerSnapshot_t;

#pragma pack(pop)

static_assert(sizeof(AP_SwarmMesh_PeerFileHeader_t) == 18, "peer file header must be exactly 18 bytes");
static_assert(sizeof(AP_SwarmMesh_PeerSnapshot_t) == 63, "peer snapshot must be exactly 63 bytes");

#endif  // AP_SWARMMESH_ENABLED && AP_FILESYSTEM_FILE_WRITING_ENABLED
