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

#include "AP_SwarmMesh.h"

#if AP_SWARMMESH_ENABLED

#include "AP_SwarmMesh_Backend.h"

#if AP_SWARMMESH_SERIAL_ENABLED
#include "AP_SwarmMesh_Serial.h"
#endif

#if AP_SWARMMESH_SITL_ENABLED
#include "AP_SwarmMesh_SITL.h"
#endif

#include <AP_Logger/AP_Logger.h>

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
#include "AP_SwarmMesh_PeerStorage.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#endif

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_SwarmMesh::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Communication backend
    // @Description: Which communication backend are you using
    // @Values: 0:None,1:Serial,10:SITL
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE", 0, AP_SwarmMesh, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _SR_POSITION
    // @DisplayName: Position stream rate
    // @Description: Rate at which GLOBAL_POSITION_INT and LOCAL_POSITION_NED are broadcast (Hz). 0 disables.
    // @Units: Hz
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("_SR_POSITION", 1, AP_SwarmMesh, stream_rate[0], 0),

    // @Param: _SR_EXT_STAT
    // @DisplayName: Extended status stream rate
    // @Description: Rate at which SYS_STATUS, NAV_CONTROLLER_OUTPUT, POSITION_TARGET_GLOBAL_INT and MISSION_CURRENT are broadcast (Hz). 0 disables.
    // @Units: Hz
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("_SR_EXT_STAT", 2, AP_SwarmMesh, stream_rate[1], 0),

    // @Param: _SR_EXTRA1
    // @DisplayName: Extra 1 stream rate
    // @Description: Rate at which ATTITUDE and EKF_STATUS_REPORT are broadcast (Hz). 0 disables.
    // @Units: Hz
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("_SR_EXTRA1", 3, AP_SwarmMesh, stream_rate[2], 0),

    // @Param: _SWARM_SIZE
    // @DisplayName: Swarm size
    // @Description: Size of swarm (peers + GCS). Constrained to AP_SWARMMESH_MAX_PEERS (board-dependent, up to 255) regardless of the value set here. 0 uses the board's compile time max.
    // @Increment: 1
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_SWARM_SIZE", 4, AP_SwarmMesh, swarm_size, 0),

    // @Param: _DESTID
    // @DisplayName: Destination ID
    // @Description: SysID of intended destination for transmitted messages
    // @Increment: 1
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("_DESTID", 5, AP_SwarmMesh, destination_id, 0),

    // @Param: _SYSID
    // @DisplayName: System ID
    // @Description: Unique system ID of this drone
    // @Increment: 1
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_SYSID", 6, AP_SwarmMesh, sysid, 0),

    // @Param: _TTL
    // @DisplayName: Time-to-Live
    // @Description: Number of hops a forwarded packet can take before being discarded
    // @Increment: 1
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_TTL", 7, AP_SwarmMesh, ttl, 255),

    // @Param: _HW_MASK
    // @DisplayName: Radio hardware capability
    // @Description: Bitmask describing the radio hardware attached. Bit 0: Full capacity radio. If clear, Lite radio assumed. The FC CPU class may further restrict to Lite regardless of radio.
    // @Bitmask: 0:Full radio
    // @User: Advanced
    AP_GROUPINFO("_HW_MASK", 8, AP_SwarmMesh, hardware_mask, 0),

    // @Param: _LOG_HZ
    // @DisplayName: RX log write rate
    // @Description: Maximum combined rate at which RX peer telemetry is written to the dataflash log, across all peers and message types. 0 disables RX logging entirely.
    // @Units: Hz
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("_LOG_HZ", 9, AP_SwarmMesh, log_rate_hz, 50),

    // @Param: _LOG_MASK
    // @DisplayName: RX log message mask
    // @Description: Bitmask of which RX message types are written to the dataflash log (still subject to LOG_HZ). Bits 9-31 are reserved for future message types.
    // @Bitmask: 0:Heartbeat,1:SysStatus,2:GlobalPositionInt,3:LocalPositionNED,4:PositionTargetGlobalInt,5:ExtendedSysState,6:Attitude,7:EkfStatusReport,8:ScaledIMU
    // @User: Advanced
    AP_GROUPINFO("_LOG_MASK", 10, AP_SwarmMesh, log_mask, 0x1FF),

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    // @Param: _SAVE_HZ
    // @DisplayName: Peer snapshot save rate
    // @Description: Rate at which the current peer table (filled and fresh entries only) is rewritten to APM/PEERS/peers.dat on the SD card, so it can be reloaded after a reset. 0 disables.
    // @Units: Hz
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("_SAVE_HZ", 11, AP_SwarmMesh, save_rate_hz, 1),
#endif

    // @Param: _PRUNE_SECS
    // @DisplayName: Peer table prune interval
    // @Description: How often (seconds) the peer table is checked for stale (non-fresh) entries and deleted to free space for new peers. 0 disables pruning.
    // @Units: s
    // @Range: 0 60
    // @User: Advanced
    AP_GROUPINFO("_PRUNE_SECS", 12, AP_SwarmMesh, prune_timeout, 10),

    // @Param: _PEER_01
    // @DisplayName: Neighbourhood peer 1
    // @Description: SysID of a peer to track. When any _PEER_* slot is non-zero only peers matching a listed SysID are tracked. 0 disables this slot. All slots zero (default) accepts any peer.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_PEER_01", 13, AP_SwarmMesh, peer_filter[0],  0),
    AP_GROUPINFO("_PEER_02", 14, AP_SwarmMesh, peer_filter[1],  0),
    AP_GROUPINFO("_PEER_03", 15, AP_SwarmMesh, peer_filter[2],  0),
    AP_GROUPINFO("_PEER_04", 16, AP_SwarmMesh, peer_filter[3],  0),
    AP_GROUPINFO("_PEER_05", 17, AP_SwarmMesh, peer_filter[4],  0),
    AP_GROUPINFO("_PEER_06", 18, AP_SwarmMesh, peer_filter[5],  0),
    AP_GROUPINFO("_PEER_07", 19, AP_SwarmMesh, peer_filter[6],  0),
    AP_GROUPINFO("_PEER_08", 20, AP_SwarmMesh, peer_filter[7],  0),
    AP_GROUPINFO("_PEER_09", 21, AP_SwarmMesh, peer_filter[8],  0),
    AP_GROUPINFO("_PEER_10", 22, AP_SwarmMesh, peer_filter[9],  0),
    AP_GROUPINFO("_PEER_11", 23, AP_SwarmMesh, peer_filter[10], 0),
    AP_GROUPINFO("_PEER_12", 24, AP_SwarmMesh, peer_filter[11], 0),
    AP_GROUPINFO("_PEER_13", 25, AP_SwarmMesh, peer_filter[12], 0),
    AP_GROUPINFO("_PEER_14", 26, AP_SwarmMesh, peer_filter[13], 0),
    AP_GROUPINFO("_PEER_15", 27, AP_SwarmMesh, peer_filter[14], 0),
    AP_GROUPINFO("_PEER_16", 28, AP_SwarmMesh, peer_filter[15], 0),

    AP_GROUPEND
};

AP_SwarmMesh::AP_SwarmMesh()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_SwarmMesh must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the AP_SwarmMesh class
void AP_SwarmMesh::init(void)
{
    if (_driver != nullptr) {
        // init called a 2nd time?
        return;
    }

    // create backend
    switch ((Type)_type) {
    case Type::Serial:
#if AP_SWARMMESH_SERIAL_ENABLED
        _driver = NEW_NOTHROW AP_SwarmMesh_Serial(*this);
#endif
        break;
#if AP_SWARMMESH_SITL_ENABLED
    case Type::SITL:
        _driver = NEW_NOTHROW AP_SwarmMesh_SITL(*this);
        break;
#endif
    case Type::None:
        break;
    }

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    // restore the peer table from disk before any radio traffic arrives
    load_peer_snapshot();
#endif
}

// return true if swarm feature is enabled
bool AP_SwarmMesh::enabled(void) const
{
    return (_type != Type::None);
}

// return true if radio is basically healthy (we are receiving data)
bool AP_SwarmMesh::healthy(void) const
{
    if (!device_ready()) {
        return false;
    }
    return _driver->healthy();
}

// update state. This should be called often from the main loop
void AP_SwarmMesh::update(void)
{
    if (!device_ready()) {
        return;
    }
    _driver->update();

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    const uint8_t rate_hz = MAX(0, (int8_t)save_rate_hz);
    if (rate_hz != 0) {
        const uint32_t now_ms = AP_HAL::millis();
        const uint32_t interval_ms = 1000U / ((rate_hz <= 10) ? rate_hz : 10);  // Max snapshot rate 10Hz
        if (now_ms - _last_save_ms >= interval_ms) {
            _last_save_ms = now_ms;
            save_peer_snapshot();
        }
    }
#endif

    // Delete expired peer table entries
    const uint8_t prune_s = MAX(0, (int8_t)prune_timeout);
    if (prune_s != 0) {
        const uint32_t now_ms = AP_HAL::millis();
        const uint32_t interval_ms = 1000U * ((prune_s <= 60) ? prune_s : 60);  // Max timeout 60s
        if (now_ms - _last_check_ms >= interval_ms) {
            _last_check_ms = now_ms;
            prune_peer_table();
        }
    }
}

// return the number of peers
uint8_t AP_SwarmMesh::count() const
{
    if (!device_ready()) {
        return 0;
    }
    return num_peers;
}

// return all peer data
bool AP_SwarmMesh::get_peer_data(uint8_t peer_id, struct PeerState& state) const
{
    if (!device_ready() || peer_id >= num_peers) {
        return false;
    }
    state = peer_state[peer_id];
    return true;
}

// find an existing peer entry by sysid without allocating. returns nullptr if absent.
const AP_SwarmMesh::PeerState *AP_SwarmMesh::find_peer_by_sysid(uint8_t peer_sysid) const
{
    if (!device_ready()) {
        return nullptr;
    }
    for (uint8_t i = 0; i < num_peers; i++) {
        if (peer_state[i].sysid == peer_sysid) {
            return &peer_state[i];
        }
    }
    return nullptr;
}

// fill loc with peer's last global pos. Returns false if the peer is unknown or its POSITION value is stale
bool AP_SwarmMesh::get_peer_location(Location& loc, uint8_t peer_sysid) const
{
    const PeerState *ps = find_peer_by_sysid(peer_sysid);
    const uint32_t pos_bit = 1U << (uint8_t)MsgFresh::GLOBAL_POSITION_INT;
    if (ps == nullptr || !(ps->freshness & pos_bit)) {
        return false;
    }
    // global_pos: [lat degE7, lon degE7, alt mm]; Location alt is in cm.
    loc = Location(ps->global_pos.x, ps->global_pos.y, ps->global_pos.z / 10, Location::AltFrame::ABSOLUTE);
    return true;
}

// fill vel_ned (m/s, NED) with peer's last vel. Velocity is carried by either GLOBAL_POSITION_INT or LOCAL_POSITION_NED, so it is fresh if either bit is set.
bool AP_SwarmMesh::get_peer_velocity_NED(Vector3f& vel_ned, uint8_t peer_sysid) const
{
    const PeerState *ps = find_peer_by_sysid(peer_sysid);
    const uint32_t vel_bits = (1U << (uint8_t)MsgFresh::GLOBAL_POSITION_INT) | (1U << (uint8_t)MsgFresh::LOCAL_POSITION_NED);
    if (ps == nullptr || !(ps->freshness & vel_bits)) {
        return false;
    }
    // velocity stored as cm/s NED; convert to m/s.
    vel_ned = Vector3f(ps->velocity[0], ps->velocity[1], ps->velocity[2]) * 0.01f;
    return true;
}

// check if the device is ready
bool AP_SwarmMesh::device_ready(void) const
{
    return ((_driver != nullptr) && (_type != Type::None));
}

// returns true if sysid is permitted by the neighbourhood filter. when all slots are 0 (default), every sysid is allowed.
bool AP_SwarmMesh::peer_is_allowed(uint8_t peer_sysid) const
{
    bool any_set = false;
    for (uint8_t i = 0; i < AP_SWARMMESH_MAX_PEER_FILTERS; i++) {
        const uint8_t f = (uint8_t)peer_filter[i];
        if (f == 0) {
            continue;
        }
        any_set = true;
        if (f == peer_sysid) {
            return true;
        }
    }
    return !any_set;
}

// find an existing peer entry by sysid, or allocate a new zeroed entry.
// returns nullptr if the table is full, the peer is not already present,
// or the peer is excluded by the neighbourhood filter.
AP_SwarmMesh::PeerState *AP_SwarmMesh::find_or_alloc_peer(uint8_t peer_sysid)
{
    // respect swarm_size if set, otherwise fall back to compile time max.
    const uint8_t limit = (swarm_size > 0) ? (uint8_t)MIN((int16_t)swarm_size, (int16_t)AP_SWARMMESH_MAX_PEERS) : AP_SWARMMESH_MAX_PEERS;

    for (uint8_t i = 0; i < num_peers; i++) {
        if (peer_state[i].sysid == peer_sysid) {
            return &peer_state[i];
        }
    }
    if (!peer_is_allowed(peer_sysid)) {
        return nullptr;
    }
    if (num_peers >= limit) {
        return nullptr;
    }
    PeerState &ps = peer_state[num_peers++];
    memset(&ps, 0, sizeof(ps));
    ps.sysid = peer_sysid;
    return &ps;
}

// periodically rewrite the on-disk peer-table snapshot (filled and fresh entries only).
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
void AP_SwarmMesh::save_peer_snapshot()
{
    if (!_save_dir_checked) {
        _save_dir_checked = true;
        EXPECT_DELAY_MS(3000);
        struct stat st;
        int ret = AP::FS().stat(AP_SWARMMESH_PEER_DIR, &st);
        if (ret == -1) {
            ret = AP::FS().mkdir(AP_SWARMMESH_PEER_DIR);
        }
        _save_dir_ok = (ret == 0) || (ret == -1 && errno == EEXIST);
        if (!_save_dir_ok) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SwarmMesh: failed to create %s", AP_SWARMMESH_PEER_DIR);
        }
    }
    if (!_save_dir_ok) {
        return;
    }

    // count eligible (filled + alive) peers first, since the header needs the count up front and we don't want to buffer all peers on the stack.
    // alive == any message type still fresh (freshness bitmask non-zero).
    uint16_t eligible = 0;
    for (uint8_t i = 0; i < num_peers; i++) {
        if (peer_state[i].freshness != 0) {
            eligible++;
        }
    }

    EXPECT_DELAY_MS(3000);
    const int fd = AP::FS().open(AP_SWARMMESH_PEER_FILE, O_WRONLY | O_CREAT | O_TRUNC);
    if (fd == -1) {
        return;
    }

    const AP_SwarmMesh_PeerFileHeader_t hdr{
        magic          : AP_SWARMMESH_PEER_FILE_MAGIC,
        version        : AP_SWARMMESH_PEER_FILE_VERSION,
        snapshot_size  : sizeof(AP_SwarmMesh_PeerSnapshot_t),
        snapshot_count : eligible,
        saved_time_us  : AP_HAL::micros64()
    };
    AP::FS().write(fd, &hdr, sizeof(hdr));

    for (uint8_t i = 0; i < num_peers; i++) {
        const PeerState &ps = peer_state[i];
        if (ps.freshness == 0) {
            continue;
        }
        const AP_SwarmMesh_PeerSnapshot_t rec{
            sysid           : ps.sysid,
            vehicle_type    : ps.vehicle_type,
            mode            : ps.mode,
            armed_state     : (uint8_t)ps.armed_state,
            landed_state    : ps.landed_state,
            failsafe_flags  : ps.failsafe_flags,
            battery_voltage : ps.battery_voltage,
            local_pos_NED   : { ps.local_pos_NED.x, ps.local_pos_NED.y, ps.local_pos_NED.z },
            global_pos      : { ps.global_pos.x, ps.global_pos.y, ps.global_pos.z },
            attitude        : { ps.attitude.x, ps.attitude.y, ps.attitude.z },
            role            : ps.role,
            task_id         : ps.task_id,
            formation_slot  : ps.formation_slot,
            target_pos      : { ps.target_pos.x, ps.target_pos.y, ps.target_pos.z },
            priority        : ps.priority
        };
        AP::FS().write(fd, &rec, sizeof(rec));
    }

    AP::FS().close(fd);
}

// called once from init(): restore the peer table from the on-disk snapshot, if one exists and is valid. Bails on a missing, foreign, or truncated file.
// TODO: the save path isn't atomic, so a power loss mid-write can leave a partial file behind
void AP_SwarmMesh::load_peer_snapshot()
{
    EXPECT_DELAY_MS(3000);
    const int fd = AP::FS().open(AP_SWARMMESH_PEER_FILE, O_RDONLY);
    if (fd == -1) {
        // no snapshot yet (normal on inital boot)
        return;
    }

    AP_SwarmMesh_PeerFileHeader_t hdr;
    if (AP::FS().read(fd, &hdr, sizeof(hdr)) != (int32_t)sizeof(hdr)) {
        AP::FS().close(fd);
        return;
    }

    if (hdr.magic != AP_SWARMMESH_PEER_FILE_MAGIC || hdr.version != AP_SWARMMESH_PEER_FILE_VERSION || hdr.snapshot_size != sizeof(AP_SwarmMesh_PeerSnapshot_t)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SwarmMesh: ignoring incompatible peer snapshot");
        AP::FS().close(fd);
        return;
    }

    const uint16_t snapshot_count = MIN(hdr.snapshot_count, (uint16_t)AP_SWARMMESH_MAX_PEERS);
    uint16_t restored = 0;

    for (uint16_t i = 0; i < snapshot_count; i++) {
        AP_SwarmMesh_PeerSnapshot_t rec;
        if (AP::FS().read(fd, &rec, sizeof(rec)) != (int32_t)sizeof(rec)) {
            // short/truncated read - stop here, keep whatever was already restored
            break;
        }

        PeerState *ps = find_or_alloc_peer(rec.sysid);
        if (ps == nullptr) {
            continue;
        }

        // freshness, last_heard_ms, last_seq, seq_seen_mask, rssi, rx_count, drop_count, prev_id, variances and health_flags are deliberately left at zero.
        ps->vehicle_type    = rec.vehicle_type;
        ps->mode            = rec.mode;
        ps->armed_state     = (rec.armed_state != 0);
        ps->landed_state    = rec.landed_state;
        ps->failsafe_flags  = rec.failsafe_flags;
        ps->battery_voltage = rec.battery_voltage;
        ps->local_pos_NED   = Vector3f(rec.local_pos_NED[0], rec.local_pos_NED[1], rec.local_pos_NED[2]);
        ps->global_pos      = Vector3l(rec.global_pos[0], rec.global_pos[1], rec.global_pos[2]);
        ps->attitude        = Vector3f(rec.attitude[0], rec.attitude[1], rec.attitude[2]);
        ps->role            = rec.role;
        ps->task_id         = rec.task_id;
        ps->formation_slot  = rec.formation_slot;
        ps->target_pos      = Vector3l(rec.target_pos[0], rec.target_pos[1], rec.target_pos[2]);
        ps->priority        = rec.priority;
        restored++;
    }

    AP::FS().close(fd);

    if (restored > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SwarmMesh: restored %u peer(s) from snapshot", (unsigned)restored);
    }
}
#endif  // AP_FILESYSTEM_FILE_WRITING_ENABLED

// delete stale peer table entries and compacts the table so the live entries stay contiguous in [0, num_peers)
void AP_SwarmMesh::prune_peer_table()
{
    uint8_t write = 0;
    for (uint8_t read = 0; read < num_peers; read++) {
        if (peer_state[read].freshness == 0) {
            continue;  // no fresh message types left -> peer is dead, drop it
        }
        if (write != read) {
            peer_state[write] = peer_state[read];
        }
        write++;
    }
    num_peers = write;
}

#if HAL_LOGGING_ENABLED
void AP_SwarmMesh::log()
{
    if (!device_ready()) {
        return;
    }
    _driver->log_stats();
}
#endif

// singleton instance
AP_SwarmMesh *AP_SwarmMesh::_singleton;

namespace AP {

AP_SwarmMesh *swarmmesh()
{
    return AP_SwarmMesh::get_singleton();
}

}

#endif