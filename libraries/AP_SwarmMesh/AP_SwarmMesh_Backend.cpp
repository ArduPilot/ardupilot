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

#include "AP_SwarmMesh_Backend.h"

#if AP_SWARMMESH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_BattMonitor/AP_BattMonitor_config.h>
#if AP_BATTERY_ENABLED
#include <AP_BattMonitor/AP_BattMonitor.h>
#endif
#include <AP_Vehicle/AP_Vehicle.h>

#if AP_SWARMMESH_POSCONTROL_ENABLED
#include <AC_AttitudeControl/AC_PosControl.h>
#endif

#include "LogStructure.h"

#if AP_SWARMMESH_COORD_ENABLED
#include "AP_SwarmMesh_coord.h"
#endif

extern const AP_HAL::HAL& hal;

// freshness budgets (ms), indexed by AP_SwarmMesh::MsgFresh. Each is ~3x the stream period. Order MUST match the MsgFresh enum
static constexpr uint32_t FRESH_BUDGET_MS[] = {
    3000,  // HEARTBEAT                  (~1 Hz)
    3000,  // SYS_STATUS
    2000,  // GLOBAL_POSITION_INT
    2000,  // LOCAL_POSITION_NED
    3000,  // POSITION_TARGET_GLOBAL_INT
    5000,  // EXTENDED_SYS_STATE
    2000,  // ATTITUDE
    5000,  // EKF_STATUS_REPORT
    2000,  // SCALED_IMU
#if AP_SWARMMESH_COORD_ENABLED
    3000,  // COORDINATION
#endif
};
static_assert(sizeof(FRESH_BUDGET_MS) / sizeof(FRESH_BUDGET_MS[0]) == AP_SwarmMesh::NUM_FRESH_TYPES, "FRESH_BUDGET_MS must have one entry per MsgFresh bit");

AP_SwarmMesh_Backend::AP_SwarmMesh_Backend(AP_SwarmMesh &frontend) :
    _frontend(frontend)
{
}

// ---- frontend accessors ----

// The vehicle's MAVLink system ID is also its mesh identity: it is the origin_id in the peer header,
// the sysid the peer stream is packed with, and the key of its peer table entry.
uint8_t AP_SwarmMesh_Backend::frontend_sysid() const
{
#if HAL_GCS_ENABLED
    return gcs().sysid_this_mav();
#else
    return 1;
#endif
}

uint8_t AP_SwarmMesh_Backend::frontend_dest_id() const
{
    return (uint8_t)_frontend.destination_id;
}

uint8_t AP_SwarmMesh_Backend::frontend_ttl() const
{
    return (uint8_t)_frontend.ttl;
}

AP_SwarmMesh::PeerState *AP_SwarmMesh_Backend::frontend_peerstate(uint8_t peer_sysid)
{
    return _frontend.find_or_alloc_peer(peer_sysid);
}

uint8_t AP_SwarmMesh_Backend::frontend_peer_count() const
{
    return _frontend.num_peers;
}

AP_SwarmMesh::PeerState *AP_SwarmMesh_Backend::frontend_peer_at(uint8_t index)
{
    if (index >= _frontend.num_peers) {
        return nullptr;
    }
    return &_frontend.peer_state[index];
}

uint8_t AP_SwarmMesh_Backend::frontend_sr_rate(uint8_t bucket) const
{
    if (bucket >= AP_SwarmMesh::NUM_BUCKETS) {
        return 0;
    }
    return MAX(0, (int8_t)_frontend.stream_rate[bucket]);
}

bool AP_SwarmMesh_Backend::frontend_uses_full() const
{
    const uint8_t hw = (uint8_t)_frontend.hardware_mask;
    if (!(hw & 0x01)) {
        return false;
    }
#if defined(STM32F4)
    return false;
#else
    return true;
#endif
}

uint16_t AP_SwarmMesh_Backend::frontend_log_rate_hz() const
{
    return MAX(0, (int16_t)_frontend.log_rate_hz);
}

uint32_t AP_SwarmMesh_Backend::frontend_log_mask() const
{
    return (uint32_t)(int32_t)_frontend.log_mask;
}

int8_t AP_SwarmMesh_Backend::frontend_fwd_port() const
{
    return _frontend.fwd_port;
}

// ---- healthy / update ----

bool AP_SwarmMesh_Backend::healthy()
{
    return (AP_HAL::millis() - _last_rx_ms) < AP_SWARMMESH_TIMEOUT_MS;
}

void AP_SwarmMesh_Backend::update()
{
    if (!transport_ready()) {
        return;
    }

    // drain available RX bytes through the parser. The budget caps max CPU per scheduler slot
    uint32_t nbytes = MIN(transport_available(), (uint32_t)AP_SWARMMESH_RX_BUDGET_BYTES);
    while (nbytes-- > 0) {
        const int16_t b = transport_read();
        if (b >= 0 && parse_byte((uint8_t)b)) {
            process_packet();
        }
    }

    // type freshness on every tick
    const uint32_t now_ms = AP_HAL::millis();
    for (uint8_t i = 0; i < frontend_peer_count(); i++) {
        AP_SwarmMesh::PeerState *ps = frontend_peer_at(i);
        if (ps == nullptr) {
            continue;
        }
        for (uint8_t b = 0; b < AP_SwarmMesh::NUM_FRESH_TYPES; b++) {
            const uint32_t bit = 1U << b;
            if ((ps->freshness & bit) && (now_ms - ps->last_heard_ms[b]) > FRESH_BUDGET_MS[b]) {
                ps->freshness &= ~bit;
            }
        }
    }

    // TX send path

    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000U;
    if (now_ms - _last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        _last_heartbeat_ms = now_ms;
        send_heartbeat();
    }

    const uint32_t hw_min_interval_ms = frontend_uses_full() ? (1000U / AP_SWARMMESH_FULL_HZ) : (1000U / AP_SWARMMESH_LITE_HZ);

    for (uint8_t i = 0; i < AP_SwarmMesh::NUM_BUCKETS; i++) {
        const uint8_t rate_hz = frontend_sr_rate(i);
        if (rate_hz == 0) {
            continue;
        }
        const uint32_t interval_ms = MAX(1000U / (uint32_t)rate_hz, hw_min_interval_ms);
        if (now_ms - _last_bucket_ms[i] < interval_ms) {
            continue;
        }
        _last_bucket_ms[i] = now_ms;
        send_stream(static_cast<Bucket>(i));
    }
}

// ---- parser ----

bool AP_SwarmMesh_Backend::parse_byte(uint8_t b)
{
    switch (_state) {

    case ParseState::WAIT_SYNC1:
        if (b == SWARMMESH_SYNC1) {
            _msgbuf[0] = b;
            _msg_len = 1;
            _crc = b;
            _state = ParseState::WAIT_SYNC2;
        }
        break;

    case ParseState::WAIT_SYNC2:
        if (b == SWARMMESH_SYNC2) {
            _msgbuf[1] = b;
            _msg_len = 2;
            _crc += b;
            _state = ParseState::HEADER;
        } else {
            _state = ParseState::WAIT_SYNC1;
        }
        break;

    case ParseState::HEADER:
        if (_msg_len < SWARMMESH_HEADER_SIZE - 1) {
            _crc += b;
        }
        _msgbuf[_msg_len++] = b;
        if (_msg_len == SWARMMESH_HEADER_SIZE) {
            const p2p_header_t *hdr = (const p2p_header_t *)_msgbuf;
            _type = hdr->type;
            _payload_len = hdr->payload_len;
            if (_crc != hdr->crc) {
                _crc_fail++;
                _state = ParseState::WAIT_SYNC1;
            } else if (_payload_len == 0) {
                _state = ParseState::WAIT_SYNC1;
                return true;
            } else {
                _state = ParseState::PAYLOAD;
            }
        }
        break;

    case ParseState::PAYLOAD: {
        if (_msg_len < SWARMMESH_MSG_BUF_MAX) {
            _msgbuf[_msg_len] = b;
        }
        _msg_len++;
        if (_msg_len == SWARMMESH_HEADER_SIZE + _payload_len) {
            _state = ParseState::WAIT_SYNC1;
            return true;
        }
        break;
    }
    }
    return false;
}

// ---- packet processing ----

void AP_SwarmMesh_Backend::process_packet()
{
    _last_rx_ms = AP_HAL::millis();

    const p2p_header_t *hdr = (const p2p_header_t *)_msgbuf;

    if (hdr->version != SWARMMESH_VERSION_01) {
        _dropped++;
        return;
    }

    AP_SwarmMesh::PeerState *ps = frontend_peerstate(hdr->origin_id);
    if (ps == nullptr) {
        _dropped++;
        return;
    }

    if (ps->seq_seen_mask == 0) {
        ps->seq_seen_mask = 1;
        ps->last_seq = hdr->seq;
    } else {
        const int16_t delta = (int16_t)(hdr->seq - ps->last_seq);
        if (delta == 0) {
            _dedup++;
            ps->drop_count++;
            return;
        } else if (delta > 0 && delta < 32) {
            ps->seq_seen_mask = (ps->seq_seen_mask << (uint8_t)delta) | 1U;
            ps->last_seq = hdr->seq;
        } else if (delta >= 32) {
            ps->seq_seen_mask = 1U;
            ps->last_seq = hdr->seq;
        } else if (delta > -32) {
            const uint32_t bit = 1U << (uint8_t)(-delta);
            if (ps->seq_seen_mask & bit) {
                _dedup++;
                ps->drop_count++;
                return;
            }
            ps->seq_seen_mask |= bit;
        } else {
            _dropped++;
            ps->drop_count++;
            return;
        }
    }

    // Staleness check. deadline_ms == 0 means "no freshness budget" (skip). Otherwise only check when the sender was GPS-synced
    // Note: every SITL instance runs its own sim clock, so this check can't be validated in sim.
    uint64_t rx_utc = 0;
    if (!(hdr->flags & SWARMMESH_NO_RTC) && hdr->deadline_ms != 0 && have_synced_utc(rx_utc)) {
        const uint64_t deadline_us = (uint64_t)hdr->deadline_ms * 1000ULL;
        if (rx_utc > hdr->origin_time_us && (rx_utc - hdr->origin_time_us) > deadline_us) {
            _stale++;
            ps->drop_count++;
            return;
        }
    }

    if (hdr->ttl == 0) {
        _ttl++;
        ps->drop_count++;
        return;
    }

    const bool is_broadcast = (hdr->dest_id == SWARMMESH_BROADCAST);    // dest_id=0 is broadcast: deliver locally to every peer without forwarding
    if (!is_broadcast && hdr->dest_id != frontend_sysid()) {
        forward_mavlink(hdr->origin_id, hdr->dest_id,
                        &_msgbuf[SWARMMESH_HEADER_SIZE],
                        hdr->deadline_ms, hdr->ttl,
                        hdr->payload_len, hdr->flags,
                        hdr->origin_time_us, hdr->seq);
        return;
    }

    if (hdr->type == SWARMMESH_TYPE_MAVLINK) {
        ps->prev_id = hdr->prev_id;
        ps->rx_count++;
        const uint8_t *payload = &_msgbuf[SWARMMESH_HEADER_SIZE];
#if HAL_GCS_ENABLED
        // hand the untouched frame to the companion computer port. This runs after the
        // CRC, duplicate, staleness and TTL checks above, so only packets we accept locally are forwarded.
        forward_to_port(payload, hdr->payload_len);
#endif
        mavlink_message_t msg;
        for (uint8_t i = 0; i < hdr->payload_len; i++) {
            if (mavlink_frame_char_buffer(&_mavlink_rxmsg, &_mavlink_rx_status, payload[i], &msg, &_mavlink_rx_status)) {
                handle_mavlink(msg, *ps);
            }
        }
    } else {
        _dropped++;
        ps->drop_count++;
    }
}

bool AP_SwarmMesh_Backend::log_rate_ok()
{
    const uint16_t rate_hz = frontend_log_rate_hz();
    if (rate_hz == 0) {
        return false;
    }
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t interval_ms = 1000U / rate_hz;
    if (now_ms - _last_log_ms < interval_ms) {
        return false;
    }
    _last_log_ms = now_ms;
    return true;
}

// map a MAVLink msgid to its MsgFresh bit position, or -1 if we don't track it.
int8_t AP_SwarmMesh_Backend::fresh_bit_for_msgid(uint32_t msgid)
{
    switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:                  return (int8_t)AP_SwarmMesh::MsgFresh::HEARTBEAT;
    case MAVLINK_MSG_ID_SYS_STATUS:                 return (int8_t)AP_SwarmMesh::MsgFresh::SYS_STATUS;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:        return (int8_t)AP_SwarmMesh::MsgFresh::GLOBAL_POSITION_INT;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:         return (int8_t)AP_SwarmMesh::MsgFresh::LOCAL_POSITION_NED;
    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: return (int8_t)AP_SwarmMesh::MsgFresh::POSITION_TARGET_GLOBAL_INT;
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:         return (int8_t)AP_SwarmMesh::MsgFresh::EXTENDED_SYS_STATE;
    case MAVLINK_MSG_ID_ATTITUDE:                   return (int8_t)AP_SwarmMesh::MsgFresh::ATTITUDE;
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT:          return (int8_t)AP_SwarmMesh::MsgFresh::EKF_STATUS_REPORT;
    case MAVLINK_MSG_ID_SCALED_IMU:                 return (int8_t)AP_SwarmMesh::MsgFresh::SCALED_IMU;
#if AP_SWARMMESH_COORD_ENABLED
    case MAVLINK_MSG_ID_TUNNEL:                     return (int8_t)AP_SwarmMesh::MsgFresh::COORDINATION;
#endif
    default:                                        return -1;
    }
}

// stamp the last_heard time for this message's type and set its freshness bit.
void AP_SwarmMesh_Backend::mark_fresh(AP_SwarmMesh::PeerState &ps, uint32_t msgid)
{
    const int8_t bit = fresh_bit_for_msgid(msgid);
    if (bit < 0) {
        return;
    }
    ps.last_heard_ms[bit] = AP_HAL::millis();
    ps.freshness |= (1U << (uint8_t)bit);
}

#if HAL_GCS_ENABLED
// Forward a received peer MAVLink frame out the serial port selected by _FWD_PORT, so a
// companion computer can consume the swarm as an ordinary multi-vehicle MAVLink stream.
//
// The frame is written exactly as the origin peer sent it, preserving its sysid, compid,
// sequence number and CRC. Nothing is re-serialised or rewritten, so the companion's
// parser sees each peer as a distinct MAVLink system.
void AP_SwarmMesh_Backend::forward_to_port(const uint8_t *frame, uint16_t len)
{
    const int8_t port = frontend_fwd_port();
    if (port < 0 || len == 0) {
        return;
    }

    // resolve the port to a MAVLink channel once, and again whenever the parameter
    // changes, rather than walking the channel list for every received frame
    if (port != _fwd_port_resolved) {
        _fwd_port_resolved = port;
        _fwd_chan = gcs().get_channel_from_port_number((uint8_t)port);
    }
    if (_fwd_chan == UINT8_MAX) {
        return;  // the selected port is not configured as a MAVLink port
    }

    GCS_MAVLINK *link = gcs().chan(_fwd_chan);
    if (link == nullptr) {
        return;
    }
    AP_HAL::UARTDriver *uart = link->get_uart();
    if (uart == nullptr) {
        return;
    }

    // write the whole frame or none of it: a partial write would desynchronise the
    // companion's parser and corrupt every frame after it
    if (uart->txspace() < len) {
        _fwd_dropped++;
        return;
    }
    uart->write(frame, len);
}
#endif  // HAL_GCS_ENABLED

// GPS UTC is the only clock source synchronized across the mesh
bool AP_SwarmMesh_Backend::have_synced_utc(uint64_t &usec) const
{
#if AP_RTC_ENABLED
    return AP::rtc().get_utc_usec(usec) &&
           AP::rtc().get_source_type() == AP_RTC::SOURCE_GPS;
#else
    (void)usec;
    return false;
#endif
}

void AP_SwarmMesh_Backend::handle_mavlink(const mavlink_message_t &msg, AP_SwarmMesh::PeerState &ps)
{
    mark_fresh(ps, msg.msgid);  // stamp per-type freshness before decoding

    switch (msg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        ps.vehicle_type = hb.type;
        ps.armed_state = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        ps.mode = (uint8_t)hb.custom_mode;
#if HAL_LOGGING_ENABLED
        if ((frontend_log_mask() & (uint32_t)LogMsg::HEARTBEAT) && log_rate_ok()) {
            const struct log_SwarmMesh_HB pkt{
            LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_HB_MSG),
            time_us         : AP_HAL::micros64(),
            sysid           : ps.sysid,
            vehicle_type    : ps.vehicle_type,
            mode            : ps.mode,
            armed_state     : (uint8_t)ps.armed_state
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t ss;
        mavlink_msg_sys_status_decode(&msg, &ss);
        ps.battery_voltage = ss.voltage_battery;
        ps.failsafe_flags = ss.onboard_control_sensors_present
                           & ss.onboard_control_sensors_enabled
                           & ~ss.onboard_control_sensors_health;
#if HAL_LOGGING_ENABLED
        if ((frontend_log_mask() & (uint32_t)LogMsg::SYS_STATUS) && log_rate_ok()) {
            const struct log_SwarmMesh_SS pkt{
                LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_SS_MSG),
                time_us     : AP_HAL::micros64(),
                sysid       : ps.sysid,
                bat_voltage : ps.battery_voltage,
                failsafe    : ps.failsafe_flags
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t gp;
        mavlink_msg_global_position_int_decode(&msg, &gp);
        ps.global_pos.x = gp.lat;
        ps.global_pos.y = gp.lon;
        ps.global_pos.z = gp.alt;
        ps.velocity[0] = gp.vx;
        ps.velocity[1] = gp.vy;
        ps.velocity[2] = gp.vz;
#if HAL_LOGGING_ENABLED
        if ((frontend_log_mask() & (uint32_t)LogMsg::GLOBAL_POSITION_INT) && log_rate_ok()) {
            const struct log_SwarmMesh_GP pkt{
                LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_GP_MSG),
                time_us : AP_HAL::micros64(),
                sysid   : ps.sysid,
                lat     : gp.lat,
                lon     : gp.lon,
                alt     : gp.alt,
                vx      : gp.vx,
                vy      : gp.vy,
                vz      : gp.vz
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t lp;
        mavlink_msg_local_position_ned_decode(&msg, &lp);
        ps.local_pos_NED.x = lp.x;
        ps.local_pos_NED.y = lp.y;
        ps.local_pos_NED.z = lp.z;
        ps.velocity[0] = (int16_t)constrain_float(lp.vx * 100.0f, INT16_MIN, INT16_MAX);
        ps.velocity[1] = (int16_t)constrain_float(lp.vy * 100.0f, INT16_MIN, INT16_MAX);
        ps.velocity[2] = (int16_t)constrain_float(lp.vz * 100.0f, INT16_MIN, INT16_MAX);
#if HAL_LOGGING_ENABLED
        if ((frontend_log_mask() & (uint32_t)LogMsg::LOCAL_POSITION_NED) && log_rate_ok()) {
            const struct log_SwarmMesh_LP pkt{
                LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_LP_MSG),
                time_us : AP_HAL::micros64(),
                sysid   : ps.sysid,
                x       : lp.x,
                y       : lp.y,
                z       : lp.z,
                vx      : lp.vx,
                vy      : lp.vy,
                vz      : lp.vz
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: {
        mavlink_position_target_global_int_t pt;
        mavlink_msg_position_target_global_int_decode(&msg, &pt);
        ps.target_pos.x = pt.lat_int;
        ps.target_pos.y = pt.lon_int;
        ps.target_pos.z = (int32_t)(pt.alt * 1000.0f);  // m -> mm
        ps.target_velocity[0] = (int16_t)constrain_float(pt.vx * 100.0f, INT16_MIN, INT16_MAX);
        ps.target_velocity[1] = (int16_t)constrain_float(pt.vy * 100.0f, INT16_MIN, INT16_MAX);
        ps.target_velocity[2] = (int16_t)constrain_float(pt.vz * 100.0f, INT16_MIN, INT16_MAX);
        ps.target_accel[0] = (int16_t)constrain_float(pt.afx * 100.0f, INT16_MIN, INT16_MAX);
        ps.target_accel[1] = (int16_t)constrain_float(pt.afy * 100.0f, INT16_MIN, INT16_MAX);
        ps.target_accel[2] = (int16_t)constrain_float(pt.afz * 100.0f, INT16_MIN, INT16_MAX);
#if HAL_LOGGING_ENABLED
        if ((frontend_log_mask() & (uint32_t)LogMsg::POSITION_TARGET_GLOBAL_INT) && log_rate_ok()) {
            const struct log_SwarmMesh_PT pkt{
                LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_PT_MSG),
                time_us : AP_HAL::micros64(),
                sysid   : ps.sysid,
                lat     : pt.lat_int,
                lon     : pt.lon_int,
                alt     : pt.alt,
                vx      : pt.vx,
                vy      : pt.vy,
                vz      : pt.vz,
                afx     : pt.afx,
                afy     : pt.afy,
                afz     : pt.afz
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
        mavlink_extended_sys_state_t es;
        mavlink_msg_extended_sys_state_decode(&msg, &es);
        ps.landed_state = es.landed_state;
#if HAL_LOGGING_ENABLED
        if ((frontend_log_mask() & (uint32_t)LogMsg::EXTENDED_SYS_STATE) && log_rate_ok()) {
            const struct log_SwarmMesh_ES pkt{
                LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_ES_MSG),
                time_us        : AP_HAL::micros64(),
                sysid          : ps.sysid,
                landed_state   : es.landed_state
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t at;
        mavlink_msg_attitude_decode(&msg, &at);
        ps.attitude.x = at.roll;
        ps.attitude.y = at.pitch;
        ps.attitude.z = at.yaw;
#if HAL_LOGGING_ENABLED
        if ((frontend_log_mask() & (uint32_t)LogMsg::ATTITUDE) && log_rate_ok()) {
            const struct log_SwarmMesh_AT pkt{
                LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_AT_MSG),
                time_us : AP_HAL::micros64(),
                sysid   : ps.sysid,
                pitch   : at.pitch,
                roll    : at.roll,
                yaw     : at.yaw
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t ek;
        mavlink_msg_ekf_status_report_decode(&msg, &ek);
        ps.pos_horiz_variance = ek.pos_horiz_variance;
        ps.pos_vert_variance = ek.pos_vert_variance;
        ps.vel_variance = ek.velocity_variance;
#if HAL_LOGGING_ENABLED
        if ((frontend_log_mask() & (uint32_t)LogMsg::EKF_STATUS_REPORT) && log_rate_ok()) {
            const struct log_SwarmMesh_EK pkt{
                LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_EK_MSG),
                time_us       : AP_HAL::micros64(),
                sysid         : ps.sysid,
                pos_horiz_var : ek.pos_horiz_variance,
                pos_vert_var  : ek.pos_vert_variance,
                vel_var       : ek.velocity_variance
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }
#endif
        break;
    }

    case MAVLINK_MSG_ID_SCALED_IMU: {
        mavlink_scaled_imu_t si;
        mavlink_msg_scaled_imu_decode(&msg, &si);
        ps.accel[0] = si.xacc;
        ps.accel[1] = si.yacc;
        ps.accel[2] = si.zacc;
#if HAL_LOGGING_ENABLED
        if ((frontend_log_mask() & (uint32_t)LogMsg::SCALED_IMU) && log_rate_ok()) {
            const struct log_SwarmMesh_IM pkt{
                LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_IM_MSG),
                time_us : AP_HAL::micros64(),
                sysid   : ps.sysid,
                xacc    : si.xacc,
                yacc    : si.yacc,
                zacc    : si.zacc
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }
#endif
        break;
    }

#if AP_SWARMMESH_COORD_ENABLED
    case MAVLINK_MSG_ID_TUNNEL:
        handle_coordination(msg, ps);
        break;
#endif

    // TODO: Add more cases (NAV_CONTROLLER_OUTPUT, MISSION_CURRENT, ...)

    default:
        _dropped++;
        ps.drop_count++;
        break;
    }
}

#if AP_SWARMMESH_COORD_ENABLED
// unpack a peer's coordination basket. A TUNNEL that is not a basket, or is a version we dont know, counts as a drop.
void AP_SwarmMesh_Backend::handle_coordination(const mavlink_message_t &msg, AP_SwarmMesh::PeerState &ps)
{
    mavlink_tunnel_t tunnel;
    mavlink_msg_tunnel_decode(&msg, &tunnel);
    if (tunnel.payload_type != SWARMMESH_COORD_PAYLOAD_TYPE ||
        tunnel.payload_length < SWARMMESH_COORD_FIXED_LEN) {
        _dropped++;
        ps.drop_count++;
        return;
    }

    swarmmesh_coord_t basket;
    memcpy(&basket, tunnel.payload, MIN((size_t)tunnel.payload_length, sizeof(basket)));
    if (basket.version != SWARMMESH_COORD_VERSION) {
        _dropped++;
        ps.drop_count++;
        return;
    }

    ps.role           = basket.role;
    ps.task_id        = basket.task_id;
    ps.formation_slot = basket.formation_slot;
    ps.priority       = basket.priority;
    ps.target_pos     = Vector3l(basket.target_pos[0], basket.target_pos[1], basket.target_pos[2]);
    memcpy(ps.target_velocity, basket.target_velocity, sizeof(ps.target_velocity));
    memcpy(ps.target_accel, basket.target_accel, sizeof(ps.target_accel));
    // a peer built with a larger AP_SWARMMESH_COORD_USER_MAX than ours is truncated, not rejected
    const uint8_t on_wire = MIN((uint16_t)basket.user_len, (uint16_t)(tunnel.payload_length - SWARMMESH_COORD_FIXED_LEN));
    ps.coord_user_len = MIN(on_wire, (uint8_t)AP_SWARMMESH_COORD_USER_MAX);
    memcpy(ps.coord_user, basket.user, ps.coord_user_len);

#if HAL_LOGGING_ENABLED
    if ((frontend_log_mask() & (uint32_t)LogMsg::COORDINATION) && log_rate_ok()) {
        const struct log_SwarmMesh_CO pkt{
            LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_CO_MSG),
            time_us        : AP_HAL::micros64(),
            sysid          : ps.sysid,
            role           : ps.role,
            task_id        : ps.task_id,
            formation_slot : ps.formation_slot,
            priority       : ps.priority,
            lat            : ps.target_pos.x,
            lon            : ps.target_pos.y,
            alt            : ps.target_pos.z * 0.001f,
            user_len       : ps.coord_user_len
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
#endif
}
#endif  // AP_SWARMMESH_COORD_ENABLED

// ---- TX path ----

void AP_SwarmMesh_Backend::send_mavlink(uint8_t dest_id, const mavlink_message_t *msg, uint16_t deadline_ms, uint8_t ttl)
{
    uint8_t payload[MAVLINK_MAX_PACKET_LEN];
    const uint16_t payload_len = mavlink_msg_to_send_buffer(payload, msg);

    if (transport_txspace() < SWARMMESH_HEADER_SIZE + payload_len) {
        _tx_dropped++;
        return;
    }

    p2p_header_t hdr {};
    hdr.stx1           = SWARMMESH_SYNC1;
    hdr.stx2           = SWARMMESH_SYNC2;
    hdr.version        = SWARMMESH_VERSION_01;
    hdr.type           = SWARMMESH_TYPE_MAVLINK;
    hdr.origin_id      = frontend_sysid();
    hdr.dest_id        = dest_id;
    hdr.prev_id        = frontend_sysid();
    hdr.ttl            = ttl;
    hdr.seq            = _tx_seq++;
    hdr.deadline_ms    = deadline_ms;
    hdr.payload_len    = payload_len;

    // Stamp NORMAL (RTC-synced) only if we actually have GPS UTC right now
    uint64_t utc_usec = 0;
    if (have_synced_utc(utc_usec)) {
        hdr.flags = SWARMMESH_NORMAL;
    } else {
        hdr.flags = SWARMMESH_NO_RTC;
        utc_usec = 0;
    }
    hdr.origin_time_us = utc_usec;

    uint8_t crc = 0;
    const uint8_t *hdr_bytes = (const uint8_t *)&hdr;
    for (uint8_t i = 0; i < SWARMMESH_HEADER_SIZE - 1; i++) {
        crc += hdr_bytes[i];
    }
    hdr.crc = crc;

    // write header + payload as a single buffer so datagram transports see one packet
    uint8_t pkt[SWARMMESH_HEADER_SIZE + MAVLINK_MAX_PACKET_LEN];
    memcpy(pkt, &hdr, SWARMMESH_HEADER_SIZE);
    memcpy(pkt + SWARMMESH_HEADER_SIZE, payload, payload_len);
    transport_write(pkt, SWARMMESH_HEADER_SIZE + payload_len);
}

void AP_SwarmMesh_Backend::forward_mavlink(uint8_t id, uint8_t dest_id, const uint8_t *payload, uint16_t deadline_ms, uint8_t ttl, uint8_t payload_len, uint8_t flags, uint64_t origin_time, uint16_t seq)
{
    if (transport_txspace() < SWARMMESH_HEADER_SIZE + payload_len) {
        _dropped++;
        return;
    }

    p2p_header_t hdr {};
    hdr.stx1           = SWARMMESH_SYNC1;
    hdr.stx2           = SWARMMESH_SYNC2;
    hdr.version        = SWARMMESH_VERSION_01;
    hdr.type           = SWARMMESH_TYPE_MAVLINK;
    hdr.flags          = flags;
    hdr.origin_id      = id;
    hdr.dest_id        = dest_id;
    hdr.prev_id        = frontend_sysid();
    hdr.ttl            = (ttl - 1);
    hdr.seq            = seq;
    hdr.deadline_ms    = deadline_ms;
    hdr.payload_len    = payload_len;
    hdr.origin_time_us = origin_time;

    uint8_t crc = 0;
    const uint8_t *hdr_bytes = (const uint8_t *)&hdr;
    for (uint8_t i = 0; i < SWARMMESH_HEADER_SIZE - 1; i++) {
        crc += hdr_bytes[i];
    }
    hdr.crc = crc;

    // write header + payload as a single buffer so datagram transports see one packet
    uint8_t pkt[SWARMMESH_HEADER_SIZE + SWARMMESH_MAX_PAYLOAD];
    memcpy(pkt, &hdr, SWARMMESH_HEADER_SIZE);
    memcpy(pkt + SWARMMESH_HEADER_SIZE, payload, payload_len);
    transport_write(pkt, SWARMMESH_HEADER_SIZE + payload_len);

    _tx_fwd++;
}

void AP_SwarmMesh_Backend::send_stream(Bucket bucket)
{
    switch (bucket) {
    case Bucket::POSITION:
#if AP_AHRS_ENABLED
        // TODO: add if/else logic so if global pos is active, local doesn't send
        send_local_position();
        send_global_position_int();
#endif
        break;
    case Bucket::EXT_STAT:
        send_sys_status();
        send_nav_controller_output();
        send_position_target_global_int();
        break;
    case Bucket::EXTRA1:
#if AP_AHRS_ENABLED
        send_attitude();
        send_ekf_status_report();
        send_scaled_imu();
#endif
        send_extended_sys_state();
        break;
#if AP_SWARMMESH_COORD_ENABLED
    case Bucket::COORD:
        send_coordination();
        break;
#endif
    }
    // TODO: Add more buckets (mode, mission state, etc.)
}

void AP_SwarmMesh_Backend::send_heartbeat()
{
    // TODO: base_mode()/system_status() are per-vehicle GCS_MAVLINK overrides we have no access to (no vehicle reference)
    const bool armed = AP_HAL::get_HAL().util->get_soft_armed();
    uint8_t base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (armed) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    const MAV_STATE system_status = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        gcs().frame_type(),
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        gcs().custom_mode(),
        system_status);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

#if AP_AHRS_ENABLED
void AP_SwarmMesh_Backend::send_global_position_int()
{
    AP_AHRS &ahrs = AP::ahrs();

    Location loc;
    UNUSED_RESULT(ahrs.get_location(loc));

    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        vel.zero();
    }

    float posD;
    ahrs.get_relative_position_D_home(posD);
    const int32_t relative_alt_mm = (int32_t)(-posD * 1000.0f);

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        AP_HAL::millis(),
        loc.lat,
        loc.lng,
        loc.alt * 10,
        relative_alt_mm,
        (int16_t)(vel.x * 100),
        (int16_t)(vel.y * 100),
        (int16_t)(vel.z * 100),
        ahrs.yaw_sensor);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

void AP_SwarmMesh_Backend::send_local_position()
{
    const AP_AHRS &ahrs = AP::ahrs();

    Vector3f pos, vel;
    if (!ahrs.get_relative_position_NED_origin_float(pos) ||
        !ahrs.get_velocity_NED(vel)) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_local_position_ned_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        AP_HAL::millis(),
        pos.x, pos.y, pos.z,
        vel.x, vel.y, vel.z);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}
#endif  // AP_AHRS_ENABLED

void AP_SwarmMesh_Backend::send_position_target_global_int()
{
    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle == nullptr) {
        return;
    }

    Location target;
    if (!vehicle->get_target_location(target)) {
        return;
    }
    if (!target.initialised()) {
        return;
    }
    float alt_amsl_m;
    if (!target.get_alt_m(Location::AltFrame::ABSOLUTE, alt_amsl_m)) {
        return;
    }

    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t NE_IGNORE =
        POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE;
    static constexpr uint16_t D_IGNORE =
        POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE;
    uint16_t type_mask =
        NE_IGNORE | D_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE |
        POSITION_TARGET_TYPEMASK_LAST_BYTE;

    // vel/accel targets require AC_PosControl, which not every vehicle type links or instantiates
    Vector3f vel_target, accel_target;
#if AP_SWARMMESH_POSCONTROL_ENABLED
    AC_PosControl *pos_control = AC_PosControl::get_singleton();
    if (pos_control != nullptr) {
        const Vector3f v = pos_control->get_vel_target_NED_ms();
        const Vector3f a = pos_control->get_accel_target_NED_mss();
        if (pos_control->NE_is_active() && pos_control->D_is_active()) {
            vel_target = v;
            accel_target = a;
            type_mask &= ~(NE_IGNORE | D_IGNORE);
        } else if (pos_control->D_is_active()) {
            vel_target.z = v.z;
            accel_target.z = a.z;
            type_mask &= ~D_IGNORE;
        }
    }
#endif

    mavlink_message_t msg;
    mavlink_msg_position_target_global_int_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        AP_HAL::millis(),
        MAV_FRAME_GLOBAL,
        type_mask,
        target.lat,
        target.lng,
        alt_amsl_m,
        vel_target.x, vel_target.y, vel_target.z,
        accel_target.x, accel_target.y, accel_target.z,
        0.0f,
        0.0f);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

void AP_SwarmMesh_Backend::send_extended_sys_state()
{
    MAV_LANDED_STATE landed_state = MAV_LANDED_STATE_UNDEFINED;
    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        if (!vehicle->get_likely_flying()) {
            landed_state = MAV_LANDED_STATE_ON_GROUND;
        } else if (vehicle->is_landing()) {
            landed_state = MAV_LANDED_STATE_LANDING;
        } else if (vehicle->is_taking_off()) {
            landed_state = MAV_LANDED_STATE_TAKEOFF;
        } else {
            landed_state = MAV_LANDED_STATE_IN_AIR;
        }
    }

    mavlink_message_t msg;
    mavlink_msg_extended_sys_state_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        MAV_VTOL_STATE_UNDEFINED,
        landed_state);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

#if AP_SWARMMESH_COORD_ENABLED
// broadcast whatever a script or companion computer last published.
// Nothing goes on the air until something has been published, so a vehicle without coordination logic stays quiet.
void AP_SwarmMesh_Backend::send_coordination()
{
    if (!_frontend._coord_state_set) {
        return;
    }
    const SwarmCoordState &state = _frontend._coord_state;

    swarmmesh_coord_t basket {};
    basket.version           = SWARMMESH_COORD_VERSION;
    basket.role              = state.role;
    basket.task_id           = state.task_id;
    basket.formation_slot    = state.formation_slot;
    basket.priority          = state.priority;
    basket.target_pos[0]     = state.target_lat;
    basket.target_pos[1]     = state.target_lng;
    basket.target_pos[2]     = state.target_alt_mm;
    memcpy(basket.target_velocity, state.target_vel_NED, sizeof(basket.target_velocity));
    memcpy(basket.target_accel, state.target_accel_NED, sizeof(basket.target_accel));
    basket.user_len = MIN(state.user_len, (uint8_t)AP_SWARMMESH_COORD_USER_MAX);
    memcpy(basket.user, state.user, basket.user_len);

    // only the user bytes actually in use go on the wire
    const uint8_t payload_len = SWARMMESH_COORD_FIXED_LEN + basket.user_len;
    uint8_t payload[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN] {};
    memcpy(payload, &basket, payload_len);

    mavlink_message_t msg;
    mavlink_msg_tunnel_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        0,                      // target_system: any peer may consume this
        0,                      // target_component
        SWARMMESH_COORD_PAYLOAD_TYPE,
        payload_len,
        payload);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}
#endif  // AP_SWARMMESH_COORD_ENABLED

#if AP_AHRS_ENABLED
void AP_SwarmMesh_Backend::send_attitude()
{
    const AP_AHRS &ahrs = AP::ahrs();
    const Vector3f omega = ahrs.get_gyro();
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        AP_HAL::millis(),
        ahrs.get_roll_rad(),
        ahrs.get_pitch_rad(),
        ahrs.get_yaw_rad(),
        omega.x,
        omega.y,
        omega.z);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

void AP_SwarmMesh_Backend::send_ekf_status_report()
{
    nav_filter_status filter_status{};
    AP::ahrs().get_filter_status(filter_status);

    uint16_t flags = 0;
    if (filter_status.flags.attitude)          { flags |= EKF_ATTITUDE; }
    if (filter_status.flags.horiz_vel)         { flags |= EKF_VELOCITY_HORIZ; }
    if (filter_status.flags.vert_vel)          { flags |= EKF_VELOCITY_VERT; }
    if (filter_status.flags.horiz_pos_rel)     { flags |= EKF_POS_HORIZ_REL; }
    if (filter_status.flags.horiz_pos_abs)     { flags |= EKF_POS_HORIZ_ABS; }
    if (filter_status.flags.vert_pos)          { flags |= EKF_POS_VERT_ABS; }
    if (filter_status.flags.terrain_alt)       { flags |= EKF_POS_VERT_AGL; }
    if (filter_status.flags.const_pos_mode)    { flags |= EKF_CONST_POS_MODE; }
    if (filter_status.flags.pred_horiz_pos_rel){ flags |= EKF_PRED_POS_HORIZ_REL; }
    if (filter_status.flags.pred_horiz_pos_abs){ flags |= EKF_PRED_POS_HORIZ_ABS; }
    if (!filter_status.flags.initalized)       { flags |= EKF_UNINITIALIZED; }
    if (filter_status.flags.gps_glitching)     { flags |= (1U << 15); }

    float velVar = 0, posVar = 0, hgtVar = 0, tasVar = 0;
    Vector3f magVar;
    AP::ahrs().get_variances(velVar, posVar, hgtVar, magVar, tasVar);

    mavlink_message_t msg;
    mavlink_msg_ekf_status_report_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        flags,
        velVar,
        posVar,
        hgtVar,
        fmaxf(fmaxf(magVar.x, magVar.y), magVar.z),
        0,
        tasVar);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

void AP_SwarmMesh_Backend::send_scaled_imu()
{
    const AP_AHRS &ahrs = AP::ahrs();
    // body frame, m/s/s, with the EKF's estimated accelerometer bias removed
    const Vector3f accel = ahrs.get_accel() - ahrs.get_accel_bias();

    mavlink_message_t msg;
    mavlink_msg_scaled_imu_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        AP_HAL::millis(),
        (int16_t)constrain_float(accel.x * 1000.0f / GRAVITY_MSS, INT16_MIN, INT16_MAX),
        (int16_t)constrain_float(accel.y * 1000.0f / GRAVITY_MSS, INT16_MIN, INT16_MAX),
        (int16_t)constrain_float(accel.z * 1000.0f / GRAVITY_MSS, INT16_MIN, INT16_MAX),
        0, 0, 0,
        0, 0, 0,
        0);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}
#endif  // AP_AHRS_ENABLED

// TODO: Fill in empty fields
void AP_SwarmMesh_Backend::send_sys_status()
{
    float voltage_mv = 0;
    float current_ca = -1;
    int8_t remaining_pct = -1;

#if AP_BATTERY_ENABLED
    const AP_BattMonitor &battery = AP::battery();
    if (battery.healthy()) {
        voltage_mv = battery.gcs_voltage() * 1000.0f;
        float amps;
        if (battery.current_amps(amps)) {
            current_ca = constrain_float(amps * 100.0f, -INT16_MAX, INT16_MAX);
        }
        uint8_t pct;
        if (battery.capacity_remaining_pct(pct)) {
            remaining_pct = (int8_t)pct;
        }
    }
#endif

    uint32_t sensors_present = 0;
    uint32_t sensors_enabled = 0;
    uint32_t sensors_health = 0;
    gcs().get_sensor_status_flags(sensors_present, sensors_enabled, sensors_health);

    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        sensors_present,
        sensors_enabled,
        sensors_health,
        0,
        (uint16_t)voltage_mv,
        (int16_t)current_ca,
        remaining_pct,
        0, 0,
        0, 0, 0, 0,
        0, 0, 0);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

// TODO: Find way to access control targets to fill in empty fields
void AP_SwarmMesh_Backend::send_nav_controller_output()
{
    // TODO: SwarmMesh has no access to mode-specific control targets (no vehicle reference)
    float nav_roll_deg = 0;
    float nav_pitch_deg = 0;

    float wp_bearing_deg = 0;
    float wp_distance_m = 0;
    float xtrack_error_m = 0;
    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        vehicle->get_wp_bearing_deg(wp_bearing_deg);
        vehicle->get_wp_distance_m(wp_distance_m);
        vehicle->get_wp_crosstrack_error_m(xtrack_error_m);
    }

    mavlink_message_t msg;
    mavlink_msg_nav_controller_output_pack(
        frontend_sysid(),
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        nav_roll_deg,
        nav_pitch_deg,
        (int16_t)wp_bearing_deg,
        (int16_t)wp_bearing_deg,
        (uint16_t)MIN(wp_distance_m, (float)UINT16_MAX),
        0,
        0,
        xtrack_error_m);

    send_mavlink(frontend_dest_id(), &msg, 0, frontend_ttl());
}

// ---- log_stats ----

#if HAL_LOGGING_ENABLED
void AP_SwarmMesh_Backend::log_stats()
{
    const struct log_SwarmMesh pkt{
       LOG_PACKET_HEADER_INIT(LOG_SWARMMESH_MSG),
       time_us         : AP_HAL::micros64(),
       crc_fail        : _crc_fail,
       stale           : _stale,
       ttl             : _ttl,
       dedup           : _dedup,
       drop            : _dropped,
       txseq           : _tx_seq,
       txfwd           : _tx_fwd,
       txdrop          : _tx_dropped
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AP_SWARMMESH_ENABLED
