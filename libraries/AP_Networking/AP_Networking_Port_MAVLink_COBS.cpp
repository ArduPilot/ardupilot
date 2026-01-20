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

#include "AP_Networking_Port_MAVLink_COBS.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_MAVLINK_COBS

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

using namespace AP_Networking_COBS_Protocol;

AP_Networking_Port_MAVLink_COBS *AP_Networking_Port_MAVLink_COBS::_singleton;

AP_Networking_Port_MAVLink_COBS::AP_Networking_Port_MAVLink_COBS(
    AP_Networking_Hub *_hub,
    const uint8_t _local_device_id[6],
    uint8_t _target_sysid,
    uint8_t _target_compid)
    : hub(_hub)
    , target_sysid(_target_sysid)
    , target_compid(_target_compid)
{
    memcpy(local_device_id, _local_device_id, 6);
    _singleton = this;
}

AP_Networking_Port_MAVLink_COBS::~AP_Networking_Port_MAVLink_COBS()
{
    if (_singleton == this) {
        _singleton = nullptr;
    }
}

bool AP_Networking_Port_MAVLink_COBS::init()
{
    last_keepalive_tx_ms = AP_HAL::millis();
    return true;
}

bool AP_Networking_Port_MAVLink_COBS::is_link_up() const
{
    if (last_rx_ms == 0) {
        return false;
    }
    return (AP_HAL::millis() - last_rx_ms) < LINK_TIMEOUT_MS;
}

bool AP_Networking_Port_MAVLink_COBS::get_remote_device_id(uint8_t id_out[6]) const
{
    if (!remote_id_known) {
        return false;
    }
    memcpy(id_out, remote_device_id, 6);
    return true;
}

/*
  Called from main thread at 10Hz
*/
void AP_Networking_Port_MAVLink_COBS::update()
{
    uint32_t now = AP_HAL::millis();
    
    // Send keepalive periodically
    if ((now - last_keepalive_tx_ms) >= KEEPALIVE_INTERVAL_MS) {
        send_keepalive();
        last_keepalive_tx_ms = now;
    }
}

/*
  COBS encode and send data as fragmented TUNNEL messages
*/
void AP_Networking_Port_MAVLink_COBS::send_cobs_frame(const uint8_t *data, size_t len)
{
    // COBS encode with delimiter using shared protocol helper
    size_t enc_len = encode_frame(data, len, tx_encode_buffer, sizeof(tx_encode_buffer));
    if (enc_len == 0) {
        return;
    }
    
    // Send in chunks via TUNNEL messages
    size_t offset = 0;
    bool first = true;
    
    while (offset < enc_len) {
        size_t chunk_len = MIN(enc_len - offset, TUNNEL_PAYLOAD_MAX);
        uint16_t type = first ? MAV_TUNNEL_PAYLOAD_TYPE_COBS_START 
                              : MAV_TUNNEL_PAYLOAD_TYPE_COBS_CONT;
        
        if (!send_tunnel(type, &tx_encode_buffer[offset], chunk_len)) {
            // Buffer full - drop rest of frame rather than blocking
            break;
        }
        
        offset += chunk_len;
        first = false;
    }
}

/*
  Send a keepalive frame
*/
void AP_Networking_Port_MAVLink_COBS::send_keepalive()
{
    WITH_SEMAPHORE(tx_sem);
    
    // Build keepalive: "KA" + device_id[6] + rx_good[2] + CRC32
    uint8_t ka_buf[KA_TOTAL_LEN];
    size_t ka_len = build_keepalive(ka_buf, sizeof(ka_buf), local_device_id, rx_good);
    if (ka_len == 0) {
        return;
    }
    
    send_cobs_frame(ka_buf, ka_len);
    ka_tx_count++;
}

/*
  Deliver an ethernet frame: add CRC, COBS encode, send as TUNNEL messages
*/
void AP_Networking_Port_MAVLink_COBS::deliver_frame(const uint8_t *frame, size_t len)
{
    if (frame == nullptr || len == 0 || len > MAX_FRAME) {
        return;
    }
    
    WITH_SEMAPHORE(tx_sem);
    
    // Build data frame: ethernet_frame + CRC32
    size_t input_len = build_data_frame(tx_input_buffer, sizeof(tx_input_buffer), frame, len);
    if (input_len == 0) {
        return;
    }
    
    send_cobs_frame(tx_input_buffer, input_len);
    tx_count++;
}

/*
  Send a single TUNNEL message
*/
bool AP_Networking_Port_MAVLink_COBS::send_tunnel(uint16_t payload_type, 
                                                   const uint8_t *data, uint8_t len)
{
    // Find an active MAVLink channel
    for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
        GCS_MAVLINK *link = gcs().chan(i);
        if (link == nullptr || !link->is_active()) {
            continue;
        }
        
        mavlink_channel_t chan = link->get_chan();
        
        if (!HAVE_PAYLOAD_SPACE(chan, TUNNEL)) {
            continue;
        }
        
        mavlink_msg_tunnel_send(
            chan,
            target_sysid,
            target_compid,
            payload_type,
            len,
            data
        );
        
        return true;
    }
    
    return false;
}

/*
  Handle a decoded COBS frame (after CRC verification)
*/
bool AP_Networking_Port_MAVLink_COBS::handle_decoded_frame(const uint8_t *data, size_t len)
{
    const uint8_t *payload;
    size_t payload_len;
    
    FrameType type = identify_frame(data, len, &payload, &payload_len);
    
    switch (type) {
    case FrameType::KEEPALIVE: {
        // Extract keepalive fields
        uint8_t their_device_id[6];
        uint16_t their_rx_good;
        parse_keepalive(payload, their_device_id, &their_rx_good);
        
        // Learn/verify remote device ID
        if (!remote_id_known) {
            memcpy(remote_device_id, their_device_id, 6);
            remote_id_known = true;
        }
        
        ka_rx_count++;
        return true;
    }
    
    case FrameType::DATA_SINGLE:
        // Deliver ethernet frame to hub
        rx_good++;
        hub->route_frame(this, payload, payload_len);
        rx_count++;
        return true;
    
    case FrameType::DATA_GANGED:
        // MAVLink doesn't support ganged mode (no multi-link striping)
        // But we can still receive ganged frames - just ignore the sequence
        rx_good++;
        hub->route_frame(this, payload, payload_len);
        rx_count++;
        return true;
    
    case FrameType::INVALID:
    default:
        crc_errors++;
        return false;
    }
}

/*
  Handle incoming TUNNEL message - feed data into COBS decoder
*/
void AP_Networking_Port_MAVLink_COBS::handle_tunnel(uint16_t payload_type,
                                                     const uint8_t *payload, uint8_t payload_len,
                                                     uint8_t src_sysid, uint8_t src_compid)
{
    if (payload_len == 0) {
        return;
    }
    
    // Learn target from first received message
    if (target_sysid == 0) {
        target_sysid = src_sysid;
        target_compid = src_compid;
    }
    
    WITH_SEMAPHORE(rx_sem);
    
    last_rx_ms = AP_HAL::millis();
    
    // COBS_START resets the decoder for a new frame
    if (payload_type == MAV_TUNNEL_PAYLOAD_TYPE_COBS_START) {
        decoder.reset();
    }
    
    // Feed data into streaming COBS decoder
    for (uint8_t i = 0; i < payload_len; i++) {
        if (decoder.process_byte(payload[i])) {
            // Frame complete - get decoded data
            size_t frame_len = sizeof(rx_frame_buffer);
            if (decoder.get_frame(rx_frame_buffer, &frame_len, sizeof(rx_frame_buffer))) {
                if (!handle_decoded_frame(rx_frame_buffer, frame_len)) {
                    rx_errors++;
                }
            } else {
                rx_errors++;
            }
            decoder.reset();
        }
    }
}

#endif // AP_NETWORKING_BACKEND_HUB_PORT_MAVLINK_COBS
