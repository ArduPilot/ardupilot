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

#include "AP_Networking_SwitchPort_MAVLink_COBS.h"

#if AP_NETWORKING_BACKEND_SWITCHPORT_MAVLINK_COBS

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

using namespace AP_Networking_COBS_Protocol;

// Static registry members
AP_Networking_Switch *AP_Networking_SwitchPort_MAVLink_COBS::_hub;
uint8_t AP_Networking_SwitchPort_MAVLink_COBS::_local_device_id[6];
HAL_Semaphore AP_Networking_SwitchPort_MAVLink_COBS::_registry_sem;
AP_Networking_SwitchPort_MAVLink_COBS *AP_Networking_SwitchPort_MAVLink_COBS::_ports[MAX_TUNNELS];
uint8_t AP_Networking_SwitchPort_MAVLink_COBS::_num_ports;

/*
  Initialize the registry with hub and local device ID
*/
void AP_Networking_SwitchPort_MAVLink_COBS::init_registry(AP_Networking_Switch *hub, 
                                                          const uint8_t local_device_id[6])
{
    WITH_SEMAPHORE(_registry_sem);
    _hub = hub;
    memcpy(_local_device_id, local_device_id, 6);
}

/*
  Get or create a port for the given (chan, sysid, compid) key
*/
AP_Networking_SwitchPort_MAVLink_COBS *AP_Networking_SwitchPort_MAVLink_COBS::get_or_create_port(
    mavlink_channel_t chan, uint8_t sysid, uint8_t compid)
{
    WITH_SEMAPHORE(_registry_sem);
    
    if (_hub == nullptr) {
        // Registry not initialized
        return nullptr;
    }
    
    // Search for existing port with matching key
    for (uint8_t i = 0; i < _num_ports; i++) {
        if (_ports[i] != nullptr &&
            _ports[i]->tx_chan == chan &&
            _ports[i]->target_sysid == sysid &&
            _ports[i]->target_compid == compid) {
            return _ports[i];
        }
    }
    
    // Not found - create new port if space available
    if (_num_ports >= MAX_TUNNELS) {
        return nullptr;
    }
    
    auto *port = NEW_NOTHROW AP_Networking_SwitchPort_MAVLink_COBS(chan, sysid, compid);
    if (port == nullptr) {
        return nullptr;
    }
    
    if (!port->init()) {
        delete port;
        return nullptr;
    }
    
    if (_hub->register_port(port) < 0) {
        delete port;
        return nullptr;
    }
    
    _ports[_num_ports++] = port;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: MAVLink COBS tunnel created (chan=%u sys=%u comp=%u)",
                  (unsigned)chan, (unsigned)sysid, (unsigned)compid);
    
    return port;
}

/*
  Update all registered ports
*/
void AP_Networking_SwitchPort_MAVLink_COBS::update_all()
{
    WITH_SEMAPHORE(_registry_sem);
    for (uint8_t i = 0; i < _num_ports; i++) {
        if (_ports[i] != nullptr) {
            _ports[i]->update();
        }
    }
}

AP_Networking_SwitchPort_MAVLink_COBS::AP_Networking_SwitchPort_MAVLink_COBS(
    mavlink_channel_t chan,
    uint8_t _target_sysid,
    uint8_t _target_compid)
    : tx_chan(chan)
    , target_sysid(_target_sysid)
    , target_compid(_target_compid)
{
}

AP_Networking_SwitchPort_MAVLink_COBS::~AP_Networking_SwitchPort_MAVLink_COBS()
{
    // Remove from registry
    WITH_SEMAPHORE(_registry_sem);
    for (uint8_t i = 0; i < _num_ports; i++) {
        if (_ports[i] == this) {
            // Shift remaining ports down
            for (uint8_t j = i; j < _num_ports - 1; j++) {
                _ports[j] = _ports[j + 1];
            }
            _ports[_num_ports - 1] = nullptr;
            _num_ports--;
            break;
        }
    }
}

bool AP_Networking_SwitchPort_MAVLink_COBS::init()
{
    // Set decoder buffer (required before use)
    decoder.set_buffer(rx_frame_buffer, sizeof(rx_frame_buffer));
    
    last_keepalive_tx_ms = AP_HAL::millis();
    return true;
}

bool AP_Networking_SwitchPort_MAVLink_COBS::is_link_up() const
{
    if (last_rx_ms == 0) {
        return false;
    }
    return (AP_HAL::millis() - last_rx_ms) < LINK_TIMEOUT_MS;
}

bool AP_Networking_SwitchPort_MAVLink_COBS::get_remote_device_id(uint8_t id_out[6]) const
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
void AP_Networking_SwitchPort_MAVLink_COBS::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    
    // Send keepalive periodically
    if ((now_ms - last_keepalive_tx_ms) >= KEEPALIVE_INTERVAL_MS) {
        send_keepalive();
        last_keepalive_tx_ms = now_ms;
    }
}

/*
  COBS encode and send data as fragmented TUNNEL messages
*/
void AP_Networking_SwitchPort_MAVLink_COBS::send_cobs_frame(const uint8_t *data, size_t len)
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
        size_t chunk_len = MIN(enc_len - offset, (size_t)TUNNEL_PAYLOAD_MAX);
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
void AP_Networking_SwitchPort_MAVLink_COBS::send_keepalive()
{
    WITH_SEMAPHORE(tx_sem);
    
    // Build keepalive: "KA" + device_id[6] + rx_good[2] + CRC32
    uint8_t ka_buf[KA_TOTAL_LEN];
    size_t ka_len = build_keepalive(ka_buf, sizeof(ka_buf), _local_device_id, rx_good);
    if (ka_len == 0) {
        return;
    }
    
    send_cobs_frame(ka_buf, ka_len);
    ka_tx_count++;
}

/*
  Deliver an ethernet frame: add CRC, COBS encode, send as TUNNEL messages
*/
void AP_Networking_SwitchPort_MAVLink_COBS::deliver_frame(const uint8_t *frame, size_t len)
{
    if (frame == nullptr || len == 0 || len > MAX_FRAME) {
        return;
    }
#if AP_NETWORKING_CAPTURE_ENABLED
    capture.capture_frame(frame, len);
#endif
    
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
  Send a single TUNNEL message on our designated channel
*/
bool AP_Networking_SwitchPort_MAVLink_COBS::send_tunnel(uint16_t payload_type, 
                                                   const uint8_t *data, uint8_t len)
{
    if (!HAVE_PAYLOAD_SPACE(tx_chan, TUNNEL)) {
        return false;
    }
    
    mavlink_msg_tunnel_send(
        tx_chan,
        target_sysid,
        target_compid,
        payload_type,
        len,
        data
    );
    
    return true;
}

/*
  Handle a decoded COBS frame (after CRC verification)
*/
bool AP_Networking_SwitchPort_MAVLink_COBS::handle_decoded_frame(const uint8_t *data, size_t len)
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
#if AP_NETWORKING_CAPTURE_ENABLED
        capture.capture_frame(payload, payload_len);
#endif
        _hub->route_frame(this, payload, payload_len);
        rx_count++;
        return true;
    
    case FrameType::DATA_BONDED:
        // MAVLink doesn't support bonded mode (no multi-link striping)
        // But we can still receive bonded frames - just ignore the sequence
        rx_good++;
#if AP_NETWORKING_CAPTURE_ENABLED
        capture.capture_frame(payload, payload_len);
#endif
        _hub->route_frame(this, payload, payload_len);
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
void AP_Networking_SwitchPort_MAVLink_COBS::handle_tunnel(uint16_t payload_type,
                                                          const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len == 0) {
        return;
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

#endif // AP_NETWORKING_BACKEND_SWITCHPORT_MAVLINK_COBS
