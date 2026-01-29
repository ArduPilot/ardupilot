#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_MAVLINK_COBS

#include "AP_Networking_Hub.h"
#include "AP_Networking_COBS.h"
#include "AP_Networking_COBS_Protocol.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>

/*
  Hub port that tunnels COBS ethernet frames over MAVLink TUNNEL messages.
  
  Uses the same protocol as AP_Networking_Port_COBS (UART):
  - Keepalive: "KA" + device_id[6] + rx_good[2] + CRC32
  - Data:      ethernet_frame + CRC32
  
  Assumes in-order MAVLink delivery. COBS-encoded data is split into
  128-byte TUNNEL payloads with two payload types:
  
  - COBS_START: First chunk (resets RX decoder)
  - COBS_CONT:  Continuation chunks
*/
class AP_Networking_Port_MAVLink_COBS : public AP_Networking_HubPort
{
public:
    // Payload types (experimental range 32768+)
    static constexpr uint16_t MAV_TUNNEL_PAYLOAD_TYPE_COBS_START = 32769;
    static constexpr uint16_t MAV_TUNNEL_PAYLOAD_TYPE_COBS_CONT  = 32770;
    
    AP_Networking_Port_MAVLink_COBS(AP_Networking_Hub *hub,
                                    const uint8_t local_device_id[6],
                                    uint8_t target_sysid,
                                    uint8_t target_compid);
    ~AP_Networking_Port_MAVLink_COBS();
    
    CLASS_NO_COPY(AP_Networking_Port_MAVLink_COBS);
    
    bool init();
    void update() override;
    
    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override { return true; }
    const char *get_name() const override { return "MAVLink_COBS"; }
    bool is_link_up() const override;
    
    // Handle incoming TUNNEL message (called from GCS)
    void handle_tunnel(uint16_t payload_type, const uint8_t *payload, uint8_t payload_len,
                       uint8_t src_sysid, uint8_t src_compid);
    
    // Check if payload type is one of ours
    static bool is_cobs_payload_type(uint16_t type) {
        return type == MAV_TUNNEL_PAYLOAD_TYPE_COBS_START ||
               type == MAV_TUNNEL_PAYLOAD_TYPE_COBS_CONT;
    }
    
    // Target identification
    uint8_t get_target_sysid() const { return target_sysid; }
    uint8_t get_target_compid() const { return target_compid; }
    
    // Remote device ID (learned from keepalives)
    bool get_remote_device_id(uint8_t id_out[6]) const;
    bool has_remote_device_id() const { return remote_id_known; }
    
    // Statistics
    uint32_t get_rx_count() const { return rx_count; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_errors() const { return rx_errors; }
    uint32_t get_ka_tx_count() const { return ka_tx_count; }
    uint32_t get_ka_rx_count() const { return ka_rx_count; }
    
    // Singleton accessor for GCS to route TUNNEL messages
    static AP_Networking_Port_MAVLink_COBS *get_singleton() { return _singleton; }

private:
    AP_Networking_Hub *hub;
    uint8_t local_device_id[6];
    uint8_t target_sysid;
    uint8_t target_compid;
    
    // Remote device (learned from keepalives)
    uint8_t remote_device_id[6];
    bool remote_id_known = false;
    
    // TX state
    HAL_Semaphore tx_sem;
    static constexpr uint8_t TUNNEL_PAYLOAD_MAX = 128;
    static constexpr size_t MAX_FRAME = AP_Networking_COBS_Protocol::MAX_FRAME;
    uint8_t tx_input_buffer[MAX_FRAME + 4];  // frame + CRC
    uint8_t tx_encode_buffer[MAX_FRAME + MAX_FRAME/254 + 2];  // COBS overhead
    uint32_t last_keepalive_tx_ms = 0;
    
    // RX state - streaming COBS decoder
    AP_Networking_COBS::Decoder decoder;
    HAL_Semaphore rx_sem;
    uint8_t rx_frame_buffer[MAX_FRAME + 6];  // for decoded frame + CRC
    uint16_t rx_good = 0;  // frames received (for keepalive stats)
    
    // Link status tracking
    uint32_t last_rx_ms = 0;
    
    // Statistics
    uint32_t rx_count = 0;
    uint32_t tx_count = 0;
    uint32_t rx_errors = 0;
    uint32_t crc_errors = 0;
    uint32_t ka_tx_count = 0;
    uint32_t ka_rx_count = 0;
    
    // Methods
    bool send_tunnel(uint16_t payload_type, const uint8_t *data, uint8_t len);
    void send_cobs_frame(const uint8_t *data, size_t len);
    void send_keepalive();
    bool handle_decoded_frame(const uint8_t *data, size_t len);
    
    static AP_Networking_Port_MAVLink_COBS *_singleton;
};

#endif // AP_NETWORKING_BACKEND_HUB_PORT_MAVLINK_COBS
