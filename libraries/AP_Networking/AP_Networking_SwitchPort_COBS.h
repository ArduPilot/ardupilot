#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS

#include "AP_Networking_Switch.h"
#include "AP_Networking_COBS.h"
#include "AP_Networking_COBS_Protocol.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/UARTDriver.h>

// Forward declaration
class AP_Networking_SwitchPort_COBS;

/*
  COBS port for Ethernet-over-serial transport.
  
  Features:
  - Point-to-point COBS-encoded Ethernet frames over UART
  - Keepalive-based link detection and statistics
  - CRC32 frame integrity verification
  - Optional bonding (multiple UARTs to same destination)
  
  Protocol:
  - Keepalive: "KA" + device_id[6] + rx_good[2] + CRC32
  - Data: ethernet_frame + CRC32
  
  Lock ordering (must acquire in this order to avoid deadlock):
    route_sem -> tx_sem
*/
class AP_Networking_COBS_Link
{
public:
    static constexpr size_t MAX_FRAME = 1522;  // Max Ethernet frame including VLAN tag
    
    AP_Networking_COBS_Link(AP_Networking_Switch *hub,
                            AP_HAL::UARTDriver *uart,
                            uint32_t baud_rate,
                            const uint8_t local_device_id[6]);
    ~AP_Networking_COBS_Link();
    
    CLASS_NO_COPY(AP_Networking_COBS_Link);
    
    bool init();
    
    // Called by shared COBS thread to process this port (one iteration)
    void thread_update();
    
    // Send a frame over this UART
    // deliver_frame: single mode, no sequence number (for single-UART bonds)
    // deliver_frame_sequenced: bonded mode with sequence number (for multi-UART bonds)
    void deliver_frame(const uint8_t *frame, size_t len);
    void deliver_frame_sequenced(const uint8_t *frame, size_t len, uint16_t seq);
    bool can_receive() const;
    bool is_link_up() const;
    
    // Discovery info (for hub to match ports)
    bool get_remote_device_id(uint8_t id_out[6]) const;
    bool has_remote_device_id() const;
    
    // Statistics
    uint32_t get_rx_count() const { return rx_count; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_errors() const { return rx_errors; }
    uint32_t get_crc_errors() const { return crc_errors; }
    uint32_t get_keepalive_tx() const { return ka_tx_count; }
    uint32_t get_keepalive_rx() const { return ka_rx_count; }
    
    // TX scheduling: estimated microseconds until TX buffer drains
    // Used by bond to select which UART to send on (pick lowest)
    uint32_t get_tx_completion_us() const;

private:
    friend class AP_Networking_SwitchPort_COBS;
    
    AP_Networking_Switch *hub;
    AP_Networking_SwitchPort_COBS *bond;  // Set by bond when added
    uint8_t local_device_id[6];
    
    // UART state
    AP_HAL::UARTDriver *uart;
    uint32_t baud;
    bool uart_initialized;  // Set true after begin(0) called from thread
    AP_Networking_COBS::Decoder decoder;
    
    // Per-port decoder buffer (can't be shared - partial frames would corrupt)
    uint8_t decoder_buffer[MAX_FRAME + 6];  // seq(2) + frame + CRC(4)
    
    // Link status
    uint8_t seen_device_id[6];
    bool device_id_valid;
    uint32_t last_rx_ms;
    uint32_t last_keepalive_tx_ms;
    
    // Stats for keepalive response
    uint16_t rx_good;  // Data frames received (reported in keepalive)
    
    // Remote device (learned from keepalives)
    uint8_t remote_device_id[6];
    bool remote_id_known;
    
    // Statistics
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t rx_errors;
    uint32_t crc_errors;
    uint32_t ka_tx_count;
    uint32_t ka_rx_count;
    
    // Last received data frame type (hint for CRC checking - avoids computing both CRCs)
    AP_Networking_COBS_Protocol::FrameType last_data_type = AP_Networking_COBS_Protocol::FrameType::DATA_SINGLE;
    
    // Constants
    static constexpr size_t MIN_ETH_FRAME = 14;  // Minimum = Ethernet header only
    static constexpr uint32_t TX_BUFFER_SIZE = 65535;
    static constexpr uint16_t KEEPALIVE_INTERVAL_MS = 500;
    static constexpr uint16_t KEEPALIVE_TIMEOUT_MS = 2000;
    
    // Methods
    void process_rx();
    bool handle_decoded_frame(const uint8_t *data, size_t len);
    bool handle_keepalive(const uint8_t *data);
    bool handle_data_frame_single(const uint8_t *data, size_t len);
    bool handle_data_frame_sequenced(uint16_t seq, const uint8_t *data, size_t len);
    bool send_keepalive();
    void check_keepalive_tx();
    void check_uart_timeout();
    
    // Shared workspace (heap-allocated on first use, shared by all COBS ports)
    // Safe because all COBS ports run in single cobs_thread for RX,
    // and TX is serialized by the shared tx_sem
    struct SharedWorkspace;
    static SharedWorkspace *workspace;
    static bool ensure_workspace();
};

/*
  Bond wrapper for multiple COBS ports to the same destination.
  
  Multiple UARTs configured with the same COBS protocol (51/52/53)
  are combined into a single logical link with load-balanced TX.
  
  - Registers as a single HubPort with the hub
  - Distributes TX frames across member UARTs (round-robin)
  - Aggregates stats from all members
  - Link is up if ANY member is up
*/
class AP_Networking_SwitchPort_COBS : public AP_Networking_SwitchPort
{
public:
    static constexpr uint8_t MAX_BOND_MEMBERS = AP_NETWORKING_COBS_MAX_BOND_MEMBERS;
    
    AP_Networking_SwitchPort_COBS(AP_Networking_Switch *hub, uint8_t bond_id);
    ~AP_Networking_SwitchPort_COBS();
    
    CLASS_NO_COPY(AP_Networking_SwitchPort_COBS);
    
    // Add a COBS port to this bond (takes ownership)
    bool add_member(AP_Networking_COBS_Link *port);
    
    // Called by shared COBS thread
    void thread_update();
    
    // AP_Networking_SwitchPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override;
    const char *get_name() const override { return "COBS"; }
    bool is_link_up() const override;
    void update() override {}
    
    // Statistics (aggregated from all members)
    uint32_t get_rx_count() const;
    uint32_t get_tx_count() const;
    uint32_t get_crc_errors() const;
    uint32_t get_keepalive_rx() const;
    uint32_t get_keepalive_tx() const;
    
    uint8_t get_bond_id() const { return bond_id; }
    uint8_t get_num_members() const { return num_members; }
    
    // Called by member ports to route received frames
    // route_rx_frame: for single-mode frames (no sequencing)
    // route_rx_frame_sequenced: for bonded-mode frames (with reordering)
    void route_rx_frame(const uint8_t *frame, size_t len);
    void route_rx_frame_sequenced(uint16_t seq, const uint8_t *frame, size_t len);
    
private:
    AP_Networking_Switch *hub;
    uint8_t bond_id;
    AP_Networking_COBS_Link *members[MAX_BOND_MEMBERS];
    uint8_t num_members;
    uint16_t tx_seq;   // Sequence number for multi-UART bonds (only used when num_members > 1)
    
    // RX reorder buffer (only allocated when num_members > 1)
    static constexpr uint8_t REORDER_BUFFER_SIZE = 8;
    static constexpr size_t MAX_FRAME = AP_Networking_COBS_Link::MAX_FRAME;
    static constexpr uint32_t REORDER_TIMEOUT_MS = 5;   // Max time to wait for missing sequence
    uint16_t rx_seq_expected;
    uint32_t reorder_stall_start_us;  // When we first noticed a gap (0 = not stalled)
    struct ReorderEntry {
        uint8_t frame[MAX_FRAME];
        size_t len;
        uint16_t seq;
        bool valid;
    };
    ReorderEntry *reorder_buffer;  // Heap-allocated when num_members > 1
    
    void deliver_reordered();       // Deliver buffered frames in sequence
    void check_reorder_timeout();   // Skip missing frames if stalled too long
};

#endif // AP_NETWORKING_BACKEND_SWITCHPORT_COBS
