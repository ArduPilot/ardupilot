#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_COBS

#include "AP_Networking_Hub.h"
#include "AP_Networking_COBS.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/UARTDriver.h>

/*
  Unified COBS port with automatic ganging support.
  
  Features:
  - Starts immediately with one UART (no discovery delay)
  - Dynamically detects and absorbs additional UARTs connected to same remote
  - Automatically enables frame striping when multiple UARTs present
  - Supports hotplug - detects device changes and splits/merges ports
  - Auto-disables bad UARTs based on relative loss stats
  
  Protocol:
  - Keepalive: "KA" + device_id[6] + rx_good[2] + CRC32
  - Single mode: ethernet_frame + CRC32
  - Ganged mode: seq[2] + ethernet_frame + CRC32(data + "GANG")
  
  Frame type detection via CRC - no negotiation needed.
*/
class AP_Networking_Port_COBS : public AP_Networking_HubPort
{
public:
    static constexpr uint8_t MAX_UARTS = 4;
    
    AP_Networking_Port_COBS(AP_Networking_Hub *hub,
                            AP_HAL::UARTDriver *uart,
                            uint32_t baud_rate,
                            const uint8_t local_device_id[6]);
    ~AP_Networking_Port_COBS();
    
    CLASS_NO_COPY(AP_Networking_Port_COBS);
    
    bool init();
    void update() override;
    
    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override;
    const char *get_name() const override { return "COBS"; }
    bool is_link_up() const override;
    
    // Gang management (called by hub)
    bool add_uart(AP_HAL::UARTDriver *uart, uint32_t baud);
    bool extract_uart(uint8_t idx, AP_HAL::UARTDriver *&uart_out, uint32_t &baud_out);
    
    // Discovery info (for hub to match ports)
    bool get_remote_device_id(uint8_t id_out[6]) const;
    bool has_remote_device_id() const { return remote_id_known; }
    
    // Status
    uint8_t get_num_uarts() const { return num_uarts; }
    bool is_ganged() const { return state == State::GANGED; }
    AP_HAL::UARTDriver *get_uart(uint8_t idx) const;
    uint32_t get_baud(uint8_t idx) const;
    
    // Statistics
    uint32_t get_rx_count() const { return rx_count; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_errors() const { return rx_errors; }
    uint32_t get_crc_errors() const { return crc_errors; }
    uint32_t get_reorder_count() const { return reorder_count; }
    uint32_t get_reorder_timeout_count() const { return reorder_timeout_count; }
    uint32_t get_keepalive_tx() const { return ka_tx_count; }
    uint32_t get_keepalive_rx() const { return ka_rx_count; }

private:
    AP_Networking_Hub *hub;
    uint8_t local_device_id[6];
    
    enum class State : uint8_t {
        SINGLE,     // 1 UART, non-sequenced frames
        GANGED      // 2+ UARTs, sequenced frames, earliest-completion TX, reorder RX
    };
    State state = State::SINGLE;
    
    // Per-UART state
    struct UARTState {
        AP_HAL::UARTDriver *uart;
        uint32_t baud;
        AP_Networking_COBS::Decoder decoder;
        
        // Link status
        uint8_t seen_device_id[6];
        bool device_id_valid;
        uint32_t last_rx_ms;
        uint32_t last_keepalive_tx_ms;
        
        // Stats for link quality (counters wrap, differences used)
        uint16_t tx_count;              // Data frames we sent on this UART
        uint16_t tx_count_at_last_ka;   // tx_count when last keepalive received
        uint16_t last_remote_rx_good;   // rx_good from last keepalive
        uint16_t lost_since_last_ka;    // Frames lost in last measurement period
        
        // Rolling loss rate (0-255 scaled, where 255 = 100% loss)
        uint8_t loss_rate_u8;
        
        // Health state
        enum class Health : uint8_t {
            GOOD,       // Normal operation, 500ms keepalives
            DISABLED    // Not used for data TX, 50ms keepalives for recovery
        };
        Health health;
        
        uint16_t rx_good;  // Data frames we received on this UART (for keepalive stats)
    };
    UARTState uarts[MAX_UARTS];
    uint8_t num_uarts = 0;
    
    // Remote device (learned from keepalives)
    uint8_t remote_device_id[6];
    bool remote_id_known = false;
    
    // TX state (ganged mode)
    uint16_t tx_seq = 0;
    HAL_Semaphore tx_sem;
    
    // TX buffers
    static constexpr size_t MAX_FRAME = 1522;
    static constexpr size_t MIN_ETH_FRAME = 14;  // Minimum = Ethernet header only
    uint8_t tx_encode_buffer[1600];
    uint8_t tx_input_buffer[MAX_FRAME + 6];  // seq(2) + frame + CRC(4)
    
    // RX reorder buffer (ganged mode) - lazily allocated, sized to num_uarts
    static constexpr uint8_t MIN_REORDER_SLOTS = 4;
    struct ReorderSlot {
        uint8_t frame[MAX_FRAME];
        size_t len;
        uint16_t seq;
        bool occupied;
    };
    ReorderSlot *reorder_buffer = nullptr;
    uint8_t reorder_window = 0;
    uint8_t reorder_mask = 0;
    uint16_t rx_expected_seq = 0;
    HAL_Semaphore rx_sem;
    
    // RX buffer
    uint8_t rx_frame_buffer[MAX_FRAME + 6];
    
    // Track last received data frame mode for CRC check optimization
    bool last_rx_was_ganged = false;
    
    // Statistics
    uint32_t rx_count = 0;
    uint32_t tx_count = 0;
    uint32_t rx_errors = 0;
    uint32_t crc_errors = 0;
    uint32_t reorder_count = 0;
    uint32_t reorder_timeout_count = 0;
    uint32_t ka_tx_count = 0;
    uint32_t ka_rx_count = 0;
    
    // Constants
    static constexpr uint16_t KEEPALIVE_INTERVAL_MS = 500;
    static constexpr uint16_t KEEPALIVE_FLOOD_INTERVAL_MS = 50;
    static constexpr uint16_t KEEPALIVE_TIMEOUT_MS = 2000;
    static constexpr uint8_t LOSS_RATE_DISABLE_PCT = 15;
    static constexpr uint8_t LOSS_RATE_DISABLE_MARGIN = 3;
    static constexpr uint8_t LOSS_RATE_ENABLE_PCT = 5;
    static constexpr uint8_t LOSS_RATE_ENABLE_RELATIVE_PCT = 10;
    static constexpr uint16_t MIN_SAMPLE_SIZE = 20;
    
    // TX buffer size (for completion time calculation)
    static constexpr uint32_t TX_BUFFER_SIZE = 16384;
    
    // Methods
    void thread();
    void process_rx(uint8_t uart_idx);
    bool handle_decoded_frame(uint8_t uart_idx, const uint8_t *data, size_t len);
    bool handle_keepalive(uint8_t uart_idx, const uint8_t *data);
    bool handle_data_frame_single(uint8_t uart_idx, const uint8_t *data, size_t len);
    bool handle_data_frame_ganged(uint8_t uart_idx, uint16_t seq, const uint8_t *data, size_t len);
    void send_keepalive(uint8_t uart_idx);
    void check_keepalive_tx();
    bool ensure_reorder_buffer();
    void deliver_reordered_frames();
    void update_uart_health();
    void check_uart_timeouts();
    uint8_t select_tx_uart(size_t frame_size);
    uint8_t get_best_loss_rate() const;
};

#endif // AP_NETWORKING_BACKEND_HUB_PORT_COBS
