#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET

#include "AP_Networking_Switch.h"
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Common/AP_Common.h>

/*
  ChibiOS Ethernet MAC port for hub.
  Owns the ChibiOS MAC driver and uses dedicated threads for RX/TX.
*/
class AP_Networking_SwitchPort_Ethernet_ChibiOS : public AP_Networking_SwitchPort
{
public:
    friend class BL_Network;
    AP_Networking_SwitchPort_Ethernet_ChibiOS(AP_Networking_Switch *hub_in, const uint8_t *macaddr);

    CLASS_NO_COPY(AP_Networking_SwitchPort_Ethernet_ChibiOS);

    bool init();
    void update() override;

    // AP_Networking_SwitchPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override;
    const char *get_name() const override { return "Ethernet"; }
    bool is_link_up() const override;

    // Statistics
    uint32_t get_rx_count() const { return rx_count; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_errors() const { return rx_errors; }
    uint32_t get_tx_errors() const { return tx_errors; }

#if AP_NETWORKING_CAPTURE_ENABLED
    // Packet capture to pcap file
    void start_capture();
    void stop_capture();
#endif

private:
    AP_Networking_Switch *hub;
    uint8_t macaddr[6];

    // Buffer allocation for MAC DMA (delegates to shared implementation)
    static bool allocate_buffers();

    // Dedicated RX/TX threads
    void rx_thread();
    void tx_thread();
    
    // MAC initialization
    bool mac_init_and_start();
    void set_promiscuous_mode(bool enable);
    bool process_one_rx_descriptor(uint32_t timeout_ms);
    void apply_errata_for_mac_KSZ9896C();

    // Promiscuous mode state tracking
    bool promiscuous_enabled;

    // TX queue
    static constexpr size_t MAX_FRAME = 1522;
    static constexpr size_t TX_QUEUE_SIZE = 8 * MAX_FRAME + 64;
    ByteBuffer *tx_queue;
    HAL_Semaphore tx_mutex;
    HAL_BinarySemaphore tx_not_empty;

    // RX buffer (used by rx_thread)
    uint8_t rx_framebuf[MAX_FRAME];

    // TX buffer (used by tx_thread)
    uint8_t tx_framebuf[MAX_FRAME];

    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t rx_errors;
    uint32_t tx_errors;

#if AP_NETWORKING_CAPTURE_ENABLED
    void capture_frame(const uint8_t *frame, size_t len);
    struct {
        HAL_Semaphore sem;
        int fd = -1;
    } capture;
#endif
};

#endif // AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET
