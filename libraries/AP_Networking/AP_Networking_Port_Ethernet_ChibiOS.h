#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

#include "AP_Networking_Hub.h"
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Common/AP_Common.h>

/*
  ChibiOS Ethernet MAC port for hub.
  Owns the ChibiOS MAC driver and uses dedicated threads for RX/TX.
*/
class AP_Networking_Port_Ethernet_ChibiOS : public AP_Networking_HubPort
{
public:
    AP_Networking_Port_Ethernet_ChibiOS(AP_Networking_Hub *hub_in, const uint8_t *macaddr);

    CLASS_NO_COPY(AP_Networking_Port_Ethernet_ChibiOS);

    bool init();
    void update() override {}

    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override;
    const char *get_name() const override { return "Ethernet"; }
    bool is_link_up() const override;

    // Statistics
    uint32_t get_rx_count() const { return rx_count; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_errors() const { return rx_errors; }
    uint32_t get_tx_errors() const { return tx_errors; }

private:
    AP_Networking_Hub *hub;
    uint8_t macaddr[6];

    // Buffer allocation for MAC DMA
    static bool allocate_buffers();

    // Dedicated RX/TX threads
    void rx_thread();
    void tx_thread();
    
    // MAC initialization
    bool mac_init_and_start();
    void set_promiscuous_mode();
    bool process_one_rx_descriptor(uint32_t timeout_ms);
    void apply_errata_for_mac_KSZ9896C();

    // TX queue
    static constexpr size_t MAX_FRAME = 1522;
    static constexpr size_t TX_QUEUE_SIZE = 8 * MAX_FRAME + 64;
    ByteBuffer *tx_queue;
    HAL_Semaphore tx_mutex;
    HAL_BinarySemaphore tx_not_empty;

    // RX buffer
    uint8_t rx_framebuf[MAX_FRAME];

    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t rx_errors;
    uint32_t tx_errors;
};

#endif // AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
