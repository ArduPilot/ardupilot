#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP

#include "AP_Networking_Hub.h"
#include <AP_HAL/Semaphores.h>

struct netif;
class AP_Networking;

/*
  lwIP port for hub.
  When hub is enabled, this port owns lwIP:
  - Initializes lwIP and creates netif
  - Runs thread to process incoming frames from hub
  - Routes outgoing frames through hub
*/
class AP_Networking_Port_lwIP : public AP_Networking_HubPort
{
public:
    AP_Networking_Port_lwIP(AP_Networking_Hub *hub_in, AP_Networking &frontend_in);

    CLASS_NO_COPY(AP_Networking_Port_lwIP);

    bool init();
    void update() override;

    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override;
    const char *get_name() const override { return "lwIP"; }
    bool is_link_up() const override { return true; }  // lwIP virtual port is always "up"

    // Statistics
    uint32_t get_rx_count() const { return rx_count; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_errors() const { return rx_errors; }
    uint32_t get_tx_errors() const { return tx_errors; }

    // Access to netif for status reporting
    struct netif *get_netif() const { return thisif; }

private:
    AP_Networking_Hub *hub;
    AP_Networking &frontend;

    // Network interface
    struct netif *thisif = nullptr;

    // Inject queue for frames from hub
    static constexpr size_t INJECT_QUEUE_SIZE = 8;
    static constexpr size_t INJECT_FRAME_MAX = 1522;
    struct {
        uint8_t buf[INJECT_FRAME_MAX];
        size_t len;
    } inject_queue[INJECT_QUEUE_SIZE];
    volatile uint8_t inject_head = 0;
    volatile uint8_t inject_tail = 0;
    HAL_BinarySemaphore inject_sem;

    // Thread and callbacks
    void thread();
    static void link_up_cb(void *p);
    static void link_down_cb(void *p);
    static int8_t ethernetif_init(struct netif *netif);
    static int8_t low_level_output(struct netif *netif, struct pbuf *p);

    // Process injected frames
    void process_inject_queue();

    static AP_Networking_Port_lwIP *singleton;

    uint32_t rx_count = 0;
    uint32_t tx_count = 0;
    uint32_t rx_errors = 0;
    uint32_t tx_errors = 0;

    // Active settings tracking
    struct {
        uint32_t ip;
        uint32_t nm;
        uint32_t gw;
        uint32_t last_change_ms;
    } activeSettings;
};

#endif // AP_NETWORKING_BACKEND_HUB_PORT_LWIP
