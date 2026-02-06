#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SWITCHPORT_LWIP

#include "AP_Networking_Switch.h"
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/utility/RingBuffer.h>
#if AP_NETWORKING_CAPTURE_ENABLED
#include "AP_Networking_Capture.h"
#endif

struct netif;
class AP_Networking;

/*
  lwIP port for hub.
  When hub is enabled, this port owns lwIP:
  - Initializes lwIP and creates netif
  - Runs thread to process incoming frames from hub
  - Routes outgoing frames through hub
*/
class AP_Networking_SwitchPort_lwIP : public AP_Networking_SwitchPort
{
public:
    AP_Networking_SwitchPort_lwIP(AP_Networking_Switch *hub_in, AP_Networking &frontend_in);
    ~AP_Networking_SwitchPort_lwIP();

    CLASS_NO_COPY(AP_Networking_SwitchPort_lwIP);

    bool init();
    void update() override;

    // AP_Networking_SwitchPort interface
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

    // Get active network settings (host byte order)
    uint32_t get_active_ip() const;
    uint32_t get_active_netmask() const;
    uint32_t get_active_gateway() const;

private:
    AP_Networking_Switch *hub;
    AP_Networking &frontend;

    // Network interface
    struct netif *thisif;

    // Inject queue for frames from hub (variable-size ring buffer)
    // Each frame stored as: [len:2 bytes][frame data:len bytes]
    // 12KB pool matches old 8-slot fixed capacity for max-size frames
    static constexpr size_t INJECT_FRAME_MAX = 1522;
    static constexpr size_t INJECT_QUEUE_SIZE = 12 * 1024;  // 12KB pool
    ByteBuffer *inject_queue;
    HAL_BinarySemaphore inject_sem;

    // TX buffer for flattening pbuf chain (used by low_level_output)
    uint8_t tx_buf[INJECT_FRAME_MAX];

    // RX buffer for reading from inject queue (used by process_inject_queue)
    uint8_t rx_buf[INJECT_FRAME_MAX];

    // Thread and callbacks
    void thread();
    static void link_up_cb(void *p);
    static void link_down_cb(void *p);
    static int8_t ethernetif_init(struct netif *netif);
    static int8_t low_level_output(struct netif *netif, struct pbuf *p);

    // Process injected frames
    void process_inject_queue();

    static AP_Networking_SwitchPort_lwIP *singleton;

#if AP_NETWORKING_CAPTURE_ENABLED
public:
    AP_Networking_Capture capture;
private:
#endif

    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t rx_errors;
    uint32_t tx_errors;

    // Active settings tracking
    struct {
        uint32_t ip;
        uint32_t nm;
        uint32_t gw;
        uint32_t last_change_ms;
    } activeSettings;
};

#endif // AP_NETWORKING_BACKEND_SWITCHPORT_LWIP
