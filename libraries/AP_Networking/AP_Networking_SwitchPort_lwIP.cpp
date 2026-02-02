#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SWITCHPORT_LWIP

#include "AP_Networking_SwitchPort_lwIP.h"
#include "AP_Networking.h"
#include <GCS_MAVLink/GCS.h>

#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <lwip/tcpip.h>
#include <lwip/netifapi.h>
#if LWIP_DHCP
#include <lwip/dhcp.h>
#endif
#include <lwip/etharp.h>

extern const AP_HAL::HAL& hal;

AP_Networking_SwitchPort_lwIP *AP_Networking_SwitchPort_lwIP::singleton = nullptr;

#define LWIP_NETIF_MTU       1500
#define LWIP_LINK_POLL_INTERVAL_MS 100

AP_Networking_SwitchPort_lwIP::AP_Networking_SwitchPort_lwIP(AP_Networking_Switch *hub_in, AP_Networking &frontend_in)
    : hub(hub_in), frontend(frontend_in)
{
}

AP_Networking_SwitchPort_lwIP::~AP_Networking_SwitchPort_lwIP()
{
    delete inject_queue;
    delete thisif;
}

bool AP_Networking_SwitchPort_lwIP::init()
{
    if (singleton != nullptr) {
        return false; // already initialized
    }

    // Allocate inject queue (variable-size ring buffer)
    inject_queue = NEW_NOTHROW ByteBuffer(INJECT_QUEUE_SIZE);
    if (inject_queue == nullptr) {
        return false;
    }

    thisif = NEW_NOTHROW netif;
    if (thisif == nullptr) {
        delete inject_queue;
        inject_queue = nullptr;
        return false;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_SwitchPort_lwIP::thread, void),
                                      "lwip",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        delete thisif;
        thisif = nullptr;
        delete inject_queue;
        inject_queue = nullptr;
        return false;
    }

    // Only set singleton after successful initialization
    singleton = this;
    return true;
}

void AP_Networking_SwitchPort_lwIP::link_up_cb(void *p)
{
#if LWIP_DHCP
    auto *port = (AP_Networking_SwitchPort_lwIP *)p;
    if (port->frontend.get_dhcp_enabled()) {
        dhcp_start(port->thisif);
    }
#endif
}

void AP_Networking_SwitchPort_lwIP::link_down_cb(void *p)
{
#if LWIP_DHCP
    auto *port = (AP_Networking_SwitchPort_lwIP *)p;
    if (port->frontend.get_dhcp_enabled()) {
        dhcp_stop(port->thisif);
    }
#endif
}

/*
 * lwIP output callback - routes frames through hub
 */
int8_t AP_Networking_SwitchPort_lwIP::low_level_output(struct netif *netif, struct pbuf *p)
{
    (void)netif;

    if (singleton == nullptr || singleton->hub == nullptr) {
        static uint32_t last_warn_ms;
        uint32_t now = AP_HAL::millis();
        if (now - last_warn_ms > 1000) {
            last_warn_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: lwIP TX blocked: singleton=%p hub=%p", 
                          (void*)singleton, singleton ? (void*)singleton->hub : nullptr);
        }
        return ERR_IF;
    }

    // Check if any downstream port can receive - provides backpressure to TCP
    if (!singleton->hub->any_port_can_receive(singleton)) {
        // No downstream port can receive - tell lwIP to retry later
        // This applies backpressure up to the application layer
        return ERR_WOULDBLOCK;
    }

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE);
#endif

    // Calculate total length first to detect oversized frames
    size_t total_len = 0;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        total_len += q->len;
    }

    // Drop oversized frames rather than silently truncating
    if (total_len > sizeof(singleton->tx_buf)) {
#if ETH_PAD_SIZE
        pbuf_header(p, ETH_PAD_SIZE);
#endif
        return ERR_BUF;
    }

    // Flatten pbuf chain into contiguous buffer (member variable)
    size_t ofs = 0;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        memcpy(&singleton->tx_buf[ofs], q->payload, q->len);
        ofs += q->len;
    }

#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE);
#endif

    // Route through hub to other ports (Ethernet, COBS, etc.)
    singleton->hub->route_frame(singleton, singleton->tx_buf, ofs);
    singleton->tx_count++;

    return ERR_OK;
}

int8_t AP_Networking_SwitchPort_lwIP::ethernetif_init(struct netif *netif)
{
    netif->state = NULL;
    netif->name[0] = 'm';
    netif->name[1] = 's';
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* maximum transfer unit */
    netif->mtu = LWIP_NETIF_MTU;

    /* device capabilities */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

#if LWIP_IGMP
    netif->flags |= NETIF_FLAG_IGMP;
#endif

    return ERR_OK;
}

void AP_Networking_SwitchPort_lwIP::deliver_frame(const uint8_t *frame, size_t len)
{
    // Hub wants to deliver a frame to lwIP
    if (inject_queue == nullptr || frame == nullptr || len == 0 || len > INJECT_FRAME_MAX) {
        rx_errors++;
        return;
    }

    // Need space for 2-byte length prefix + frame data
    if (inject_queue->space() < len + 2) {
        // Queue full
        rx_errors++;
        return;
    }

    // Write length prefix (little-endian)
    uint16_t len16 = (uint16_t)len;
    inject_queue->write((const uint8_t *)&len16, 2);
    inject_queue->write(frame, len);
    rx_count++;

    inject_sem.signal();
}

bool AP_Networking_SwitchPort_lwIP::can_receive() const
{
    // Check if queue has space for a max-size frame
    if (inject_queue == nullptr) {
        return false;
    }
    return inject_queue->space() >= INJECT_FRAME_MAX + 2;
}

void AP_Networking_SwitchPort_lwIP::process_inject_queue()
{
    while (inject_queue->available() >= 2) {
        // Read length prefix (2 bytes, little-endian)
        uint16_t len;
        if (inject_queue->peekbytes((uint8_t *)&len, 2) != 2) {
            break;
        }
        
        // Validate length
        if (len == 0 || (size_t)len > INJECT_FRAME_MAX || inject_queue->available() < (size_t)len + 2) {
            // Corrupted or incomplete - shouldn't happen, clear queue
            inject_queue->clear();
            break;
        }
        
        // Advance past length prefix and read frame into rx buffer
        inject_queue->advance(2);
        inject_queue->read(rx_buf, len);

        struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
        if (p != nullptr) {
            // Use pbuf_take to properly handle chained pbufs
            pbuf_take(p, rx_buf, len);
            struct eth_hdr *ethhdr = (struct eth_hdr *)p->payload;
            uint16_t type = htons(ethhdr->type);

            if (type == ETHTYPE_IP || type == ETHTYPE_ARP) {
                err_t err = thisif->input(p, thisif);
                if (err != ERR_OK) {
                    static uint32_t last_err_ms;
                    uint32_t now = AP_HAL::millis();
                    if (now - last_err_ms > 1000) {
                        last_err_ms = now;
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: lwIP input err=%d type=0x%04x link=%u up=%u",
                                      (int)err, type, 
                                      (unsigned)netif_is_link_up(thisif),
                                      (unsigned)netif_is_up(thisif));
                    }
                    pbuf_free(p);
                }
            } else {
                pbuf_free(p);
            }
        }
    }
}

/*
  lwIP thread
*/
void AP_Networking_SwitchPort_lwIP::thread()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    /* Start tcpip thread */
    tcpip_init(NULL, NULL);

    /* Get MAC address */
    frontend.get_macaddr(thisif->hwaddr);

    struct {
        ip4_addr_t ip, gateway, netmask;
    } addr {};

#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
    if (!frontend.get_dhcp_enabled()) {
        addr.ip.addr = htonl(frontend.get_ip_param());
        addr.gateway.addr = htonl(frontend.get_gateway_param());
        addr.netmask.addr = htonl(frontend.get_netmask_param());
    }
#endif

    /* Add interface */
    auto result = netifapi_netif_add(thisif, &addr.ip, &addr.netmask, &addr.gateway, NULL, ethernetif_init, tcpip_input);
    if (result != ERR_OK) {
        AP_HAL::panic("Failed to initialise netif");
    }

    netifapi_netif_set_default(thisif);
    netifapi_netif_set_up(thisif);

    uint32_t last_link_check_ms = 0;

    while (true) {
        // Wait for inject signal or timeout for periodic link check
        inject_sem.wait(LWIP_LINK_POLL_INTERVAL_MS * 1000U);

        // Process any injected frames
        process_inject_queue();

        // Periodic link status check
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_link_check_ms >= LWIP_LINK_POLL_INTERVAL_MS) {
            last_link_check_ms = now_ms;

            // Link status based on hub ports - up if at least 2 ports up (lwIP + one other)
            uint8_t num_up = (hub != nullptr) ? hub->get_num_ports_link_up() : 0;
            bool current_link_status = num_up >= 2;
            bool netif_link = netif_is_link_up(thisif);
            if (current_link_status != netif_link) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: lwIP link %s (ports_up=%u netif_link=%u)",
                              current_link_status ? "UP" : "DOWN", num_up, (unsigned)netif_link);
                if (current_link_status) {
                    tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_up, thisif, 0);
                    tcpip_callback_with_block(link_up_cb, this, 0);
                } else {
                    tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_down, thisif, 0);
                    tcpip_callback_with_block(link_down_cb, this, 0);
                }
            }
        }
    }
}

void AP_Networking_SwitchPort_lwIP::update()
{
    if (thisif == nullptr) {
        return;
    }

    const uint32_t ip = ntohl(thisif->ip_addr.addr);
    const uint32_t nm = ntohl(thisif->netmask.addr);
    const uint32_t gw = ntohl(thisif->gw.addr);

    if (ip != activeSettings.ip ||
        nm != activeSettings.nm ||
        gw != activeSettings.gw) {
        activeSettings.ip = ip;
        activeSettings.gw = gw;
        activeSettings.nm = nm;
        activeSettings.last_change_ms = AP_HAL::millis();
    }
}

uint32_t AP_Networking_SwitchPort_lwIP::get_active_ip() const
{
    if (thisif == nullptr) {
        return 0;
    }
    return ntohl(thisif->ip_addr.addr);
}

uint32_t AP_Networking_SwitchPort_lwIP::get_active_netmask() const
{
    if (thisif == nullptr) {
        return 0;
    }
    return ntohl(thisif->netmask.addr);
}

uint32_t AP_Networking_SwitchPort_lwIP::get_active_gateway() const
{
    if (thisif == nullptr) {
        return 0;
    }
    return ntohl(thisif->gw.addr);
}

#endif // AP_NETWORKING_BACKEND_SWITCHPORT_LWIP
