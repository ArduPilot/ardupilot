/*
  USB CDC-ECM networking backend for ArduPilot (experimental spike).
 */

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_USB_ECM

#include "AP_Networking_USB_ECM.h"
#include <GCS_MAVLink/GCS.h>

#include <lwip/tcpip.h>
#include <lwip/netifapi.h>
#include <lwip/etharp.h>
#include <netif/ethernet.h>
#include <string.h>

#include <hal.h>
#include <AP_HAL_ChibiOS/hwdef/common/usbcfg.h>

extern const AP_HAL::HAL& hal;

#ifndef USB_ECM_NETIF_MTU
#define USB_ECM_NETIF_MTU 1500
#endif

/* Fixed MAC matching HAL_USB_ECM_MAC_STRING ("020000000014"). */
static const uint8_t USB_ECM_FIXED_MAC[6] = {
    0x02, 0x00, 0x00, 0x00, 0x00, 0x14
};

#if LWIP_ARP
static void ecm_garp_cb(void *arg)
{
    struct netif *n = (struct netif *)arg;
    if (n != nullptr) {
        etharp_gratuitous(n);
    }
}
#endif

bool AP_Networking_USB_ECM::init()
{
    thisif = NEW_NOTHROW netif;
    if (thisif == nullptr) {
        return false;
    }
    memset(thisif, 0, sizeof(*thisif));
    link_was_up = false;

    const bool ok = hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&AP_Networking_USB_ECM::thread, void),
        "usb_ecm",
        2048,
        AP_HAL::Scheduler::PRIORITY_NET,
        0);
    if (!ok) {
        return false;
    }
    return true;
}

void AP_Networking_USB_ECM::update()
{
    /* Quiet in normal use — no periodic STATUSTEXT spam.
     * Keep link up and occasional GARP for hosts that drop ARP. */
    static uint32_t last_ms;
    const uint32_t now = AP_HAL::millis();
    if (now - last_ms < 2000) {
        return;
    }
    last_ms = now;
    if (!usb_ecm_link_up() || thisif == nullptr) {
        return;
    }
    if (!netif_is_link_up(thisif)) {
        netifapi_netif_set_link_up(thisif);
    }
#if LWIP_ARP
    static uint8_t garp_div;
    if ((++garp_div % 15) == 0) {
        tcpip_callback_with_block((tcpip_callback_fn)ecm_garp_cb, thisif, 0);
    }
#endif
}


int8_t AP_Networking_USB_ECM::low_level_output(struct netif *netif, struct pbuf *p)
{
    (void)netif;
    if (p == nullptr || !usb_ecm_link_up()) {
        return ERR_IF;
    }

    /* static: avoid Wframe-larger-than stack limit */
    static uint8_t frame[USB_ECM_MAX_FRAME];
    const uint16_t total = p->tot_len;
    if (total == 0 || total > USB_ECM_MAX_FRAME) {
        return ERR_BUF;
    }

    uint16_t off = 0;
    for (struct pbuf *q = p; q != nullptr; q = q->next) {
        if (off + q->len > total) {
            return ERR_BUF;
        }
        memcpy(&frame[off], q->payload, q->len);
        off = (uint16_t)(off + q->len);
    }

    /* Ethernet minimum 60 bytes without FCS */
    uint16_t send_len = total;
    if (send_len < 60) {
        memset(&frame[send_len], 0, 60 - send_len);
        send_len = 60;
    }

    if (!usb_ecm_send_frame(frame, send_len)) {
        return ERR_IF;
    }
    return ERR_OK;
}

int8_t AP_Networking_USB_ECM::ethernetif_init(struct netif *netif)
{
    netif->name[0] = 'u';
    netif->name[1] = 'e';
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    memcpy(netif->hwaddr, USB_ECM_FIXED_MAC, 6);
    netif->mtu = USB_ECM_NETIF_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
    return ERR_OK;
}

void AP_Networking_USB_ECM::thread()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    tcpip_init(NULL, NULL);

    ip4_addr_t ip;
    ip4_addr_t gateway;
    ip4_addr_t netmask;
    ip.addr = htonl(frontend.get_ip_param());
    gateway.addr = htonl(frontend.get_gateway_param());
    netmask.addr = htonl(frontend.get_netmask_param());

    const err_t result = netifapi_netif_add(
        thisif,
        &ip,
        &netmask,
        &gateway,
        this,
        ethernetif_init,
        tcpip_input);
    if (result != ERR_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: ECM netif add failed");
        return;
    }

    netifapi_netif_set_default(thisif);
    netifapi_netif_set_up(thisif);

    activeSettings.ip = frontend.get_ip_param();
    activeSettings.gw = frontend.get_gateway_param();
    activeSettings.nm = frontend.get_netmask_param();
    memcpy(activeSettings.macaddr, USB_ECM_FIXED_MAC, 6);
    activeSettings.last_change_ms = AP_HAL::millis();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: USB-ECM ready (static)");
    if (usb_ecm_link_up()) {
        netifapi_netif_set_link_up(thisif);
#if LWIP_ARP
        /* Announce our IP/MAC so the host can fill ARP without a prior request */
        tcpip_callback_with_block((tcpip_callback_fn)ecm_garp_cb, thisif, 0);
#endif
        link_was_up = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: USB-ECM link up");
    }

    static uint8_t rxbuf[USB_ECM_MAX_FRAME];

    while (true) {
        const bool up = usb_ecm_link_up();
        if (up != link_was_up) {
            link_was_up = up;
            if (up) {
                tcpip_callback_with_block(
                    (tcpip_callback_fn)netif_set_link_up, thisif, 0);
#if LWIP_ARP
                tcpip_callback_with_block(
                    (tcpip_callback_fn)ecm_garp_cb, thisif, 0);
#endif
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: USB-ECM link up");
            } else {
                tcpip_callback_with_block(
                    (tcpip_callback_fn)netif_set_link_down, thisif, 0);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: USB-ECM link down");
            }
        }

        bool got = false;
        while (true) {
            const uint16_t n = usb_ecm_recv_frame(rxbuf, sizeof(rxbuf));
            if (n == 0) {
                break;
            }
            got = true;
            struct pbuf *p = pbuf_alloc(PBUF_RAW, n, PBUF_POOL);
            if (p == nullptr) {
                continue;
            }
            if (pbuf_take(p, rxbuf, n) != ERR_OK) {
                pbuf_free(p);
                continue;
            }
            if (thisif->input(p, thisif) != ERR_OK) {
                pbuf_free(p);
            }
        }

        if (!got) {
            hal.scheduler->delay_microseconds(500);
        }
    }
}

#endif // AP_NETWORKING_BACKEND_USB_ECM
