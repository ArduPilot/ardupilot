
#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_PPP

#include "AP_Networking_PPP.h"
#include <GCS_MAVLink/GCS.h>

#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <netif/ppp/ppp_opts.h>
#include <netif/ppp/pppapi.h>
#include <netif/ppp/pppos.h>
#include <lwip/tcpip.h>
#include <stdio.h>


extern const AP_HAL::HAL& hal;

#if LWIP_TCPIP_CORE_LOCKING
#define LWIP_TCPIP_LOCK() sys_lock_tcpip_core()
#define LWIP_TCPIP_UNLOCK() sys_unlock_tcpip_core()
#else
#define LWIP_TCPIP_LOCK()
#define LWIP_TCPIP_UNLOCK()
#endif

#define PPP_DEBUG_TX 0
#define PPP_DEBUG_RX 0

// timeout for PPP link, if no packets in this time then restart the link
#ifndef PPP_LINK_TIMEOUT_MS
#define PPP_LINK_TIMEOUT_MS 5000U
#endif

/*
  output some data to the uart
 */
uint32_t AP_Networking_PPP::ppp_output_cb(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx)
{
    auto &driver = *(AP_Networking_PPP *)ctx;
    LWIP_UNUSED_ARG(pcb);
    uint32_t remaining = len;
    const uint8_t *ptr = (const uint8_t *)data;

    if (pcb->phase == PPP_PHASE_TERMINATE) {
        // don't output while terminating
        return 0;
    }

#if PPP_DEBUG_TX
    bool flag_end = false;
    if (ptr[len-1] == 0x7E) {
        flag_end = true;
        remaining--;
    }
    if (ptr[0] == 0x7E) {
        // send byte size
        if (pkt_size > 0) {
            printf("PPP: tx[%lu] %u\n", tx_index++,  pkt_size);
        }
        // dump the packet
        if (!(tx_index % 10)) {
            for (uint32_t i = 0; i < pkt_size; i++) {
                printf(" %02X", tx_bytes[i]);
            }
            printf("\n");
        }
        pkt_size = 0;
    }
#endif
    if (driver.uart->txspace() < remaining) {
        /*
          unfortunately there is nothing we can do if we can't fit the
          data in the uart transmit buffer. We can't block here as
          this function is called with the TCPIP lock held, so any
          blocking can block other threads

          to prevent the link going down we need to lie about sending
          this frame
         */
        return remaining;
    }
    auto ret = driver.uart->write(ptr, remaining);

#if PPP_DEBUG_TX
    memcpy(&tx_bytes[pkt_size], data, len);
    pkt_size += len;
    if (flag_end) {
        driver.uart->write(0x7E);
        printf("PPP: tx[%lu] %u\n", tx_index++,  pkt_size);
        // dump the packet
        if (!(tx_index % 10)) {
            for (uint32_t i = 0; i < pkt_size; i++) {
                printf(" %02X", tx_bytes[i]);
            }
            printf("\n");
        }
        pkt_size = 0;
    }
#endif
    return ret;
}

/*
  callback on link status change
 */
void AP_Networking_PPP::ppp_status_callback(struct ppp_pcb_s *pcb, int code, void *ctx)
{
    auto &driver = *(AP_Networking_PPP *)ctx;
    struct netif *pppif = ppp_netif(pcb);

    switch (code) {
    case PPPERR_NONE:
        // got new addresses for the link
#if AP_NETWORKING_PPP_GATEWAY_ENABLED
        if (driver.frontend.option_is_set(AP_Networking::OPTION::PPP_ETHERNET_GATEWAY)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP: got addresses");
        } else
#endif
        {
            driver.activeSettings.ip = ntohl(netif_ip4_addr(pppif)->addr);
            driver.activeSettings.gw = ntohl(netif_ip4_gw(pppif)->addr);
            driver.activeSettings.nm = ntohl(netif_ip4_netmask(pppif)->addr);
            driver.activeSettings.last_change_ms = AP_HAL::millis();
        }
        break;

    case PPPERR_OPEN:
    case PPPERR_CONNECT:
    case PPPERR_PEERDEAD:
    case PPPERR_IDLETIMEOUT:
    case PPPERR_CONNECTTIME:
        driver.need_restart = true;
        break;

    case PPPERR_USER:
        // this happens on reconnect
        break;

    default:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP: error %d", code);
        break;
    }
}


/*
  initialise PPP network backend using LWIP
 */
bool AP_Networking_PPP::init()
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_PPP, 0);
    if (uart == nullptr) {
        return false;
    }

    pppif = NEW_NOTHROW netif;
    if (pppif == nullptr) {
        return false;
    }

    const bool ethernet_gateway = frontend.option_is_set(AP_Networking::OPTION::PPP_ETHERNET_GATEWAY);
    if (!ethernet_gateway) {
        // initialise TCP/IP thread
        LWIP_TCPIP_LOCK();
        tcpip_init(NULL, NULL);
        LWIP_TCPIP_UNLOCK();
    }

    hal.scheduler->delay(100);
    
    // create ppp connection
    LWIP_TCPIP_LOCK();

    ppp = pppos_create(pppif, ppp_output_cb, ppp_status_callback, this);
    if (ppp == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP: failed to create link");
        return false;
    }
    LWIP_TCPIP_UNLOCK();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP: started");
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_PPP::ppp_loop, void),
                                 "ppp",
                                 2048, AP_HAL::Scheduler::PRIORITY_NET, 0);

    return true;
}

#if AP_NETWORKING_PPP_GATEWAY_ENABLED
/*
  promote a network interface to the front of the list
 */
static void netif_promote(struct netif *iface)
{
    extern struct netif *netif_list;
    if (netif_list == nullptr || netif_list == iface) {
        // already first in the list
        return;
    }
    for (struct netif *prev = netif_list;
         prev != nullptr;
         prev = prev->next) {
        if (prev->next == iface) {
            // found it, move it to the front
            prev->next = iface->next;
            iface->next = netif_list;
            netif_list = iface;
            break;
        }
    }
}
#endif

/*
  main loop for PPP
 */
void AP_Networking_PPP::ppp_loop(void)
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }
    const bool ppp_gateway = frontend.option_is_set(AP_Networking::OPTION::PPP_ETHERNET_GATEWAY);
    if (ppp_gateway) {
        // wait for the ethernet interface to be up
        AP::network().startup_wait();
    }

    // ensure this thread owns the uart
    uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_PPP, 0));
    uart->set_unbuffered_writes(true);

    while (true) {
        uint8_t buf[1024];

        // connect and set as default route
        LWIP_TCPIP_LOCK();

#if AP_NETWORKING_PPP_GATEWAY_ENABLED
        if (ppp_gateway) {
            /*
              when bridging setup the ppp interface with the same IP
              as the ethernet interface, and set the remote IP address
              as the local address + 1
             */
            ip4_addr_t our_ip, his_ip;
            const uint32_t ip = frontend.get_ip_active();
            uint32_t rem_ip = frontend.param.remote_ppp_ip.get_uint32();
            if (rem_ip == 0) {
                // use ethernet IP +1 by default
                rem_ip = ip+1;
            }
            our_ip.addr = htonl(ip);
            his_ip.addr = htonl(rem_ip);
            ppp_set_ipcp_ouraddr(ppp, &our_ip);
            ppp_set_ipcp_hisaddr(ppp, &his_ip);
            if (netif_list != nullptr) {
                const uint32_t nmask = frontend.get_netmask_param();
                if ((ip & nmask) == (rem_ip & nmask)) {
                    // remote PPP IP is on the same subnet as the
                    // local ethernet IP, so enable proxyarp to avoid
                    // users having to setup routes in all devices
                    netif_set_proxyarp(netif_list, &his_ip);
                }
            }
        }

        // connect to the remote end
        ppp_connect(ppp, 0);

        if (ppp_gateway) {
            /*
              when we are setup as a PPP gateway we want the pppif to be
              first in the list so routing works if it is on the same
              subnet
            */
            netif_promote(pppif);
        } else {
            netif_set_default(pppif);
        }
#else
        // normal PPP link, connect to the remote end and set as the
        // default route
        ppp_connect(ppp, 0);
        netif_set_default(pppif);
#endif // AP_NETWORKING_PPP_GATEWAY_ENABLED

        LWIP_TCPIP_UNLOCK();

        need_restart = false;

        uint32_t last_read_ms = AP_HAL::millis();

        while (!need_restart) {
            const uint32_t now_ms = AP_HAL::millis();
            auto n = uart->read(buf, sizeof(buf));
            if (n > 0) {
                LWIP_TCPIP_LOCK();
                pppos_input(ppp, buf, n);
                LWIP_TCPIP_UNLOCK();
                if (ppp->if4_up) {
                    // only consider the link active if IPv4 is up
                    // so we will restart PPP negotiation if we
                    // are out of sync with the other side
                    last_read_ms = now_ms;
                }
#if PPP_LINK_TIMEOUT_MS
            } else if (now_ms - last_read_ms > PPP_LINK_TIMEOUT_MS) {
                break;
#endif
            } else {
                hal.scheduler->delay_microseconds(200);
            }
            if (ppp->err_code == PPPERR_PEERDEAD ||
                ppp->phase == PPP_PHASE_TERMINATE) {
                // reached LCP echo failure threshold LCP_MAXECHOFAILS
                break;
            }
#if PPP_DEBUG_RX
            auto pppos = (pppos_pcb *)ppp->link_ctx_cb;
            for (uint32_t i = 0; i < n; i++) {
                if (buf[i] == 0x7E && last_ppp_frame_size != 1) {
                    // dump the packet
                    if (pppos->bad_pkt) {
                        printf("PPP: rx[%lu] %u\n", rx_index, last_ppp_frame_size);
                        for (uint32_t j = 0; j < last_ppp_frame_size; j++) {
                            printf("0x%02X,", rx_bytes[j]);
                        }
                        printf("\n");
                        hal.scheduler->delay(1);
                    }
                    rx_index++;
                    last_ppp_frame_size = 0;
                }
                rx_bytes[last_ppp_frame_size++] = buf[i];
            }
#endif
        }

        // close with carrier loss, tear down the interface and
        // re-create to restart link
        LWIP_TCPIP_LOCK();
        ppp->phase = 0;
        ppp_close(ppp, 1);
        LWIP_TCPIP_UNLOCK();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP: reconnecting");
    }
}

#endif // AP_NETWORKING_BACKEND_PPP
