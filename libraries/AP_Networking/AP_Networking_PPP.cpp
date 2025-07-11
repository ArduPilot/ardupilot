
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

// PPP protocol
#ifndef PPP_BUFSIZE_RX
#define PPP_BUFSIZE_RX 4096
#endif
#ifndef PPP_BUFSIZE_TX
#define PPP_BUFSIZE_TX 8192
#endif

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
#define PPP_LINK_TIMEOUT_MS 15000U
#endif

/*
  output some data to the uart
 */
uint32_t AP_Networking_PPP::ppp_output_cb(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx)
{
    auto &driver = *(AP_Networking_PPP::PPP_Instance *)ctx;
    LWIP_UNUSED_ARG(pcb);
    uint32_t remaining = len;
    const uint8_t *ptr = (const uint8_t *)data;

    if (pcb->phase == PPP_PHASE_TERMINATE) {
        // don't output while terminating
        return 0;
    }

    if (driver.uart->txspace() < remaining) {
        /*
          if we can't send the whole frame then don't send any of it. This
          minimises issues with the PPP state machine
         */
        return 0;
    }

    return driver.uart->write(ptr, remaining);
}

/*
  callback on link status change
 */
void AP_Networking_PPP::ppp_status_callback(struct ppp_pcb_s *pcb, int code, void *ctx)
{
    auto &driver = *(AP_Networking_PPP::PPP_Instance *)ctx;
    struct netif *pppif = ppp_netif(pcb);

    switch (code) {
    case PPPERR_NONE:
        // got new addresses for the link
#if AP_NETWORKING_PPP_GATEWAY_ENABLED
        if (driver.backend->frontend.option_is_set(AP_Networking::OPTION::PPP_ETHERNET_GATEWAY)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: got addresses", unsigned(driver.idx));
        } else
#endif
        {
            driver.backend->activeSettings.ip = ntohl(netif_ip4_addr(pppif)->addr);
            driver.backend->activeSettings.gw = ntohl(netif_ip4_gw(pppif)->addr);
            driver.backend->activeSettings.nm = ntohl(netif_ip4_netmask(pppif)->addr);
            driver.backend->activeSettings.last_change_ms = AP_HAL::millis();
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: error %d", unsigned(driver.idx), code);
        break;
    }
}


/*
  initialise PPP network backend using LWIP
 */
bool AP_Networking_PPP::init()
{
    auto &sm = AP::serialmanager();
    bool need_thread = false;

    for (uint8_t i=0; i<AP_NETWORKING_PPP_NUM_INTERFACES; i++) {
        auto &inst = iface[i];

        inst.backend = this;
        inst.idx = i;

        auto *uart = sm.find_serial(AP_SerialManager::SerialProtocol_PPP, i);
        if (uart == nullptr) {
            break;
        }

        inst.pppif = NEW_NOTHROW netif;
        if (inst.pppif == nullptr) {
            break;
        }

        const bool ethernet_gateway = frontend.option_is_set(AP_Networking::OPTION::PPP_ETHERNET_GATEWAY);
        if (!ethernet_gateway && !need_thread) {
            // initialise TCP/IP thread
            LWIP_TCPIP_LOCK();
            tcpip_init(NULL, NULL);
            LWIP_TCPIP_UNLOCK();
        }

        hal.scheduler->delay(100);
    
        // create ppp connection
        LWIP_TCPIP_LOCK();
        inst.ppp = pppos_create(inst.pppif, ppp_output_cb, ppp_status_callback, &inst);
        if (inst.ppp == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: failed to create link", unsigned(i));
            break;
        }
        LWIP_TCPIP_UNLOCK();

        inst.uart = uart;

        need_thread = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: started", unsigned(i));
    }

    if (need_thread) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_PPP::ppp_loop, void),
                                     "ppp",
                                     2048, AP_HAL::Scheduler::PRIORITY_NET, 0);
        return true;
    }
    return false;
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

    // ensure this thread owns the uarts
    for (uint8_t i=0; i<AP_NETWORKING_PPP_NUM_INTERFACES; i++) {
        auto &inst = iface[i];
        if (inst.uart == nullptr) {
            continue;
        }
        // use a larger buffer space for TX to allow for large downloads (eg. MAVFTP)
        inst.uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_PPP, i), PPP_BUFSIZE_RX, PPP_BUFSIZE_TX);
        inst.uart->set_unbuffered_writes(true);

        restart_instance(i);
    }

    while (true) {
        bool read_data = false;

        for (auto &inst : iface) {
            if (inst.uart != nullptr) {
                read_data |= update_instance(inst.idx);
            }
        }
        if (!read_data) {
            // ensure we give up some time
            hal.scheduler->delay_microseconds(200);
        }
    }
}

/*
  restart link on an instance
 */
void AP_Networking_PPP::restart_instance(const uint8_t idx)
{
    auto &inst = iface[idx];
    // connect and set as default route
    LWIP_TCPIP_LOCK();

#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    // assume PPP/ethernet gateway is first instance only
    const bool ppp_gateway = idx == 0 && frontend.option_is_set(AP_Networking::OPTION::PPP_ETHERNET_GATEWAY);
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
        ppp_set_ipcp_ouraddr(inst.ppp, &our_ip);
        ppp_set_ipcp_hisaddr(inst.ppp, &his_ip);
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
    ppp_connect(inst.ppp, 0);

    if (idx == 0) {
        if (ppp_gateway) {
            /*
              when we are setup as a PPP gateway we want the pppif to be
              first in the list so routing works if it is on the same
              subnet
            */
            netif_promote(inst.pppif);
        } else {
            netif_set_default(inst.pppif);
        }
    }
#else
    // normal PPP link, connect to the remote end and set as the
    // default route
    ppp_connect(inst.ppp, 0);
    if (idx == 0) {
        netif_set_default(inst.pppif);
    }
#endif // AP_NETWORKING_PPP_GATEWAY_ENABLED
    LWIP_TCPIP_UNLOCK();

    inst.last_read_ms = AP_HAL::millis();
}

/*
  update an instance, return true if we have read some data from the uart
 */
bool AP_Networking_PPP::update_instance(const uint8_t idx)
{
    auto &inst = iface[idx];
    uint8_t buf[1024];

    if (inst.need_restart) {
        inst.need_restart = false;

        LWIP_TCPIP_LOCK();
        inst.ppp->phase = 0;
        ppp_close(inst.ppp, 1);
        LWIP_TCPIP_UNLOCK();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: reconnecting", unsigned(idx));

        restart_instance(idx);
    }

    const uint32_t now_ms = AP_HAL::millis();
    auto n = inst.uart->read(buf, sizeof(buf));
    if (n > 0) {
        LWIP_TCPIP_LOCK();
        pppos_input(inst.ppp, buf, n);
        LWIP_TCPIP_UNLOCK();
        if (inst.ppp->if4_up) {
            // only consider the link active if IPv4 is up
            // so we will restart PPP negotiation if we
            // are out of sync with the other side
            inst.last_read_ms = now_ms;
        }
#if PPP_LINK_TIMEOUT_MS
    } else if (!frontend.option_is_set(AP_Networking::OPTION::PPP_TIMEOUT_DISABLE) &&
               now_ms - inst.last_read_ms > PPP_LINK_TIMEOUT_MS) {
        inst.need_restart = true;
    }
#endif

    if (inst.ppp->err_code == PPPERR_PEERDEAD ||
        inst.ppp->phase == PPP_PHASE_TERMINATE) {
        // reached LCP echo failure threshold LCP_MAXECHOFAILS
        inst.need_restart = true;
    }

    // allow the echo timeout to be disabled
    if (frontend.option_is_set(AP_Networking::OPTION::PPP_ECHO_LIMIT_DISABLE)) {
        inst.ppp->settings.lcp_echo_fails = 0;
    } else {
        inst.ppp->settings.lcp_echo_fails = LCP_MAXECHOFAILS;
    }

    return n > 0;
}

// hook for custom routes
struct netif *AP_Networking_PPP::routing_hook(uint32_t dest)
{
    for (uint8_t i=0; i<AP_NETWORKING_MAX_ROUTES; i++) {
        auto &r = routes[i];
        if (r.enabled && (r.dest_ip & r.netmask) == (dest & r.netmask)) {
            if (r.iface_idx >= AP_NETWORKING_PPP_NUM_INTERFACES) {
                continue;
            }
            return iface[r.iface_idx].pppif;
        }
    }
    return nullptr;
}

#endif // AP_NETWORKING_BACKEND_PPP
