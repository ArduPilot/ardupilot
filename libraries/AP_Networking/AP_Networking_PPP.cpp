
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

/*
  output some data to the uart
 */
uint32_t AP_Networking_PPP::ppp_output_cb(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx)
{
    auto &driver = *(AP_Networking_PPP *)ctx;
    LWIP_UNUSED_ARG(pcb);
    uint32_t remaining = len;
    const uint8_t *ptr = (const uint8_t *)data;
    while (remaining > 0) {
        const auto n = driver.uart->write(ptr, remaining);
        if (n > 0) {
            remaining -= n;
            ptr += n;
        } else {
            hal.scheduler->delay_microseconds(100);
        }
    }
    return len;
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

    pppif = new netif;
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
            extern struct netif *netif_list;
            /*
              when we are setup as a PPP gateway we want the pppif to be
              first in the list so routing works if it is on the same
              subnet
            */
            if (netif_list != nullptr &&
                netif_list->next != nullptr &&
                netif_list->next->next == pppif) {
                netif_list->next->next = nullptr;
                pppif->next = netif_list;
                netif_list = pppif;
            }
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

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP: connected");

        while (!need_restart) {
            auto n = uart->read(buf, sizeof(buf));
            if (n > 0) {
                LWIP_TCPIP_LOCK();
                pppos_input(ppp, buf, n);
                LWIP_TCPIP_UNLOCK();
            } else {
                hal.scheduler->delay_microseconds(200);
            }
        }
    }
}

#endif // AP_NETWORKING_BACKEND_PPP
