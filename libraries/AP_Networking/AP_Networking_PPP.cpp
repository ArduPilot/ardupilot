
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
        driver.activeSettings.ip = ntohl(netif_ip4_addr(pppif)->addr);
        driver.activeSettings.gw = ntohl(netif_ip4_gw(pppif)->addr);
        driver.activeSettings.nm = ntohl(netif_ip4_netmask(pppif)->addr);
        driver.activeSettings.last_change_ms = AP_HAL::millis();
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

    // initialise TCP/IP thread
    LWIP_TCPIP_LOCK();
    tcpip_init(NULL, NULL);
    LWIP_TCPIP_UNLOCK();

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

    // ensure this thread owns the uart
    uart->begin(0);
    uart->set_unbuffered_writes(true);

    while (true) {
        uint8_t buf[1024];

        // connect and set as default route
        LWIP_TCPIP_LOCK();
        ppp_connect(ppp, 0);

        netif_set_default(pppif);
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
