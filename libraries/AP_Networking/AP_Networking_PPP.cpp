
#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_PPP

#include "AP_Networking_PPP.h"
#include <GCS_MAVLink/GCS.h>

#include <lwipthread.h>
#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <netif/ppp/ppp_opts.h>
#include <netif/ppp/pppapi.h>
#include <netif/ppp/pppos.h>
#include <lwip/tcpip.h>


extern const AP_HAL::HAL& hal;

/*
  uint32_t timestamp in smallest available units
 */
uint32_t sys_jiffies(void)
{
    return AP_HAL::micros();
}

/*
  output some data to the uart
 */
uint32_t AP_Networking_PPP::ppp_output_cb(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx)
{
    auto &driver = *(AP_Networking_PPP *)ctx;
    LWIP_UNUSED_ARG(pcb);
    uint32_t remaining = len;
    uint8_t *ptr = const_cast<uint8_t*>((const uint8_t *)data);
    while (remaining > 0) {
        auto n = driver.uart->write(ptr, remaining);
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
    const char *msg = nullptr;

    switch (code) {
    case PPPERR_NONE: {
        // got addresses
        driver.activeSettings.ip = ntohl(netif_ip4_addr(pppif)->addr);
        driver.activeSettings.gw = ntohl(netif_ip4_gw(pppif)->addr);
        driver.activeSettings.nm = ntohl(netif_ip4_netmask(pppif)->addr);
        driver.activeSettings.last_change_ms = AP_HAL::millis();

        break;
    }
    case PPPERR_PARAM: {           /* Invalid parameter. */
        msg = "PPPERR_PARAM";
        break;
    }
    case PPPERR_OPEN: {            /* Unable to open PPP session. */
        msg = "PPPERR_OPEN";
        break;
    }
    case PPPERR_DEVICE: {          /* Invalid I/O device for PPP. */
        msg = "PPPERR_DEVICE";
        break;
    }
    case PPPERR_ALLOC: {           /* Unable to allocate resources. */
        msg = "PPPERR_ALLOC";
        break;
    }
    case PPPERR_USER: {            /* User interrupt. */
        msg = "PPPERR_USER";
        break;
    }
    case PPPERR_CONNECT: {         /* Connection lost. */
        msg = "PPPERR_CONNECT";
        break;
    }
    case PPPERR_AUTHFAIL: {        /* Failed authentication challenge. */
        msg = "PPPERR_AUTHFAIL";
        break;
    }
    case PPPERR_PROTOCOL: {        /* Failed to meet protocol. */
        msg = "PPPERR_PROTOCOL";
        break;
    }
    case PPPERR_PEERDEAD: {        /* Connection timeout */
        msg = "PPPERR_PEERDEAD";
        break;
    }
    case PPPERR_IDLETIMEOUT: {     /* Idle Timeout */
        msg = "PPPERR_IDLETIMEOUT";
        break;
    }
    case PPPERR_CONNECTTIME: {     /* Max connect time reached */
        msg = "PPPERR_CONNECTTIME";
        break;
    }
    case PPPERR_LOOPBACK: {        /* Loopback detected */
        msg = "PPPERR_LOOPBACK";
        break;
    }
    }
    if (msg != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP: %s", msg);
    }
}


/*
  initialise PPP network backend using LWIP
 */
bool AP_Networking_PPP::init()
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_PPP, 0);
    if (uart == nullptr) {
        return false;
    }
    uart->set_unbuffered_writes(true);
    
    pppif = new netif;
    if (pppif == nullptr) {
        return false;
    }

    // initialise TCP/IP thread
    tcpip_init(NULL, NULL);
    hal.scheduler->delay(100);
    
    // create ppp connection
    ppp = pppos_create(pppif, ppp_output_cb, ppp_status_callback, this);
    if (ppp == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP: failed to create link");
        return false;
    }

    // connect and set as default route
    ppp_connect(ppp, 0);
    netif_set_default(pppif);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP: started");
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_PPP::ppp_loop, void),
                                 "ppp",
                                 8192, AP_HAL::Scheduler::PRIORITY_UART, 0);

    return true;
}

/*
  main loop for PPP
 */
void AP_Networking_PPP::ppp_loop(void)
{
    // ensure this thread owns the uart
    uart->begin(0);

    while (true) {
        uint8_t buf[1024];
        auto n = uart->read(buf, sizeof(buf));
        if (n > 0) {
            pppos_input(ppp, buf, n);
        } else {
            hal.scheduler->delay_microseconds(100);
        }
    }
}

#endif // AP_NETWORKING_BACKEND_PPP
