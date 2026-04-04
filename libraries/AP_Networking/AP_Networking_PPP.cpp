
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

#if AP_NETWORKING_CAPTURE_ENABLED
#include <AP_Filesystem/AP_Filesystem.h>
#endif

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

// each PPP frame has a 0x7E at start and end, which needs
// to be stripped for network capture
#define PPP_FRAME_BYTE 0x7E
#define PPP_ESCAPE_BYTE 0x7D

/*
  output some data to the uart
 */
uint32_t AP_Networking_PPP::ppp_output_cb(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx)
{
    auto &driver = *(AP_Networking_PPP::PPP_Instance *)ctx;
    LWIP_UNUSED_ARG(pcb);
    const uint8_t *ptr = (const uint8_t *)data;

    if (pcb->phase == PPP_PHASE_TERMINATE) {
        // don't output while terminating
        return 0;
    }

    if (driver.uart->txspace() < len) {
        /*
          if we can't send the whole frame then don't send any of it. This
          minimises issues with the PPP state machine
         */
        return 0;
    }


#if AP_NETWORKING_CAPTURE_ENABLED
    /*
      capture outgoing data
     */
    if (driver.backend->frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
        driver.capture_data(ptr, len);
    }
#endif

    return driver.uart->write(ptr, len);
}

#if AP_NETWORKING_CAPTURE_ENABLED
/*
  capture a outgoing data packet
 */
void AP_Networking_PPP::PPP_Instance::capture_data(const uint8_t *ptr, uint32_t len)
{
    WITH_SEMAPHORE(capture.sem);
    if (capture.fd == -1) {
        return;
    }
    if (len >= 2 && ptr[0] == PPP_FRAME_BYTE) {
        // strip framing flags
        ptr++;
        len -= 2;
    }
    /*
      for PPP outgoing we need to unescape the data, but we
      need the length for the header, which means we need to
      walk the data twice
    */
    uint32_t dlen = 0;
    for (uint32_t i = 0; i < len; i++) {
        if (ptr[i] == PPP_ESCAPE_BYTE) {
            i++;
        }
        dlen++;
    }
    auto &fs = AP::FS();
    backend->capture_header(capture.fd, dlen+1); // +1 for direction byte
    const uint8_t direction = 0; // direction out
    fs.write(capture.fd, &direction, 1);

    uint8_t buf[32];
    uint32_t buflen = 0;
    for (uint32_t i = 0; i < len; i++) {
        if (ptr[i] == PPP_ESCAPE_BYTE && i + 1 < len) {
            buf[buflen++] = ptr[++i] ^ 0x20;
        } else {
            buf[buflen++] = ptr[i];
        }
        if (buflen == sizeof(buf)) {
            fs.write(capture.fd, buf, buflen);
            buflen = 0;
        }
    }
    if (buflen > 0) {
        fs.write(capture.fd, buf, buflen);
    }
}
#endif // AP_NETWORKING_CAPTURE_ENABLED

/*
  callback on link status change
 */
void AP_Networking_PPP::ppp_status_callback(ppp_pcb *pcb, int code, void *ctx)
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

#if AP_NETWORKING_PPP_GATEWAY_ENABLED
        const bool ethernet_gateway = frontend.option_is_set(AP_Networking::OPTION::PPP_ETHERNET_GATEWAY);
#else
        const bool ethernet_gateway = false;
#endif
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

#if AP_NETWORKING_CAPTURE_ENABLED
// start a pcap network capture
void AP_Networking_PPP::start_capture(void)
{
    for (auto &inst : iface) {
        if (inst.uart == nullptr) {
            continue;
        }
        auto &capture = inst.capture;
        if (capture.fd != -1) {
            // called at 1Hz, flush the file
            AP::FS().fsync(capture.fd);
            continue;
        }
        struct pcap_hdr {
            uint32_t magic_number;   // 0xa1b2c3d4
            uint16_t version_major;  // 2
            uint16_t version_minor;  // 4
            int32_t  thiszone;       // GMT to local
            uint32_t sigfigs;
            uint32_t snaplen;
            uint32_t network;        // DLT = 204 DLT_PPP_WITH_DIR
        } hdr {
            0xa1b2c3d4, 2, 4, 0, 0, UINT16_MAX, 204
        };
        char fname[] = "pppN.cap";
        fname[3] = '0' + inst.idx;
        WITH_SEMAPHORE(capture.sem);
        capture.fd = AP::FS().open(fname, O_WRONLY|O_CREAT|O_TRUNC);
        if (capture.fd != -1) {
            AP::FS().write(capture.fd, (const void *)&hdr, sizeof(hdr));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Capturing to %s", fname);
        }
    }
}

// stop a pcap network capture
void AP_Networking_PPP::stop_capture(void)
{
    for (auto &inst : iface) {
        if (inst.uart == nullptr) {
            continue;
        }
        auto &capture = inst.capture;
        if (capture.fd != -1) {
            int fd = capture.fd;
            capture.fd = -1;
            AP::FS().close(fd);
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

#if AP_NETWORKING_CAPTURE_ENABLED
            if (frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
                start_capture();
            } else {
                stop_capture();
            }
#endif
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

#if AP_NETWORKING_CAPTURE_ENABLED
/*
  hook for capturing all incoming PPP packets
 */
void AP_Networking_PPP::PPP_Instance::capture_hook(const struct pbuf *pb)
{
    WITH_SEMAPHORE(capture.sem);
    if (capture.fd == -1) {
        return;
    }
    /*
      calculate length with trimming of frame bytes
    */
    uint32_t len = pb->tot_len;
    bool need_trim = false;
    if (len >= 2 && pb->len > 0 && *(uint8_t*)pb->payload == PPP_FRAME_BYTE) {
        len -= 2;
        need_trim = true;
    }
    auto &fs = AP::FS();
    backend->capture_header(capture.fd, len+1); // +1 for direction byte
    const uint8_t direction = 1; // direction In
    fs.write(capture.fd, &direction, 1);
    /*
      copy the pbuf segments, trimming the framing bytes as needed
    */
    for (auto *pp = pb; pp; pp = pp->next) {
        const uint8_t *data = (const uint8_t *)pp->payload;
        auto plen = pp->len;
        if (need_trim && pp == pb) {
            data++;
            plen--;
        }
        if (need_trim && pp->next == nullptr) {
            plen--;
        }
        fs.write(capture.fd, data, plen);
    }
}

/*
  hook for capturing all incoming PPP packets
 */
void AP_Networking_PPP::capture_hook(const ppp_pcb *pcb, const struct pbuf *pb)
{
    auto &frontend = AP::network();
    if (!frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
        return;
    }
    AP_Networking_Backend *driver = frontend.backend;
#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    if (frontend.backend_PPP != nullptr) {
        driver = frontend.backend_PPP;
    }
#endif
    for (auto &inst : ((AP_Networking_PPP*)driver)->iface) {
        if (inst.uart != nullptr && inst.ppp == pcb) {
            inst.capture_hook(pb);
            break;
        }
    }
}
#endif // AP_NETWORKING_CAPTURE_ENABLED

void ap_ppp_capture_hook(const struct ppp_pcb_s *pcb, const struct pbuf *pb)
{
#if AP_NETWORKING_CAPTURE_ENABLED
    AP_Networking_PPP::capture_hook(pcb, pb);
#endif
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
