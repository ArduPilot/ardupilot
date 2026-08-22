
#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_PPP

#include "AP_Networking_PPP.h"
#include "AP_Networking_PPP_SoftFlow.h"
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS.h>

#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <netif/ppp/ppp_opts.h>
#include <netif/ppp/pppapi.h>
#include <netif/ppp/pppos.h>
#include <netif/ppp/ppp_impl.h>
#include <lwip/tcpip.h>
#include <stdio.h>

#if AP_NETWORKING_CAPTURE_ENABLED
#include <AP_Filesystem/AP_Filesystem.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
#include <SITL/SITL.h>
#endif

// PPP protocol
#ifndef PPP_BUFSIZE_RX
#define PPP_BUFSIZE_RX 4096
#endif
#ifndef PPP_BUFSIZE_TX
#define PPP_BUFSIZE_TX 8192
#endif


// Keep the HAL queue short only on the software-flow-controlled path. The
// normal path retains PPP_BUFSIZE_TX for high-rate links using hardware flow
// control.
static constexpr uint16_t PPP_SOFT_FLOW_UART_TX_SIZE = 256;

struct AP_Networking_PPP::SoftFlowState {
    explicit SoftFlowState(PPP_Instance &_instance) :
        instance(&_instance),
        rx_buffer(PPP_BUFSIZE_RX),
        tx_buffer(PPP_BUFSIZE_TX)
    {}

    enum class TxState : uint8_t {
        IDLE,
        WAIT_NORMAL_DRAIN,
        WAIT_COMMAND_DRAIN,
    };

    PPP_Instance *const instance;
    ByteBuffer rx_buffer;
    ByteBuffer tx_buffer;
    AP_Networking_PPP_SoftFlow flow;
    TxState tx_state = TxState::IDLE;
    AP_Networking_PPP_SoftFlow::Command tx_command = AP_Networking_PPP_SoftFlow::Command::NONE;
    bool hal_stopped = false;
};
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

/*
  Queue a complete PPP frame for the software-flow-controlled transport.
  This callback is selected only when software flow control is enabled, so
  the normal high-rate output path above remains unchanged.
 */
uint32_t AP_Networking_PPP::ppp_output_soft_flow_cb(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx)
{
    auto &state = *(AP_Networking_PPP::SoftFlowState *)ctx;
    LWIP_UNUSED_ARG(pcb);
    const uint8_t *ptr = (const uint8_t *)data;

    if (state.tx_buffer.space() < len) {
        // Keep PPP frames atomic in the staging ring.
        return 0;
    }

#if AP_NETWORKING_CAPTURE_ENABLED
    auto &driver = *state.instance;
    if (driver.backend->frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
        driver.capture_data(ptr, len);
    }
#endif

    return state.tx_buffer.write(ptr, len);
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
  Adapt the enabled-only callback context to the ordinary status callback.
 */
void AP_Networking_PPP::ppp_status_soft_flow_callback(ppp_pcb *pcb, int code, void *ctx)
{
    auto &state = *(AP_Networking_PPP::SoftFlowState *)ctx;
    ppp_status_callback(pcb, code, state.instance);
}


/*
  initialise PPP network backend using LWIP
 */
bool AP_Networking_PPP::init()
{
    auto &sm = AP::serialmanager();
    bool need_thread = false;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // Seed activeSettings from the static IP params so PPP-gateway-mode IPCP
    // setup in restart_instance() has valid addresses on its first run. On
    // ChibiOS+Ethernet the ChibiOS backend does this; in pure-PPP SITL there
    // is no other backend to set it.
    const uint32_t param_ip = frontend.get_ip_param();
    if (param_ip != 0) {
        activeSettings.ip = param_ip;
        activeSettings.nm = frontend.get_netmask_param();
        activeSettings.gw = frontend.get_gateway_param();
        activeSettings.last_change_ms = AP_HAL::millis();
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    const bool software_flow_control =
        frontend.option_is_set(AP_Networking::OPTION::PPP_SOFTWARE_FLOW_CONTROL);
    if (software_flow_control) {
        uint8_t num_ppp_interfaces = 0;
        while (num_ppp_interfaces < AP_NETWORKING_PPP_NUM_INTERFACES) {
            if (sm.find_serial(AP_SerialManager::SerialProtocol_PPP, num_ppp_interfaces) == nullptr) {
                break;
            }
            num_ppp_interfaces++;
        }

        for (uint8_t i = 0; i < num_ppp_interfaces; i++) {
            auto *uart = sm.find_serial(AP_SerialManager::SerialProtocol_PPP, i);
            if (!uart->stop_transmit(false)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                              "PPP[%u]: transmit pause unavailable",
                              unsigned(i));
                return false;
            }
        }
    }

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

        SoftFlowState *flow_state = nullptr;
        pppos_output_cb_fn output_cb = ppp_output_cb;
        ppp_link_status_cb_fn status_cb = ppp_status_callback;
        void *callback_ctx = &inst;
        if (software_flow_control) {
            flow_state = NEW_NOTHROW SoftFlowState(inst);
            if (flow_state == nullptr ||
                flow_state->rx_buffer.get_size() != PPP_BUFSIZE_RX ||
                flow_state->tx_buffer.get_size() != PPP_BUFSIZE_TX) {
                delete flow_state;
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                              "PPP[%u]: flow buffer allocation failed",
                              unsigned(i));
                break;
            }
            output_cb = ppp_output_soft_flow_cb;
            status_cb = ppp_status_soft_flow_callback;
            callback_ctx = flow_state;
        }

        // create ppp connection
        LWIP_TCPIP_LOCK();
        inst.ppp = pppos_create(inst.pppif, output_cb, status_cb, callback_ctx);
        if (inst.ppp == nullptr) {
            delete flow_state;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: failed to create link", unsigned(i));
            LWIP_TCPIP_UNLOCK();
            break;
        }
        LWIP_TCPIP_UNLOCK();

        if (software_flow_control) {
            // Reserve XON and XOFF for out-of-band flow-control codewords.
            // The peer will escape any occurrences inside normal PPP frames.
            ppp_set_asyncmap(inst.ppp, (1UL << 0x11) | (1UL << 0x13));
        }

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
    for (auto &inst : iface) {
        if (inst.uart == nullptr) {
            continue;
        }
        if (inst.ppp->ctx_cb == &inst) {
            ppp_loop_normal();
        } else {
            ppp_loop_soft_flow();
        }
        return;
    }
}

/*
  Original direct PPP path, kept separate so high-rate hardware-flow-controlled
  links do not allocate staging buffers or execute software-flow checks.
 */
void AP_Networking_PPP::ppp_loop_normal(void)
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
  PPP path with software flow control. Its staging buffers and smaller HAL TX
  queue are used only when the option was enabled during init().
 */
void AP_Networking_PPP::ppp_loop_soft_flow(void)
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }
    const bool ppp_gateway = frontend.option_is_set(AP_Networking::OPTION::PPP_ETHERNET_GATEWAY);
    if (ppp_gateway) {
        AP::network().startup_wait();
    }

    for (uint8_t i = 0; i < AP_NETWORKING_PPP_NUM_INTERFACES; i++) {
        auto &inst = iface[i];
        if (inst.uart == nullptr) {
            continue;
        }
        inst.uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_PPP, i),
                         PPP_BUFSIZE_RX,
                         PPP_SOFT_FLOW_UART_TX_SIZE);
        inst.uart->set_unbuffered_writes(true);
        inst.uart->stop_transmit(false);
        auto &state = *(SoftFlowState *)inst.ppp->ctx_cb;
        restart_instance(i, &state);
    }

    while (true) {
        bool read_data = false;

        for (auto &inst : iface) {
            if (inst.uart != nullptr) {
                read_data |= update_instance_soft_flow(inst.idx);
            }
        }
        if (!read_data) {
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
void AP_Networking_PPP::restart_instance(const uint8_t idx, SoftFlowState *soft_flow_state)
{
    auto &inst = iface[idx];
    LWIP_TCPIP_LOCK();
    if (soft_flow_state != nullptr) {
        inst.uart->stop_transmit(false);
        soft_flow_state->hal_stopped = false;
        soft_flow_state->flow.reset();
        soft_flow_state->rx_buffer.clear();
        soft_flow_state->tx_buffer.clear();
        soft_flow_state->tx_state = SoftFlowState::TxState::IDLE;
        soft_flow_state->tx_command = AP_Networking_PPP_SoftFlow::Command::NONE;
    }

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
#endif // AP_NETWORKING_CAPTURE_ENABLED

/*
  hook for valid incoming PPP packets
 */
void AP_Networking_PPP::input_hook(const ppp_pcb *pcb, const struct pbuf *pb)
{
    LWIP_UNUSED_ARG(pb);
    auto &frontend = AP::network();
    AP_Networking_Backend *driver = frontend.backend;
#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    if (frontend.backend_PPP != nullptr) {
        driver = frontend.backend_PPP;
    }
#endif
    for (auto &inst : ((AP_Networking_PPP*)driver)->iface) {
        if (inst.uart != nullptr && inst.ppp == pcb) {
            // The lwIP hook runs only after a frame passes its FCS check. Count
            // valid LCP/IPCP packets during negotiation as link activity too;
            // requiring IPv4 to already be up can make the peer timeout while
            // it is responding to a legitimate renegotiation.
            inst.last_read_ms = AP_HAL::millis();
#if AP_NETWORKING_CAPTURE_ENABLED
            if (frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
                inst.capture_hook(pb);
            }
#endif
            break;
        }
    }
}

void ap_ppp_capture_hook(const struct ppp_pcb_s *pcb, const struct pbuf *pb)
{
    AP_Networking_PPP::input_hook(pcb, pb);
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

        if (!inst.restart_pending) {
            inst.restart_pending = true;

            LWIP_TCPIP_LOCK();
            if (inst.ppp->phase != PPP_PHASE_DEAD) {
                // Record this as a local close before completing the carrier
                // teardown. This clears a latched PPPERR_PEERDEAD, which would
                // otherwise make every new connection restart immediately.
                ppp_close(inst.ppp, 1);
                if (inst.ppp->phase != PPP_PHASE_DEAD) {
                    // ppp_close(..., nocarrier=1) only terminates immediately
                    // from RUNNING. In ESTABLISH or TERMINATE, force the lwIP
                    // carrier-loss path so both peers start a fresh session.
                    link_terminated(inst.ppp);
                }
            }
            LWIP_TCPIP_UNLOCK();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: reconnecting", unsigned(idx));
        }
    }

    if (inst.restart_pending && inst.ppp->phase == PPP_PHASE_DEAD) {
        inst.restart_pending = false;
        inst.need_restart = false;
        restart_instance(idx);
    }

    auto n = inst.uart->read(buf, sizeof(buf));
    if (n > 0) {
        LWIP_TCPIP_LOCK();
        pppos_input(inst.ppp, buf, n);
        LWIP_TCPIP_UNLOCK();
    }
#if PPP_LINK_TIMEOUT_MS
    const uint32_t now_ms = AP_HAL::millis();
    if (!frontend.option_is_set(AP_Networking::OPTION::PPP_TIMEOUT_DISABLE) &&
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

/*
  Pump queued PPP data and priority flow-control codewords into the UART.
 */
void AP_Networking_PPP::update_soft_flow_tx(PPP_Instance &inst)
{
    auto &state = *(SoftFlowState *)inst.ppp->ctx_cb;
    const uint32_t now_ms = AP_HAL::millis();

    if (state.tx_state == SoftFlowState::TxState::WAIT_COMMAND_DRAIN) {
        if (inst.uart->tx_pending()) {
            return;
        }
        state.flow.command_sent(state.tx_command, now_ms);
        state.tx_command = AP_Networking_PPP_SoftFlow::Command::NONE;
        state.tx_state = SoftFlowState::TxState::IDLE;
    }

    if (state.tx_state == SoftFlowState::TxState::WAIT_NORMAL_DRAIN) {
        if (inst.uart->tx_pending()) {
            return;
        }
        const auto current_command = state.flow.pending_command(now_ms);
        if (current_command != AP_Networking_PPP_SoftFlow::Command::NONE) {
            state.tx_command = current_command;
        }
        if (state.tx_command == AP_Networking_PPP_SoftFlow::Command::NONE) {
            state.tx_state = SoftFlowState::TxState::IDLE;
        } else if (inst.uart->txspace() >= AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH) {
            const uint32_t written = inst.uart->write(
                                         AP_Networking_PPP_SoftFlow::codeword(state.tx_command),
                                         AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH);
            if (written == AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH) {
                state.tx_state = SoftFlowState::TxState::WAIT_COMMAND_DRAIN;
            }
            return;
        } else {
            return;
        }
    }

    const auto command = state.flow.pending_command(now_ms);
    if (command != AP_Networking_PPP_SoftFlow::Command::NONE) {
        if (state.hal_stopped) {
            inst.uart->stop_transmit(false);
            state.hal_stopped = false;
        }
        state.tx_command = command;
        state.tx_state = SoftFlowState::TxState::WAIT_NORMAL_DRAIN;
        return;
    }

    const bool transmit_paused = state.flow.transmit_paused(now_ms);
    if (state.hal_stopped != transmit_paused) {
        inst.uart->stop_transmit(transmit_paused);
        state.hal_stopped = transmit_paused;
    }
    if (transmit_paused) {
        return;
    }

    uint32_t available_bytes = 0;
    const uint8_t *data = state.tx_buffer.readptr(available_bytes);
    if (data == nullptr || available_bytes == 0) {
        return;
    }
    const uint32_t tx_space = inst.uart->txspace();
    const uint32_t len = available_bytes < tx_space ? available_bytes : tx_space;
    if (len > 0) {
        state.tx_buffer.advance(inst.uart->write(data, len));
    }
}

/*
  Software-flow-controlled input path. Raw UART data is drained into a PPP
  staging ring before it is handed to lwIP, allowing STOP/START codewords to
  be recognised even when the PPP consumer is temporarily slow.
 */
bool AP_Networking_PPP::update_instance_soft_flow(const uint8_t idx)
{
    auto &inst = iface[idx];
    auto &state = *(SoftFlowState *)inst.ppp->ctx_cb;
    uint8_t buf[1024];

    if (inst.need_restart) {
        inst.need_restart = false;

        if (!inst.restart_pending) {
            inst.restart_pending = true;

            LWIP_TCPIP_LOCK();
            if (inst.ppp->phase != PPP_PHASE_DEAD) {
                ppp_close(inst.ppp, 1);
                if (inst.ppp->phase != PPP_PHASE_DEAD) {
                    link_terminated(inst.ppp);
                }
            }
            LWIP_TCPIP_UNLOCK();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: reconnecting", unsigned(idx));
        }
    }

    if (inst.restart_pending && inst.ppp->phase == PPP_PHASE_DEAD) {
        inst.restart_pending = false;
        inst.need_restart = false;
        restart_instance(idx, &state);
    }

    uint32_t now_ms = AP_HAL::millis();
    state.flow.update_receive_space(state.rx_buffer.space(),
                                    state.rx_buffer.get_size(),
                                    now_ms);
    update_soft_flow_tx(inst);

    const ssize_t raw_n = inst.uart->read(buf, sizeof(buf));
    if (raw_n > 0) {
        uint16_t data_len = 0;
        now_ms = AP_HAL::millis();
        for (uint16_t i = 0; i < raw_n; i++) {
            if (!state.flow.process_byte(buf[i], now_ms)) {
                buf[data_len++] = buf[i];
            }
        }
        if (data_len > 0) {
            if (state.rx_buffer.write(buf, data_len) != data_len) {
                // Flow-control thresholds leave headroom for bytes already in
                // flight. If that is ever insufficient, recover explicitly
                // instead of silently passing a damaged stream to lwIP.
                inst.need_restart = true;
            }
        }
    }

    state.flow.update_receive_space(state.rx_buffer.space(),
                                    state.rx_buffer.get_size(),
                                    AP_HAL::millis());
    update_soft_flow_tx(inst);

    bool input_stalled = false;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
    const auto *sitl = AP::sitl();
    if (sitl != nullptr && sitl->uart_read_stall_port >= 0) {
        input_stalled = inst.uart == hal.serial(uint8_t(sitl->uart_read_stall_port));
    }
#endif

    uint32_t input_n = 0;
    if (!input_stalled) {
        input_n = state.rx_buffer.read(buf, sizeof(buf));
        if (input_n > 0) {
            LWIP_TCPIP_LOCK();
            pppos_input(inst.ppp, buf, input_n);
            LWIP_TCPIP_UNLOCK();
        }
    }

    state.flow.update_receive_space(state.rx_buffer.space(),
                                    state.rx_buffer.get_size(),
                                    AP_HAL::millis());
    update_soft_flow_tx(inst);

#if PPP_LINK_TIMEOUT_MS
    now_ms = AP_HAL::millis();
    if (!frontend.option_is_set(AP_Networking::OPTION::PPP_TIMEOUT_DISABLE) &&
        now_ms - inst.last_read_ms > PPP_LINK_TIMEOUT_MS) {
        inst.need_restart = true;
    }
#endif

    if (inst.ppp->err_code == PPPERR_PEERDEAD ||
        inst.ppp->phase == PPP_PHASE_TERMINATE) {
        inst.need_restart = true;
    }

    if (frontend.option_is_set(AP_Networking::OPTION::PPP_ECHO_LIMIT_DISABLE)) {
        inst.ppp->settings.lcp_echo_fails = 0;
    } else {
        inst.ppp->settings.lcp_echo_fails = LCP_MAXECHOFAILS;
    }

    return raw_n > 0 || input_n > 0;
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
