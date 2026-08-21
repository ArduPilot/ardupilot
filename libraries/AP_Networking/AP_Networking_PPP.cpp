
#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_PPP

#include "AP_Networking_PPP.h"
#include "AP_Networking_PPP_SoftFlow.h"
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <netif/ppp/ppp_opts.h>
#include <netif/ppp/pppapi.h>
#include <netif/ppp/pppos.h>
#include <netif/ppp/ppp_impl.h>
#include <lwip/tcpip.h>
#include <atomic>
#include <stdio.h>
#include <string.h>

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

// largest escaped frame: payload, header and FCS can all double in
// size, plus the two flag bytes
#define PPP_MAX_FRAME_SIZE (2U*(PPP_MAXMRU+8U))
static_assert(PPP_BUFSIZE_TX >= PPP_MAX_FRAME_SIZE, "PPP TX buffer must hold a maximum size frame");

// data held in the HAL TX queue is limited to about 3ms worth, within
// these bounds, so a flow-control codeword is never far behind data. A
// peer that has stopped us can still be sent this much when we need to
// send it a command of our own, which is inside the space it keeps free
// when it sends STOP
static constexpr uint16_t PPP_SOFT_FLOW_TX_MIN = 32;
static constexpr uint16_t PPP_SOFT_FLOW_TX_MAX = 1024;
static constexpr uint16_t PPP_SOFT_FLOW_TX_RETRY_MS = 20;
static constexpr uint32_t PPP_SOFT_FLOW_CONNECT_TIMEOUT_MS = 60000;
// queue space data frames leave free, so they cannot crowd out LCP echoes
static constexpr uint16_t PPP_SOFT_FLOW_CTRL_RESERVE = 256;
// a ByteBuffer holds one byte less than its size
static_assert(PPP_BUFSIZE_TX > PPP_MAX_FRAME_SIZE + PPP_SOFT_FLOW_CTRL_RESERVE, "PPP soft flow TX buffer must hold a maximum size frame");

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
    TxState tx_state;
    AP_Networking_PPP_SoftFlow::Command tx_command;
    uint32_t tx_state_start_ms;
    uint32_t tx_retry_start_ms;
    uint32_t connect_start_ms;
    uint32_t rx_total;
    uint32_t drain_timeout_ms;
    uint32_t tx_backlog_limit;
    uint16_t tx_limit;
    bool peer_warned;
    bool rx_blocked;
    bool tx_retry_pending;
    bool restart_pending;
    std::atomic<bool> flow_control_ready;
    bool hal_stopped;
};

extern const AP_HAL::HAL& hal;

#if LWIP_TCPIP_CORE_LOCKING
#define LWIP_TCPIP_LOCK() sys_lock_tcpip_core()
#define LWIP_TCPIP_TRYLOCK() sys_trylock_tcpip_core()
#define LWIP_TCPIP_UNLOCK() sys_unlock_tcpip_core()
#else
#define LWIP_TCPIP_LOCK()
#define LWIP_TCPIP_TRYLOCK() true
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
  true if an escaped PPP frame carries a control protocol (LCP, IPCP
  etc), which always have a protocol number of 0x8000 or above
 */
static bool ppp_frame_is_control(const uint8_t *ptr, uint32_t len)
{
    uint8_t hdr[3] {};
    uint8_t n = 0;
    for (uint32_t i = 0; i < len && n < sizeof(hdr); i++) {
        uint8_t c = ptr[i];
        if (c == PPP_FRAME_BYTE) {
            // leading flag
            continue;
        }
        if (c == PPP_ESCAPE_BYTE) {
            if (++i >= len) {
                break;
            }
            c = ptr[i] ^ 0x20;
        }
        hdr[n++] = c;
    }
    // the address and control bytes may be omitted on data frames
    const uint8_t proto = (hdr[0] == 0xFF) ? hdr[2] : hdr[0];
    return (proto & 0x80) != 0;
}

/*
  output callback from lwip. A frame arrives in chunks, so stage them
  and only write complete frames to the uart. A frame that does not
  fit waits for space in update_instance() rather than being partly
  sent
 */
uint32_t AP_Networking_PPP::ppp_output_cb(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx)
{
    auto &driver = *(AP_Networking_PPP::PPP_Instance *)ctx;
    const uint8_t *ptr = (const uint8_t *)data;
    auto &tx = driver.tx;

    WITH_SEMAPHORE(tx.sem);

    if (pcb->phase == PPP_PHASE_TERMINATE) {
        // don't output while terminating
        tx.len = 0;
        tx.pending = false;
        return 0;
    }

    if (tx.buf == nullptr || len == 0) {
        return 0;
    }

    if (tx.pending) {
        // a data frame is waiting for uart space. Let a complete
        // control frame past it so the link stays up, refuse the rest
        // and lwip will retry them
        if (ptr[len-1] == PPP_FRAME_BYTE && ppp_frame_is_control(ptr, len) &&
            driver.uart->txspace() >= len) {
#if AP_NETWORKING_CAPTURE_ENABLED
            if (driver.backend->frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
                driver.capture_data(ptr, len);
            }
#endif
            driver.uart->write(ptr, len);
            return len;
        }
        return 0;
    }

    if (tx.len + len > PPP_MAX_FRAME_SIZE) {
        tx.len = 0;
        return 0;
    }

    memcpy(&tx.buf[tx.len], ptr, len);
    tx.len += len;

    if (ptr[len-1] != PPP_FRAME_BYTE) {
        // more chunks of this frame to come
        return len;
    }

#if AP_NETWORKING_CAPTURE_ENABLED
    if (driver.backend->frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
        driver.capture_data(tx.buf, tx.len);
    }
#endif

    if (!driver.write_frame()) {
        tx.pending = true;
    }
    return len;
}

/*
  write the staged frame to the uart if there is space for all of it
 */
bool AP_Networking_PPP::PPP_Instance::write_frame(void)
{
    if (tx.uart_capacity > 0 && tx.len > tx.uart_capacity) {
        // the requested UART buffer may have been reduced under memory
        // pressure, so this frame can never fit
        tx.len = 0;
        tx.pending = false;
        return true;
    }
    if (uart->txspace() < tx.len) {
        return false;
    }
    uart->write(tx.buf, tx.len);
    tx.len = 0;
    tx.pending = false;
    return true;
}

/*
  output callback from lwip when software flow control is enabled
 */
uint32_t AP_Networking_PPP::ppp_output_soft_flow_cb(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx)
{
    auto &state = *(AP_Networking_PPP::SoftFlowState *)ctx;
    auto &driver = *state.instance;
    auto &tx = driver.tx;
    const uint8_t *ptr = (const uint8_t *)data;

    WITH_SEMAPHORE(tx.sem);

    // stage the chunks so only complete frames are queued
    if (pcb->phase == PPP_PHASE_TERMINATE || len == 0 ||
        tx.len + len > PPP_MAX_FRAME_SIZE) {
        tx.len = 0;
        return 0;
    }

    memcpy(&tx.buf[tx.len], ptr, len);
    tx.len += len;

    if (ptr[len-1] != PPP_FRAME_BYTE) {
        // more chunks of this frame to come
        return len;
    }

    const uint32_t frame_len = tx.len;
    tx.len = 0;
    const bool is_control = ppp_frame_is_control(tx.buf, frame_len);
    const uint32_t reserve = is_control ? 0 : PPP_SOFT_FLOW_CTRL_RESERVE;
    if (state.tx_buffer.space() < frame_len + reserve) {
        return 0;
    }
    // limit how long an LCP echo can wait behind queued data
    if (!is_control && state.tx_buffer.available() + frame_len > state.tx_backlog_limit) {
        return 0;
    }

#if AP_NETWORKING_CAPTURE_ENABLED
    if (driver.backend->frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
        driver.capture_data(tx.buf, frame_len);
    }
#endif

    state.tx_buffer.write(tx.buf, frame_len);
    return len;
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
        driver.need_restart.store(true, std::memory_order_relaxed);
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

    if (code != PPPERR_NONE) {
        state.flow_control_ready.store(false, std::memory_order_release);
        ppp_status_callback(pcb, code, state.instance);
        return;
    }

    const uint32_t required_asyncmap = (1UL << 0x11) | (1UL << 0x13);
    // an unnegotiated ACCM also escapes XON/XOFF, but does not show
    // the peer supports the codewords, so require explicit negotiation
    const bool flow_control_ready =
        pcb->lcp_gotoptions.neg_asyncmap &&
        pcb->lcp_hisoptions.neg_asyncmap &&
        (pcb->lcp_gotoptions.asyncmap & required_asyncmap) == required_asyncmap &&
        (pcb->lcp_hisoptions.asyncmap & required_asyncmap) == required_asyncmap;
    ppp_status_callback(pcb, code, state.instance);
    state.flow_control_ready.store(flow_control_ready, std::memory_order_release);
    if (!flow_control_ready) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "PPP[%u]: peer software flow unavailable",
                      unsigned(state.instance->idx));
    }
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
        inst.tx.buf = NEW_NOTHROW uint8_t[PPP_MAX_FRAME_SIZE];
        if (inst.pppif == nullptr || inst.tx.buf == nullptr) {
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
  PPP loop without software flow control
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
        inst.tx.uart_capacity = inst.uart->txspace();

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
  PPP loop with software flow control
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
        const uint32_t baud = AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_PPP, i);
        inst.uart->begin(baud, PPP_BUFSIZE_RX, PPP_SOFT_FLOW_TX_MAX);
        inst.uart->set_unbuffered_writes(true);
        inst.uart->stop_transmit(false);
        inst.tx.uart_capacity = inst.uart->txspace();
        auto &state = *(SoftFlowState *)inst.ppp->ctx_cb;
        state.tx_limit = MAX(PPP_SOFT_FLOW_TX_MIN, MIN(PPP_SOFT_FLOW_TX_MAX, baud / 3600U));
        // about a second of data, and always room for one frame
        state.tx_backlog_limit = MAX(baud / 10U, PPP_MAX_FRAME_SIZE);
        // twice the time to send a full queue and a codeword
        state.drain_timeout_ms = MAX(50U, (state.tx_limit + AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH) * 20000U / MAX(baud, 1U));
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
    {
        // drop any frame left over from the old link
        WITH_SEMAPHORE(inst.tx.sem);
        inst.tx.len = 0;
        inst.tx.pending = false;
    }
    LWIP_TCPIP_LOCK();
    if (soft_flow_state != nullptr) {
        inst.uart->stop_transmit(false);
        soft_flow_state->hal_stopped = false;
        soft_flow_state->flow.reset();
        soft_flow_state->rx_buffer.clear();
        soft_flow_state->tx_buffer.clear();
        soft_flow_state->tx_state = SoftFlowState::TxState::IDLE;
        soft_flow_state->tx_command = AP_Networking_PPP_SoftFlow::Command::NONE;
        soft_flow_state->tx_state_start_ms = 0;
        soft_flow_state->tx_retry_start_ms = 0;
        soft_flow_state->connect_start_ms = AP_HAL::millis();
        soft_flow_state->tx_retry_pending = false;
        soft_flow_state->restart_pending = false;
        soft_flow_state->flow_control_ready.store(false, std::memory_order_release);
        soft_flow_state->peer_warned = false;
        soft_flow_state->rx_blocked = false;
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

    if (inst.need_restart.exchange(false, std::memory_order_relaxed)) {

        LWIP_TCPIP_LOCK();
        inst.ppp->phase = 0;
        ppp_close(inst.ppp, 1);
        LWIP_TCPIP_UNLOCK();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: reconnecting", unsigned(idx));

        restart_instance(idx);
    }

    {
        WITH_SEMAPHORE(inst.tx.sem);
        if (inst.tx.uart_capacity == 0 && !inst.uart->tx_pending()) {
            inst.tx.uart_capacity = inst.uart->txspace();
        }
        if (inst.tx.pending) {
            inst.write_frame();
        }
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
        inst.need_restart.store(true, std::memory_order_relaxed);
#endif
    }

    if (inst.ppp->err_code == PPPERR_PEERDEAD ||
        inst.ppp->phase == PPP_PHASE_TERMINATE) {
        // reached LCP echo failure threshold LCP_MAXECHOFAILS
        inst.need_restart.store(true, std::memory_order_relaxed);
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
    const bool tx_state_timed_out =
        now_ms - state.tx_state_start_ms > state.drain_timeout_ms;

    const uint32_t tx_space = inst.uart->txspace();
    if (inst.tx.uart_capacity == 0) {
        // nothing is written before the size of the empty queue is known
        inst.tx.uart_capacity = tx_space;
        if (tx_space == 0) {
            return;
        }
    }
    const uint32_t tx_queued = inst.tx.uart_capacity > tx_space ?
                               inst.tx.uart_capacity - tx_space : 0;

    if (state.tx_state == SoftFlowState::TxState::WAIT_COMMAND_DRAIN) {
        if (tx_queued != 0) {
            if (!tx_state_timed_out) {
                return;
            }
            state.tx_state = SoftFlowState::TxState::IDLE;
            state.tx_command = AP_Networking_PPP_SoftFlow::Command::NONE;
            state.tx_retry_start_ms = now_ms;
            state.tx_retry_pending = true;
            return;
        }
        state.flow.command_sent(state.tx_command, now_ms);
        state.tx_command = AP_Networking_PPP_SoftFlow::Command::NONE;
        state.tx_state = SoftFlowState::TxState::IDLE;
    }

    if (state.tx_state == SoftFlowState::TxState::WAIT_NORMAL_DRAIN) {
        if (tx_queued != 0) {
            if (!tx_state_timed_out) {
                return;
            }
            state.tx_state = SoftFlowState::TxState::IDLE;
            state.tx_command = AP_Networking_PPP_SoftFlow::Command::NONE;
            state.tx_retry_start_ms = now_ms;
            state.tx_retry_pending = true;
            return;
        }
        const auto current_command = state.flow.pending_command(now_ms);
        if (current_command != AP_Networking_PPP_SoftFlow::Command::NONE) {
            state.tx_command = current_command;
        }
        if (state.tx_command == AP_Networking_PPP_SoftFlow::Command::NONE) {
            state.tx_state = SoftFlowState::TxState::IDLE;
        } else if (tx_space >= AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH) {
            const uint32_t written = inst.uart->write(
                                         AP_Networking_PPP_SoftFlow::codeword(state.tx_command),
                                         AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH);
            if (written == AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH) {
                state.tx_state = SoftFlowState::TxState::WAIT_COMMAND_DRAIN;
                state.tx_state_start_ms = now_ms;
            }
            return;
        } else if (!tx_state_timed_out) {
            return;
        } else {
            state.tx_state = SoftFlowState::TxState::IDLE;
            state.tx_command = AP_Networking_PPP_SoftFlow::Command::NONE;
            state.tx_retry_start_ms = now_ms;
            state.tx_retry_pending = true;
            return;
        }
    }

    if (state.tx_retry_pending) {
        if (now_ms - state.tx_retry_start_ms < PPP_SOFT_FLOW_TX_RETRY_MS) {
            return;
        }
        state.tx_retry_pending = false;
    }

    if (!state.flow_control_ready.load(std::memory_order_acquire)) {
        state.flow.reset();
    }

    const auto command = state.flow.pending_command(now_ms);
    if (command != AP_Networking_PPP_SoftFlow::Command::NONE) {
        if (state.hal_stopped) {
            inst.uart->stop_transmit(false);
            state.hal_stopped = false;
        }
        state.tx_command = command;
        state.tx_state = SoftFlowState::TxState::WAIT_NORMAL_DRAIN;
        state.tx_state_start_ms = now_ms;
        return;
    }

    // with the staging buffer full we are not reading the peer's
    // codewords, so cannot know that it wants data
    const bool transmit_paused = state.flow.transmit_paused(now_ms) || state.rx_blocked;
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
    if (tx_queued >= state.tx_limit) {
        return;
    }
    const uint32_t len = MIN(MIN(available_bytes, tx_space),
                             state.tx_limit - tx_queued);
    if (len > 0) {
        state.tx_buffer.advance(inst.uart->write(data, len));
    }
}

/*
  update an instance with software flow control, return true if we
  have read some data from the uart
 */
bool AP_Networking_PPP::update_instance_soft_flow(const uint8_t idx)
{
    auto &inst = iface[idx];
    auto &state = *(SoftFlowState *)inst.ppp->ctx_cb;
    uint8_t buf[1024 + AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH];

    // the lwIP lock is never waited for here, as that would hold up
    // flow control on every PPP interface
    if ((inst.need_restart.load(std::memory_order_relaxed) || state.restart_pending) &&
        LWIP_TCPIP_TRYLOCK()) {
        bool closing = false;
        if (inst.need_restart.exchange(false, std::memory_order_relaxed) &&
            !state.restart_pending) {
            state.restart_pending = true;
            closing = true;
            // also clears the error from a link that is already dead
            ppp_close(inst.ppp, 1);
            if (inst.ppp->phase != PPP_PHASE_DEAD) {
                link_terminated(inst.ppp);
            }
        }
        if (state.restart_pending && inst.ppp->phase == PPP_PHASE_DEAD) {
            state.restart_pending = false;
            inst.need_restart.store(false, std::memory_order_relaxed);
            restart_instance(idx, &state);
        }
        LWIP_TCPIP_UNLOCK();
        if (closing) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PPP[%u]: reconnecting", unsigned(idx));
        }
    }

    const bool flow_control_ready =
        state.flow_control_ready.load(std::memory_order_acquire);
    uint32_t now_ms = AP_HAL::millis();

    // the receive window is the staging buffer, which has a known size
    // and is filled here whether or not lwIP is ready for the data
    if (flow_control_ready) {
        state.flow.update_receive_space(state.rx_buffer.space(), PPP_BUFSIZE_RX, now_ms);
    }
    update_soft_flow_tx(inst);

    // empty the UART into the staging buffer, so that data lwIP is slow
    // to take fills the staging buffer and not the UART
    uint32_t raw_total = 0;
    while (true) {
        const uint32_t stage_space = state.rx_buffer.space();
        // held codeword candidates can add CODEWORD_LENGTH bytes to a read
        state.rx_blocked = stage_space <= AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH;
        if (state.rx_blocked) {
            break;
        }
        now_ms = AP_HAL::millis();
        uint16_t data_len = 0;
        if (flow_control_ready) {
            data_len = state.flow.flush(now_ms, buf);
        }
        uint8_t *const raw = &buf[AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH];
        const uint32_t want = MIN(1024U, stage_space - AP_Networking_PPP_SoftFlow::CODEWORD_LENGTH);
        const ssize_t raw_n = inst.uart->read(raw, want);
        if (raw_n > 0) {
            raw_total += raw_n;
            if (flow_control_ready) {
                for (uint16_t i = 0; i < raw_n; i++) {
                    const uint8_t byte = raw[i];
                    data_len += state.flow.process_byte(byte, now_ms, &buf[data_len]);
                }
            } else {
                memmove(buf, raw, raw_n);
                data_len = raw_n;
            }
        }
        state.rx_buffer.write(buf, data_len);
        if (raw_n < ssize_t(want)) {
            break;
        }
    }
    state.rx_total += raw_total;

    if (flow_control_ready) {
        now_ms = AP_HAL::millis();
        state.flow.update_receive_space(state.rx_buffer.space(), PPP_BUFSIZE_RX, now_ms);
        // a stopped peer can send a full HAL queue ahead of a STOP of its own
        if (!state.peer_warned &&
            state.flow.stop_ignored(state.rx_total + inst.uart->available(),
                                    state.tx_limit + 256U, now_ms)) {
            state.peer_warned = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "PPP[%u]: peer ignoring flow control", unsigned(idx));
        }
    }
    update_soft_flow_tx(inst);

    bool consumer_stalled = false;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
    const auto *sitl = AP::sitl();
    if (sitl != nullptr && sitl->ppp_stall_port >= 0) {
        consumer_stalled = inst.uart == hal.serial(uint8_t(sitl->ppp_stall_port));
    }
#endif

    // a link paused in either direction carries no data or echo replies,
    // the codewords show that it is alive
    now_ms = AP_HAL::millis();
    const bool link_paused = flow_control_ready &&
                             (state.flow.receive_stopped() || state.flow.transmit_paused(now_ms));

    // never wait for lwIP, so STOP can be sent while it is busy
    uint32_t input_n = 0;
    if ((link_paused || state.rx_buffer.available() > 0) && LWIP_TCPIP_TRYLOCK()) {
        if (link_paused) {
            inst.ppp->lcp_echos_pending = 0;
        }
        if (!consumer_stalled) {
            input_n = state.rx_buffer.read(buf, 1024);
            if (input_n > 0) {
                pppos_input(inst.ppp, buf, input_n);
            }
        }
        LWIP_TCPIP_UNLOCK();
    }
    if (input_n > 0 || link_paused) {
        // negotiation traffic counts as link activity here, the
        // connect timeout bounds a negotiation that never completes
        inst.last_read_ms = now_ms;
    }

    update_soft_flow_tx(inst);

#if PPP_LINK_TIMEOUT_MS
    now_ms = AP_HAL::millis();
    if (inst.ppp->if4_up) {
        // time a later renegotiation from when the link was last up
        state.connect_start_ms = now_ms;
    }
    if (!frontend.option_is_set(AP_Networking::OPTION::PPP_TIMEOUT_DISABLE) &&
        (now_ms - inst.last_read_ms > PPP_LINK_TIMEOUT_MS ||
         (!inst.ppp->if4_up &&
          now_ms - state.connect_start_ms > PPP_SOFT_FLOW_CONNECT_TIMEOUT_MS))) {
        inst.need_restart.store(true, std::memory_order_relaxed);
    }
#endif

    if (inst.ppp->err_code == PPPERR_PEERDEAD ||
        inst.ppp->phase == PPP_PHASE_TERMINATE) {
        inst.need_restart.store(true, std::memory_order_relaxed);
    }

    if (frontend.option_is_set(AP_Networking::OPTION::PPP_ECHO_LIMIT_DISABLE)) {
        inst.ppp->settings.lcp_echo_fails = 0;
    } else {
        inst.ppp->settings.lcp_echo_fails = LCP_MAXECHOFAILS;
    }

    return raw_total > 0 || input_n > 0;
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
