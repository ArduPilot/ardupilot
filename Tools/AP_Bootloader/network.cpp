/*
  support for networking enabled bootloader
 */

#include "network.h"

#if AP_BOOTLOADER_NETWORK_ENABLED

#include <AP_Networking/AP_Networking.h>
#if !AP_BOOTLOADER_NETWORK_PPP_ENABLED
#include <AP_Networking/AP_Networking_ChibiOS.h>
#endif
#if AP_NETWORKING_CAN_MCAST_ENABLED
#include <AP_Networking/AP_Networking_CAN.h>
#endif

#include <lwip/ip_addr.h>
#include <lwip/tcpip.h>
#include <lwip/netifapi.h>
#include <lwip/sockets.h>
#if AP_BOOTLOADER_NETWORK_PPP_ENABLED
#include <netif/ppp/pppos.h>
#else
#include <lwip/etharp.h>
#endif
#if LWIP_DHCP
#include <lwip/dhcp.h>
#endif
#include <hal.h>
#include "../../modules/ChibiOS/os/various/evtimer.h"
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL/utility/Socket.h>
#include <AP_ROMFS/AP_ROMFS.h>
#include "support.h"
#include "bl_protocol.h"
#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include "app_comms.h"
#include "can.h"
#include <stdio.h>
#include <stdarg.h>
#include <AP_HAL_ChibiOS/hwdef/common/flash.h>

// can_printf() only exists when the bootloader has CAN
#define BL_NETWORK_HAVE_CAN (HAL_USE_CAN == TRUE || HAL_NUM_CAN_IFACES)
#if BL_NETWORK_HAVE_CAN
#include <AP_HAL_ChibiOS/CANIface.h>
#endif

#ifndef AP_NETWORKING_BOOTLOADER_DEFAULT_MAC_ADDR
#define AP_NETWORKING_BOOTLOADER_DEFAULT_MAC_ADDR "C2:AF:51:03:CF:46"
#endif

#ifndef AP_NETWORKING_BOOTLOADER_DEFAULT_IP
#define AP_NETWORKING_BOOTLOADER_DEFAULT_IP "192.168.1.100"
#endif

#ifndef AP_NETWORKING_BOOTLOADER_DEFAULT_GATEWAY
#define AP_NETWORKING_BOOTLOADER_DEFAULT_GATEWAY "192.168.1.1"
#endif

#ifndef AP_NETWORKING_BOOTLOADER_DEFAULT_NETMASK
#define AP_NETWORKING_BOOTLOADER_DEFAULT_NETMASK "255.255.255.0"
#endif

#ifndef AP_BOOTLOADER_NETWORK_USE_DHCP
#define AP_BOOTLOADER_NETWORK_USE_DHCP 0
#endif

// firmware is received into RAM this much at a time between flash operations
#ifndef AP_BOOTLOADER_NETWORK_FLASH_CHUNK_SIZE
#define AP_BOOTLOADER_NETWORK_FLASH_CHUNK_SIZE 65536
#endif

#define LWIP_SEND_TIMEOUT_MS 50
#define LWIP_NETIF_MTU       1500
#define LWIP_LINK_POLL_INTERVAL TIME_S2I(5)

#define PERIODIC_TIMER_ID       1
#define FRAME_RECEIVED_ID       2

#define MIN(a,b) ((a)<(b)?(a):(b))

#if AP_NETWORKING_CAN_MCAST_ENABLED
static AP_Networking_CAN mcast_server;
#endif

void BL_Network::link_up_cb(void *p)
{
    auto *driver = (BL_Network *)p;
#if AP_BOOTLOADER_NETWORK_USE_DHCP
    dhcp_start(driver->thisif);
#endif
#if BL_NETWORK_HAVE_CAN
    char ipstr[IP4_STR_LEN];
    can_printf("IP %s", SocketAPM::inet_addr_to_str(ntohl(driver->thisif->ip_addr.addr), ipstr, sizeof(ipstr)));
#else
    (void)driver;
#endif

#if AP_NETWORKING_CAN_MCAST_ENABLED
    // start mcast CAN server
    mcast_server.start((1U<<HAL_NUM_CAN_IFACES)-1);
#endif
}

void BL_Network::link_down_cb(void *p)
{
#if AP_BOOTLOADER_NETWORK_USE_DHCP
    auto *driver = (BL_Network *)p;
    dhcp_stop(driver->thisif);
#endif
}

#if !AP_BOOTLOADER_NETWORK_PPP_ENABLED
/*
 * This function does the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
int8_t BL_Network::low_level_output(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;
    MACTransmitDescriptor td;
    (void)netif;

    if (macWaitTransmitDescriptor(&ETHD1, &td, TIME_MS2I(LWIP_SEND_TIMEOUT_MS)) != MSG_OK) {
        return ERR_TIMEOUT;
    }

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE);        /* drop the padding word */
#endif

    /* Iterates through the pbuf chain. */
    for(q = p; q != NULL; q = q->next) {
        macWriteTransmitDescriptor(&td, (uint8_t *)q->payload, (size_t)q->len);
    }
    macReleaseTransmitDescriptorX(&td);

#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE);         /* reclaim the padding word */
#endif

    return ERR_OK;
}

/*
 * Receives a frame.
 * Allocates a pbuf and transfers the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
bool BL_Network::low_level_input(struct netif *netif, struct pbuf **pbuf)
{
    MACReceiveDescriptor rd;
    struct pbuf *q;
    u16_t len;

    (void)netif;

    if (macWaitReceiveDescriptor(&ETHD1, &rd, TIME_IMMEDIATE) != MSG_OK) {
        return false;
    }

    len = (u16_t)rd.size;

#if ETH_PAD_SIZE
    len += ETH_PAD_SIZE;        /* allow room for Ethernet padding */
#endif

    /* We allocate a pbuf chain of pbufs from the pool. */
    *pbuf = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    if (*pbuf != nullptr) {
#if ETH_PAD_SIZE
        pbuf_header(*pbuf, -ETH_PAD_SIZE); /* drop the padding word */
#endif

        /* Iterates through the pbuf chain. */
        for(q = *pbuf; q != NULL; q = q->next) {
            macReadReceiveDescriptor(&rd, (uint8_t *)q->payload, (size_t)q->len);
        }
        macReleaseReceiveDescriptorX(&rd);

#if ETH_PAD_SIZE
        pbuf_header(*pbuf, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
    } else {
        macReleaseReceiveDescriptorX(&rd);     // Drop packet
    }
  
    return true;
}

int8_t BL_Network::ethernetif_init(struct netif *netif)
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
    // also enable multicast
    netif->flags |= NETIF_FLAG_IGMP;
#endif

    return ERR_OK;
}

/*
  networking thread
*/
void BL_Network::net_thread()
{
    /* start tcpip thread */
    tcpip_init(NULL, NULL);

    thread_sleep_ms(100);

    AP_Networking::convert_str_to_macaddr(AP_NETWORKING_BOOTLOADER_DEFAULT_MAC_ADDR, thisif->hwaddr);

    ip4_addr_t ip, gateway, netmask;
    if (addr.ip == 0) {
        // no IP from AP_Periph, use defaults
        ip.addr = htonl(SocketAPM::inet_str_to_addr(AP_NETWORKING_BOOTLOADER_DEFAULT_IP));
        gateway.addr = htonl(SocketAPM::inet_str_to_addr(AP_NETWORKING_BOOTLOADER_DEFAULT_GATEWAY));
        netmask.addr = htonl(SocketAPM::inet_str_to_addr(AP_NETWORKING_BOOTLOADER_DEFAULT_NETMASK));
    } else {
        // use addresses from AP_Periph
        ip.addr = htonl(addr.ip);
        netmask.addr = htonl(addr.netmask);
        gateway.addr = htonl(addr.gateway);
    }

    const MACConfig mac_config = {thisif->hwaddr};
    macStart(&ETHD1, &mac_config);

    /* Add interface. */

    auto result = netifapi_netif_add(thisif, &ip, &netmask, &gateway, NULL, ethernetif_init, tcpip_input);
    if (result != ERR_OK) {
        AP_HAL::panic("Failed to initialise netif");
    }

    netifapi_netif_set_default(thisif);
    netifapi_netif_set_up(thisif);

    /* Setup event sources.*/
    event_timer_t evt;
    event_listener_t el0, el1;
    
    evtObjectInit(&evt, LWIP_LINK_POLL_INTERVAL);
    evtStart(&evt);
    chEvtRegisterMask(&evt.et_es, &el0, PERIODIC_TIMER_ID);
    chEvtRegisterMaskWithFlags(macGetEventSource(&ETHD1), &el1,
                               FRAME_RECEIVED_ID, MAC_FLAGS_RX);
    chEvtAddEvents(PERIODIC_TIMER_ID | FRAME_RECEIVED_ID);

    while (true) {
        eventmask_t mask = chEvtWaitAny(ALL_EVENTS);
        if (mask & PERIODIC_TIMER_ID) {
            bool current_link_status = macPollLinkStatus(&ETHD1);
            if (current_link_status != netif_is_link_up(thisif)) {
                if (current_link_status) {
                    tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_up, thisif, 0);
                    tcpip_callback_with_block(link_up_cb, this, 0);
                }
                else {
                    tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_down, thisif, 0);
                    tcpip_callback_with_block(link_down_cb, this, 0);
                }
            }
        }

        if (mask & FRAME_RECEIVED_ID) {
            struct pbuf *p;
            while (low_level_input(thisif, &p)) {
                if (p != NULL) {
                    struct eth_hdr *ethhdr = (struct eth_hdr *)p->payload;
                    switch (htons(ethhdr->type)) {
                        /* IP or ARP packet? */
                    case ETHTYPE_IP:
                    case ETHTYPE_ARP:
                        /* full packet send to tcpip_thread to process */
                        if (thisif->input(p, thisif) == ERR_OK) {
                            break;
                        }
                        /* Falls through */
                    default:
                        pbuf_free(p);
                    }
                }
            }
        }
    }
}

#else  // AP_BOOTLOADER_NETWORK_PPP_ENABLED

#ifndef AP_BOOTLOADER_NETWORK_PPP_BAUD
#define AP_BOOTLOADER_NETWORK_PPP_BAUD 921600
#endif

#ifndef AP_BOOTLOADER_NETWORK_PPP_FLOW_CONTROL
#define AP_BOOTLOADER_NETWORK_PPP_FLOW_CONTROL 0
#endif

#if AP_BOOTLOADER_NETWORK_PPP_FLOW_CONTROL && (!defined(AP_BOOTLOADER_NETWORK_PPP_RTS_LINE) || !defined(AP_BOOTLOADER_NETWORK_PPP_RTS_AF))
#error "PPP flow control needs AP_BOOTLOADER_NETWORK_PPP_RTS_LINE and AP_BOOTLOADER_NETWORK_PPP_RTS_AF"
#endif

#if !AP_NETWORKING_BACKEND_PPP
// lwIP calls this for every incoming PPP frame; the bootloader does
// not support packet capture. AP_Networking_PPP.cpp provides it when
// that backend is enabled
void ap_ppp_capture_hook(const struct ppp_pcb_s *pcb, const struct pbuf *pb)
{
}
#endif

/*
  start the serial port the PPP link runs over
 */
void BL_Network::ppp_start_serial(void)
{
    static SerialConfig ppp_sercfg;
    ppp_sercfg.speed = AP_BOOTLOADER_NETWORK_PPP_BAUD;
#if defined(STM32H7)
    // the H7 UART clock is 100MHz; rates above 6.25MBaud (up to
    // 12.5MBaud) need 8x rather than 16x oversampling
    if (ppp_sercfg.speed > 100000000UL / 16U) {
        ppp_sercfg.cr1 |= USART_CR1_OVER8;
    }
#endif
#ifdef USART_CR1_FIFOEN
    // the serial driver takes an interrupt per byte, and at high
    // baudrates kernel critical sections delay it for longer than two
    // byte times; the FIFO covers that and the driver drains it
    ppp_sercfg.cr1 |= USART_CR1_FIFOEN;
#endif
#if AP_BOOTLOADER_NETWORK_PPP_FLOW_CONTROL
    // hardware CTS, and hardware RTS, which is deasserted whenever the
    // receive FIFO is full, including while flash operations stall
    // the CPU
    ppp_sercfg.cr3 |= USART_CR3_CTSE | USART_CR3_RTSE;
#endif
    sdStart(&AP_BOOTLOADER_NETWORK_PPP_SERIAL, &ppp_sercfg);
#if AP_BOOTLOADER_NETWORK_PPP_FLOW_CONTROL
    // hwdef makes RTS a GPIO output for the HAL's software RTS; give
    // it to the UART instead
    palSetLineMode(AP_BOOTLOADER_NETWORK_PPP_RTS_LINE, PAL_MODE_ALTERNATE(AP_BOOTLOADER_NETWORK_PPP_RTS_AF));
#endif
}

/*
  send PPP data to the serial port
 */
uint32_t BL_Network::ppp_output_cb(struct ppp_pcb_s *pcb, const void *data, uint32_t len, void *ctx)
{
    return chnWriteTimeout(&AP_BOOTLOADER_NETWORK_PPP_SERIAL, (const uint8_t *)data, len, TIME_MS2I(LWIP_SEND_TIMEOUT_MS));
}

/*
  PPP link status change
 */
void BL_Network::ppp_status_cb(struct ppp_pcb_s *pcb, int code, void *ctx)
{
    auto *driver = (BL_Network *)ctx;
    switch (code) {
    case PPPERR_NONE:
        // link is up and we have our address from the peer
        link_up_cb(driver);
        break;
    case PPPERR_USER:
        break;
    default:
        // the link is dead; start negotiating again
        driver->ppp_need_restart = true;
        break;
    }
}

/*
  networking thread
*/
void BL_Network::net_thread()
{
    /* start tcpip thread */
    tcpip_init(NULL, NULL);

    thread_sleep_ms(100);

    ppp_start_serial();

    // our address comes from the peer over IPCP
    LOCK_TCPIP_CORE();
    ppp = pppos_create(thisif, ppp_output_cb, ppp_status_cb, this);
    if (ppp == nullptr) {
        AP_HAL::panic("Failed to create PPP link");
    }
    netif_set_default(thisif);
    ppp_connect(ppp, 0);
    UNLOCK_TCPIP_CORE();

    uint8_t buf[256];
    while (true) {
        if (ppp_need_restart) {
            ppp_need_restart = false;
            LOCK_TCPIP_CORE();
            ppp_connect(ppp, 0);
            UNLOCK_TCPIP_CORE();
        }

        // wait for some data, then take whatever else is buffered
        size_t n = chnReadTimeout(&AP_BOOTLOADER_NETWORK_PPP_SERIAL, buf, 1, TIME_MS2I(100));
        if (n == 0) {
            continue;
        }
        n += chnReadTimeout(&AP_BOOTLOADER_NETWORK_PPP_SERIAL, &buf[1], sizeof(buf)-1, TIME_IMMEDIATE);

        LOCK_TCPIP_CORE();
        pppos_input(ppp, buf, n);
        UNLOCK_TCPIP_CORE();
    }
}

#endif  // AP_BOOTLOADER_NETWORK_PPP_ENABLED

void BL_Network::net_thread_trampoline(void *ctx)
{
    auto *net = (BL_Network *)ctx;
    net->net_thread();
}

void BL_Network::web_server_trampoline(void *ctx)
{
    auto *net = (BL_Network *)ctx;
    net->web_server();
}

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

BL_Network::web_var BL_Network::variables[] = {
    { "BOARD_NAME", HAL_BOARD_NAME },
    { "BOARD_ID", STR(APJ_BOARD_ID)  },
    { "FLASH_SIZE", STR(BOARD_FLASH_SIZE)  },
};

/*
  substitute variables of the form {VARNAME}
 */
char *BL_Network::substitute_vars(const char *str, uint32_t size)
{
    // assume 1024 is enough room for new variables
    char *result = (char *)malloc(strlen(str) + 1024);
    if (result == nullptr) {
        return nullptr;
    }
    char *p = result;
    const char *str0 = str;
    while (*str && str-str0<size) {
        if (*str != '{') {
            *p++ = *str++;
            continue;
        }
        char *q = strchr(str+1, '}');
        if (q == nullptr) {
            *p++ = *str++;
            continue;
        }
        const uint32_t len = (q - str)-1;
        bool found = false;
        for (auto &v : variables) {
            if (strlen(v.name) == len && strncmp(v.name, str+1, len) == 0) {
                found = true;
                strcpy(p, v.value);
                p += strlen(v.value);
                str = q+1;
                break;
            }
        }
        if (found) {
            continue;
        }
        *p++ = *str++;
    }
    *p = '\0';
    return result;
}

/*
  read HTTP headers, returing as a single string
 */
char *BL_Network::read_headers(SocketAPM *sock)
{
    char *ret = (char *)malloc(1024);
    char *p = ret;
    while (true) {
        char c;
        auto n = sock->recv(&c, 1, 100);
        if (n != 1) {
            break;
        }
        *p++ = c;
        if (p-ret >= 4 && strcmp(p-4, "\r\n\r\n") == 0) {
            break;
        }
    }
    return ret;
}

/*
  wait until the sender is blocked by our TCP receive window, or has
  nothing left to send, so that nothing arrives while flash is
  busy. remaining is the number of bytes the sender has yet to send
  that we have not read
 */
void BL_Network::wait_sender_blocked(SocketAPM *sock, uint32_t remaining)
{
    // the sender may hold back less than a segment rather than send a
    // runt, so a window within a segment of full counts as full
    const uint32_t blocked = MIN(remaining, uint32_t(TCP_WND - TCP_MSS));
    int last_avail = -1;
    const uint32_t start_ms = AP_HAL::millis();
    uint32_t last_change_ms = start_ms;
    while (true) {
        int avail = 0;
        lwip_ioctl(sock->get_read_fd(), FIONREAD, &avail);
        const uint32_t now_ms = AP_HAL::millis();
        if (avail != last_avail) {
            last_avail = avail;
            last_change_ms = now_ms;
        }
        const uint32_t quiet_ms = now_ms - last_change_ms;
        if ((uint32_t(avail) >= blocked && quiet_ms >= 20) ||
            quiet_ms >= 200 ||
            now_ms - start_ms >= 2000) {
            break;
        }
        thread_sleep_ms(5);
    }
}

/*
  handle an incoming HTTP POST request
 */
void BL_Network::handle_post(SocketAPM *sock, uint32_t content_length)
{
    /*
      skip over form boundary. We don't care about the filename as we
      only support one file
     */
    uint8_t state = 0;
    while (true) {
        char c;
        if (sock->recv(&c, 1, 100) != 1) {
            return;
        }
        content_length--;
        // absorb up to \r\n\r\n
        if (c == '\r') {
            if (state == 2) {
                state = 3;
            } else {
                state = 1;
            }
        } else if (c == '\n') {
            if (state == 1 || state == 3) {
                state++;
            } else {
                state = 0;
            }
            if (state == 4) {
                break;
            }
        } else {
            state = 0;
        }
    }
    /*
      receive the file a chunk at a time into RAM, erasing flash
      sectors as the chunks reach them. Erasing and writing flash can
      stall the CPU, and the network interface with it, for long
      enough to lose incoming data, so before touching flash we wait
      for the sender to be blocked by our TCP receive window
     */
    uint32_t chunk_size = AP_BOOTLOADER_NETWORK_FLASH_CHUNK_SIZE;
    uint32_t *chunk = nullptr;
    while (chunk_size >= 1024 && (chunk = (uint32_t *)malloc(chunk_size)) == nullptr) {
        chunk_size /= 2;
    }
    if (chunk == nullptr) {
        status_printf("Flash failed: no memory");
        return;
    }

    uint32_t flash_size = 0;
    for (uint32_t sec=0; flash_func_sector_size(sec) != 0; sec++) {
        flash_size += flash_func_sector_size(sec);
    }

    // the body after the form headers: the file, then the closing form boundary
    const uint32_t body_length = content_length;

    // must be multiple of 4
    content_length &= ~3;

    const uint32_t max_ofs = MIN(flash_size, content_length);
    uint32_t ofs = 0;
    uint32_t erased_ofs = 0;
    uint32_t erase_sec = 0;
    bool failed = false;
    uint8_t last_pct = 0;
    status_printf("Flashing 0%%");
    flash_set_keep_unlocked(true);
    while (ofs < max_ofs && !failed) {
        const uint32_t want = MIN(chunk_size, max_ofs - ofs);
        uint32_t len = 0;
        while (len < want) {
            const auto n = sock->recv(((uint8_t *)chunk) + len, want - len, 10000);
            if (n <= 0) {
                break;
            }
            len += n;
        }
        if (len < want) {
            status_printf("Flash failed: upload stopped at %u of %u", unsigned(ofs+len), unsigned(max_ofs));
            failed = true;
            break;
        }
        wait_sender_blocked(sock, content_length - (ofs + len));
        while (erased_ofs < ofs + len) {
            const uint32_t size = flash_func_sector_size(erase_sec);
            if (size == 0 || !flash_func_erase_sector(erase_sec)) {
                status_printf("Flash failed: erase of sector %u", unsigned(erase_sec));
                failed = true;
                break;
            }
            erased_ofs += size;
            erase_sec++;
        }
        for (uint32_t i=0; i<len && !failed; i += 512) {
            const uint32_t n = MIN(512U, len - i);
            if (!flash_write_buffer(ofs + i, &chunk[i/4], n/4)) {
                status_printf("Flash failed: write at %u", unsigned(ofs + i));
                failed = true;
            }
        }
        ofs += len;
        const uint8_t pct = ofs*100/max_ofs;
        // don't replace the reason for a failure
        if (!failed && pct / 10 != last_pct / 10) {
            last_pct = pct;
            status_printf("Flashing %u%%", unsigned(pct));
        }
    }
    if (!failed) {
        // read the rest of the body; closing the socket with data
        // unread resets the connection, losing our reply
        uint8_t discard[64];
        uint32_t left = body_length - ofs;
        while (left > 0) {
            const auto n = sock->recv(discard, MIN(sizeof(discard), left), 1000);
            if (n <= 0) {
                break;
            }
            left -= n;
        }
    }
    if (!failed && ofs % 32 != 0) {
        // pad to 32 bytes
        memset(chunk, 0xff, 32);
        flash_write_buffer(ofs, chunk, (32 - ofs%32)/4);
    }
    free(chunk);
    // flush even after a failure, so that nothing of this upload is
    // left buffered to be written into the next one
    const bool flushed = flash_write_flush();
    if (!failed && !flushed) {
        status_printf("Flash failed: write");
        failed = true;
    }
    flash_set_keep_unlocked(false);
    if (failed) {
        const char *str = "<html><body>Flash failed, see status</body></html>";
        sock->send(str, strlen(str));
        return;
    }
#if AP_CHECK_FIRMWARE_ENABLED
    const auto ok = check_good_firmware();
#else
    const auto ok = check_fw_result_t::CHECK_FW_OK;
#endif
    if (ok == check_fw_result_t::CHECK_FW_OK) {
        need_launch = true;
        status_printf("Flash done: OK");
        const char *str = "<html><head><meta http-equiv=\"refresh\" content=\"4; url=/\"></head><body>Flash OK, booting</body></html>";
        sock->send(str, strlen(str));
    } else {
        status_printf("Flash done: ERR:%u", unsigned(ok));
    }
}

/*
  handle an incoming HTTP request
 */
void BL_Network::handle_request(SocketAPM *sock)
{
    /*
      read HTTP headers
     */
    char *headers = read_headers(sock);

    const char *header = "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Connection: close\r\n"
        "\r\n";
    sock->send(header, strlen(header));

    if (strncmp(headers, "POST / ", 7) == 0) {
        const char *clen1 = "\r\nContent-Length:";
        const char *clen2 = "\r\ncontent-length:";
        const char *p = strstr(headers, clen1);
        if (p == nullptr) {
            p = strstr(headers, clen2);
        }
        if (p != nullptr) {
            p += strlen(clen1);
            const uint32_t content_length = atoi(p);
            handle_post(sock, content_length);
            delete headers;
            delete sock;
            return;
        }
    }

    /*
      check for async status
     */
    const char *get_status = "GET /bootloader_status.html";
    if (strncmp(headers, get_status, strlen(get_status)) == 0) {
        {
            WITH_SEMAPHORE(status_mtx);
            sock->send(bl_status, strlen(bl_status));
        }
        delete headers;
        delete sock;
        return;
    }

    const char *get_reboot = "GET /REBOOT";
    if (strncmp(headers, get_reboot, strlen(get_reboot)) == 0) {
        need_reboot = true;
    }
    
    uint32_t size = 0;
    const auto *msg = AP_ROMFS::find_decompress("index.html", size);
    if (need_reboot) {
        const char *str = "<html><head><meta http-equiv=\"refresh\" content=\"2; url=/\"></head></html>";
        sock->send(str, strlen(str));
    } else {
        char *msg2 = substitute_vars((const char *)msg, size);
        sock->send(msg2, strlen(msg2));
        delete msg2;
    }
    delete headers;
    delete sock;
    AP_ROMFS::free(msg);
}

struct req_context {
    BL_Network *driver;
    SocketAPM *sock;
};

void BL_Network::net_request_trampoline(void *ctx)
{
    auto *req = (req_context *)ctx;
    req->driver->handle_request(req->sock);

    auto *driver = req->driver;
    auto *thd = chThdGetSelfX();
    delete req;

    WITH_SEMAPHORE(driver->web_delete_mtx);
    thd->delete_next = driver->web_delete_list;
    driver->web_delete_list = thd;
}

/*
  web server thread
 */
void BL_Network::web_server(void)
{
    auto *listen_socket = NEW_NOTHROW SocketAPM(0);
    listen_socket->bind("0.0.0.0", 80);
    listen_socket->listen(20);

    while (true) {
        auto *sock = listen_socket->accept(20);
        if (need_reboot) {
            need_reboot = false;
            NVIC_SystemReset();
        }
        if (need_launch) {
            need_launch = false;
            thread_sleep_ms(1000);
            jump_to_app();
        }
        if (sock == nullptr) {
            continue;
        }
        // a new thread for each connection to allow for AJAX
        auto *req = NEW_NOTHROW req_context;
        req->driver = this;
        req->sock = sock;
        thread_create_alloc(THD_WORKING_AREA_SIZE(2048),
                            "web_request",
                            60,
                            net_request_trampoline,
                            req);

        // cleanup any finished threads
        WITH_SEMAPHORE(web_delete_mtx);
        while (web_delete_list != nullptr) {
            auto *thd = web_delete_list;
            web_delete_list = thd->delete_next;
            chThdRelease(thd);
        }
    }
}

/*
  initialise bootloader networking
 */
void BL_Network::init()
{
#if AP_BOOTLOADER_NETWORK_PPP_ENABLED
    // received PPP data is processed through the whole IP stack on
    // the network thread
    const uint32_t net_thread_stack = 4096;
#else
    AP_Networking_ChibiOS::allocate_buffers();

    macInit();

    const uint32_t net_thread_stack = 2048;
#endif

    thisif = NEW_NOTHROW netif;

    net_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(net_thread_stack),
                                         "network",
                                         60,
                                         net_thread_trampoline,
                                         this);

    thread_create_alloc(THD_WORKING_AREA_SIZE(2048),
                        "web_server",
                        60,
                        web_server_trampoline,
                        this);
}

/*
  restore IP address stashed away by AP_Periph
 */
void BL_Network::restore_comms_ip(void)
{
    struct app_bootloader_comms *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
    if (comms->magic == APP_BOOTLOADER_COMMS_MAGIC && comms->ip != 0) {
        addr.ip = comms->ip;
        addr.netmask = comms->netmask;
        addr.gateway = comms->gateway;
    }
}

/*
  update status message
 */
void BL_Network::status_printf(const char *fmt, ...)
{
    WITH_SEMAPHORE(status_mtx);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(bl_status, sizeof(bl_status), fmt, ap);
    va_end(ap);
#if BL_NETWORK_HAVE_CAN
    can_printf("%s", bl_status);
#endif
}

#endif // AP_BOOTLOADER_NETWORK_ENABLED

