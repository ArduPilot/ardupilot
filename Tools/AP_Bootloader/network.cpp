/*
  support for networking enabled bootloader
 */

#include "network.h"

#if AP_BOOTLOADER_NETWORK_ENABLED

#include <AP_Networking/AP_Networking.h>
#include <AP_Networking/AP_Networking_ChibiOS.h>

#include <lwip/ip_addr.h>
#include <lwip/tcpip.h>
#include <lwip/netifapi.h>
#include <lwip/etharp.h>
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

#define LWIP_SEND_TIMEOUT_MS 50
#define LWIP_NETIF_MTU       1500
#define LWIP_LINK_POLL_INTERVAL TIME_S2I(5)

#define PERIODIC_TIMER_ID       1
#define FRAME_RECEIVED_ID       2

#define MIN(a,b) ((a)<(b)?(a):(b))

void BL_Network::link_up_cb(void *p)
{
#if AP_BOOTLOADER_NETWORK_USE_DHCP
    auto *driver = (AP_Networking_ChibiOS *)p;
    dhcp_start(driver->thisif);
#endif
}

void BL_Network::link_down_cb(void *p)
{
#if AP_BOOTLOADER_NETWORK_USE_DHCP
    auto *driver = (BL_Network *)p;
    dhcp_stop(driver->thisif);
#endif
}

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
    { "BOARD_NAME", CHIBIOS_BOARD_NAME },
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
      erase all of flash
     */
    flash_set_keep_unlocked(true);
    uint32_t sec=0;
    while (flash_func_erase_sector(sec)) {
        sec++;
    }
    /*
      receive file and write to flash
     */
    uint32_t buf[128];
    uint32_t ofs = 0;

    // must be multiple of 4
    content_length &= ~3;

    const uint32_t max_ofs = MIN(BOARD_FLASH_SIZE*1024, content_length);
    while (ofs < max_ofs) {
        const uint32_t needed = MIN(sizeof(buf), max_ofs-ofs);
        auto n = sock->recv((void*)buf, needed, 10000);
        if (n <= 0) {
            break;
        }
        // we need a whole number of words
        if (n % 4 != 0 && n < needed) {
            auto n2 = sock->recv(((uint8_t*)buf)+n, 4 - n%4, 10000);
            if (n2 > 0) {
                n += n2;
            }
        }
        flash_write_buffer(ofs, buf, n/4);
        ofs += n;
    }
    if (ofs % 32 != 0) {
        // pad to 32 bytes
        memset(buf, 0xff, sizeof(buf));
        flash_write_buffer(ofs, buf, (32 - ofs%32)/4);
    }
    flash_write_flush();
    flash_set_keep_unlocked(false);
    const auto ok = check_good_firmware();
    if (ok == check_fw_result_t::CHECK_FW_OK) {
        jump_to_app();
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

    if (strncmp(headers, "POST / ", 7) == 0) {
        const char *clen = "\r\nContent-Length:";
        const char *p = strstr(headers, clen);
        if (p != nullptr) {
            p += strlen(clen);
            const uint32_t content_length = atoi(p);
            handle_post(sock, content_length);
        }
    }

    uint32_t size = 0;
    /*
      we only need one URL in the bootloader
    */
    const char *header = "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Connection: close\r\n"
        "\r\n";
    const auto *msg = AP_ROMFS::find_decompress("index.html", size);
    sock->send(header, strlen(header));
    char *msg2 = substitute_vars((const char *)msg, size);
    sock->send(msg2, strlen(msg2));
    delete msg2;
    delete headers;
    AP_ROMFS::free(msg);
}

/*
  web server thread
 */
void BL_Network::web_server(void)
{
    auto *listen_socket = new SocketAPM(0);
    listen_socket->bind("0.0.0.0", 80);
    listen_socket->listen(20);

    while (true) {
        auto *sock = listen_socket->accept(20);
        if (sock == nullptr) {
            continue;
        }
        handle_request(sock);
        delete sock;
    }
}

/*
  initialise bootloader networking
 */
void BL_Network::init()
{
    AP_Networking_ChibiOS::allocate_buffers();

    macInit();

    thisif = new netif;

    net_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(2048),
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
  save IP address from AP_Periph
 */
void BL_Network::save_comms_ip(void)
{
    struct app_bootloader_comms *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
    if (comms->magic == APP_BOOTLOADER_COMMS_MAGIC && comms->ip != 0) {
        addr.ip = comms->ip;
        addr.netmask = comms->netmask;
        addr.gateway = comms->gateway;
    }
}

#endif // AP_BOOTLOADER_NETWORK_ENABLED

