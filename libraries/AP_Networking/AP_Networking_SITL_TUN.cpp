/*
  SITL networking backend: bridge lwIP to a Linux TAP device.

  AP_FLAKE8_CLEAN
 */

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SITL_TUN

#include "AP_Networking_SITL_TUN.h"
#include "AP_Networking.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/if.h>
#include <linux/if_tun.h>

#include <lwip/tcpip.h>
#include <lwip/netifapi.h>
#include <lwip/etharp.h>

extern const AP_HAL::HAL& hal;

#if LWIP_TCPIP_CORE_LOCKING
#define LWIP_TCPIP_LOCK() sys_lock_tcpip_core()
#define LWIP_TCPIP_UNLOCK() sys_unlock_tcpip_core()
#else
#define LWIP_TCPIP_LOCK()
#define LWIP_TCPIP_UNLOCK()
#endif

#define SITL_TUN_NETIF_MTU 1500

#ifndef SITL_TUN_DEFAULT_DEV
#define SITL_TUN_DEFAULT_DEV "sitltap"
#endif

/*
  initialise the TAP-backed lwIP netif. The TAP device must already exist
  (use Tools/scripts/Networking/sitl_network.sh up).
 */
bool AP_Networking_SITL_TUN::init()
{
    const char *dev = getenv("SITL_TUN_DEV");
    if (dev == nullptr) {
        dev = SITL_TUN_DEFAULT_DEV;
    }
    snprintf(ifname, sizeof(ifname), "%s", dev);

    tap_fd = open("/dev/net/tun", O_RDWR);
    if (tap_fd >= 0) {
        struct ifreq ifr {};
        ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
        snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname);
        if (ioctl(tap_fd, TUNSETIFF, &ifr) < 0) {
            close(tap_fd);
            tap_fd = -1;
        }
    }
    if (tap_fd < 0) {
        // TAP not available - skip the host bridge but still expose IP
        // params via activeSettings so a paired PPP backend can still do
        // proper IPCP-aware PPPGW. Tools/scripts/Networking/sitl_network.sh up
        // brings the TAP device up if you want host-side reachability.
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "NET: TAP %s unavailable; PPPGW running without host bridge",
                      ifname);
        activeSettings.ip = frontend.get_ip_param();
        activeSettings.nm = frontend.get_netmask_param();
        activeSettings.gw = frontend.get_gateway_param();
        activeSettings.last_change_ms = AP_HAL::millis();
        return true;
    }

    // bring lwIP up if no other backend has done so yet
    LWIP_TCPIP_LOCK();
    tcpip_init(NULL, NULL);
    LWIP_TCPIP_UNLOCK();

    thisif = NEW_NOTHROW netif;
    if (thisif == nullptr) {
        return false;
    }

    // MAC address: take the user-configured one (with platform-UUID suffix
    // already mixed in by AP_Networking::init).
    frontend.param.macaddr.get_address(macaddr);

    ip4_addr_t ip, nm, gw;
    ip.addr = htonl(frontend.get_ip_param());
    nm.addr = htonl(frontend.get_netmask_param());
    gw.addr = htonl(frontend.get_gateway_param());

    auto result = netifapi_netif_add(thisif, &ip, &nm, &gw,
                                     this, tap_netif_init, tcpip_input);
    if (result != ERR_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: netif_add failed (%d)", (int)result);
        return false;
    }
    netifapi_netif_set_default(thisif);
    netifapi_netif_set_up(thisif);
    netifapi_netif_set_link_up(thisif);

    // seed activeSettings so PPP-gateway-mode IPCP setup sees real IPs
    activeSettings.ip = ntohl(ip.addr);
    activeSettings.nm = ntohl(nm.addr);
    activeSettings.gw = ntohl(gw.addr);
    activeSettings.last_change_ms = AP_HAL::millis();

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_Networking_SITL_TUN::thread, void),
            "tap_net", 8192, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: tap_net thread create failed");
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: TAP %s up", ifname);
    return true;
}

/*
  lwIP netif init callback. The TAP device gives us raw ethernet frames so
  this is set up as a standard L2 netif with ARP support.
 */
int8_t AP_Networking_SITL_TUN::tap_netif_init(struct netif *netif)
{
    auto *driver = (AP_Networking_SITL_TUN *)netif->state;

    netif->name[0] = 't';
    netif->name[1] = 'p';
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    memcpy(netif->hwaddr, driver->macaddr, ETHARP_HWADDR_LEN);
    netif->mtu = SITL_TUN_NETIF_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#if LWIP_IGMP
    netif->flags |= NETIF_FLAG_IGMP;
#endif

    // stash the backend pointer for the linkoutput callback to find later
    netif->state = driver;
    return ERR_OK;
}

/*
  lwIP wants to transmit a frame. Write the pbuf chain to the TAP fd.
 */
int8_t AP_Networking_SITL_TUN::low_level_output(struct netif *netif, struct pbuf *p)
{
    auto *driver = (AP_Networking_SITL_TUN *)netif->state;
    uint8_t buf[SITL_TUN_NETIF_MTU + 18];   // worst-case ethernet frame
    if (p->tot_len > sizeof(buf)) {
        return ERR_BUF;
    }
    const u16_t n = pbuf_copy_partial(p, buf, p->tot_len, 0);
    const ssize_t w = write(driver->tap_fd, buf, n);
    if (w != n) {
        return ERR_IF;
    }
    return ERR_OK;
}

/*
  reader thread: pull frames off the TAP fd and hand them to lwIP.
 */
void AP_Networking_SITL_TUN::thread(void)
{
    while (true) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(tap_fd, &rfds);
        struct timeval tv = { 0, 100000 };  // 100ms
        const int r = select(tap_fd + 1, &rfds, NULL, NULL, &tv);
        if (r <= 0 || !FD_ISSET(tap_fd, &rfds)) {
            continue;
        }

        uint8_t buf[SITL_TUN_NETIF_MTU + 18];
        const ssize_t n = read(tap_fd, buf, sizeof(buf));
        if (n <= 0) {
            continue;
        }

        struct pbuf *p = pbuf_alloc(PBUF_RAW, n, PBUF_POOL);
        if (p == nullptr) {
            continue;
        }
        if (pbuf_take(p, buf, n) != ERR_OK) {
            pbuf_free(p);
            continue;
        }

        // hand off to lwIP TCP/IP thread; netif->input == tcpip_input,
        // which takes ownership of the pbuf (or frees it on error)
        if (thisif->input(p, thisif) != ERR_OK) {
            pbuf_free(p);
        }
    }
}

void AP_Networking_SITL_TUN::update()
{
    // nothing to do; activeSettings is static for the TAP backend
}

#endif // AP_NETWORKING_BACKEND_SITL_TUN
