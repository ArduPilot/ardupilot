/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by @davidbuzz and Claude
 */
#include "WiFiDriver.h"

#if defined(AP_ZEPHYR_WIFI_ENABLED) && AP_ZEPHYR_WIFI_ENABLED

/* Zephyr headers MUST come before AP_Math: AP_Math.h does `#undef MAX`, which
 * breaks Zephyr's own use of it. */
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/sys/printk.h>

#include <AP_Math/AP_Math.h>

#include <string.h>
#include <errno.h>

using namespace Zephyr;

extern const AP_HAL::HAL &hal;

/* Monitor stand-down window (Scheduler.cpp); nonzero = uptime-ms until
   which the main-loop watchdog must not reset. */
extern "C" volatile uint32_t ap_zephyr_grace_until_ms;

/* Blob API (esp_wifi.h, units of 0.25 dBm). Declared directly - the
   esp-idf headers are not on the AP include path. */
extern "C" int esp_wifi_set_max_tx_power(int8_t power);

/* ── shared softAP bring-up ─────────────────────────────────────────── */

static struct net_mgmt_event_callback s_wifi_ap_cb;

/* Event state, written by the handler and PRINTED FROM OUR OWN THREAD, because
 * the handler runs in a context where printing can deadlock. */
static volatile int s_ap_enable_status = -1000;   /* -1000 = no event yet */
static volatile uint8_t s_ap_enable_events;
static volatile uint8_t s_sta_joined_events, s_sta_left_events;
static volatile uint8_t s_sta_last_mac[6];
/* AP-death forensics (2026-08-16): the softAP silently vanished after
   minutes of unattended running - vehicle healthy, WiFi task not even
   error-looping. Every wifi mgmt event is recorded so the reporter can
   narrate the death live on the next occurrence. */
static volatile uint32_t s_any_event_count;
static volatile uint64_t s_last_event_id;
static volatile uint64_t s_event_ring[8];
static volatile uint8_t s_ap_disable_events;

static void wifi_ap_event_handler(struct net_mgmt_event_callback *cb,
                                  uint64_t mgmt_event, struct net_if *iface)
{
    s_event_ring[s_any_event_count & 7] = mgmt_event;
    s_any_event_count++;
    s_last_event_id = mgmt_event;
    if (mgmt_event == NET_EVENT_WIFI_AP_ENABLE_RESULT) {
        const struct wifi_status *st = (const struct wifi_status *)cb->info;
        s_ap_enable_status = st ? st->status : -999;
        s_ap_enable_events++;
        return;
    }
    if (mgmt_event == NET_EVENT_WIFI_AP_DISABLE_RESULT) {
        s_ap_disable_events++;
        return;
    }
    if (mgmt_event == NET_EVENT_WIFI_AP_STA_CONNECTED ||
        mgmt_event == NET_EVENT_WIFI_AP_STA_DISCONNECTED) {
        const struct wifi_ap_sta_info *sta =
            (const struct wifi_ap_sta_info *)cb->info;
        for (int i = 0; i < 6; i++) {
            s_sta_last_mac[i] = sta->mac[i];
        }
        if (mgmt_event == NET_EVENT_WIFI_AP_STA_CONNECTED) {
            s_sta_joined_events++;
        } else {
            s_sta_left_events++;
        }
    }
}

/* Called periodically from the TCP service thread (preemptible, low
   stakes) to report event-state changes on the console. */
static void wifi_report_events(void)
{
    static uint8_t seen_enable, seen_join, seen_left;
    if (seen_enable != s_ap_enable_events) {
        seen_enable = s_ap_enable_events;
        printk("WiFi: AP enable result status=%d\n", s_ap_enable_status);
    }
    if (seen_join != s_sta_joined_events) {
        seen_join = s_sta_joined_events;
        printk("WiFi: client %02x:%02x:%02x:%02x:%02x:%02x joined "
               "(UDP:14550 broadcasting to it now)\n",
               s_sta_last_mac[0], s_sta_last_mac[1], s_sta_last_mac[2],
               s_sta_last_mac[3], s_sta_last_mac[4], s_sta_last_mac[5]);
    }
    if (seen_left != s_sta_left_events) {
        seen_left = s_sta_left_events;
        printk("WiFi: a client left\n");
    }
    static uint8_t seen_disable;
    if (seen_disable != s_ap_disable_events) {
        seen_disable = s_ap_disable_events;
        printk("WiFi: *** AP DISABLE event ***\n");
    }
    static uint32_t seen_any;
    while (seen_any < s_any_event_count) {
        const uint64_t id = s_event_ring[seen_any & 7];
        seen_any++;
        printk("WiFi: mgmt event #%u id=0x%08x%08x\n", (unsigned)seen_any,
               (unsigned)(id >> 32), (unsigned)(id & 0xffffffff));
    }
    /* 30 s AP liveness heartbeat: polls the interface's oper state so a
       silent beacon death shows up in the console record even when no
       mgmt event fired at all */
    static int64_t last_hb_ms;
    const int64_t now = k_uptime_get();
    if (now - last_hb_ms >= 30000) {
        last_hb_ms = now;
        struct net_if *iface = net_if_get_first_wifi();
        printk("WiFi: heartbeat iface %s admin-%s oper=%d\n",
               iface ? "ok" : "NULL",
               (iface && net_if_is_admin_up(iface)) ? "up" : "DOWN",
               iface ? (int)net_if_oper_state(iface) : -1);
    }
}

K_MUTEX_DEFINE(s_wifi_start_mutex);

bool Zephyr::wifi_softap_start(void)
{
    static bool started;
    k_mutex_lock(&s_wifi_start_mutex, K_FOREVER);
    if (started) {
        k_mutex_unlock(&s_wifi_start_mutex);
        return true;
    }

    /* Radio start drops the USB Serial-JTAG link for a moment; console
       writes during that flap busy-wait in poll_out and can stall even
       the main loop past the monitor's reset limit. Tell the monitor to
       stand down across the bring-up window. */
    ap_zephyr_grace_until_ms = k_uptime_get_32() + 8000;

    net_mgmt_init_event_callback(&s_wifi_ap_cb, wifi_ap_event_handler,
                                 NET_EVENT_WIFI_AP_ENABLE_RESULT |
                                 NET_EVENT_WIFI_AP_DISABLE_RESULT |
                                 NET_EVENT_WIFI_AP_STA_CONNECTED |
                                 NET_EVENT_WIFI_AP_STA_DISCONNECTED);
    net_mgmt_add_event_callback(&s_wifi_ap_cb);

    struct net_if *iface = net_if_get_wifi_sap();
    if (iface == nullptr) {
        /* single-iface builds register the one WiFi iface as STA+SAP */
        iface = net_if_get_first_wifi();
    }
    if (iface == nullptr) {
        printk("WiFi: no WiFi interface (CONFIG_WIFI_ESP32 missing?)\n");
        k_mutex_unlock(&s_wifi_start_mutex);
        return false;
    }

    /* Zephyr network interfaces can start administratively down; the AP
       iface must be up before (or as) the radio starts beaconing. */
    net_if_up(iface);

    static const char ssid[] = HAL_ZEPHYR_WIFI_SSID;
    static const char psk[]  = HAL_ZEPHYR_WIFI_PASSWORD;
    struct wifi_connect_req_params ap = {};
    ap.ssid = (const uint8_t *)ssid;
    ap.ssid_length = strlen(ssid);
    ap.psk = (const uint8_t *)psk;
    ap.psk_length = strlen(psk);
    ap.channel = HAL_ZEPHYR_WIFI_CHANNEL;
    ap.band = WIFI_FREQ_BAND_2_4_GHZ;
    ap.security = WIFI_SECURITY_TYPE_PSK;

    int ret = net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, iface, &ap, sizeof(ap));
    if (ret != 0) {
        printk("WiFi: AP enable failed (%d)\n", ret);
        k_mutex_unlock(&s_wifi_start_mutex);
        return false;
    }

    /* Cap TX power at 10 dBm: the default 20 dBm browns out small boards' 3.3 V rail. */
    ret = esp_wifi_set_max_tx_power(40);
    if (ret != 0) {
        printk("WiFi: set_max_tx_power failed (%d)\n", ret);
    }

    /* static 192.168.4.1/24, the AP_HAL_ESP32 convention */
    struct in_addr addr, netmask, base;
    addr.s_addr    = htonl(0xC0A80401);   /* 192.168.4.1 */
    netmask.s_addr = htonl(0xFFFFFF00);   /* 255.255.255.0 */
    base.s_addr    = htonl(0xC0A80402);   /* DHCP pool from .2 */
    if (net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0) == nullptr) {
        printk("WiFi: static IP add failed\n");
    }
    net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask);

    ret = net_dhcpv4_server_start(iface, &base);
    if (ret != 0) {
        printk("WiFi: DHCPv4 server start failed (%d)\n", ret);
        /* not fatal: a client with a static IP still works */
    }

    printk("WiFi: softAP '%s' up, 192.168.4.1 (TCP:5760/UDP:14550 per SERIAL_ORDER)\n",
           ssid);

    started = true;
    k_mutex_unlock(&s_wifi_start_mutex);
    return true;
}

/* ── TCP server driver (:5760) ──────────────────────────────────────── */

WiFiDriver::WiFiDriver()
{
    _state = NOT_INITIALIZED;
    _accept_socket = -1;
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION; ++i) {
        _socket_list[i] = -1;
    }
}

void WiFiDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (_state != NOT_INITIALIZED) {
        return;
    }
    _readbuf.set_size(MAX(rxS, (uint16_t)RX_BUF_SIZE));
    _writebuf.set_size(MAX(txS, (uint16_t)TX_BUF_SIZE));
    _state = INITIALIZED;

    /* +2: PREEMPT(7), ABOVE the main loop (8). On a saturated single core the socket
     * service thread must outrank the loop or it never runs. */
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&WiFiDriver::_wifi_thread_fn, void),
            "APM_WIFI_TCP", 4096, AP_HAL::Scheduler::PRIORITY_UART, 2)) {
        printk("WiFi: TCP thread create failed\n");
        _state = NOT_INITIALIZED;
    }
}

void WiFiDriver::_end()
{
    /* sockets are owned by the service thread; nothing safe to do here */
}

void WiFiDriver::_flush()
{
}

bool WiFiDriver::is_initialized()
{
    return _state != NOT_INITIALIZED;
}

bool WiFiDriver::tx_pending()
{
    return _writebuf.available() > 0;
}

uint32_t WiFiDriver::_available()
{
    if (_state != CONNECTED) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t WiFiDriver::txspace()
{
    if (_state != CONNECTED) {
        return 0;
    }
    /* reserve a quarter so a burst can never wedge the ring full,
       matching AP_HAL_ESP32 */
    const int result = _writebuf.space() - TX_BUF_SIZE / 4;
    return MAX(result, 0);
}

ssize_t WiFiDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (_state != CONNECTED) {
        return 0;
    }
    return _readbuf.read(buffer, count);
}

size_t WiFiDriver::_write(const uint8_t *buffer, size_t size)
{
    if (_state != CONNECTED) {
        return 0;
    }
    WITH_SEMAPHORE(_write_mutex);
    return _writebuf.write(buffer, size);
}

bool WiFiDriver::_discard_input()
{
    _readbuf.clear();
    return true;
}

bool WiFiDriver::start_listen()
{
    _accept_socket = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (_accept_socket < 0) {
        return false;
    }
    int opt = 1;
    zsock_setsockopt(_accept_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_addr.s_addr = htonl(INADDR_ANY);
    dest.sin_port = htons(5760);
    if (zsock_bind(_accept_socket, (struct sockaddr *)&dest, sizeof(dest)) != 0 ||
        zsock_listen(_accept_socket, WIFI_MAX_CONNECTION) != 0) {
        zsock_close(_accept_socket);
        _accept_socket = -1;
        return false;
    }
    /* non-blocking accept so the service loop keeps pumping data */
    zsock_fcntl(_accept_socket, F_SETFL, O_NONBLOCK);
    return true;
}

bool WiFiDriver::try_accept()
{
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION; ++i) {
        if (_socket_list[i] == -1) {
            struct sockaddr_in src;
            socklen_t len = sizeof(src);
            const int fd = zsock_accept(_accept_socket, (struct sockaddr *)&src, &len);
            if (fd >= 0) {
                zsock_fcntl(fd, F_SETFL, O_NONBLOCK);
                _socket_list[i] = fd;
                _state = CONNECTED;
                return true;
            }
            return false;
        }
    }
    return false;
}

bool WiFiDriver::read_data()
{
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION; ++i) {
        if (_socket_list[i] < 0) {
            continue;
        }
        int count;
        do {
            count = zsock_recv(_socket_list[i], _buffer, sizeof(_buffer), 0);
            if (count > 0) {
                _readbuf.write(_buffer, count);
            } else if (count == 0 || (count < 0 && errno != EAGAIN)) {
                /* orderly close or hard error: drop this client */
                zsock_close(_socket_list[i]);
                _socket_list[i] = -1;
                break;
            }
        } while (count > 0);
    }
    return true;
}

bool WiFiDriver::write_data()
{
    WITH_SEMAPHORE(_write_mutex);
    int count;
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count <= 0) {
            break;
        }
        /* send to every connected client; advance by what the ring held */
        bool any_sent = false;
        for (unsigned short i = 0; i < WIFI_MAX_CONNECTION; ++i) {
            if (_socket_list[i] < 0) {
                continue;
            }
            const int sent = zsock_send(_socket_list[i], _buffer, count, 0);
            if (sent > 0) {
                any_sent = true;
            } else if (sent < 0 && errno != EAGAIN) {
                zsock_close(_socket_list[i]);
                _socket_list[i] = -1;
            }
        }
        if (!any_sent) {
            break;
        }
        _writebuf.advance(count);
    } while (count > 0);
    return true;
}

void WiFiDriver::_wifi_thread_fn()
{
    /* Radio bring-up is DEFERRED to here (not begin()): boot spew and the
       radio-init USB Serial-JTAG re-enumeration otherwise land in the
       same window and eat each other's console output. */
    hal.scheduler->delay(3000);
    while (!wifi_softap_start()) {
        hal.scheduler->delay(2000);
    }
    while (!start_listen()) {
        hal.scheduler->delay(1000);
    }
    printk("WiFi: TCP listening on :5760\n");

    /* NEVER issue NET_REQUEST_WIFI_SCAN while the softAP should keep
       beaconing: an AP-mode scan drags the radio off-channel and was
       observed (2026-08-15) suppressing our own beacons - the AP became
       invisible to host scans on every boot that ran one. */

    while (true) {
        wifi_report_events();
        try_accept();

        bool any = false;
        for (unsigned short i = 0; i < WIFI_MAX_CONNECTION; ++i) {
            if (_socket_list[i] >= 0) {
                any = true;
            }
        }
        if (!any && _state == CONNECTED) {
            _state = INITIALIZED;
        }
        if (any) {
            read_data();
            write_data();
        }
        hal.scheduler->delay(2);
    }
}

/* ── UDP driver (:14550) ────────────────────────────────────────────── */

WiFiUdpDriver::WiFiUdpDriver()
{
    _state = NOT_INITIALIZED;
    _socket = -1;
}

void WiFiUdpDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (_state != NOT_INITIALIZED) {
        return;
    }
    _readbuf.set_size(MAX(rxS, (uint16_t)RX_BUF_SIZE));
    _writebuf.set_size(MAX(txS, (uint16_t)TX_BUF_SIZE));

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&WiFiUdpDriver::_wifi_thread_fn, void),
            "APM_WIFI_UDP", 4096, AP_HAL::Scheduler::PRIORITY_UART, 2)) {  /* +2 = above main, see TCP note */
        printk("WiFi: UDP thread create failed\n");
        return;
    }
    /* UDP is connectionless: usable as soon as the socket exists */
    _state = CONNECTED;
}

void WiFiUdpDriver::_end()
{
}

void WiFiUdpDriver::_flush()
{
}

bool WiFiUdpDriver::is_initialized()
{
    return _state != NOT_INITIALIZED;
}

bool WiFiUdpDriver::tx_pending()
{
    return _writebuf.available() > 0;
}

uint32_t WiFiUdpDriver::_available()
{
    if (_state != CONNECTED) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t WiFiUdpDriver::txspace()
{
    if (_state != CONNECTED) {
        return 0;
    }
    const int result = _writebuf.space() - TX_BUF_SIZE / 4;
    return MAX(result, 0);
}

ssize_t WiFiUdpDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (_state != CONNECTED) {
        return 0;
    }
    return _readbuf.read(buffer, count);
}

size_t WiFiUdpDriver::_write(const uint8_t *buffer, size_t size)
{
    if (_state != CONNECTED) {
        return 0;
    }
    WITH_SEMAPHORE(_write_mutex);
    return _writebuf.write(buffer, size);
}

bool WiFiUdpDriver::_discard_input()
{
    _readbuf.clear();
    return true;
}

bool WiFiUdpDriver::open_socket()
{
    _socket = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_socket < 0) {
        return false;
    }
    struct sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(14550);
    if (zsock_bind(_socket, (struct sockaddr *)&local, sizeof(local)) != 0) {
        zsock_close(_socket);
        _socket = -1;
        return false;
    }
    int bcast = 1;
    zsock_setsockopt(_socket, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));
    zsock_fcntl(_socket, F_SETFL, O_NONBLOCK);
    return true;
}

bool WiFiUdpDriver::read_data()
{
    int count;
    do {
        struct sockaddr src;
        socklen_t len = sizeof(src);
        count = zsock_recvfrom(_socket, _buffer, sizeof(_buffer), 0,
                               &src, &len);
        if (count > 0) {
            _readbuf.write(_buffer, count);
        }
    } while (count > 0);
    return true;
}

bool WiFiUdpDriver::write_data()
{
    WITH_SEMAPHORE(_write_mutex);
    int count;
    do {
        /* one MAVLink-friendly datagram at a time */
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count <= 0) {
            break;
        }
        /* ALWAYS subnet-broadcast (maintainer requirement 2026-08-15):
           every client that joins the softAP receives the MAVLink stream
           automatically on :14550, no peer lock-on - a second and third
           listener work without sending a byte first. */
        struct sockaddr_in bcast = {};
        bcast.sin_family = AF_INET;
        bcast.sin_addr.s_addr = htonl(0xC0A804FF);  /* 192.168.4.255 */
        bcast.sin_port = htons(14550);
        const int sent = zsock_sendto(_socket, _buffer, count, 0,
                                      (const struct sockaddr *)&bcast,
                                      sizeof(bcast));
        if (sent <= 0) {
            break;
        }
        _writebuf.advance(sent);
    } while (count > 0);
    return true;
}

void WiFiUdpDriver::_wifi_thread_fn()
{
    hal.scheduler->delay(10000);
    while (!wifi_softap_start()) {
        hal.scheduler->delay(2000);
    }
    while (!open_socket()) {
        hal.scheduler->delay(1000);
    }
    printk("WiFi: UDP on :14550 (broadcast until first client)\n");
    while (true) {
        read_data();
        write_data();
        hal.scheduler->delay(2);
    }
}

#endif  /* AP_ZEPHYR_WIFI_ENABLED */
