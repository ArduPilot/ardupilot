
#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED

#include "AP_Networking.h"
#include "AP_Networking_Backend.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/crc.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/CANIface.h>

extern const AP_HAL::HAL& hal;

#if AP_NETWORKING_BACKEND_CHIBIOS
#include <hal_mii.h>
#endif

#if AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET
#include "AP_Networking_SwitchPort_Ethernet_ChibiOS.h"
#endif

#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS || AP_NETWORKING_BACKEND_SWITCH
#include "AP_Networking_SwitchPort_COBS.h"
#endif

#if AP_NETWORKING_BACKEND_SWITCHPORT_LWIP
#include "AP_Networking_SwitchPort_lwIP.h"
#endif

#if AP_NETWORKING_BACKEND_SWITCH
#include "AP_Networking_Switch.h"
#endif

#if AP_NETWORKING_BACKEND_SWITCHPORT_MAVLINK_COBS
#include "AP_Networking_SwitchPort_MAVLink_COBS.h"
#endif

#if AP_NETWORKING_NEED_LWIP
#include <lwipopts.h>
#endif

#include <AP_HAL/utility/Socket.h>

#if AP_NETWORKING_BACKEND_PPP
#include "AP_Networking_PPP.h"
#endif

#if AP_NETWORKING_BACKEND_SITL
#include "AP_Networking_SITL.h"
#endif

const AP_Param::GroupInfo AP_Networking::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Networking Enable
    // @Description: Networking Enable
    // @Values: 0:Disable,1:Enable
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE",  1, AP_Networking, param.enabled, 0, AP_PARAM_FLAG_ENABLE),

#if AP_NETWORKING_BACKEND_SWITCHPORT_LWIP
    // @Param: IPSTACK_EN
    // @DisplayName: Enable IP stack
    // @Description: Enable/Disable the TCP/IP (lwIP) stack. When disabled, the system still bridges Ethernet and UART at Layer 2.
    // @Values: 0:Disable,1:Enable
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("IPSTACK_EN",  13, AP_Networking, param.ipstack_enabled, AP_NETWORKING_DEFAULT_IP_ENABLE),
#endif

#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
    // @Group: IPADDR
    // @Path: AP_Networking_address.cpp
    AP_SUBGROUPINFO(param.ipaddr, "IPADDR", 2,  AP_Networking, AP_Networking_IPV4),

    // @Param: NETMASK
    // @DisplayName: IP Subnet mask
    // @Description: Allows setting static subnet mask. The value is a count of consecutive bits. Examples: 24 = 255.255.255.0, 16 = 255.255.0.0
    // @Range: 0 32
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("NETMASK", 3,  AP_Networking,    param.netmask,   AP_NETWORKING_DEFAULT_NETMASK),

#if AP_NETWORKING_DHCP_AVAILABLE
    // @Param: DHCP
    // @DisplayName: DHCP client
    // @Description: Enable/Disable DHCP client
    // @Values: 0:Disable, 1:Enable
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("DHCP", 4,  AP_Networking,    param.dhcp,   AP_NETWORKING_DEFAULT_DHCP_ENABLE),
#endif

    // @Group: GWADDR
    // @Path: AP_Networking_address.cpp
    AP_SUBGROUPINFO(param.gwaddr, "GWADDR", 5,  AP_Networking, AP_Networking_IPV4),

    // @Group: MACADDR
    // @Path: AP_Networking_macaddr.cpp
    AP_SUBGROUPINFO(param.macaddr, "MACADDR", 6,  AP_Networking, AP_Networking_MAC),
#endif // AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED

#if AP_NETWORKING_TESTS_ENABLED
    // @Param: TESTS
    // @DisplayName: Test enable flags
    // @Description: Enable/Disable networking tests
    // @Bitmask: 0:UDP echo test,1:TCP echo test, 2:TCP discard test, 3:TCP reflect test
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("TESTS", 7,  AP_Networking,    param.tests,   0),

    // @Group: TEST_IP
    // @Path: AP_Networking_address.cpp
    AP_SUBGROUPINFO(param.test_ipaddr, "TEST_IP", 8,  AP_Networking, AP_Networking_IPV4),
#endif

    // @Param: OPTIONS
    // @DisplayName: Networking options
    // @Description: Networking options
    // @Bitmask: 0:EnablePPP Ethernet gateway, 1:Enable CAN1 multicast endpoint, 2:Enable CAN2 multicast endpoint, 3:Enable CAN1 multicast bridged, 4:Enable CAN2 multicast bridged, 5:DisablePPPTimeout, 6:DisablePPPEchoLimit, 8:Debug messages, 9:Debug switch packets (ARP/ICMP)
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 9,  AP_Networking,    param.options, 0),

#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    // @Group: REMPPP_IP
    // @Path: AP_Networking_address.cpp
    AP_SUBGROUPINFO(param.remote_ppp_ip, "REMPPP_IP", 10,  AP_Networking, AP_Networking_IPV4),
#endif

#if AP_NETWORKING_CAPTURE_ENABLED
    // @Param: CAPMASK
    // @DisplayName: Packet capture bitmask
    // @Description: Enable packet capture to pcap files by port type. Each enabled port type writes to a separate file.
    // @Bitmask: 0:lwIP (lwip0.cap), 1:Ethernet (eth0.cap), 2:COBS bonds (cobsN.cap), 3:MAVLink COBS (mavcobsN.cap), 4:PPP (pppN.cap)
    // @User: Advanced
    AP_GROUPINFO("CAPMASK", 11, AP_Networking, param.capture_mask, 0),
#endif
    
    AP_GROUPEND
};

/*
  constructor
 */
AP_Networking::AP_Networking(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("AP_Networking must be singleton");
    }
#endif
    singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  initialise networking subsystem
 */
void AP_Networking::init()
{
    if (!param.enabled || backend != nullptr) {
        return;
    }

#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
    // set default MAC Address as lower 3 bytes of the CRC of the UID
    uint8_t uid[50];
    uint8_t uid_len = sizeof(uid);
    if (hal.util->get_system_id_unformatted(uid, uid_len)) {
        union {
            uint8_t bytes[4];
            uint32_t value32;
        } crc;
        crc.value32 = crc_crc32(0, uid, uid_len);

        param.macaddr.set_default_address_byte(3, crc.bytes[0]);
        param.macaddr.set_default_address_byte(4, crc.bytes[1]);
        param.macaddr.set_default_address_byte(5, crc.bytes[2]);
    }
#endif

    // Create hub and ports when switch backend is enabled
#if AP_NETWORKING_BACKEND_SWITCH
    if (hub == nullptr) {
        hub = NEW_NOTHROW AP_Networking_Switch();
    }
    if (hub != nullptr) {
        // Ethernet port - owns the MAC with dedicated RX/TX threads
#if AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET
        if (port_eth == nullptr) {
            uint8_t macaddr_tmp[6] {};
            get_macaddr(macaddr_tmp);
            port_eth = NEW_NOTHROW AP_Networking_SwitchPort_Ethernet_ChibiOS(hub, macaddr_tmp);
            if (port_eth != nullptr) {
                if (port_eth->init()) {
                    UNUSED_RESULT(hub->register_port(port_eth));
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: Ethernet port init failed");
                    delete port_eth;
                    port_eth = nullptr;
                }
            }
        }
#endif // AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET

#if AP_NETWORKING_BACKEND_SWITCHPORT_LWIP
        // lwIP port - owns lwIP when hub is enabled
        if (get_ipstack_enabled() && port_lwip == nullptr) {
            port_lwip = NEW_NOTHROW AP_Networking_SwitchPort_lwIP(hub, *this);
            if (port_lwip != nullptr) {
                if (port_lwip->init()) {
                    UNUSED_RESULT(hub->register_port(port_lwip));
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: lwIP port init failed");
                    delete port_lwip;
                    port_lwip = nullptr;
                }
            }
        }
#endif // AP_NETWORKING_BACKEND_SWITCHPORT_LWIP

        // Create COBS bonds for protocols 51/52/53
#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS
        uint8_t local_device_id[6];
        get_macaddr(local_device_id);
        
        static const AP_SerialManager::SerialProtocol bond_protocols[3] = {
            AP_SerialManager::SerialProtocol_COBS_ETH,   // Bond 1 (protocol 51)
            AP_SerialManager::SerialProtocol_COBS_ETH2,  // Bond 2 (protocol 52)
            AP_SerialManager::SerialProtocol_COBS_ETH3,  // Bond 3 (protocol 53)
        };

        uint8_t total_ports = 0;
        num_cobs_bonds = 0;

        for (uint8_t bond_idx = 0; bond_idx < 3; bond_idx++) {
            AP_SerialManager::SerialProtocol proto = bond_protocols[bond_idx];
            AP_Networking_SwitchPort_COBS *bond = nullptr;

            // Find all UARTs with this protocol
            for (uint8_t inst = 0; inst < SERIALMANAGER_NUM_PORTS; inst++) {
                if (!AP::serialmanager().have_serial(proto, inst)) {
                    continue;
                }
                auto *uart = AP::serialmanager().find_serial(proto, inst);
                const uint32_t baud = AP::serialmanager().find_baudrate(proto, inst);
                if (uart == nullptr || baud == 0) {
                    continue;
                }

                // Create bond on first port
                if (bond == nullptr) {
                    bond = NEW_NOTHROW AP_Networking_SwitchPort_COBS(hub, bond_idx + 1);
                    if (bond == nullptr) {
                        break;
                    }
                }

                // Create and add COBS port to bond
                auto *p = NEW_NOTHROW AP_Networking_COBS_Link(hub, uart, baud, local_device_id);
                if (p == nullptr) {
                    continue;
                }
                if (!p->init()) {
                    delete p;
                    continue;
                }
                if (!bond->add_member(p)) {
                    delete p;
                    continue;
                }
                total_ports++;
            }

            // Register bond with hub if it has members
            if (bond != nullptr) {
                if (hub->register_port(bond) < 0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: hub full, COBS bond %u skipped", bond_idx + 1);
                    delete bond;
                } else {
                    hub->register_cobs_bond(bond);
                    cobs_bonds[num_cobs_bonds++] = bond;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS bond %u: %u port(s)",
                                  bond_idx + 1, bond->get_num_members());
                }
            }
        }

        if (total_ports > 0) {
            // Start the shared COBS thread
            hub->init_cobs();
        }
#endif // AP_NETWORKING_BACKEND_SWITCHPORT_COBS

#if AP_NETWORKING_BACKEND_SWITCHPORT_MAVLINK_COBS
        // Initialize MAVLink COBS tunnel registry
        // Tunnel ports are created dynamically when TUNNEL messages arrive from new endpoints
        uint8_t mav_cobs_device_id[6];
        get_macaddr(mav_cobs_device_id);
        AP_Networking_SwitchPort_MAVLink_COBS::init_registry(hub, mav_cobs_device_id);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: MAVLink COBS tunnels enabled (max %u)",
                      (unsigned)AP_Networking_SwitchPort_MAVLink_COBS::MAX_TUNNELS);
#endif // AP_NETWORKING_BACKEND_SWITCHPORT_MAVLINK_COBS
    }
#endif // AP_NETWORKING_BACKEND_SWITCH

#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    if (option_is_set(OPTION::PPP_ETHERNET_GATEWAY)) {
        /*
          PPP/Ethernet gateway mode - PPP provides pppif, hub provides netif via lwIP port.
          lwIP IP_FORWARD routes between them.
         */
        // Hub mode: hub provides netif via lwIP port, just create PPP backend for pppif
        backend_PPP = NEW_NOTHROW AP_Networking_PPP(*this);
    }
#endif

#if AP_NETWORKING_BACKEND_PPP
    if (backend == nullptr && AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_PPP, 0)) {
        backend = NEW_NOTHROW AP_Networking_PPP(*this);
    }
#endif

#if AP_NETWORKING_BACKEND_SITL
    if (backend == nullptr) {
        backend = NEW_NOTHROW AP_Networking_SITL(*this);
    }
#endif

    // With switch enabled, we may not have a backend but still have a hub
#if AP_NETWORKING_BACKEND_SWITCH
    if (backend == nullptr && hub == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: no backend or hub");
        return;
    }
#else
    if (backend == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: backend failed");
        return;
    }
#endif

    if (backend != nullptr && !backend->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: backend init failed");
        backend = nullptr;
#if !AP_NETWORKING_BACKEND_SWITCH
        return;
#endif
    }

#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    if (backend_PPP != nullptr && !backend_PPP->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: backend_PPP init failed");
        backend_PPP = nullptr;
    }
#endif

    announce_address_changes();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: Initialized");

#if AP_NETWORKING_TESTS_ENABLED
    start_tests();
#endif

#if AP_NETWORKING_CAN_MCAST_BRIDGING_ENABLED
    if (option_is_set(OPTION::CAN1_MCAST_ENDPOINT) ||
        option_is_set(OPTION::CAN2_MCAST_ENDPOINT)) {
        // get mask of enabled buses
        uint8_t bus_mask = 0;
        if (option_is_set(OPTION::CAN1_MCAST_ENDPOINT)) {
            bus_mask |= (1U<<0);
        }
        if (option_is_set(OPTION::CAN2_MCAST_ENDPOINT)) {
            bus_mask |= (1U<<1);
        }
        mcast_server.start(bus_mask);
    }
#endif
    
#if AP_NETWORKING_REGISTER_PORT_ENABLED
    // init network mapped serialmanager ports
    ports_init();
#endif
}

/*
  check if we should announce changes to IP addresses
 */
void AP_Networking::announce_address_changes()
{
    const auto &as = backend->activeSettings;

    if (as.last_change_ms == 0 || as.last_change_ms == announce_ms) {
        // nothing changed and we've already printed it at least once. Nothing to do.
        return;
    }

#if AP_HAVE_GCS_SEND_TEXT
    char ipstr[16];
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: IP      %s", SocketAPM::inet_addr_to_str(get_ip_active(), ipstr, sizeof(ipstr)));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: Mask    %s", SocketAPM::inet_addr_to_str(get_netmask_active(), ipstr, sizeof(ipstr)));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: Gateway %s", SocketAPM::inet_addr_to_str(get_gateway_active(), ipstr, sizeof(ipstr)));
#endif

    announce_ms = as.last_change_ms;
}

/*
  update called at 10Hz
 */
void AP_Networking::update()
{
    if (!is_healthy()) {
        return;
    }
    backend->update();
#if AP_NETWORKING_CAPTURE_ENABLED
    // Manage per-port-type packet captures based on NET_CAPMASK
#if AP_NETWORKING_BACKEND_SWITCHPORT_LWIP
    if (port_lwip != nullptr) {
        if (capture_is_set(CAPTURE_LWIP)) {
            port_lwip->capture.start("lwip0.cap");
        } else {
            port_lwip->capture.stop();
        }
    }
#endif
#if AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET
    if (port_eth != nullptr) {
        if (capture_is_set(CAPTURE_ETHERNET)) {
            port_eth->capture.start("eth0.cap");
        } else {
            port_eth->capture.stop();
        }
    }
#endif
#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS
    for (uint8_t i = 0; i < num_cobs_bonds; i++) {
        if (cobs_bonds[i] != nullptr) {
            if (capture_is_set(CAPTURE_COBS)) {
                char fname[16];
                hal.util->snprintf(fname, sizeof(fname), "cobs%u.cap", i);
                cobs_bonds[i]->capture.start(fname);
            } else {
                cobs_bonds[i]->capture.stop();
            }
        }
    }
#endif
#if AP_NETWORKING_BACKEND_SWITCHPORT_MAVLINK_COBS
    for (uint8_t i = 0; i < AP_Networking_SwitchPort_MAVLink_COBS::MAX_TUNNELS; i++) {
        auto *port = AP_Networking_SwitchPort_MAVLink_COBS::get_port(i);
        if (port != nullptr) {
            if (capture_is_set(CAPTURE_MAV_COBS)) {
                char fname[20];
                hal.util->snprintf(fname, sizeof(fname), "mavcobs%u.cap", i);
                port->capture.start(fname);
            } else {
                port->capture.stop();
            }
        }
    }
#endif
#endif // AP_NETWORKING_CAPTURE_ENABLED
    announce_address_changes();
}

uint32_t AP_Networking::convert_netmask_bitcount_to_ip(const uint32_t netmask_bitcount)
{
    if (netmask_bitcount >= 32) {
        return 0xFFFFFFFFU;
    }
    return ~((1U<<(32U-netmask_bitcount))-1U);
}

uint8_t AP_Networking::convert_netmask_ip_to_bitcount(const uint32_t netmask_ip)
{
    uint32_t netmask_bitcount = 0;
    for (uint32_t i=0; i<32; i++) {
        // note, netmask LSB is IP MSB
        if ((netmask_ip & (1UL<<i)) == 0) {
            break;
        }
        netmask_bitcount++;
    }
    return netmask_bitcount;
}

/*
  convert a string to an ethernet MAC address
 */
bool AP_Networking::convert_str_to_macaddr(const char *mac_str, uint8_t addr[6])
{
    if (strlen(mac_str) != 17) {
        return false;
    }
    char s2[18];
    strncpy(s2, mac_str, sizeof(s2)-1);
    s2[17] = 0;
    char *ptr = nullptr;
    const char *s = strtok_r(s2, ":", &ptr);
    for (uint8_t i=0; i<6; i++) {
        if (s == nullptr) {
            return false;
        }
        auto v = strtoul(s, nullptr, 16);
        if (v > 255) {
            return false;
        }
        addr[i] = v;
        s = strtok_r(nullptr, ":", &ptr);
    }
    return true;
}

// returns the 32bit value of the active IP address that is currently in use
uint32_t AP_Networking::get_ip_active() const
{
    return backend?backend->activeSettings.ip:0;
}

// returns the 32bit value of the active Netmask that is currently in use
uint32_t AP_Networking::get_netmask_active() const
{
    return backend?backend->activeSettings.nm:0;
}

uint32_t AP_Networking::get_gateway_active() const
{
    return backend?backend->activeSettings.gw:0;
}

/*
  wait for networking to be active
 */
void AP_Networking::startup_wait(void) const
{
    if (hal.scheduler->in_main_thread()) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
#if AP_NETWORKING_BACKEND_CHIBIOS
    do {
        hal.scheduler->delay(250);
    } while (get_ip_active() == 0);
#endif
}

/*
  send the rest of a file to a socket
 */
bool AP_Networking::sendfile(SocketAPM *sock, int fd)
{
    WITH_SEMAPHORE(sem);
    if (sendfile_buf == nullptr) {
        uint32_t bufsize = AP_NETWORKING_SENDFILE_BUFSIZE;
        do {
            sendfile_buf = (uint8_t *)hal.util->malloc_type(bufsize, AP_HAL::Util::MEM_FILESYSTEM);
            if (sendfile_buf != nullptr) {
                sendfile_bufsize = bufsize;
                break;
            }
            bufsize /= 2;
        } while (bufsize >= 4096);
        if (sendfile_buf == nullptr) {
            return false;
        }
    }
    if (!sendfile_thread_started) {
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::sendfile_check, void),
                                          "sendfile",
                                          2048, AP_HAL::Scheduler::PRIORITY_UART, 0)) {
            return false;
        }
        sendfile_thread_started = true;
    }
    for (auto &s : sendfiles) {
        if (s.sock == nullptr) {
            s.sock = sock->duplicate();
            if (s.sock == nullptr) {
                return false;
            }
            s.fd = fd;
            return true;
        }
    }
    return false;
}

void AP_Networking::SendFile::close(void)
{
    AP::FS().close(fd);
    delete sock;
    sock = nullptr;
}

#include <stdio.h>
/*
  check for sendfile updates
 */
void AP_Networking::sendfile_check(void)
{
    while (true) {
        hal.scheduler->delay(1);
        WITH_SEMAPHORE(sem);
        bool none_active = true;
        for (auto &s : sendfiles) {
            if (s.sock == nullptr) {
                continue;
            }
            none_active = false;
            if (!s.sock->pollout(0)) {
                continue;
            }
            const auto nread = AP::FS().read(s.fd, sendfile_buf, sendfile_bufsize);
            if (nread <= 0) {
                s.close();
                continue;
            }
            const auto nsent = s.sock->send(sendfile_buf, nread);
            if (nsent <= 0) {
                s.close();
                continue;
            }
            if (nsent < nread) {
                AP::FS().lseek(s.fd, nsent - nread, SEEK_CUR);
            }
        }
        if (none_active) {
            free(sendfile_buf);
            sendfile_buf = nullptr;
        }
    }
}

AP_Networking *AP_Networking::singleton;

namespace AP
{
AP_Networking &network()
{
    return *AP_Networking::get_singleton();
}
}

/*
  debug printfs from LWIP
 */
int ap_networking_printf(const char *fmt, ...)
{
    WITH_SEMAPHORE(AP::network().get_semaphore());
#ifdef AP_NETWORKING_LWIP_DEBUG_FILE
    static int fd = -1;
    if (fd == -1) {
        fd = AP::FS().open(AP_NETWORKING_LWIP_DEBUG_FILE, O_WRONLY|O_CREAT|O_TRUNC, 0644);
        if (fd == -1) {
            return -1;
        }
    }
    va_list ap;
    va_start(ap, fmt);
    vdprintf(fd, fmt, ap);
    va_end(ap);
#else
    va_list ap;
    va_start(ap, fmt);
    hal.console->vprintf(fmt, ap);
    va_end(ap);
#endif
    return 0;
}

// address to string using a static return buffer
const char *AP_Networking::address_to_str(uint32_t addr)
{
    static char buf[16]; // 16 for aaa.bbb.ccc.ddd
    return SocketAPM::inet_addr_to_str(addr, buf, sizeof(buf));
}

/*
  send network stats to GCS for debugging
 */
void AP_Networking::send_network_stats()
{
#if AP_NETWORKING_BACKEND_SWITCH
    if (hub != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: SW routed=%lu dropped=%lu",
                      (unsigned long)hub->get_frames_routed(),
                      (unsigned long)hub->get_frames_dropped());
    }
#if AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET
    if (port_eth != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: ETH rx=%lu tx=%lu rxerr=%lu txerr=%lu link=%u",
                      (unsigned long)port_eth->get_rx_count(),
                      (unsigned long)port_eth->get_tx_count(),
                      (unsigned long)port_eth->get_rx_errors(),
                      (unsigned long)port_eth->get_tx_errors(),
                      (unsigned)port_eth->is_link_up());
    }
#endif
#if AP_NETWORKING_BACKEND_SWITCHPORT_LWIP
    if (port_lwip != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: LWIP rx=%lu tx=%lu rxerr=%lu txerr=%lu",
                      (unsigned long)port_lwip->get_rx_count(),
                      (unsigned long)port_lwip->get_tx_count(),
                      (unsigned long)port_lwip->get_rx_errors(),
                      (unsigned long)port_lwip->get_tx_errors());
    }
#endif
#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS
    for (uint8_t i = 0; i < num_cobs_bonds; i++) {
        auto *bond = cobs_bonds[i];
        if (bond != nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS_G%u(%u) rx=%lu tx=%lu",
                          i + 1, bond->get_num_members(),
                          (unsigned long)bond->get_rx_count(),
                          (unsigned long)bond->get_tx_count());
        }
    }
#endif // AP_NETWORKING_BACKEND_SWITCHPORT_COBS
#endif // AP_NETWORKING_BACKEND_SWITCH
}

#ifdef LWIP_PLATFORM_ASSERT
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || defined(HAL_DEBUG_BUILD)
void ap_networking_platform_assert(const char *msg, int line)
{
    AP_HAL::panic("LWIP:%u %s", line, msg);
}
#else
void ap_networking_platform_assert(int line)
{
    AP_HAL::panic("LWIP:%u", line);
}
#endif
#endif

#ifdef LWIP_HOOK_IP4_ROUTE
#include <lwip/ip4_addr.h>
struct netif *ap_networking_routing_hook(const struct ip4_addr *dest_ip)
{
    if (dest_ip == nullptr) {
        return nullptr;
    }
    return AP::network().routing_hook(ntohl(dest_ip->addr));
}
#endif

/*
  check for custom routes
 */
struct netif *AP_Networking::routing_hook(uint32_t dest)
{
    if (backend) {
        auto *iface = backend->routing_hook(dest);
        if (iface != nullptr) {
            return iface;
        }
    }
#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    if (backend_PPP) {
        auto *iface = backend_PPP->routing_hook(dest);
        if (iface != nullptr) {
            return iface;
        }
    }
#endif
    return nullptr;
}

// add new routes for an interface.
// Returns true if the route is added or the route already exists
bool AP_Networking::add_route(uint8_t backend_idx, uint8_t iface_idx, uint32_t dest_ip, uint8_t mask_len)
{
    if (backend_idx == 0 &&
        backend != nullptr &&
        backend->add_route(iface_idx, dest_ip, mask_len)) {
        return true;
    }
#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    if (backend_idx == 1 &&
        backend_PPP != nullptr &&
        backend_PPP->add_route(iface_idx, dest_ip, mask_len)) {
        return true;
    }
#endif
    return false;
}


#endif // AP_NETWORKING_ENABLED
