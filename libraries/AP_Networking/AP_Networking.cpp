
#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED

#include "AP_Networking.h"
#include "AP_Networking_ChibiOS.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/crc.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal_mii.h>
#include <lwip/sockets.h>
#else
#include <arpa/inet.h>
#endif


const AP_Param::GroupInfo AP_Networking::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Networking Enable
    // @Description: Networking Enable
    // @Values: 0:Disable,1:Enable
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED",  1, AP_Networking, param.enabled, 0, AP_PARAM_FLAG_ENABLE),

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

    // @Param: DHCP
    // @DisplayName: DHCP client
    // @Description: Enable/Disable DHCP client
    // @Values: 0:Disable, 1:Enable
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("DHCP", 4,  AP_Networking,    param.dhcp,   AP_NETWORKING_DEFAULT_DHCP_ENABLE),

    // @Group: GWADDR
    // @Path: AP_Networking_address.cpp
    AP_SUBGROUPINFO(param.gwaddr, "GWADDR", 5,  AP_Networking, AP_Networking_IPV4),

    // @Group: MACADDR
    // @Path: AP_Networking_macaddr.cpp
    AP_SUBGROUPINFO(param.macaddr, "MACADDR", 6,  AP_Networking, AP_Networking_MAC),

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

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    backend = new AP_Networking_ChibiOS(*this);
#endif

    if (backend == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: backend failed");
        return;
    }

    if (!backend->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: backend init failed");
        // the backend init function creates a thread which references the backend pointer; that thread may be running so don't remove the backend allocation.
        backend = nullptr;
        return;
    }

    announce_address_changes();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: Initialized");
}

/*
  check if we should announce changes to IP addresses
 */
void AP_Networking::announce_address_changes()
{
    auto &as = backend->activeSettings;

    if (as.last_change_ms == 0 || as.last_change_ms == announce_ms) {
        // nothing changed and we've already printed it at least once. Nothing to do.
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: IP      %s", get_ip_active_str());
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: Mask    %s", get_netmask_active_str());
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: Gateway %s", get_gateway_active_str());

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

uint32_t AP_Networking::convert_str_to_ip(const char* ip_str)
{
    uint32_t ip = 0;
    inet_pton(AF_INET, ip_str, &ip);
    return ntohl(ip);
}

const char* AP_Networking::convert_ip_to_str(uint32_t ip)
{
    ip = htonl(ip);
    static char _str_buffer[20];
    inet_ntop(AF_INET, &ip, _str_buffer, sizeof(_str_buffer));
    return _str_buffer;
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

AP_Networking *AP_Networking::singleton;

namespace AP
{
AP_Networking &network()
{
    return *AP_Networking::get_singleton();
}
}

#endif // AP_NETWORKING_ENABLED
