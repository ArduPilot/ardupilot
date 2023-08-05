/*
  classes for holding IPv4 and ethernet MAC address parameters
 */

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED

#include "AP_Networking.h"

const AP_Param::GroupInfo AP_Networking_IPV4::var_info[] = {
    // @Param: 0
    // @DisplayName: IPv4 Address 1st byte
    // @Description: IPv4 address. Example: 192.xxx.xxx.xxx
    // @Range: 0 255
    // @RebootRequired: True
    AP_GROUPINFO("0", 1,  AP_Networking_IPV4, addr[0], 0),

    // @Param: 1
    // @DisplayName: IPv4 Address MSB
    // @Description: IPv4 address. Example: xxx.168.xxx.xxx
    // @Range: 0 255
    // @RebootRequired: True
    AP_GROUPINFO("1", 2,  AP_Networking_IPV4, addr[1], 0),

    // @Param: 2
    // @DisplayName: IPv4 Address MSB
    // @Description: IPv4 address. Example: xxx.xxx.13.xxx
    // @Range: 0 255
    // @RebootRequired: True
    AP_GROUPINFO("2", 3,  AP_Networking_IPV4, addr[2], 0),

    // @Param: 3
    // @DisplayName: IPv4 Address MSB
    // @Description: IPv4 address. Example: xxx.xxx.xxx.14
    // @Range: 0 255
    // @RebootRequired: True
    AP_GROUPINFO("3", 4,  AP_Networking_IPV4, addr[3], 0),

    AP_GROUPEND
};

const AP_Param::GroupInfo AP_Networking_MAC::var_info[] = {
    // @Param: 0
    // @DisplayName: MAC Address 1st byte
    // @Description: MAC address 1st byte
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("0", 1,  AP_Networking_MAC, addr[0], 0),

    // @Param: 1
    // @DisplayName: MAC Address 2nd byte
    // @Description: MAC address 2nd byte
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("1", 2,  AP_Networking_MAC, addr[1], 0),

    // @Param: 2
    // @DisplayName: MAC Address 3rd byte
    // @Description: MAC address 3rd byte
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("2", 3,  AP_Networking_MAC, addr[2], 0),

    // @Param: 3
    // @DisplayName: MAC Address 4th byte
    // @Description: MAC address 4th byte
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("3", 4,  AP_Networking_MAC, addr[3], 0),

    // @Param: 4
    // @DisplayName: MAC Address 5th byte
    // @Description: MAC address 5th byte
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("4", 5,  AP_Networking_MAC, addr[4], 0),

    // @Param: 5
    // @DisplayName: MAC Address 6th byte
    // @Description: MAC address 6th byte
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("5", 6,  AP_Networking_MAC, addr[5], 0),
    
    AP_GROUPEND
};

/*
  IPV4 address parameter class
 */
AP_Networking_IPV4::AP_Networking_IPV4(const char *default_addr)
{
    AP_Param::setup_object_defaults(this, var_info);
    set_default_uint32(AP_Networking::convert_str_to_ip(default_addr));
}

uint32_t AP_Networking_IPV4::get_uint32(void) const
{
    uint32_t v = 0;
    uint8_t *b = (uint8_t*)&v;
    for (uint8_t i=0; i<4; i++) {
        b[3-i] = uint8_t(addr[i].get());
    }
    return v;
}

void AP_Networking_IPV4::set_default_uint32(uint32_t v)
{
    uint8_t *b = (uint8_t*)&v;
    for (uint8_t i=0; i<ARRAY_SIZE(addr); i++) {
        addr[3-i].set_default(b[i]);
    }
}

/*
  ethernet MAC address parameter class
 */
AP_Networking_MAC::AP_Networking_MAC(const char *default_addr)
{
    AP_Param::setup_object_defaults(this, var_info);
    uint8_t b[6];
    if (AP_Networking::convert_str_to_macaddr(default_addr, b)) {
        for (uint8_t i=0; i<ARRAY_SIZE(addr); i++) {
            addr[i].set_default(b[i]);
        }
    }
}

void AP_Networking_MAC::get_address(uint8_t v[6]) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(addr); i++) {
        v[i] = uint8_t(addr[i].get());
    }
}

void AP_Networking_MAC::set_default_address_byte(uint8_t idx, uint8_t b)
{
    if (idx < ARRAY_SIZE(addr)) {
        addr[idx].set_default(b);
    }
}

#endif // AP_NETWORKING_ENABLED
