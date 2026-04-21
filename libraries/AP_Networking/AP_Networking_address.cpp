/*
  class for holding IPv4 address parameters
 */

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED

#include <arpa/inet.h>
#include "AP_Networking.h"
#include <AP_HAL/utility/Socket.h>

const AP_Param::GroupInfo AP_Networking_IPV4::var_info[] = {
    // @Param: 0
    // @DisplayName: IPv4 Address 1st byte
    // @Description: IPv4 address. Example: 192.xxx.xxx.xxx
    // @Range: 0 255
    // @RebootRequired: True
    AP_GROUPINFO("0", 1,  AP_Networking_IPV4, addr[0], 0),

    // @Param: 1
    // @DisplayName: IPv4 Address 2nd byte
    // @Description: IPv4 address. Example: xxx.168.xxx.xxx
    // @Range: 0 255
    // @RebootRequired: True
    AP_GROUPINFO("1", 2,  AP_Networking_IPV4, addr[1], 0),

    // @Param: 2
    // @DisplayName: IPv4 Address 3rd byte
    // @Description: IPv4 address. Example: xxx.xxx.144.xxx
    // @Range: 0 255
    // @RebootRequired: True
    AP_GROUPINFO("2", 3,  AP_Networking_IPV4, addr[2], 0),

    // @Param: 3
    // @DisplayName: IPv4 Address 4th byte
    // @Description: IPv4 address. Example: xxx.xxx.xxx.14
    // @Range: 0 255
    // @RebootRequired: True
    AP_GROUPINFO("3", 4,  AP_Networking_IPV4, addr[3], 0),

    AP_GROUPEND
};


/*
  IPV4 address parameter class
 */
AP_Networking_IPV4::AP_Networking_IPV4(const char *default_addr)
{
    AP_Param::setup_object_defaults(this, var_info);
    set_default_uint32(SocketAPM::inet_str_to_addr(default_addr));
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

const char* AP_Networking_IPV4::get_str()
{
    const auto ip = get_uint32();
    return SocketAPM::inet_addr_to_str(ip, strbuf, sizeof(strbuf));
}

#endif // AP_NETWORKING_ENABLED
