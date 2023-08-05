
#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>


#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "lwipthread.h"
#include "lwip/udp.h"
#include <lwip/ip_addr.h>
#else
#include <AP_Common/missing/byteswap.h>
#define IP4_ADDR_VALUE(a,b,c,d) be32toh(UINT32_VALUE(a,b,c,d))
#endif

#include "AP_Networking_address.h"

/*
  Note! all uint32_t IPv4 addresses are in network byte order (big-endian)
*/

class AP_Networking
{
public:
    AP_Networking();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking);

    // initialize the library. This should only be run once
    void init();

    // update task. For most applications this should run as fast as possible. Default is 1000Hz for lowest latency
    void update();

    static AP_Networking *get_singleton(void)
    {
        return _singleton;
    }

    // Networking interface is enabled and initialized
    bool is_healthy() const
    {
        return _param.enabled && _init.done;
    }

    // returns true if DHCP is enabled
    bool get_dhcp_enabled() const
    {
        return _param.dhcp;
    }

    // Sets DHCP to be enabled or disabled
    void set_dhcp_enable(const bool enable)
    {
        _param.dhcp.set(enable);
    }

    // returns the 32bit value of the active IP address that is currently in use
    uint32_t get_ip_active() const
    {
        return _activeSettings.ip;
    }

    // returns the 32bit value of the user-parameter static IP address
    uint32_t get_ip_param() const
    {
        return _param.ipaddr.get_uint32();
    }

    /*
      returns a null terminated string of the active IP address. Example: "192.168.12.13"
      Note that the returned
    */
    const char *get_ip_active_str() const
    {
        return convert_ip_to_str(get_ip_active());
    }

    // sets the user-parameter static IP address from a 32bit value
    void set_ip_param(const uint32_t ip)
    {
        _param.ipaddr.set_uint32(ip);
    }

    // returns the 32bit value of the active Netmask that is currently in use
    uint32_t get_netmask_active() const
    {
        return _activeSettings.nm;
    }

    // returns the 32bit value of the of the user-parameter static Netmask
    uint32_t get_netmask_param() const
    {
        return convert_netmask_bitcount_to_ip(_param.netmask.get());
    }

    // returns a null terminated string of the active Netmask address. Example: "192.168.12.13"
    const char *get_netmask_active_str()
    {
        return convert_ip_to_str(get_netmask_active());
    }

    const char *get_netmask_param_str()
    {
        return convert_ip_to_str(get_netmask_param());
    }

    void set_netmask_param_str(const char* nm_str)
    {
        set_netmask_param(convert_str_to_ip((char*)nm_str));
    }

    void set_netmask_param(const uint32_t nm)
    {
        _param.netmask.set(convert_netmask_ip_to_bitcount(nm));
    }

    uint32_t get_gateway_active() const
    {
        return _activeSettings.gw;
    }

    uint32_t get_gateway_param() const
    {
        return _param.gwaddr.get_uint32();
    }

    const char *get_gateway_active_str()
    {
        return convert_ip_to_str(get_gateway_active());
    }

    const char *get_gateway_param_str()
    {
        return convert_ip_to_str(get_gateway_param());
    }

    void set_gateway_param_str(const char* gw_str)
    {
        set_gateway_param(convert_str_to_ip((char*)gw_str));
    }

    void set_gateway_param(const uint32_t gw)
    {
        _param.gwaddr.set_uint32(gw);
    }


    // helper functions to convert between 32bit IP addresses and null terminated strings and back
    static uint32_t convert_str_to_ip(const char* ip_str);
    static const char* convert_ip_to_str(const uint8_t ip[4]);
    static const char* convert_ip_to_str(const uint32_t ip);

    // convert string to ethernet mac address
    static bool convert_str_to_macaddr(const char *mac_str, uint8_t addr[6]);

    // helper functions to convert between 32bit Netmask and counting consecutive bits and back
    static uint32_t convert_netmask_bitcount_to_ip(const uint32_t netmask_bitcount);
    static uint8_t convert_netmask_ip_to_bitcount(const uint32_t netmask_ip);


#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    static int32_t send_udp(struct udp_pcb *pcb, const ip4_addr_t &ip4_addr, const uint16_t port, const uint8_t* data, uint16_t data_len);
#endif

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_Networking *_singleton;

    uint8_t     _num_instances;         // number of feature instances

    void announce_address_changes();

    struct {
        bool done;
    } _init;

    struct {
        AP_Networking_IPV4 ipaddr{AP_NETWORKING_DEFAULT_STATIC_IP_ADDR};
        AP_Int8 netmask;    // bits to mask. example: (16 == 255.255.0.0) and (24 == 255.255.255.0)
        AP_Networking_IPV4 gwaddr{AP_NETWORKING_DEFAULT_STATIC_GW_ADDR};

        AP_Int8 dhcp;
        AP_Networking_MAC macaddr{AP_NETWORKING_DEFAULT_MAC_ADDR};
        AP_Int8 enabled;
        AP_Int32 options;
    } _param;

    struct {
        uint32_t ip;
        uint32_t nm;
        uint32_t gw;
        uint32_t announce_ms;
        uint8_t macaddr[6];
        bool announce_at_boot_done;
    } _activeSettings;
    struct lwipthread_opts lwip_options;

    HAL_Semaphore _sem;
};

namespace AP
{
AP_Networking &network();
};

#endif // AP_NETWORKING_ENABLED
