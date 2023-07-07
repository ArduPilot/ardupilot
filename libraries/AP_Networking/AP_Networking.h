
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

#define IP4_ADDR_VALUE_FROM_ARRAY(array) IP4_ADDR_VALUE(array[0],array[1],array[2],array[3])
#define IP4_ADDR_FROM_ARRAY(dest_ip, array) IP4_ADDR(dest_ip, array[0],array[1],array[2],array[3])
#define IP_ADDR_FROM_ARRAY(dest_ip, array) IP4_ADDR_FROM_ARRAY(dest_ip, array)

class AP_Networking
{
public:
    AP_Networking();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking);

    void init();

    void update();

    static AP_Networking *get_singleton(void)
    {
        return _singleton;
    }

    bool is_healthy() const
    {
        return _param.enabled && _init.done;
    }
    bool get_dhcp_enabled() const
    {
        return _param.dhcp;
    }
    void set_dhcp_enable(const bool enable)
    {
        _param.dhcp.set(enable);
    }

    uint32_t get_ip_active() const
    {
        return _activeSettings.ip;
    }
    uint32_t get_ip_param() const
    {
        return IP4_ADDR_VALUE_FROM_ARRAY(_param.ipaddr);
    }
    char*    get_ip_active_str() const
    {
        return convert_ip_to_str(get_ip_active());
    }
    char*    get_ip_param_str() const
    {
        return convert_ip_to_str(get_ip_param());
    }
    void     set_ip_param_str(const char* ip_str)
    {
        set_ip_param(convert_str_to_ip((char*)ip_str));
    }
    void     set_ip_param(const uint32_t ip)
    {
        //put_le32_ptr(_param.ipaddr->get(), ip);
        _param.ipaddr[3].set_and_save((ip >> 24) & 0xff);
        _param.ipaddr[2].set_and_save((ip >> 16) & 0xff);
        _param.ipaddr[1].set_and_save((ip >> 8) & 0xff);
        _param.ipaddr[0].set_and_save(ip & 0xff);
    }

    uint32_t get_netmask_active() const
    {
        return _activeSettings.nm;
    }
    uint32_t get_netmask_param() const
    {
        return convert_netmask_bitcount_to_ip(_param.netmask.get());
    }
    char*    get_netmask_active_str()
    {
        return convert_ip_to_str(get_netmask_active());
    }
    char*    get_netmask_param_str()
    {
        return convert_ip_to_str(get_netmask_param());
    }
    void     set_netmask_param_str(const char* nm_str)
    {
        set_netmask_param(convert_str_to_ip((char*)nm_str));
    }
    void     set_netmask_param(const uint32_t nm)
    {
        _param.netmask.set(convert_netmask_ip_to_bitcount(nm));
    }

    uint32_t get_gateway_active() const
    {
        return _activeSettings.gw;
    }
    uint32_t get_gateway_param() const
    {
        return IP4_ADDR_VALUE_FROM_ARRAY(_param.gwaddr);
    }
    char*    get_gateway_active_str()
    {
        return convert_ip_to_str(get_gateway_active());
    }
    char*    get_gateway_param_str()
    {
        return convert_ip_to_str(get_gateway_param());
    }
    void     set_gateway_param_str(const char* gw_str)
    {
        set_gateway_param(convert_str_to_ip((char*)gw_str));
    }
    void     set_gateway_param(const uint32_t gw)
    {
        //put_le32_ptr(_param.gwaddr->get(), gw);
        _param.gwaddr[3].set_and_save((gw >> 24) & 0xff);
        _param.gwaddr[2].set_and_save((gw >> 16) & 0xff);
        _param.gwaddr[1].set_and_save((gw >> 8) & 0xff);
        _param.gwaddr[0].set_and_save(gw & 0xff);
    }


    static uint32_t convert_str_to_ip(char* ip_str);
    static char* convert_ip_to_str(const uint8_t ip[4]);
    static char* convert_ip_to_str(const uint32_t ip);

    static uint32_t convert_netmask_bitcount_to_ip(const uint32_t netmask_bitcount);
    static uint8_t convert_netmask_ip_to_bitcount(const uint32_t netmask_ip);


#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    static int32_t send_udp(struct udp_pcb *pcb, const ip4_addr_t &ip4_addr, const uint16_t port, const uint8_t* data, uint16_t data_len);
#endif

    static const struct AP_Param::GroupInfo var_info[];

private:
    static int tftp_fd;
    static AP_Networking *_singleton;

    uint8_t     _num_instances;         // number of feature instances

    void announce_address_changes();

    struct {
        bool done;
    } _init;

    struct {
        AP_Int16 ipaddr[4];
        AP_Int8 netmask;    // bits to mask. example: (16 == 255.255.0.0) and (24 == 255.255.255.0)
        AP_Int16 gwaddr[4];

        AP_Int8 dhcp;
        AP_Int16 macaddr[6];
        AP_Int8 enabled;
        AP_Int32 options;
    } _param;

    struct {
        uint32_t ip;
        uint32_t nm;
        uint32_t gw;
        uint32_t announce_ms;
        bool announce_at_boot_done;
    } _activeSettings;

    HAL_Semaphore _sem;
};

namespace AP
{
AP_Networking &network();
};

#endif // AP_NETWORKING_ENABLED
