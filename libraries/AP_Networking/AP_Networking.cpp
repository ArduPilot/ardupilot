
#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED

#include "AP_Math/definitions.h"
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    #include <hal_mii.h>
    #include <lwip/sockets.h>

    #if AP_NETWORKING_SNTP_ENABLED
    #include <AP_RTC/AP_RTC.h>
    #include <lwip/apps/sntp.h>

    /* Configure behaviour depending on native, microsecond or second precision.
    * Treat NTP timestamps as signed two's-complement integers. This way,
    * timestamps that have the MSB set simply become negative offsets from
    * the epoch (Feb 7, 2036 06:28:16 UTC). Representable dates range from
    * 1968 to 2104.
    */
    void sntp_set_system_time_us(uint32_t sec, uint32_t us) {
        AP::rtc().set_utc_usec((uint64_t)sec*1E6 + (uint64_t)us, AP_RTC::SOURCE_NTP);
    }

    void sntp_get_system_time_us(uint32_t* sec_, uint32_t* usec_) {
        uint64_t time_usec;
        if (AP::rtc().get_utc_usec(time_usec)) {
            *sec_ = time_usec / AP_USEC_PER_SEC;
            *usec_ = time_usec % AP_USEC_PER_SEC;
        }
    }
    #endif // AP_NETWORKING_SNTP_ENABLED

    #if AP_NETWORKING_NETBIOS_ENABLED
    #include <lwip/apps/netbiosns.h>
    #include <AP_BoardConfig/AP_BoardConfig.h>

    #ifndef NETBIOS_NAME_LEN
        #define NETBIOS_NAME_LEN 16
    #endif
    #endif // AP_NETWORKING_NETBIOS_ENABLED

    #if AP_NETWORKING_TFTP_ENABLED
    #include <AP_Filesystem/AP_Filesystem.h>
    #include <lwip/apps/tftp_server.h>
    #endif
    
#else
    #include <arpa/inet.h>
    // #include <sys/ioctl.h>
    // #include <sys/types.h>
    #include <sys/socket.h>
    // #include <netinet/in.h>
    // #include <netinet/tcp.h>
    // #include <sys/select.h>
    // #include <termios.h>
    // #include <sys/time.h>
#endif

#include "AP_Networking_SpeedTest.h"
#include "AP_Networking_LatencyTest.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Networking::var_info[] = {

    // @Param: _ENABLED
    // @DisplayName: Enable Networking
    // @Description: Enable Networking
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLED",  0, AP_Networking, _param.enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _IPV4ADDR0
    // @DisplayName: IPv4 Address MSB
    // @Description: Allows setting static IP address. Example: 192.xxx.xxx.xxx
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_IPV4ADDR0", 1,  AP_Networking,    _param.ipaddr[0],   AP_NETWORKING_DEFAULT_STATIC_IPV4_ADDR0),

    // @Param: _IPV4ADDR1
    // @DisplayName: IPv4 Address 2nd byte
    // @Description: Allows setting static IP address. Example: xxx.168.xxx.xxx
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_IPV4ADDR1", 2,  AP_Networking,    _param.ipaddr[1],   AP_NETWORKING_DEFAULT_STATIC_IPV4_ADDR1),

    // @Param: _IPV4ADDR2
    // @DisplayName: IPv4 Address 3rd byte
    // @Description: Allows setting static IP address. Example: xxx.xxx.13.xxx
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_IPV4ADDR2", 3,  AP_Networking,    _param.ipaddr[2],   AP_NETWORKING_DEFAULT_STATIC_IPV4_ADDR2),

    // @Param: _IPV4ADDR3
    // @DisplayName: IPv4 Address LSB
    // @Description: Allows setting static IP address. Example: xxx.xxx.xxx.14
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_IPV4ADDR3", 4,  AP_Networking,    _param.ipaddr[3],   AP_NETWORKING_DEFAULT_STATIC_IPV4_ADDR3),

    // @Param: _NETMASK
    // @DisplayName: IP Subnet mask
    // @Description: Allows setting static subnet mask. The value is a count of consecutive bits. Examples: 24 = 255.255.255.0, 16 = 255.255.0.0
    // @Range: 0 32
    // @User: Advanced
    AP_GROUPINFO("_NETMASK", 5,  AP_Networking,    _param.netmask,   AP_NETWORKING_DEFAULT_NETMASK),

    // @Param: _DHCP
    // @DisplayName: DHCP client
    // @Description: Enable/Disable DHCP client
    // @Values: 0:Disable, 1:Enable
    // @User: Advanced
    AP_GROUPINFO("_DHCP", 6,  AP_Networking,    _param.dhcp,   AP_NETWORKING_DEFAULT_DHCP_ENABLE),

    // @Param: _GWADDR0
    // @DisplayName: Gateway IP Address MSB
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_GWADDR0", 7,  AP_Networking,    _param.gwaddr[0],   AP_NETWORKING_DEFAULT_STATIC_GW_ADDR0),

    // @Param: _GWADDR1
    // @DisplayName: Gateway IP Address 2nd byte
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_GWADDR1", 8,  AP_Networking,    _param.gwaddr[1],   AP_NETWORKING_DEFAULT_STATIC_GW_ADDR1),

    // @Param: _GWADDR2
    // @DisplayName: Gateway IP Address 3rd byte
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_GWADDR2", 9,  AP_Networking,    _param.gwaddr[2],   AP_NETWORKING_DEFAULT_STATIC_GW_ADDR2),

    // @Param: _GWADDR3
    // @DisplayName: Gateway IP Address LSB
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_GWADDR3", 10,  AP_Networking,    _param.gwaddr[3],   AP_NETWORKING_DEFAULT_STATIC_GW_ADDR3),

    // @Param: _MACADDR0
    // @DisplayName: MAC Address MSbyte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR0", 11,  AP_Networking,    _param.macaddr[0],  AP_NETWORKING_DEFAULT_MAC_ADDR0),

    // @Param: _MACADDR1
    // @DisplayName: MAC Address 2nd byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR1", 12,  AP_Networking,    _param.macaddr[1],   AP_NETWORKING_DEFAULT_MAC_ADDR1),

    // @Param: _MACADDR2
    // @DisplayName: MAC Address 3rd byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR2", 13,  AP_Networking,    _param.macaddr[2],   AP_NETWORKING_DEFAULT_MAC_ADDR2),

    // @Param: _MACADDR3
    // @DisplayName: MAC Address 4th byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR3", 14,  AP_Networking,    _param.macaddr[3],   AP_NETWORKING_DEFAULT_MAC_ADDR3),

    // @Param: _MACADDR4
    // @DisplayName: MAC Address 5th byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR4", 15,  AP_Networking,    _param.macaddr[4],   AP_NETWORKING_DEFAULT_MAC_ADDR4),

    // @Param: _MACADDR5
    // @DisplayName: MAC Address LSb
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR5", 16,  AP_Networking,    _param.macaddr[5],   AP_NETWORKING_DEFAULT_MAC_ADDR5),

    // @Param: _OPTIONS
    // @DisplayName: Network Options
    // @Description: Network Options
    // @Bitmask: 0:TFTP, 1:NetBIOS, 2:SNTP
    // @User: Advanced
    AP_GROUPINFO("_OPTIONS", 17,  AP_Networking,    _param.options,   AP_NETWORKING_DEFAULT_OPTIONS),

#if AP_NETWORKING_MAX_INSTANCES >= 1
    // @Group: 1_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 40, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[0], "1_", 41, AP_Networking, backend_var_info[0]),
#endif
#if AP_NETWORKING_MAX_INSTANCES >= 2
    // @Group: 2_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 42, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[1], "2_", 43, AP_Networking, backend_var_info[1]),
#endif
#if AP_NETWORKING_MAX_INSTANCES >= 3
    // @Group: 3_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 44, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[2], "3_", 45, AP_Networking, backend_var_info[2]),
#endif
#if AP_NETWORKING_MAX_INSTANCES >= 4
    // @Group: 4_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 46, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[3], "4_", 47, AP_Networking, backend_var_info[3]),
#endif
#if AP_NETWORKING_MAX_INSTANCES >= 5
    // @Group: 5_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[4], "5_", 48, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[4], "5_", 49, AP_Networking, backend_var_info[4]),
#endif
#if AP_NETWORKING_MAX_INSTANCES >= 6
    // @Group: 6_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[5], "6_", 50, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[5], "6_", 51, AP_Networking, backend_var_info[5]),
#endif
#if AP_NETWORKING_MAX_INSTANCES >= 7
    // @Group: 7_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[6], "7_", 52, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[6], "7_", 63, AP_Networking, backend_var_info[6]),
#endif
#if AP_NETWORKING_MAX_INSTANCES >= 8
    // @Group: 8_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[7], "8_", 54, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[7], "8_", 55, AP_Networking, backend_var_info[7]),
#endif
#if AP_NETWORKING_MAX_INSTANCES >= 9
    // @Group: 9_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[8], "9_", 56, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[8], "9_", 57, AP_Networking, backend_var_info[8]),
#endif

    AP_GROUPEND
};
const AP_Param::GroupInfo *AP_Networking::backend_var_info[AP_NETWORKING_MAX_INSTANCES];

AP_Networking::AP_Networking(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Networking must be singleton");
    }
#endif

    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}


void AP_Networking::init()
{
#ifdef HAL_GPIO_ETH_ENABLE
    hal.gpio->write(HAL_GPIO_ETH_ENABLE, _param.enabled ? 1 : 0);
#endif

    if (!_param.enabled) {
        return;
    }

    // set default MAC Address lower 3 bytes to UUID if possible
    uint8_t uuid[12];
    uint8_t uuid_len = sizeof(uuid);
    const bool udid_is_ok = hal.util->get_system_id_unformatted(uuid, uuid_len) && uuid_len >= 3;
    if (udid_is_ok) {
        _param.macaddr[3].set_default(uuid[uuid_len-2]);
        _param.macaddr[4].set_default(uuid[uuid_len-1]);
        _param.macaddr[5].set_default(uuid[uuid_len-0]);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    _activeSettings.ip = get_ip_param();
    _activeSettings.nm = get_netmask_param();
    _activeSettings.gw = get_gateway_param();
    net_addr_mode_t addrMode = NET_ADDRESS_STATIC;

    const uint8_t localMACAddress[6] = {(uint8_t)_param.macaddr[0].get(),
                                        (uint8_t)_param.macaddr[1].get(),
                                        (uint8_t)_param.macaddr[2].get(),
                                        (uint8_t)_param.macaddr[3].get(),
                                        (uint8_t)_param.macaddr[4].get(),
                                        (uint8_t)_param.macaddr[5].get() };

#if AP_NETWORKING_DEFAULT_DHCP_ENABLE
    if (get_dhcp_enabled()) {
        _activeSettings.ip = 0;
        _activeSettings.nm = 0;
        _activeSettings.gw = 0;
        addrMode = NET_ADDRESS_DHCP;
    }
#else
    set_dhcp_enable(false);
#endif

    struct lwipthread_opts netOptions = { (uint8_t *) localMACAddress,
                                        _activeSettings.ip,
                                        _activeSettings.nm,
                                        _activeSettings.gw,
                                        addrMode };

    lwipInit(&netOptions);
#endif

    // create each instance
    for (uint8_t instance = 0; instance < AP_NETWORKING_MAX_INSTANCES; instance++) {
        switch (get_type(instance)) {
#if AP_NETWORKING_SPEEDTEST_ENABLED
            case AP_Networking_Params::Type::SpeedTest:
                _drivers[instance] = new AP_Networking_SpeedTest(*this, _state[instance], _params[instance]);
                break;
#endif

#if AP_NETWORKING_LATENCYTEST_ENABLED
            case AP_Networking_Params::Type::LatencyTest:
                _drivers[instance] = new AP_Networking_LatencyTest(*this, _state[instance], _params[instance]);
                break;
#endif

            case AP_Networking_Params::Type::None:
            default:
                break;
        }

        // call init function for each backend
        if (_drivers[instance] != nullptr) {
            // if the backend has some local parameters then make those available in the tree
            if (_state[instance].var_info != nullptr) {
                backend_var_info[instance] = _state[instance].var_info;
                AP_Param::load_object_from_eeprom(_drivers[instance], backend_var_info[instance]);

                // param count could have changed
                AP_Param::invalidate_count();
            }
            _drivers[instance]->init();
            // _num_instances is actually the index for looping over instances
            _num_instances = instance + 1;
        }
    }

#if AP_NETWORKING_SNTP_ENABLED
    if (option_is_set(Options::SNTP)) {
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, AP_NETWORKING_SNTP_SERVERNAME);
        sntp_init();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: SNTP Ready: %s", AP_NETWORKING_SNTP_SERVERNAME);
    }
#endif

#if AP_NETWORKING_NETBIOS_ENABLED
    if (option_is_set(Options::NETBIOS)) {
        netbiosns_init();

        // Set the name of the device with example MAC:12-34-56-78-9A-BC
        //
        // BRD_SERIAL_NUM = 0
        // name: "ARDUPILOT789ABC"
        //
        // BRD_SERIAL_NUM = 123
        // name: "ARDUPILOT123"

        char name[NETBIOS_NAME_LEN] = {};
        const int32_t serial_number = (AP::boardConfig() != nullptr) ? AP::boardConfig()->get_serial_number() : 0;

        if (serial_number != 0) {
            hal.util->snprintf(name, ARRAY_SIZE(name), "%s%d",
                AP_NETWORKING_NETBIOS_NAME,
                (int)serial_number);

        } else {
            hal.util->snprintf(name, ARRAY_SIZE(name), "%s%02X%02X%02X",
                AP_NETWORKING_NETBIOS_NAME,
                (unsigned)localMACAddress[3],
                (unsigned)localMACAddress[4],
                (unsigned)localMACAddress[5]);
        }

        netbiosns_set_name(name);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: NETBIOS Ready: %s", name);
    }
#endif

#if AP_NETWORKING_TFTP_ENABLED
    if (option_is_set(Options::TFTP)) {
        static const struct tftp_context ctx { .open = tftp_open,
                                                .close = tftp_close,
                                                .read = tftp_read,
                                                .write = tftp_write};

        const err_t err = tftp_init(&ctx);
        if (err == ERR_OK) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: TFTP Ready");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"NET: TFTP Init error: %d", (int)err);
        }
    }
#endif

#if AP_NETWORKING_DEFAULT_DHCP_ENABLE
    if (get_dhcp_enabled()) {
        // give DHCP a chance to get an address before we show the boot-up address
        _activeSettings.announce_ms = AP_HAL::millis();
    }
#endif

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: Initialized");
    _init.done = true;
}

void AP_Networking::announce_address_changes()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _activeSettings.announce_ms < 1000) {
        // Never announce changes any faster than 1 sec
        return;
    }

    const uint32_t ip = lwipGetIp();
    const uint32_t nm = lwipGetNetmask();
    const uint32_t gw = lwipGetGateway();

    // struct netif thisif = lwipGetNetIf();
    // const uint32_t ip = thisif.ip_addr.u_addr.ip4.addr;
    // const uint32_t nm = thisif.netmask.u_addr.ip4.addr;
    // const uint32_t gw = thisif.gw.u_addr.ip4.addr;


    if (_activeSettings.announce_at_boot_done &&
        ip == _activeSettings.ip &&
        nm == _activeSettings.nm &&
        gw == _activeSettings.gw)
    {
        // nothing changed and we've already printed it at least once. Nothing to do.
        return;
    }

    _activeSettings.ip = ip;
    _activeSettings.nm = nm;
    _activeSettings.gw = gw;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: IP      %s", get_ip_active_str());
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: Mask    %s", get_netmask_active_str());
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: Gateway %s", get_gateway_active_str());

    if (!_activeSettings.announce_at_boot_done && ip == 0 && nm == 0 && gw == 0 && get_dhcp_enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: DHCP enabled, waiting for IP");
    }

    _activeSettings.announce_ms = now_ms;
    _activeSettings.announce_at_boot_done = true;
}

void AP_Networking::update()
{
    if (!_param.enabled || !_init.done) {
        return;
    }

    announce_address_changes();

    for (uint8_t i=0; i<_num_instances; i++) {
        if (_drivers[i] != nullptr && get_type(i) != AP_Networking_Params::Type::None) {
            _drivers[i]->update();
        }
    }
}

AP_Networking_Params::Type AP_Networking::get_type(const uint8_t instance) const
{
    if (instance >= AP_NETWORKING_MAX_INSTANCES) {
        return AP_Networking_Params::Type::None;
    }
    return (AP_Networking_Params::Type)_params[instance].type.get();
}

uint32_t AP_Networking::convert_netmask_bitcount_to_ip(const uint32_t netmask_bitcount)
{
    if (netmask_bitcount > 32) {
        return 0;
    }

    uint32_t netmask_ip = 0;
    for (uint32_t i=0; i<netmask_bitcount; i++) {
        netmask_ip |= (1UL << i);
    }
    return netmask_ip;
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

uint32_t AP_Networking::convert_str_to_ip(char* ip_str)
{
    // struct sockaddr_in antelope[2];
    // inet_pton(AF_INET, "10.0.0.2", &(antelope[0].sin_addr));
    // return inet_addr(ip_str);

    uint32_t ip = 0;
    inet_pton(AF_INET, ip_str, &ip);
    return ip;
}

char* AP_Networking::convert_ip_to_str(const uint8_t ip[4])
{
    static char _str_buffer[20];
    if (hal.util->snprintf(_str_buffer, sizeof(_str_buffer), "%u.%u.%u.%u", (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3]) == 0) {
        _str_buffer[0] = '\0';
    }
    return _str_buffer;
}
char* AP_Networking::convert_ip_to_str(const uint32_t ip)
{
    uint8_t ip_array[4];
        ip_array[3] = ((ip >> 24) & 0xff);
        ip_array[2] = ((ip >> 16) & 0xff);
        ip_array[1] = ((ip >> 8) & 0xff);
        ip_array[0] = (ip & 0xff);

    return convert_ip_to_str(ip_array);
}

int32_t AP_Networking::send_udp(struct udp_pcb *pcb, const ip4_addr_t &ip4_addr, const uint16_t port, const uint8_t* data, uint16_t data_len)
{
    if (pcb == nullptr) {
        return ERR_ARG;
    }
    
    data_len = (data == nullptr) ? 0 : data_len;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, data_len, PBUF_RAM);
    if (p == nullptr) {
        return ERR_MEM;
    }

    ip_addr_t dst;
    ip_addr_copy_from_ip4(dst, ip4_addr);

    if (data_len > 0) {
        // memcpy(p->payload, data, data_len);
        memcpy(&((uint8_t*)p->payload)[0], data, data_len);
    }

    const err_t err = udp_sendto(pcb, p, &dst, port);
    pbuf_free(p);

    return err == ERR_OK ? data_len : err;
}

#if AP_NETWORKING_TFTP_ENABLED
int AP_Networking::tftp_fd = -1;

// tftp wrappers for AP::Filesystem
void* AP_Networking::tftp_open(const char* fname, const char* mode, u8_t write)
{
    if (fname == nullptr || mode == nullptr) {
        return nullptr;
    }
    if (strcmp(mode, "octet") != 0) {
        return nullptr;
    }
    if (write) {
        tftp_fd = AP::FS().open(fname, O_WRONLY | O_CREAT | O_TRUNC);
    } else {
        tftp_fd = AP::FS().open(fname, O_RDONLY);
    }
    return &tftp_fd;
}

void AP_Networking::tftp_close(void* handle)
{
    if (&tftp_fd != handle) {
        return;
    }
    if (tftp_fd == -1) {
        return;
    }
    AP::FS().close(tftp_fd);
}

int AP_Networking::tftp_read(void* handle, void* buf, int bytes)
{
    if ((&tftp_fd != handle) || buf == nullptr) {
        return -1;
    }
    if (tftp_fd == -1) {
        return -1;
    }
    return AP::FS().read(tftp_fd, buf, bytes);
}

int AP_Networking::tftp_write(void* handle, struct pbuf* p)
{
    if ((&tftp_fd != handle) || p == nullptr) {
        return -1;
    }
    if (tftp_fd == -1) {
        return -1;
    }
    return AP::FS().write(tftp_fd, p->payload, p->len);
}
#endif // AP_NETWORKING_TFTP_ENABLED

AP_Networking *AP_Networking::_singleton;
namespace AP { 
    AP_Networking &network() {
        return *AP_Networking::get_singleton();
    }
}

#endif // AP_NETWORKING_ENABLED
