
#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED

#include "AP_Math/definitions.h"
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal_mii.h>
#include <lwip/sockets.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>

#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Networking::var_info[] = {

    // @Param: ENABLED
    // @DisplayName: Enable Networking
    // @Description: Enable Networking
    // @Values: 0:Disable,1:Enable
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED",  0, AP_Networking, _param.enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: IPADDR0
    // @DisplayName: IPv4 Address MSB
    // @Description: Allows setting static IP address. Example: 192.xxx.xxx.xxx
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("IPADDR0", 1,  AP_Networking,    _param.ipaddr[0],   AP_NETWORKING_DEFAULT_STATIC_IP_ADDR0),

    // @Param: IPADDR1
    // @DisplayName: IPv4 Address 2nd byte
    // @Description: Allows setting static IP address. Example: xxx.168.xxx.xxx
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("IPADDR1", 2,  AP_Networking,    _param.ipaddr[1],   AP_NETWORKING_DEFAULT_STATIC_IP_ADDR1),

    // @Param: IPADDR2
    // @DisplayName: IPv4 Address 3rd byte
    // @Description: Allows setting static IP address. Example: xxx.xxx.13.xxx
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("IPADDR2", 3,  AP_Networking,    _param.ipaddr[2],   AP_NETWORKING_DEFAULT_STATIC_IP_ADDR2),

    // @Param: IPADDR3
    // @DisplayName: IPv4 Address LSB
    // @Description: Allows setting static IP address. Example: xxx.xxx.xxx.14
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("IPADDR3", 4,  AP_Networking,    _param.ipaddr[3],   AP_NETWORKING_DEFAULT_STATIC_IP_ADDR3),

    // @Param: NETMASK
    // @DisplayName: IP Subnet mask
    // @Description: Allows setting static subnet mask. The value is a count of consecutive bits. Examples: 24 = 255.255.255.0, 16 = 255.255.0.0
    // @Range: 0 32
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("NETMASK", 5,  AP_Networking,    _param.netmask,   AP_NETWORKING_DEFAULT_NETMASK),

    // @Param: DHCP
    // @DisplayName: DHCP client
    // @Description: Enable/Disable DHCP client
    // @Values: 0:Disable, 1:Enable
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("DHCP", 6,  AP_Networking,    _param.dhcp,   AP_NETWORKING_DEFAULT_DHCP_ENABLE),

    // @Param: GWADDR0
    // @DisplayName: Gateway IP Address MSB
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("GWADDR0", 7,  AP_Networking,    _param.gwaddr[0],   AP_NETWORKING_DEFAULT_STATIC_GW_ADDR0),

    // @Param: GWADDR1
    // @DisplayName: Gateway IP Address 2nd byte
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("GWADDR1", 8,  AP_Networking,    _param.gwaddr[1],   AP_NETWORKING_DEFAULT_STATIC_GW_ADDR1),

    // @Param: GWADDR2
    // @DisplayName: Gateway IP Address 3rd byte
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("GWADDR2", 9,  AP_Networking,    _param.gwaddr[2],   AP_NETWORKING_DEFAULT_STATIC_GW_ADDR2),

    // @Param: GWADDR3
    // @DisplayName: Gateway IP Address LSB
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("GWADDR3", 10,  AP_Networking,    _param.gwaddr[3],   AP_NETWORKING_DEFAULT_STATIC_GW_ADDR3),

    // @Param: MACADDR0
    // @DisplayName: MAC Address MSbyte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("MACADDR0", 11,  AP_Networking,    _param.macaddr[0],  AP_NETWORKING_DEFAULT_MAC_ADDR0),

    // @Param: MACADDR1
    // @DisplayName: MAC Address 2nd byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("MACADDR1", 12,  AP_Networking,    _param.macaddr[1],   AP_NETWORKING_DEFAULT_MAC_ADDR1),

    // @Param: MACADDR2
    // @DisplayName: MAC Address 3rd byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("MACADDR2", 13,  AP_Networking,    _param.macaddr[2],   AP_NETWORKING_DEFAULT_MAC_ADDR2),

    // @Param: MACADDR3
    // @DisplayName: MAC Address 4th byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("MACADDR3", 14,  AP_Networking,    _param.macaddr[3],   AP_NETWORKING_DEFAULT_MAC_ADDR3),

    // @Param: MACADDR4
    // @DisplayName: MAC Address 5th byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("MACADDR4", 15,  AP_Networking,    _param.macaddr[4],   AP_NETWORKING_DEFAULT_MAC_ADDR4),

    // @Param: MACADDR5
    // @DisplayName: MAC Address LSb
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("MACADDR5", 16,  AP_Networking,    _param.macaddr[5],   AP_NETWORKING_DEFAULT_MAC_ADDR5),

    AP_GROUPEND
};

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

#if defined(STM32_ETH_BUFFERS_EXTERN) && (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS)
stm32_eth_rx_descriptor_t *__eth_rd;
stm32_eth_tx_descriptor_t *__eth_td;
uint32_t *__eth_rb[STM32_MAC_RECEIVE_BUFFERS];
uint32_t *__eth_tb[STM32_MAC_TRANSMIT_BUFFERS];

static bool allocate_buffers()
{
    #define AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE ((((STM32_MAC_BUFFERS_SIZE - 1) | 3) + 1) / 4) // typically == 381
    // check total size of buffers
    const uint32_t total_size = sizeof(stm32_eth_rx_descriptor_t)*STM32_MAC_RECEIVE_BUFFERS +
                                sizeof(stm32_eth_tx_descriptor_t)*STM32_MAC_TRANSMIT_BUFFERS +
                                sizeof(uint32_t)*STM32_MAC_RECEIVE_BUFFERS*AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE +
                                sizeof(uint32_t)*STM32_MAC_TRANSMIT_BUFFERS*AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE; // typically == 9240

    // ensure that we allocate 32-bit aligned memory, and mark it non-cacheable
    uint32_t size = 2;
    uint8_t rasr = 0;
    // find size closest to power of 2
    while (size < total_size) {
        size = size << 1;
        rasr++;
    }
    void *mem = malloc_eth_safe(size);
    // ensure our memory is aligned
    // ref. Cortex-M7 peripherals PM0253, section 4.6.4 MPU region base address register
    if (((uint32_t)mem) % size) {
        AP_HAL::panic("Bad alignment of ETH memory");
    }
    if (mem == nullptr) {
        return false;
    }

    // for total_size == 9240, size should be 16384 and (rasr-1) should be 13 (MPU_RASR_SIZE_16K)
    const uint32_t rasr_size = MPU_RASR_SIZE(rasr-1);

    // set up MPU region for buffers
    mpuConfigureRegion(STM32_NOCACHE_MPU_REGION_ETH,
                       (uint32_t)mem,
                       MPU_RASR_ATTR_AP_RW_RW |
                       MPU_RASR_ATTR_NON_CACHEABLE |
                       MPU_RASR_ATTR_S |
                       rasr_size |
                       MPU_RASR_ENABLE);
    mpuEnable(MPU_CTRL_PRIVDEFENA);
    SCB_CleanInvalidateDCache();

    // assign buffers
    __eth_rd = (stm32_eth_rx_descriptor_t *)mem;
    __eth_td = (stm32_eth_tx_descriptor_t *)&__eth_rd[STM32_MAC_RECEIVE_BUFFERS];
    __eth_rb[0] = (uint32_t*)&__eth_td[STM32_MAC_TRANSMIT_BUFFERS];
    for (uint16_t i = 1; i < STM32_MAC_RECEIVE_BUFFERS; i++) {
        __eth_rb[i] = &(__eth_rb[i-1][AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE]);
    }
    __eth_tb[0] = &(__eth_rb[STM32_MAC_RECEIVE_BUFFERS-1][AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE]);
    for (uint16_t i = 1; i < STM32_MAC_TRANSMIT_BUFFERS; i++) {
        __eth_tb[i] = &(__eth_tb[i-1][AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE]);
    }
    return true;
}
#endif

void AP_Networking::init()
{
#ifdef HAL_GPIO_ETH_ENABLE
    hal.gpio->pinMode(HAL_GPIO_ETH_ENABLE, HAL_GPIO_OUTPUT);
    hal.gpio->write(HAL_GPIO_ETH_ENABLE, _param.enabled ? 1 : 0);
#endif

    if (!_param.enabled || _init.done) {
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
#ifdef STM32_ETH_BUFFERS_EXTERN
    if (!allocate_buffers()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: Failed to allocate buffers");
        return;
    }
#endif
    if (!macInit()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: macInit failed");
        return;
    }

    const uint8_t localMACAddress[6] = {(uint8_t)_param.macaddr[0].get(),
                                        (uint8_t)_param.macaddr[1].get(),
                                        (uint8_t)_param.macaddr[2].get(),
                                        (uint8_t)_param.macaddr[3].get(),
                                        (uint8_t)_param.macaddr[4].get(),
                                        (uint8_t)_param.macaddr[5].get()
                                       };

#if !AP_NETWORKING_DHCP_AVAILABLE
    set_dhcp_enable(false);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: DHCP Not Supported");
#endif

    net_addr_mode_t addrMode;
    if (get_dhcp_enabled()) {
        _activeSettings.ip = 0;
        _activeSettings.nm = 0;
        _activeSettings.gw = 0;
        addrMode = NET_ADDRESS_DHCP;
    } else {
        _activeSettings.ip = get_ip_param();
        _activeSettings.nm = get_netmask_param();
        _activeSettings.gw = get_gateway_param();
        addrMode = NET_ADDRESS_STATIC;
    }

    struct lwipthread_opts netOptions = { (uint8_t *) localMACAddress,
               _activeSettings.ip,
               _activeSettings.nm,
               _activeSettings.gw,
               addrMode
    };

    lwipInit(&netOptions);
#endif

#if AP_NETWORKING_DHCP_AVAILABLE
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

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    const uint32_t ip = lwipGetIp();
    const uint32_t nm = lwipGetNetmask();
    const uint32_t gw = lwipGetGateway();
    // struct netif thisif = lwipGetNetIf();
    // const uint32_t ip = thisif.ip_addr.u_addr.ip4.addr;
    // const uint32_t nm = thisif.netmask.u_addr.ip4.addr;
    // const uint32_t gw = thisif.gw.u_addr.ip4.addr;
#else
    const uint32_t ip = 0;
    const uint32_t nm = 0;
    const uint32_t gw = 0;
#endif

    if (_activeSettings.announce_at_boot_done &&
        ip == _activeSettings.ip &&
        nm == _activeSettings.nm &&
        gw == _activeSettings.gw) {
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
    if (!is_healthy()) {
        return;
    }

    announce_address_changes();
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

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
int32_t AP_Networking::send_udp(struct udp_pcb *pcb, const ip4_addr_t &ip4_addr, const uint16_t port, const uint8_t* data, uint16_t data_len)
{
    if (!AP::network().is_healthy()) {
        return ERR_IF;
    }

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
        memcpy(p->payload, data, data_len);
    }

    const err_t err = udp_sendto(pcb, p, &dst, port);
    pbuf_free(p);

    return err == ERR_OK ? data_len : err;
}
#endif // #if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

AP_Networking *AP_Networking::_singleton;
namespace AP
{
AP_Networking &network()
{
    return *AP_Networking::get_singleton();
}
}

#endif // AP_NETWORKING_ENABLED
