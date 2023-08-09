
#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include "AP_Networking.h"
#include "AP_Networking_ChibiOS.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include <lwipthread.h>
#include <lwip/udp.h>
#include <lwip/ip_addr.h>

extern const AP_HAL::HAL& hal;

#ifndef STM32_ETH_BUFFERS_EXTERN
#error "Must use external ethernet buffers"
#endif

/*
  these are referenced as globals inside lwip
 */
stm32_eth_rx_descriptor_t *__eth_rd;
stm32_eth_tx_descriptor_t *__eth_td;
uint32_t *__eth_rb[STM32_MAC_RECEIVE_BUFFERS];
uint32_t *__eth_tb[STM32_MAC_TRANSMIT_BUFFERS];

/*
  allocate buffers for LWIP
 */
bool AP_Networking_ChibiOS::allocate_buffers()
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
    if (mem == nullptr) {
        return false;
    }
    // ensure our memory is aligned
    // ref. Cortex-M7 peripherals PM0253, section 4.6.4 MPU region base address register
    if (((uint32_t)mem) % size) {
        AP_HAL::panic("Bad alignment of ETH memory");
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

/*
  initialise ChibiOS network backend using LWIP
 */
bool AP_Networking_ChibiOS::init()
{
#ifdef HAL_GPIO_ETH_ENABLE
    hal.gpio->pinMode(HAL_GPIO_ETH_ENABLE, HAL_GPIO_OUTPUT);
    hal.gpio->write(HAL_GPIO_ETH_ENABLE, frontend.param.enabled ? 1 : 0);
#endif

    if (!allocate_buffers()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: Failed to allocate buffers");
        return false;
    }

    if (!macInit()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: macInit failed");
        return false;
    }

#if !AP_NETWORKING_DHCP_AVAILABLE
    frontend.set_dhcp_enable(false);
#endif

    lwip_options = new lwipthread_opts;

    if (frontend.get_dhcp_enabled()) {
        lwip_options->addrMode = NET_ADDRESS_DHCP;
    } else {
        lwip_options->addrMode = NET_ADDRESS_STATIC;
        lwip_options->address = htonl(frontend.get_ip_param());
        lwip_options->netmask = htonl(frontend.get_netmask_param());
        lwip_options->gateway = htonl(frontend.get_gateway_param());
    }
    frontend.param.macaddr.get_address(macaddr);
    lwip_options->macaddress = macaddr;

    lwipInit(lwip_options);

    return true;
}

/*
  update called at 10Hz
 */
void AP_Networking_ChibiOS::update()
{
    const uint32_t ip = ntohl(lwipGetIp());
    const uint32_t nm = ntohl(lwipGetNetmask());
    const uint32_t gw = ntohl(lwipGetGateway());

    if (ip != activeSettings.ip ||
        nm != activeSettings.nm ||
        gw != activeSettings.gw) {
        activeSettings.ip = ip;
        activeSettings.gw = gw;
        activeSettings.nm = nm;
        activeSettings.last_change_ms = AP_HAL::millis();
    }
}

/*
  send a UDP packet
 */
int32_t AP_Networking_ChibiOS::send_udp(struct udp_pcb *pcb, const ip4_addr_t &ip4_addr, const uint16_t port, const uint8_t* data, uint16_t data_len)
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
        memcpy(p->payload, data, data_len);
    }

    const err_t err = udp_sendto(pcb, p, &dst, port);
    pbuf_free(p);

    return err == ERR_OK ? data_len : err;
}

#endif // AP_NETWORKING_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

