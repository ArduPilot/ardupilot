#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

#include "AP_Networking_Port_Ethernet_ChibiOS.h"
#include "AP_Networking_Hub.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <hal.h>

#ifdef NEEDS_KSZ9896C_ERRATA
#include <hal_mii.h>
#endif


extern const AP_HAL::HAL& hal;

#ifndef STM32_ETH_BUFFERS_EXTERN
#error "Must use external ethernet buffers"
#endif

/*
  MAC DMA buffers - referenced as globals by ChibiOS MAC driver
*/
stm32_eth_rx_descriptor_t *__eth_rd;
stm32_eth_tx_descriptor_t *__eth_td;
uint32_t *__eth_rb[STM32_MAC_RECEIVE_BUFFERS];
uint32_t *__eth_tb[STM32_MAC_TRANSMIT_BUFFERS];

#define LWIP_SEND_TIMEOUT_MS 50

AP_Networking_Port_Ethernet_ChibiOS::AP_Networking_Port_Ethernet_ChibiOS(AP_Networking_Hub *hub_in, const uint8_t *mac)
    : hub(hub_in)
{
    memcpy(macaddr, mac, 6);
}

/*
  Allocate buffers for MAC DMA
*/
bool AP_Networking_Port_Ethernet_ChibiOS::allocate_buffers()
{
#define AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE ((((STM32_MAC_BUFFERS_SIZE - 1) | 3) + 1) / 4)
    const uint32_t total_size = sizeof(stm32_eth_rx_descriptor_t)*STM32_MAC_RECEIVE_BUFFERS +
        sizeof(stm32_eth_tx_descriptor_t)*STM32_MAC_TRANSMIT_BUFFERS +
        sizeof(uint32_t)*STM32_MAC_RECEIVE_BUFFERS*AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE +
        sizeof(uint32_t)*STM32_MAC_TRANSMIT_BUFFERS*AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE;

    uint32_t size = 1;
    uint8_t rasr = 0;
    while (size < total_size) {
        size = size << 1;
        rasr++;
    }
    void *mem = malloc_eth_safe(size);
    if (mem == nullptr) {
        return false;
    }

#ifndef HAL_BOOTLOADER_BUILD
    if (((uint32_t)mem) % size) {
        return false;  // Bad alignment
    }
#endif

    const uint32_t rasr_size = MPU_RASR_SIZE(rasr-1);

    mpuConfigureRegion(STM32_NOCACHE_MPU_REGION_ETH,
                       (uint32_t)mem,
                       MPU_RASR_ATTR_AP_RW_RW |
                       MPU_RASR_ATTR_NON_CACHEABLE |
                       MPU_RASR_ATTR_S |
                       rasr_size |
                       MPU_RASR_ENABLE);
    mpuEnable(MPU_CTRL_PRIVDEFENA);
    SCB_CleanInvalidateDCache();

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

bool AP_Networking_Port_Ethernet_ChibiOS::init()
{
#ifdef HAL_GPIO_ETH_ENABLE
    hal.gpio->pinMode(HAL_GPIO_ETH_ENABLE, HAL_GPIO_OUTPUT);
    hal.gpio->write(HAL_GPIO_ETH_ENABLE, 0); // reset
    hal.scheduler->delay(25);
    hal.gpio->write(HAL_GPIO_ETH_ENABLE, 1); // enable
    hal.scheduler->delay(10);
#endif

    if (!allocate_buffers()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: Failed to allocate MAC buffers");
        return false;
    }
    if (!mac_init_and_start()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: MAC init/start failed");
        return false;
    }
    // allocate TX queue
    if (tx_queue == nullptr) {
        tx_queue = NEW_NOTHROW ByteBuffer(TX_QUEUE_SIZE);
        if (tx_queue == nullptr) {
            return false;
        }
    }
    // start dedicated RX thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_Port_Ethernet_ChibiOS::rx_thread, void),
                                      "eth_rx",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: eth_rx thread start failed");
        return false;
    }
    // start dedicated TX thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_Port_Ethernet_ChibiOS::tx_thread, void),
                                      "eth_tx",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: eth_tx thread start failed");
        return false;
    }
    return true;
}

bool AP_Networking_Port_Ethernet_ChibiOS::mac_init_and_start()
{
    if (!macInit()) {
        return false;
    }

    const MACConfig mac_config = {macaddr};
    macStart(&ETHD1, &mac_config);

#ifdef NEEDS_KSZ9896C_ERRATA
    apply_errata_for_mac_KSZ9896C();
#endif

    set_promiscuous_mode();
    return true;
}

void AP_Networking_Port_Ethernet_ChibiOS::set_promiscuous_mode()
{
#if defined(LWIP_IGMP) && LWIP_IGMP
    if (ETH != nullptr) {
        ETH->MACPFR |= ETH_MACPFR_PM;
    }
#endif
    if (ETH != nullptr) {
#ifdef ETH_MACPFR_PR
        ETH->MACPFR |= ETH_MACPFR_PR;
#endif
    }
}

bool AP_Networking_Port_Ethernet_ChibiOS::process_one_rx_descriptor(uint32_t timeout_ms)
{
    MACReceiveDescriptor rd;
    sysinterval_t timeout = (timeout_ms == 0) ? TIME_IMMEDIATE : 
                            (timeout_ms == UINT32_MAX) ? TIME_INFINITE : TIME_MS2I(timeout_ms);
    
    if (macWaitReceiveDescriptor(&ETHD1, &rd, timeout) != MSG_OK) {
        return false;
    }
    const size_t len = (size_t)rd.size;
    if (len == 0 || len > MAX_FRAME) {
        macReleaseReceiveDescriptorX(&rd);
        rx_errors++;
        return true;
    }
    macReadReceiveDescriptor(&rd, rx_framebuf, len);
    macReleaseReceiveDescriptorX(&rd);
    hub->route_frame(this, rx_framebuf, len);
    rx_count++;
    return true;
}

void AP_Networking_Port_Ethernet_ChibiOS::rx_thread()
{
    while (true) {
        // Block until a frame arrives
        if (!process_one_rx_descriptor(UINT32_MAX)) {
            continue;
        }
        // Drain any additional pending frames without blocking
        while (process_one_rx_descriptor(0)) {
        }
    }
}

void AP_Networking_Port_Ethernet_ChibiOS::tx_thread()
{
    static uint8_t buf[MAX_FRAME];
    while (true) {
        if (tx_queue == nullptr) {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }
        // Block until signaled
        tx_not_empty.wait_blocking();
        // Drain all available frames
        while (true) {
            tx_mutex.take_blocking();
            if (tx_queue->available() < 2) {
                tx_mutex.give();
                break;
            }
            uint8_t hdr[2];
            if (tx_queue->peekbytes(hdr, 2) != 2) {
                tx_mutex.give();
                break;
            }
            const uint16_t l = (uint16_t)(hdr[0] | (uint16_t(hdr[1]) << 8));
            if (l == 0 || l > MAX_FRAME) {
                (void)tx_queue->advance(tx_queue->available());
                tx_mutex.give();
                tx_errors++;
                break;
            }
            if (tx_queue->available() < (uint32_t)(2 + l)) {
                tx_mutex.give();
                break;
            }
            (void)tx_queue->advance(2);
            if (tx_queue->read(buf, l) != l) {
                tx_mutex.give();
                break;
            }
            tx_mutex.give();
            // Transmit via MAC (outside mutex)
            MACTransmitDescriptor td;
            if (macWaitTransmitDescriptor(&ETHD1, &td, TIME_MS2I(LWIP_SEND_TIMEOUT_MS)) != MSG_OK) {
                tx_errors++;
                continue;
            }
            macWriteTransmitDescriptor(&td, buf, (size_t)l);
            macReleaseTransmitDescriptorX(&td);
            tx_count++;
        }
    }
}

void AP_Networking_Port_Ethernet_ChibiOS::deliver_frame(const uint8_t *frame, size_t len)
{
    if (frame == nullptr || len == 0 || len > AP_Networking_Hub::MAX_ETH_FRAME) {
        tx_errors++;
        return;
    }
    if (tx_queue == nullptr) {
        return;
    }
    // Serialize writers; avoid blocking the caller
    if (!tx_mutex.take(1)) {
        tx_errors++;
        return;
    }
    const uint32_t need = (uint32_t)(2 + len);
    if (tx_queue->space() < need) {
        tx_errors++;
        tx_mutex.give();
        return;
    }
    uint16_t l = (uint16_t)len;
    const uint8_t hdr[2] = { (uint8_t)(l & 0xFF), (uint8_t)((l >> 8) & 0xFF) };
    (void)tx_queue->write(hdr, 2);
    (void)tx_queue->write(frame, (uint32_t)len);
    tx_mutex.give();
    tx_not_empty.signal();
}

bool AP_Networking_Port_Ethernet_ChibiOS::can_receive() const
{
    return tx_queue != nullptr && tx_queue->space() >= (2 + MAX_FRAME);
}

bool AP_Networking_Port_Ethernet_ChibiOS::is_link_up() const
{
    return macPollLinkStatus(&ETHD1);
}

#ifdef NEEDS_KSZ9896C_ERRATA
void AP_Networking_Port_Ethernet_ChibiOS::apply_errata_for_mac_KSZ9896C()
{

    /// Apply Erratas
    for (uint8_t phyaddr = 1; phyaddr <= 5; phyaddr++) {

        ETHD1.phyaddr = phyaddr << ETH_MACMDIOAR_PA_Pos;

        // Hardware Design Checklist
        // https://ww1.microchip.com/downloads/en/DeviceDoc/KSZ989x-KSZ956x-KSZ9477-Hardware-Design-Checklist-00004151.pdf
        // 6.4: 10/100 Mbps Ethernet Only
        mii_write(&ETHD1, 0x00, 0x3100); // disable 1000Gbps, enable auto-negotiate for 10/100
        mii_write(&ETHD1, 0x09, 0x0400); // disable 1000Gbps announcements
        mii_write(&ETHD1, 0x00, 0x3100 | (1 << 9)); // restart auto-negotiate

        // Erratas:
        // http://ww1.microchip.com/downloads/en/DeviceDoc/80000757A.pdf
        const uint16_t mmd[22][3] = {
            //[MMD], [register],[data]

            // module 1: Register settings are needed to improve PHY receive performance
            {0x01, 0x6F, 0xDD0B},
            {0x01, 0x8F, 0x6032},
            {0x01, 0x9D, 0x248C},
            {0x01, 0x75, 0x0060},
            {0x01, 0xD3, 0x7777},
            {0x1C, 0x06, 0x3008},
            {0x1C, 0x08, 0x2001},

            // module 2: Transmit waveform amplitude can be improved
            {0x1C, 0x04F, 0x00D0},

            // module 3: Energy Efficient Ethernet (EEE) feature select must be manually disabled
            {0x07, 0x03C, 0x0000},

            // module 4: Toggling PHY Powerdown can cause errors or link failures in adjacent PHYs
            #if STM32_MAC_ETH1_CHANGE_PHY_STATE
            #error "MII_KSZ9896C_ID Errata module 4 requires STM32_MAC_ETH1_CHANGE_PHY_STATE = FALSE"
            #endif

            // module 6: Register settings are required to meet data sheet supply current specifications
            {0x1C, 0x013, 0x6EFF}, // This particular Register is critical for an unknown reason.
            {0x1C, 0x014, 0xE6FF},
            {0x1C, 0x015, 0x6EFF},
            {0x1C, 0x016, 0xE6FF},
            {0x1C, 0x017, 0x00FF},
            {0x1C, 0x018, 0x43FF},
            {0x1C, 0x019, 0xC3FF},
            {0x1C, 0x01A, 0x6FFF},
            {0x1C, 0x01B, 0x07FF},
            {0x1C, 0x01C, 0x0FFF},
            {0x1C, 0x01D, 0xE7FF},
            {0x1C, 0x01E, 0xEFFF},
            {0x1C, 0x020, 0xEEEE},
        };

        for (uint8_t i=0; i<22; i++) {
            const uint16_t deviceAddress = (mmd[i][0] & 0x001F);
            mii_write(&ETHD1, 0x0D, 0x0000 | deviceAddress);
            mii_write(&ETHD1, 0x0E, mmd[i][1]);

            mii_write(&ETHD1, 0x0D, 0x4000 | deviceAddress);
            mii_write(&ETHD1, 0x0E, mmd[i][2]);
        }
    }
    ETHD1.phyaddr = BOARD_PHY_ADDRESS;
}
#endif // NEEDS_KSZ9896C_ERRATA

#endif // AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
