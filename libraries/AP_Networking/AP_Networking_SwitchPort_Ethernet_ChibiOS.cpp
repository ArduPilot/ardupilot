#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET

#include "AP_Networking_SwitchPort_Ethernet_ChibiOS.h"
#include "AP_Networking_Switch.h"
#include "AP_Networking_MAC_Buffers_ChibiOS.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <hal.h>
#if AP_NETWORKING_CAPTURE_ENABLED
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_RTC/AP_RTC.h>
#endif

#ifdef NEEDS_KSZ9896C_ERRATA
#include <hal_mii.h>
#endif


extern const AP_HAL::HAL& hal;

#define LWIP_SEND_TIMEOUT_MS 50

AP_Networking_SwitchPort_Ethernet_ChibiOS::AP_Networking_SwitchPort_Ethernet_ChibiOS(AP_Networking_Switch *hub_in, const uint8_t *mac)
    : hub(hub_in)
{
    memcpy(macaddr, mac, 6);
}

/*
  Allocate buffers for MAC DMA
*/
bool AP_Networking_SwitchPort_Ethernet_ChibiOS::allocate_buffers()
{
    return AP_Networking_MAC_Buffers::allocate_buffers();
}

bool AP_Networking_SwitchPort_Ethernet_ChibiOS::init()
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
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_SwitchPort_Ethernet_ChibiOS::rx_thread, void),
                                      "eth_rx",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: eth_rx thread start failed");
        return false;
    }
    // start dedicated TX thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_SwitchPort_Ethernet_ChibiOS::tx_thread, void),
                                      "eth_tx",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: eth_tx thread start failed");
        return false;
    }
    return true;
}

bool AP_Networking_SwitchPort_Ethernet_ChibiOS::mac_init_and_start()
{
    if (!macInit()) {
        return false;
    }

    const MACConfig mac_config = {macaddr};
    macStart(&ETHD1, &mac_config);

#ifdef NEEDS_KSZ9896C_ERRATA
    apply_errata_for_mac_KSZ9896C();
#endif

    // Start with promiscuous mode disabled - will be enabled dynamically if needed
    set_promiscuous_mode(false);
    return true;
}

void AP_Networking_SwitchPort_Ethernet_ChibiOS::update()
{
    // Promiscuous mode is needed when there are multiple external connections
    // (e.g., Ethernet + COBS bridge), so we can forward frames between them.
    // With only Ethernet + lwIP, we only need to receive frames for our MAC.
    const bool need_promiscuous = (hub != nullptr) && (hub->get_num_external_ports_link_up() > 1);
    
    if (need_promiscuous != promiscuous_enabled) {
        set_promiscuous_mode(need_promiscuous);
    }
}

void AP_Networking_SwitchPort_Ethernet_ChibiOS::set_promiscuous_mode(bool enable)
{
    if (ETH == nullptr) {
        return;
    }
    
    promiscuous_enabled = enable;
    
#if defined(LWIP_IGMP) && LWIP_IGMP
    // Pass all multicast for IGMP support
    if (enable) {
        ETH->MACPFR |= ETH_MACPFR_PM;
    } else {
        ETH->MACPFR &= ~ETH_MACPFR_PM;
    }
#endif

#ifdef ETH_MACPFR_PR
    // Promiscuous mode - receive all frames regardless of destination MAC
    if (enable) {
        ETH->MACPFR |= ETH_MACPFR_PR;
    } else {
        ETH->MACPFR &= ~ETH_MACPFR_PR;
    }
#endif
}

bool AP_Networking_SwitchPort_Ethernet_ChibiOS::process_one_rx_descriptor(uint32_t timeout_ms)
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
#if AP_NETWORKING_CAPTURE_ENABLED
    capture_frame(rx_framebuf, len);
#endif
    hub->route_frame(this, rx_framebuf, len);
    rx_count++;
    return true;
}

void AP_Networking_SwitchPort_Ethernet_ChibiOS::rx_thread()
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

void AP_Networking_SwitchPort_Ethernet_ChibiOS::tx_thread()
{
    if (tx_queue == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: SwitchPort.Eth tx_queue is null");
        return;
    }

    while (true) {
        // Block until signaled
        tx_not_empty.wait_blocking();
        // Drain all available frames.
        // Manual take/give instead of WITH_SEMAPHORE because the mutex
        // is acquired and released within each loop iteration, not at
        // function scope.
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
            if (tx_queue->read(tx_framebuf, l) != l) {
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
#if AP_NETWORKING_CAPTURE_ENABLED
            capture_frame(tx_framebuf, l);
#endif
            macWriteTransmitDescriptor(&td, tx_framebuf, (size_t)l);
            macReleaseTransmitDescriptorX(&td);
            tx_count++;
        }
    }
}

void AP_Networking_SwitchPort_Ethernet_ChibiOS::deliver_frame(const uint8_t *frame, size_t len)
{
    if (frame == nullptr || len == 0 || len > AP_Networking_Switch::MAX_ETH_FRAME) {
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

bool AP_Networking_SwitchPort_Ethernet_ChibiOS::can_receive() const
{
    return tx_queue != nullptr && tx_queue->space() >= (2 + MAX_FRAME);
}

bool AP_Networking_SwitchPort_Ethernet_ChibiOS::is_link_up() const
{
    return macPollLinkStatus(&ETHD1);
}

#ifdef NEEDS_KSZ9896C_ERRATA
void AP_Networking_SwitchPort_Ethernet_ChibiOS::apply_errata_for_mac_KSZ9896C()
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

#if AP_NETWORKING_CAPTURE_ENABLED
/*
  start a pcap network capture
 */
void AP_Networking_SwitchPort_Ethernet_ChibiOS::start_capture()
{
    if (capture.fd != -1) {
        // called at 1Hz, flush the file
        AP::FS().fsync(capture.fd);
        return;
    }
    const struct pcap_hdr {
        uint32_t magic_number;   // 0xa1b2c3d4
        uint16_t version_major;  // 2
        uint16_t version_minor;  // 4
        int32_t  thiszone;       // GMT to local correction
        uint32_t sigfigs;        // accuracy of timestamps
        uint32_t snaplen;        // max length of captured packets, in octets
        uint32_t network;        // data link type (1 for Ethernet)
    } hdr = {
        0xa1b2c3d4, 2, 4, 0, 0, 1500, 1
    };
    const char *fname = "eth0.cap";
    WITH_SEMAPHORE(capture.sem);
    auto &fs = AP::FS();
    capture.fd = fs.open(fname, O_WRONLY|O_CREAT|O_TRUNC);
    if (capture.fd != -1) {
        fs.write(capture.fd, (const void *)&hdr, sizeof(hdr));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Capturing to %s", fname);
    }
}

/*
  stop a pcap network capture
 */
void AP_Networking_SwitchPort_Ethernet_ChibiOS::stop_capture()
{
    WITH_SEMAPHORE(capture.sem);
    if (capture.fd != -1) {
        int fd = capture.fd;
        capture.fd = -1;
        AP::FS().close(fd);
    }
}

/*
  capture a frame to pcap file
 */
void AP_Networking_SwitchPort_Ethernet_ChibiOS::capture_frame(const uint8_t *frame, size_t len)
{
    WITH_SEMAPHORE(capture.sem);
    if (capture.fd == -1) {
        return;
    }
    uint64_t utc_usec = 0;
#if AP_RTC_ENABLED
    AP::rtc().get_utc_usec(utc_usec);
#endif
    if (utc_usec == 0) {
        utc_usec = AP_HAL::micros64();
    }
    const struct pcaprec_hdr {
        uint32_t ts_sec;
        uint32_t ts_usec;
        uint32_t incl_len;
        uint32_t orig_len;
    } rec {
        .ts_sec = uint32_t(utc_usec / 1000000ULL),
        .ts_usec = uint32_t(utc_usec % 1000000ULL),
        .incl_len = uint32_t(len),
        .orig_len = uint32_t(len)
    };
    auto &fs = AP::FS();
    fs.write(capture.fd, (const void *)&rec, sizeof(rec));
    fs.write(capture.fd, frame, len);
}
#endif // AP_NETWORKING_CAPTURE_ENABLED

#endif // AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET
