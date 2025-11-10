/*
  CAN UDP multicast server
 */

#include "AP_Networking_Config.h"

#if AP_NETWORKING_CAN_MCAST_ENABLED

#include "AP_Networking.h"

#include <AP_HAL/utility/Socket.h>
#include <AP_HAL/CANIface.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/crc.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "hal.h"
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#endif

#define MCAST_ADDRESS_BASE "239.65.82.0"
#define MCAST_PORT 57732U
#define MCAST_MAGIC 0x2934U
#define MCAST_FLAG_CANFD 0x0001
#define MCAST_MAX_PKT_LEN 74 // 64 byte data + 10 byte header

struct PACKED mcast_pkt {
    uint16_t magic;
    uint16_t crc;
    uint16_t flags;
    uint32_t message_id;
    uint8_t data[MCAST_MAX_PKT_LEN-10];
};

#define MCAST_HDR_LENGTH offsetof(mcast_pkt, data)

extern const AP_HAL::HAL& hal;

#ifdef HAL_BOOTLOADER_BUILD
void AP_Networking_CAN::mcast_trampoline(void *ctx)
{
    auto *mcast = (AP_Networking_CAN *)ctx;
    mcast->mcast_server();
}
extern ChibiOS::CANIface can_iface[HAL_NUM_CAN_IFACES];
extern void thread_sleep_us(uint32_t us);
#endif // HAL_BOOTLOADER_BUILD

/*
  get CAN interface for a bus
 */
AP_HAL::CANIface *AP_Networking_CAN::get_caniface(uint8_t bus) const
{
#ifdef HAL_BOOTLOADER_BUILD
    return &can_iface[bus];
#else
    return hal.can[bus];
#endif
}

/*
  start the CAN multicast server
 */
void AP_Networking_CAN::start(const uint8_t _bus_mask)
{
    const uint32_t stack_size = 8192;
    bus_mask = _bus_mask;

#ifdef HAL_BOOTLOADER_BUILD
    thread_create_alloc(THD_WORKING_AREA_SIZE(stack_size),
                        "CAN_MCAST",
                        60,
                        mcast_trampoline,
                        this);
#else
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_CAN::mcast_server, void),
                                 "CAN_MCAST",
                                 stack_size, AP_HAL::Scheduler::PRIORITY_CAN, -1);
#endif
}

/*
  main thread for CAN multicast server
 */
void AP_Networking_CAN::mcast_server(void)
{
#ifndef HAL_BOOTLOADER_BUILD
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
#endif
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN_MCAST: starting");

    ObjectBuffer<AP_HAL::CANFrame> *frame_buffers[HAL_NUM_CAN_IFACES] {};

    for (uint8_t bus=0; bus<HAL_NUM_CAN_IFACES; bus++) {
        auto *cbus = get_caniface(bus);
        if (cbus == nullptr) {
            continue;
        }
        if (bus_mask & (1U<<bus)) {
            mcast_sockets[bus] = NEW_NOTHROW SocketAPM(true);
            if (mcast_sockets[bus] == nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "CAN_MCAST[%u]: failed to create socket", unsigned(bus));
                continue;
            }
        }
        char address[] = MCAST_ADDRESS_BASE;
        const uint32_t buffer_size = 20; // good for fw upload
        uint8_t callback_id = 0;

        address[strlen(address)-1] = '0' + bus;
        if (!mcast_sockets[bus]->connect(address, MCAST_PORT)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "CAN_MCAST[%u]: failed to connect", unsigned(bus));
            goto de_allocate;
        }

        if (!cbus->register_frame_callback(
                FUNCTOR_BIND_MEMBER(&AP_Networking_CAN::can_frame_callback, void, uint8_t, const AP_HAL::CANFrame &, AP_HAL::CANIface::CanIOFlags),
                callback_id)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "CAN_MCAST[%u]: failed to register", unsigned(bus));
            goto de_allocate;
        }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        // tell the ethernet interface that we want to receive all
        // multicast packets
        ETH->MACPFR |= ETH_MACPFR_PM;
#endif

        frame_buffers[bus] = NEW_NOTHROW ObjectBuffer<AP_HAL::CANFrame>(buffer_size);
        if (frame_buffers[bus] == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "CAN_MCAST[%u]: failed to allocate buffers", unsigned(bus));
            goto de_allocate;
        }
        continue;

    de_allocate:
        delete mcast_sockets[bus];
        mcast_sockets[bus] = nullptr;
    }


    // main loop
    while (true) {
        const uint32_t delay_us = 100; // limit to 10k packets/s
#ifndef HAL_BOOTLOADER_BUILD
        hal.scheduler->delay_microseconds(delay_us);
#else
        thread_sleep_us(delay_us);
#endif
        for (uint8_t bus=0; bus<HAL_NUM_CAN_IFACES; bus++) {
            if (mcast_sockets[bus] == nullptr) {
                continue;
            }

            struct mcast_pkt pkt;
            const ssize_t ret = mcast_sockets[bus]->recv((void*)&pkt, sizeof(pkt), 0);
            if (ret > MCAST_HDR_LENGTH && ret <= sizeof(pkt)) {
                const uint8_t data_len = ret - MCAST_HDR_LENGTH;
                bool is_canfd = false;
#if HAL_CANFD_SUPPORTED
                is_canfd = (pkt.flags & MCAST_FLAG_CANFD) != 0;
#endif
                if (pkt.magic != MCAST_MAGIC) {
                    continue;
                }
                const auto crc = crc16_ccitt((uint8_t*)&pkt.flags, ret - offsetof(mcast_pkt,flags), 0xFFFFU);
                if (pkt.crc != crc) {
                    continue;
                }

                // push into queue
                frame_buffers[bus]->push(AP_HAL::CANFrame(pkt.message_id, pkt.data, data_len, is_canfd));
            }

            /*
              send pending frames
            */
            AP_HAL::CANFrame frame;
            const uint16_t timeout_us = 2000;
#if AP_NETWORKING_CAN_MCAST_BRIDGING_ENABLED
            const bool bridged = AP::network().is_can_mcast_ep_bridged(bus);
#endif

            while (frame_buffers[bus]->peek(frame)) {
                auto *cbus = get_caniface(bus);
                if (cbus == nullptr) {
                    break;
                }
#if AP_NETWORKING_CAN_MCAST_BRIDGING_ENABLED
                if (bridged) {
                    auto retcode = cbus->send(frame, AP_HAL::micros64() + timeout_us,
                                                 AP_HAL::CANIface::IsForwardedFrame);
                    if (retcode == 0) {
                        break;
                    }
                } else
#endif
                {
                    // only call the AP_HAL::CANIface send if we are not in bridged mode
                    cbus->AP_HAL::CANIface::send(frame, AP_HAL::micros64() + timeout_us,
                                                 AP_HAL::CANIface::IsForwardedFrame);
                }

                // we either sent it or there was an error, either way we discard the frame
                frame_buffers[bus]->pop();
            }
        }
    }
}

/*
  handler for CAN frames from the registered callback, sending frames
  out as multicast UDP
 */
void AP_Networking_CAN::can_frame_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags)
{
    if (bus >= HAL_NUM_CAN_IFACES || mcast_sockets[bus] == nullptr) {
        return;
    }

#if AP_NETWORKING_CAN_MCAST_BRIDGING_ENABLED
    // check if bridged mode is enabled
    const bool bridged = AP::network().is_can_mcast_ep_bridged(bus);
#else
    // never bridge in bootloader, as we can cause loops if multiple
    // bootloaders with mcast are running on the same network and CAN Bus
    const bool bridged = false;
#endif

    if ((flags & AP_HAL::CANIface::IsForwardedFrame) && !bridged) {
        // we don't forward frames that we received from the multicast
        // server if not in bridged mode
        return;
    }

    struct mcast_pkt pkt {};
    pkt.magic = MCAST_MAGIC;
    pkt.flags = 0;
#if HAL_CANFD_SUPPORTED
    if (frame.isCanFDFrame()) {
        pkt.flags |= MCAST_FLAG_CANFD;
    }
#endif
    pkt.message_id = frame.id;

    const uint8_t data_length = AP_HAL::CANFrame::dlcToDataLength(frame.dlc);

    memcpy(pkt.data, frame.data, data_length);
    // 6 is the size of the flags and message_id, ie header data after crc
    pkt.crc = crc16_ccitt((uint8_t*)&pkt.flags, data_length+6, 0xFFFFU);

    mcast_sockets[bus]->send((void*)&pkt, data_length+MCAST_HDR_LENGTH);
}

#endif // AP_NETWORKING_ENABLED
