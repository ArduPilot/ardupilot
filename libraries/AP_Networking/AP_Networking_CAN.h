#pragma once

#include "AP_Networking.h"
#include <AP_HAL/utility/Socket.h>

class AP_Networking_CAN {
public:
    void start(const uint8_t bus_mask);

private:
    void mcast_server(void);
    void can_frame_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags);
    SocketAPM *mcast_sockets[HAL_NUM_CAN_IFACES];

    uint8_t bus_mask;

    AP_HAL::CANIface *get_caniface(uint8_t) const;

#ifdef HAL_BOOTLOADER_BUILD
    static void mcast_trampoline(void *ctx);
#endif
};

