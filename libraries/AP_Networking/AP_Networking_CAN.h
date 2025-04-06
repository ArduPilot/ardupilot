#pragma once

#include "AP_Networking.h"
#include <AP_HAL/utility/Socket.h>
#include <AP_HAL/CANIface.h>

class AP_Networking_CAN {
public:
    void start(const uint8_t bus_mask);

    FUNCTOR_TYPEDEF(FrameCb, void, uint8_t, const AP_HAL::CANFrame &, AP_HAL::CANIface::CanIOFlags);

    // register a frame callback function
    bool register_frame_callback(uint8_t bus, FrameCb cb);
    void unregister_frame_callback(uint8_t bus);

    void can_frame_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags = 0);
    
private:

    HAL_Semaphore cb_sem;
    // allow up to 3 callbacks, one for each interface
    FrameCb cb[3];

    void mcast_server(void);
    SocketAPM *mcast_sockets[HAL_NUM_CAN_IFACES];

    uint8_t bus_mask;

    AP_HAL::CANIface *get_caniface(uint8_t) const;

#ifdef HAL_BOOTLOADER_BUILD
    static void mcast_trampoline(void *ctx);
#endif
};

