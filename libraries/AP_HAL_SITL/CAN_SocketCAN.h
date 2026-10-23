/*
  socketcan transport for SITL CAN
 */
#pragma once

#include "CAN_Transport.h"

#if HAL_NUM_CAN_IFACES && HAL_CAN_WITH_SOCKETCAN

class CAN_SocketCAN : public CAN_Transport {
public:
    bool init(uint8_t instance) override;
    bool send(const AP_HAL::CANFrame &frame) override;
    bool receive(AP_HAL::CANFrame &frame) override;
    int get_read_fd(void) const override {
        return fd;
    }

private:
    int fd = -1;
};

#endif // HAL_NUM_CAN_IFACES
