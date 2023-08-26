/*
  multicast UDP transport for SITL CAN
 */
#pragma once

#include "CAN_Transport.h"

#if HAL_NUM_CAN_IFACES

class CAN_Multicast : public CAN_Transport {
public:
    bool init(uint8_t instance) override;
    bool send(const AP_HAL::CANFrame &frame) override;
    bool receive(AP_HAL::CANFrame &frame) override;
    int get_read_fd(void) const override {
        return fd_in;
    }

private:
    int fd_in = -1;
    int fd_out = -1;
};

#endif // HAL_NUM_CAN_IFACES
