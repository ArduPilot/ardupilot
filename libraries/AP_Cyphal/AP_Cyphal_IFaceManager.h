#pragma once

#include <AP_HAL/AP_HAL.h>

#include "cyphal/canard.h"


class CyphalTransportIface
{
public:
    CyphalTransportIface() {};
    void attach_can_iface(AP_HAL::CANIface* new_can_iface);
    bool receive(CanardFrame* can_frame);
    bool send(const CypTxQueueItem* transfer);

private:
    AP_HAL::CANIface* _can_iface = nullptr;
};
