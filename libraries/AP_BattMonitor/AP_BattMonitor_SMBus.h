/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_BATTMONITOR_SMBUS_H
#define AP_BATTMONITOR_SMBUS_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include "AP_BattMonitor_Backend.h"

#define AP_BATTMONITOR_SMBUS_MAX_CELLS 4

class AP_BattMonitor_SMBus : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_SMBus(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state) :
        AP_BattMonitor_Backend(mon, instance, mon_state)
    {}

    // virtual destructor to reduce compiler warnings
    virtual ~AP_BattMonitor_SMBus() {}

};

// include specific implementations
#include "AP_BattMonitor_SMBus_PX4.h"
#include "AP_BattMonitor_SMBus_I2C.h"

#endif // AP_BATTMONITOR_SMBUS_H
