#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_FRSky_SPort : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_FRSky_SPort(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    //bool has_consumed_energy() const override { return has_current(); }

    /// returns true if battery monitor provides current info
    bool has_current() const override;
    bool has_cell_voltages();

    void init(void);

private:
    uint16_t _id;
};
