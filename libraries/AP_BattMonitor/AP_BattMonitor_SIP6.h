#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_SIP6 : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_SIP6(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    // initialise
    void init(void) override;

    /// Read the battery voltage.  Should be called at 10hz
    void read() override;

    /// return true if battery provides current info
    bool has_current() const override { return false; }

protected:

    int ultra_snd_fd;
};
