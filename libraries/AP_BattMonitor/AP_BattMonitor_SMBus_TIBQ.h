#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SMBUS_TIBQ_ENABLED

#include "AP_BattMonitor_SMBus_Generic.h"

class AP_BattMonitor_SMBus_TIBQ : public AP_BattMonitor_SMBus_Generic
{
public:
    // Constructor
    AP_BattMonitor_SMBus_TIBQ(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

private:
    void timer(void) override;

    // returns true if the battery can be shutdown with shutdown()
    bool can_shutdown() override { return true; };
    // shuts the battery down if supported
    bool shutdown() override;

    bool _exit_emshut;
};

#endif // AP_BATTERY_SMBUS_TIBQ_ENABLED
