#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#include <AP_Generator/AP_Generator_RichenPower.h>

class AP_BattMonitor_Generator : public AP_BattMonitor_Backend
{
public:

    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    void init(void) override {}

};
