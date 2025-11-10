#pragma once

#include <AP_Generator/AP_Generator.h>

#if HAL_GENERATOR_ENABLED

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

// Sub class for generator electrical
class AP_BattMonitor_Generator_Elec : public AP_BattMonitor_Backend
{
public:

    // Inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    void init(void) override {};

    // Read the battery voltage and current
    void read(void) override;

    bool has_current(void) const override;

    bool has_consumed_energy(void) const override;

    // Override backend update_failsafes
    AP_BattMonitor::Failsafe update_failsafes() override;
};

// Sub class for generator fuel
class AP_BattMonitor_Generator_FuelLevel : public AP_BattMonitor_Backend
{
public:

    // Inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    void init(void) override;

    // Read the fuel level
    void read(void) override;

    // This is where we tell the battery monitor 'we have current' if we want to report a fuel level remaining
    bool has_current(void) const override;

    // This is where we tell the battery monitor 'we have consumed energy' if we want to report a fuel level remaining
    bool has_consumed_energy(void) const override;

    // reset remaining percentage to given value
    bool reset_remaining(float percentage) override;

    // capacity_remaining_ml - returns true if the capacity remaining in (mL) is valid and writes to capacity_ml
    bool capacity_remaining_ml(float &capacity_ml) const;

    // capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    // Override backend update_failsafes
    AP_BattMonitor::Failsafe update_failsafes() override;
    
    // by default we asume 100% full tank, we can use MAV_CMD_BATTERY_RESET mavlink command to specify otherwise
    // this is only used in case the generator doesn't provide a fuel percentage on its own. This variable
    // is needed to support tank refills or different than 100% initial fuel level
    uint8_t _initial_fuel_pct = 100.0f;
};
#endif
