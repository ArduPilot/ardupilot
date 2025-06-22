#pragma once

#include "AP_BattMonitor_SMBus.h"

#if AP_BATTERY_SMBUS_GENERIC_ENABLED

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define BATTMONITOR_SMBUS_NUM_CELLS_MAX 14
#else
#define BATTMONITOR_SMBUS_NUM_CELLS_MAX 12
#endif

class AP_BattMonitor_SMBus_Generic : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_Generic(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

private:

    void timer(void) override;

    // Override capacity scaler, use the current multiplier from SpecificationInfo()
    uint16_t get_capacity_scaler() const override { return _i_multiplier; }

    // Read SpecificationInfo() function, check if PEC is supported and update voltage and current multipliers
    // returns true once SpecificationInfo() is returned from the Smart Battery
    bool read_specification_info();

    bool _specification_info_confirmed = false; // specification info successfully read
    uint16_t _v_multiplier = 1;    // voltage multiplier = 10^VScale
    uint16_t _i_multiplier = 1;   // current multiplier = 10^IPScale
    uint32_t _last_cell_update_us[BATTMONITOR_SMBUS_NUM_CELLS_MAX]; // system time of last successful read of cell voltage
    uint32_t _cell_count_check_start_us;  // system time we started attempting to count the number of cells
    uint8_t _cell_count;    // number of cells returning voltages
    bool _cell_count_fixed; // true when cell count check is complete
};

#endif  // AP_BATTERY_SMBUS_GENERIC_ENABLED
