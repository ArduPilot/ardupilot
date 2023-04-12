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

    // check if PEC supported with the version value in SpecificationInfo() function
    // returns true once PEC is confirmed as working or not working
    bool check_pec_support();

    uint8_t _pec_confirmed; // count of the number of times PEC has been confirmed as working
    uint32_t _last_cell_update_us[BATTMONITOR_SMBUS_NUM_CELLS_MAX]; // system time of last successful read of cell voltage
    uint32_t _cell_count_check_start_us;  // system time we started attempting to count the number of cells
    uint8_t _cell_count;    // number of cells returning voltages
    bool _cell_count_fixed; // true when cell count check is complete
};

#endif  // AP_BATTERY_SMBUS_GENERIC_ENABLED
