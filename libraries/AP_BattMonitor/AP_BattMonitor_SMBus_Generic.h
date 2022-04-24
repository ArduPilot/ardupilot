#pragma once

#include "AP_BattMonitor_SMBus.h"

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

#if HAL_SMART_BATTERY_INFO_ENABLED
    // returns true if the number of cells in series can be provided and fills it in
    bool get_cells_in_series(uint8_t &cells_in_series) const override;
#endif

private:

    void timer(void) override;

    // Read initialization messages once: PEC, cycles, serial number, device name, etc.
    // Only returns false if pec check is not confirmed yet
    bool initialize(uint32_t tnow);

    // check if PEC supported with the version value in SpecificationInfo() function
    // returns true once PEC is confirmed as working or not working
    bool check_pec_support();

    bool _pec_confirmed;    // true if PEC has been confirmed as working
    uint32_t _last_cell_update_us[BATTMONITOR_SMBUS_NUM_CELLS_MAX]; // system time of last successful read of cell voltage
    uint32_t _cell_count_check_start_us;  // system time we started attempting to count the number of cells
    uint8_t _cell_count;    // number of cells returning voltages
    bool _cell_count_fixed; // true when cell count check is complete
    uint32_t _info_read_start_us;   // system time we started attempting to read the battery information registers
    bool _info_read;                // true when checking the battery information registers is complete
};
