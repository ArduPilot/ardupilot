#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_SMBus.h"
#include <AP_HAL/I2CDevice.h>

#define BATTMONITOR_SMBUS_MAXELL_NUM_CELLS 6

class AP_BattMonitor_SMBus_Maxell : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_Maxell(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params,
                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

private:

    void timer(void) override;

    // check if PEC supported with the version value in SpecificationInfo() function
    // returns true once PEC is confirmed as working or not working
    bool check_pec_support();

    // read_block - returns number of characters read if successful, zero if unsuccessful
    uint8_t read_block(uint8_t reg, uint8_t* data, bool append_zero) const;

    uint8_t _pec_confirmed; // count of the number of times PEC has been confirmed as working
    uint32_t _last_cell_update_ms[BATTMONITOR_SMBUS_MAXELL_NUM_CELLS];  // system time of last successful read of cell voltage
};
