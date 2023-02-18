#pragma once

#include "AP_BattMonitor_SMBus.h"

// Base SUI class
class AP_BattMonitor_SMBus_SUI : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_SUI(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params,
                             uint8_t cell_count
                            );

    void init(void) override;

private:
    void timer(void) override;
    void read_cell_voltages();
    void update_health();

    // read_block_bare - returns number of characters read if successful, zero if unsuccessful
    bool read_block_bare(uint8_t reg, uint8_t* data, uint8_t len) const;

    const uint8_t cell_count;
    bool phase_voltages;
    uint32_t last_volt_read_us;
};
