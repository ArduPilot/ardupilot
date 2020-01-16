#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_SMBus.h"
#include <AP_HAL/I2CDevice.h>

class AP_BattMonitor_SMBus_NeoDesign : public AP_BattMonitor_SMBus
{
public:
    AP_BattMonitor_SMBus_NeoDesign(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params,
                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

private:

    void timer(void) override;
    uint8_t read_block_bare(uint8_t reg, uint8_t* data, uint8_t max_len) const;

    uint8_t _cell_count;
};
