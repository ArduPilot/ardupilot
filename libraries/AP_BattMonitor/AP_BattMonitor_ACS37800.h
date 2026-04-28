/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ACS37800_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_ACS37800 : public AP_BattMonitor_Backend
{
public:
    AP_BattMonitor_ACS37800(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    bool has_current() const override { return true; }

    void init(void) override;
    void read() override;
    void update();

    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_HAL::I2CDevice *_dev;

    AP_Int8 i2c_bus;
    AP_Int8 i2c_address;
    AP_Float volt_fine_mult;
    AP_Float curr_fine_mult;

    struct {
        uint16_t count;
        float volt_sum;
        float current_sum;
        HAL_Semaphore sem;
    } accumulate;

    float convert_voltage(int16_t vcode);
    float convert_current(int16_t icode);

    bool write_word(const uint8_t reg, const uint32_t data) const;
    bool read_word(const uint8_t reg, uint32_t& data) const;
};

#endif // AP_BATTERY_ACS37800_ENABLED
