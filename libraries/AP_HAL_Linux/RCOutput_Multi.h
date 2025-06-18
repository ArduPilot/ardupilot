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

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_LINUX_RCOUTPUT_MULTI_ENABLED
#define HAL_LINUX_RCOUTPUT_MULTI_ENABLED 0
#endif

#if HAL_LINUX_RCOUTPUT_MULTI_ENABLED

#include "AP_HAL/RCOutput.h"
#include <stdarg.h>

namespace Linux
{

class RCOutput_Multi : public AP_HAL::RCOutput
{
public:
    RCOutput_Multi(uint8_t num_outputs, ...);
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;

    void     cork() override;
    void     push() override;

    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;

    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    bool     force_safety_on() override;
    void     force_safety_off() override;
    bool     supports_gpio() override { return false; };

    typedef struct {
        uint8_t num_channels;
        RCOutput *output;
    } RCOutputGroup;

private:
    uint8_t num_outputs;
    RCOutputGroup *outputs;
    bool resolve_channel(uint8_t ch, uint8_t & gid, uint8_t &cid);
};

}

#endif // HAL_LINUX_RCOUTPUT_MULTI_ENABLED

