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

#include "RGBLed.h"
#include <AP_Common/AP_Common.h>
#include <AP_IOMCU/AP_IOMCU.h>

#if HAL_WITH_IO_MCU && AP_IOMCU_PROFILED_SUPPORT_ENABLED

class ProfiLED_IOMCU : public RGBLed {
public:
    ProfiLED_IOMCU() : RGBLed(0, 0xFF, 0x7F, 0x33) {}

    bool init(void) override { return true; }

protected:
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override {
        const auto iomcu = AP::iomcu();
        if (iomcu == nullptr) {
            return false;
        }
        iomcu->set_profiled(r, g, b);
        return true;
    }
};

#endif