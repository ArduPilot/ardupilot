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
#include "NotifyDevice.h"
#include "OreoLED_I2C.h"
#include <AP_Common/AP_Common.h>

class NeoPixel_Oreo: public NotifyDevice {
public:
    NeoPixel_Oreo(uint8_t theme);
    
    // init - initialised the device
    bool init(void) override;

    // update - updates device according to timed_updated.  Should be
    // called at 50Hz
    void update() override;

private:

    uint16_t enable_mask;

    OreoLED_I2C *oreo;

    // perdiodic tick to re-init
    uint32_t    _last_init_ms;

    // periodic callback
    void timer();

    HAL_Semaphore_Recursive _sem;
};

