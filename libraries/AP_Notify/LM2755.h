/*
   LM2755 I2C driver

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

#include "AP_Notify_config.h"

#if AP_NOTIFY_LM2755_ENABLED

#define LM2755_LED_I2C_ADDR 0x67    // default I2C bus address

#include <AP_HAL/I2CDevice.h>
#include "RGBLed.h"

class LM2755 : public RGBLed
{
public:
    LM2755(uint8_t bus, uint8_t addr);
    bool init(void) override;
protected:
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t _bus;
    uint8_t addr;

    void _timer(void);
    bool _need_update;
    uint8_t rgb[3];
    uint8_t last_sent_rgb[3];
};

#endif  // AP_NOTIFY_LM2755_ENABLED
