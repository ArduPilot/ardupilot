/*
   IS31FL3195 I2C driver

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

   Datasheet: https://www.lumissil.com/assets/pdf/core/IS31FL3195_DS.pdf

 */
#pragma once

#include "AP_Notify_config.h"

#if AP_NOTIFY_IS31FL3195_ENABLED

#include <AP_HAL/I2CDevice.h>
#include "RGBLed.h"

class IS31FL3195 : public RGBLed
{
public:
    IS31FL3195(uint8_t bus, uint8_t addr);
    ~IS31FL3195() { delete _dev; }

    bool init(void) override;
protected:
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    AP_HAL::I2CDevice *_dev;
    uint8_t _bus;
    uint8_t _addr;

    void _timer(void);
    bool _need_update;
    uint8_t rgb[3];
};

#endif  // AP_NOTIFY_IS31FL3195_ENABLED
