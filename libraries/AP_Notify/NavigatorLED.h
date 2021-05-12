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

class NavigatorLED: public RGBLed {
public:
    NavigatorLED();
    bool init(void) override;

protected:
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    uint8_t _data[24];
    void _setup_data(uint8_t red, uint8_t green, uint8_t blue);
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
};
