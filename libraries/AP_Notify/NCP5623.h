/*
  NCP5623 Linux driver
*/
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

#include <AP_HAL/I2CDevice.h>
#include "RGBLed.h"

class NCP5623 : public RGBLed {
public:
    NCP5623(uint8_t bus);
protected:
    bool hw_init(void) override;
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;
private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    void _timer(void);
    bool write(uint8_t reg, uint8_t data);
    bool write_pwm(uint8_t rgb[3]);
    uint8_t rgb[3];
    bool _need_update;
    uint8_t _bus;
};
