/*
   ToshibaLED I2C driver

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
#ifndef __TOSHIBA_LED_I2C_H__
#define __TOSHIBA_LED_I2C_H__

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#endif

class ToshibaLED_I2C : public ToshibaLED
{
public:
    bool hw_init(void);
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b);
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
private:
    AP_HAL::DigitalSource *enable_pin;
#endif
};

#endif // __TOSHIBA_LED_I2C_H__
