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
#include "Display_OLED_I2C.h"
#include "Display_SH1106_I2C.h"
#include "Display_SSD1306_I2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_Notify.h"

Display_OLED_I2C* Display_OLED_I2C::getInstance()
{
    if (nullptr == _instance.get()) {
        switch (pNotify->_display_type) {
            case DISPLAY_SSD1306:
                _instance = new Display_SSD1306_I2C();
                break;

            case DISPLAY_SH1106:
                _instance = new Display_SH1106_I2C();
                break;

            case DISPLAY_OFF:
            default:
                break;
        }
    }
    return _instance.get();
}