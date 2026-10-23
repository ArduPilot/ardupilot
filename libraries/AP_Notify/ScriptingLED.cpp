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

#include "AP_Notify/AP_Notify.h"
#include "ScriptingLED.h"

#if AP_SCRIPTING_ENABLED

ScriptingLED *ScriptingLED::_singleton;

ScriptingLED::ScriptingLED() :
    RGBLed(0, 255, 170, 85)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("ScriptingLED must be singleton");
    }
    _singleton = this;
}

void ScriptingLED::get_rgb(uint8_t& red, uint8_t& green, uint8_t& blue)
{
    red = _red_curr;
    green = _green_curr;
    blue = _blue_curr;
}

#endif
