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

#if ENABLE_SCRIPTING

#include <AP_Scripting/AP_Scripting.h>
#include <AP_HAL/AP_HAL.h>

AP_Scripting::AP_Scripting() {       
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Scripting must be a singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}



AP_Scripting *AP_Scripting::_singleton = nullptr;

namespace AP {

AP_Scripting *scripting() {
    return AP_Scripting::get_singleton();
}

};

#endif // ENABLE_SCRIPTING
