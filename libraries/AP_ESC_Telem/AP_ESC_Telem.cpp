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

#include "AP_ESC_Telem.h"
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN
  #include <AP_BoardConfig/AP_BoardConfig_CAN.h>
  #include <AP_Common/AP_Common.h>
  #include <AP_Vehicle/AP_Vehicle.h>
  #include <AP_UAVCAN/AP_UAVCAN.h>
  #include <AP_KDECAN/AP_KDECAN.h>
  #include <AP_ToshibaCAN/AP_ToshibaCAN.h>
#endif

extern const AP_HAL::HAL& hal;

AP_ESC_Telem::AP_ESC_Telem()
{
    if (_singleton) {
        AP_HAL::panic("Too many AP_ESC_Telem instances");
    }
    _singleton = this;
}

// get an individual ESC's usage time in seconds if available, returns true on success
bool AP_ESC_Telem::get_usage_seconds(uint8_t esc_id, uint32_t& usage_sec) const
{
#if HAL_WITH_UAVCAN
    const uint8_t num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < num_drivers; i++) {
        if (AP::can().get_protocol_type(i) == AP_BoardConfig_CAN::Protocol_Type_ToshibaCAN) {
            AP_ToshibaCAN *tcan = AP_ToshibaCAN::get_tcan(i);
            if (tcan != nullptr) {
                usage_sec = tcan->get_usage_seconds(esc_id);
                return true;
            }
            return true;
        }
    }
#endif
    return false;
}

AP_ESC_Telem *AP_ESC_Telem::_singleton = nullptr;

/*
 * Get the AP_InertialSensor singleton
 */
AP_ESC_Telem *AP_ESC_Telem::get_singleton()
{
    return AP_ESC_Telem::_singleton;
}

namespace AP {

AP_ESC_Telem &esc_telem()
{
    return *AP_ESC_Telem::get_singleton();
}

};
