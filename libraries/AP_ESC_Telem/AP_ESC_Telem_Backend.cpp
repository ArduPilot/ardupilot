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

#include "AP_ESC_Telem_Backend.h"
#include "AP_ESC_Telem.h"
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_ESC_TELEM

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

AP_ESC_Telem_Backend::AP_ESC_Telem_Backend() {
    _frontend = AP_ESC_Telem::_singleton;
    if (_frontend == nullptr) {
        AP_HAL::panic("No ESC frontend");
    }
}

// callback to update the rpm in the frontend, should be called by the driver when new data is available
void AP_ESC_Telem_Backend::update_rpm(const uint8_t esc_index, const uint16_t new_rpm, const float error_rate) {
    _frontend->update_rpm(esc_index, new_rpm, error_rate);
}

// callback to update the data in the frontend, should be called by the driver when new data is available
void AP_ESC_Telem_Backend::update_telem_data(const uint8_t esc_index, const TelemetryData& new_data, const uint16_t data_present_mask) {
    _frontend->update_telem_data(esc_index, new_data, data_present_mask);
}

#endif