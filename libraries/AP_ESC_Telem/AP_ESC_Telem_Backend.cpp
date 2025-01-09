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
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

AP_ESC_Telem_Backend::AP_ESC_Telem_Backend() {
    _frontend = AP_ESC_Telem::_singleton;
#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    // we allow for no frontend in example fw and tools to make it
    // possible to run them on hardware with IOMCU
    if (_frontend == nullptr) {
        AP_HAL::panic("No ESC frontend");
    }
#endif
}

// callback to update the rpm in the frontend, should be called by the driver when new data is available
void AP_ESC_Telem_Backend::update_rpm(const uint8_t esc_index, const float new_rpm, const float error_rate) {
    _frontend->update_rpm(esc_index, new_rpm, error_rate);
}

// callback to update the data in the frontend, should be called by the driver when new data is available
void AP_ESC_Telem_Backend::update_telem_data(const uint8_t esc_index, const TelemetryData& new_data, const uint16_t data_present_mask) {
    _frontend->update_telem_data(esc_index, new_data, data_present_mask);
}

/*
  return true if the data is stale
 */
bool AP_ESC_Telem_Backend::TelemetryData::stale() const volatile
{
    return last_update_ms == 0 || !any_data_valid;
}

/*
  return true if the requested types of data are available and not stale
 */
bool AP_ESC_Telem_Backend::TelemetryData::valid(const uint16_t type_mask) const volatile
{
    if ((types & type_mask) == 0) {
        // Requested type not available
        return false;
    }
    return !stale();
}

#endif
