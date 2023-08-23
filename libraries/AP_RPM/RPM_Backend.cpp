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

#include "RPM_Backend.h"

#if AP_RPM_ENABLED

#include "AP_RPM.h"

#if HAL_WITH_ESC_TELEM
#include "AP_ESC_Telem/AP_ESC_Telem.h"
#endif

/*
  base class constructor. 
*/
AP_RPM_Backend::AP_RPM_Backend(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state) :
        ap_rpm(_ap_rpm),
        state(_state) 
{
    state.instance = instance;
}

#if AP_RPM_ESC_TELEM_OUTBOUND_ENABLED
void AP_RPM_Backend::update_esc_telem_outbound()
{
    const uint8_t esc_index = ap_rpm._params[state.instance].esc_telem_outbound_index;
    if (esc_index == 0) {
        // Disabled if there's no ESC identified to route the data to
        return;
    }
    if (!ap_rpm.healthy(state.instance)) {
        // If we're unhealthy don't update the telemetry. Let it
        // timeout on it's own instead of showing potentially wrong data
        return;
    }

    AP::esc_telem().update_rpm(esc_index-1, state.rate_rpm, 0);
}
#endif

#endif  // AP_RPM_ENABLED
