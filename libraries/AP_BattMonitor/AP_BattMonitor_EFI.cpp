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

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_EFI_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_EFI/AP_EFI.h>

#include "AP_BattMonitor_EFI.h"

// update state
void AP_BattMonitor_EFI::read()
{
    AP_EFI *efi = AP::EFI();
    if (efi == nullptr) {
        return;
    }

    if (!efi->is_healthy()) {
        _state.healthy = false;
        return;
    }

    struct EFI_State efi_state;
    /*
      a simple mapping of 1 Amp == 1 litre/hour and 1Ah = 1Litre
     */
    efi->get_state(efi_state);

    _state.current_amps = efi_state.fuel_consumption_rate_cm3pm*0.001*60; // litres/hour
    // use arbitrary 1.0 Volts
    _state.voltage = 1.0;
    _state.consumed_mah = efi_state.estimated_consumed_fuel_volume_cm3; // millilitres
    _state.consumed_wh = 0.001 * _state.consumed_mah;
    _state.healthy = true;
    _state.last_time_micros = AP_HAL::micros();
}

#endif // AP_BATTERY_EFI_ENABLED
