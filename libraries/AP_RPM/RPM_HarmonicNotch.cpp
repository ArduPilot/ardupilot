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

#include "AP_RPM_config.h"

#if AP_RPM_HARMONICNOTCH_ENABLED

#include "RPM_HarmonicNotch.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

void AP_RPM_HarmonicNotch::update(void)
{
    const AP_InertialSensor& ins = AP::ins();
    for (const auto &notch : ins.harmonic_notches) {
        if (notch.params.enabled() &&
            notch.params.tracking_mode() != HarmonicNotchDynamicMode::Fixed) {
            state.rate_rpm = notch.calculated_notch_freq_hz[0] * 60;
            state.rate_rpm *= ap_rpm._params[state.instance].scaling;
            state.signal_quality = 0.5f;
            state.last_reading_ms = AP_HAL::millis();
        }
    }
}

#endif  // AP_RPM_HARMONICNOTCH_ENABLED
