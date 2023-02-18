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

#include <AP_HAL/AP_HAL.h>

#include "RPM_Generator.h"

#if AP_RPM_GENERATOR_ENABLED
extern const AP_HAL::HAL& hal;

void AP_RPM_Generator::update(void)
{
    const auto *generator = AP::generator();
    if (generator == nullptr) {
        return;
    }
    if (generator->healthy()) {
        state.rate_rpm = generator->get_rpm();
        state.rate_rpm *= ap_rpm._params[state.instance].scaling;
        state.signal_quality = 0.5;
    } else {
        state.signal_quality = 0;
    }
}

#endif // AP_RPM_GENERATOR_ENABLED
