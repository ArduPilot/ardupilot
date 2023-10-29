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

#include "AP_ESC_Telem_SITL.h"
#include "AP_ESC_Telem.h"
#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

AP_ESC_Telem_SITL::AP_ESC_Telem_SITL()
{
}

void AP_ESC_Telem_SITL::update()
{
    SITL::SIM* sitl = AP::sitl();

    if (!sitl) {
        return;
    }

#if HAL_WITH_ESC_TELEM

    if (AP_HAL::millis64() < 6000) {
        // this prevents us sending blank data at startup, which triggers
        // ESC telem messages for all channels
        return;
    }
    uint32_t mask = sitl->state.motor_mask;

    /*
      mask out motors we should not be providing telemetry for. On
      AP_Periph SIM_CAN_SRV_MSK are the outputs we will provide
      telemetry for, on the main firmware it is the ones we don't
      provide telemetry for
    */
#if defined(HAL_BUILD_AP_PERIPH)
    mask &= uint32_t(sitl->can_servo_mask);
#else
    mask &= ~uint32_t(sitl->can_servo_mask);
#endif
    uint8_t bit;

    while ((bit = __builtin_ffs(mask)) != 0) {
        uint8_t motor = bit-1;
        mask &= ~(1U<<motor);

        const float min_rpm = hal.util->get_soft_armed()? sitl->esc_rpm_armed : 0;
        update_rpm(motor, MAX(min_rpm, sitl->state.rpm[motor]));

        // some fake values so that is_telemetry_active() returns true
        TelemetryData t {
            .temperature_cdeg = 3200,
            .voltage = 16.8f,
            .current = 0.8f,
            .consumption_mah = 1.0f,
        };

        update_telem_data(motor, t,
            AP_ESC_Telem_Backend::TelemetryType::CURRENT
                | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
                | AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION
                | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
    }
#endif
}

#endif
