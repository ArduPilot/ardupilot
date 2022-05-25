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
    for (uint8_t i = 0; i < sitl->state.num_motors; i++) {
        // some fake values so that is_telemetry_active() returns true
        TelemetryData t {
            .temperature_cdeg = 32,
            .voltage = 16.8f,
            .current = 0.8f,
            .consumption_mah = 1.0f,
        };

        update_telem_data(i, t,
            AP_ESC_Telem_Backend::TelemetryType::CURRENT
                | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
                | AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION
                | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
    }

    if (is_zero(sitl->throttle)) {
        if (!is_zero(sitl->esc_rpm_armed) && hal.util->get_soft_armed()) {
            for (uint8_t i = 0; i < sitl->state.num_motors; i++) {
                update_rpm(i, sitl->esc_rpm_armed);
            }
        }
        return;
    }

    for (uint8_t i = 0; i < sitl->state.num_motors; i++) {
        update_rpm(i, MAX(sitl->state.rpm[sitl->state.vtol_motor_start+i], sitl->esc_rpm_armed));
    }
#endif

}

#endif