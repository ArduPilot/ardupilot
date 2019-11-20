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
/*
  control IMU heater
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_IOMCU/AP_IOMCU.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_BoardConfig.h"

#if HAL_HAVE_IMU_HEATER

extern const AP_HAL::HAL& hal;

void AP_BoardConfig::set_imu_temp(float current)
{
    int8_t target = heater.imu_target_temperature.get();
    // pass to HAL for Linux
    hal.util->set_imu_target_temp((int8_t *)&heater.imu_target_temperature);
    hal.util->set_imu_temp(current);

    if (target == -1) {
        // nothing to do
        return;
    }


    // limit to 65 degrees to prevent damage
    target = constrain_int16(target, -1, 65);
    
    // average over temperatures to remove noise
    heater.count++;
    heater.sum += current;

    // update at 10Hz
    uint32_t now = AP_HAL::millis();
    if (now - heater.last_update_ms < 100) {
#if defined(HAL_HEATER_GPIO_PIN)
        // output as duty cycle to local pin. Use a random sequence to
        // prevent a periodic change to magnetic field
        bool heater_on = (get_random16() < uint32_t(heater.output) * 0xFFFFU / 100U);
        hal.gpio->write(HAL_HEATER_GPIO_PIN, heater_on);
#endif
        return;
    }
    float dt = (now - heater.last_update_ms) * 0.001;
    dt = constrain_float(dt, 0, 0.5);

    heater.last_update_ms = now;

    float avg = heater.sum / heater.count;
    heater.sum = 0;
    heater.count = 0;

    if (target < 0) {
        heater.output = 0;
    } else {
        heater.output = heater.pi_controller.update(avg, target, dt);
        heater.output = constrain_float(heater.output, 0, 100);
    }

    if (now - heater.last_log_ms >= 1000) {
        AP::logger().Write("HEAT", "TimeUS,Temp,Targ,P,I,Out", "Qfbfff",
                           AP_HAL::micros64(),
                           avg, target,
                           heater.pi_controller.get_P(),
                           heater.pi_controller.get_I(),
                           heater.output);
        heater.last_log_ms = now;
    }
#if 0
    gcs().send_text(MAV_SEVERITY_INFO, "Heater: Out=%.1f Temp=%.1f",
                    double(heater.output),
                    double(avg));
#endif

#if HAL_WITH_IO_MCU
    if (io_enabled()) {
        AP_IOMCU *iomcu = AP::iomcu();
        if (iomcu) {
            // tell IOMCU to setup heater
            iomcu->set_heater_duty_cycle(heater.output);
        }
    }
#endif
}

#endif // HAL_HAVE_IMU_HEATER
