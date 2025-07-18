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

#ifndef HAL_HEATER_GPIO_ON
#define HAL_HEATER_GPIO_ON 1
#endif

#if HAL_HAVE_IMU_HEATER2
#ifndef HAL_HEATER2_GPIO_ON
#define HAL_HEATER2_GPIO_ON 1
#endif
#endif

void AP_BoardConfig::set_imu_temp(float current, uint8_t heater_instance)
{
    int8_t target;
    if (heater_instance == 0) {
        target = heater.imu_target_temperature.get();

        // pass to HAL for Linux
        hal.util->set_imu_target_temp((int8_t *)&heater.imu_target_temperature);
        hal.util->set_imu_temp(current);

        heater_update_and_log(heater, current, target);
    }
#if HAL_HAVE_IMU_HEATER2
    else if (heater_instance == 1) {
        target = heater2.imu_target_temperature.get();

        heater_update_and_log(heater2, current, target);
        return;
    }
#endif

#if 0
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Heater: Out=%.1f Temp=%.1f",
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

/**
 * @param instance IMU heater instance
 * @param current IMU temperature in degrees Celsius
 * @param target IMU heater target temperature in degrees Celsius
 */
void AP_BoardConfig::heater_update_and_log(struct imu_heater &instance, float current, int8_t target)
{
    if (target == -1) {
        // nothing to do, make sure heater is left off
        if (&instance == &heater) {
#if defined(HAL_HEATER_GPIO_PIN)
            hal.gpio->write(HAL_HEATER_GPIO_PIN, !HAL_HEATER_GPIO_ON);
#endif
#if defined(HAL_HEATER_GPIO_PIN2)
            hal.gpio->write(HAL_HEATER_GPIO_PIN2, !HAL_HEATER_GPIO_ON);
#endif
        }
#if HAL_HAVE_IMU_HEATER2
        else if (&instance == &heater2) {
#if defined(HAL_HEATER2_GPIO_PIN)
            hal.gpio->write(HAL_HEATER2_GPIO_PIN, !HAL_HEATER2_GPIO_ON);
#endif
        }
#endif // HAL_HAVE_IMU_HEATER2
        return;
    }

    // limit to 65 degrees to prevent damage
    target = constrain_int16(target, -1, 65);
    
    // average over temperatures to remove noise
    instance.count++;
    instance.sum += current;

    // update at 10Hz
    uint32_t now = AP_HAL::millis();
    if (now - instance.last_update_ms < 100) {
        // output as duty cycle to local pin. Use a random sequence to
        // prevent a periodic change to magnetic field
        if (&instance == &heater) {
#if defined(HAL_HEATER_GPIO_PIN)
            bool heater_on = (get_random16() < uint32_t(instance.output) * 0xFFFFU / 100U);
            hal.gpio->write(HAL_HEATER_GPIO_PIN, heater_on?HAL_HEATER_GPIO_ON : !HAL_HEATER_GPIO_ON);
#if defined(HAL_HEATER_GPIO_PIN2)
            hal.gpio->write(HAL_HEATER_GPIO_PIN2, heater_on?HAL_HEATER_GPIO_ON : !HAL_HEATER_GPIO_ON);
#endif
#endif
        }
#if HAL_HAVE_IMU_HEATER2
        else if (&instance == &heater2) {
#if defined(HAL_HEATER2_GPIO_PIN)
            bool heater_on = (get_random16() < uint32_t(instance.output) * 0xFFFFU / 100U);
            hal.gpio->write(HAL_HEATER2_GPIO_PIN, heater_on?HAL_HEATER2_GPIO_ON : !HAL_HEATER2_GPIO_ON);
#endif
        }
#endif // HAL_HAVE_IMU_HEATER2
        return;
    }
    float dt = (now - instance.last_update_ms) * 0.001;
    dt = constrain_float(dt, 0, 0.5);

    instance.last_update_ms = now;

    instance.temperature = instance.sum / instance.count;
    instance.sum = 0;
    instance.count = 0;

    if (target < 0) {
        instance.output = 0;
    } else {
        instance.output = instance.pi_controller.update(instance.temperature, target, dt);
        instance.output = constrain_float(instance.output, 0, 100);
    }

#if HAL_LOGGING_ENABLED
    if (now - instance.last_log_ms >= 1000) {
// @LoggerMessage: HEAT
// @Description: IMU Heater data
// @Field: TimeUS: Time since system startup
// @Field: Id: IMU Heater instance number
// @Field: Temp: Current IMU temperature
// @Field: Targ: Target IMU temperature
// @Field: P: Proportional portion of response
// @Field: I: Integral portion of response
// @Field: Out: Controller output to heating element
        AP::logger().WriteStreaming("HEAT",
                           "TimeUS,Id,Temp,Targ,P,I,Out",
                           "s#OO--%",
                           "F-00---",
                           "QBfbfff",
                           AP_HAL::micros64(), ((&instance == &heater) ? 0 : 1),
                           instance.temperature, target,
                           instance.pi_controller.get_P(),
                           instance.pi_controller.get_I(),
                           instance.output);
        instance.last_log_ms = now;
    }
#endif // HAL_LOGGING_ENABLED
}

// getter for current temperature, return false if heater disabled
bool AP_BoardConfig::get_board_heater_temperature(float &temperature) const
{
    if (heater.imu_target_temperature == -1) {
        return false; // heater disabled
    }
    temperature = heater.temperature;
    return true;
}

// getter for min arming temperature, return false if heater disabled or min check disabled
bool AP_BoardConfig::get_board_heater_arming_temperature(int8_t &temperature) const
{
    if ((heater.imu_target_temperature == -1) || (imu_arming_temperature_margin_low == 0)) {
        return false; // heater or temperature check disabled
    }
    temperature = heater.imu_target_temperature - imu_arming_temperature_margin_low;
    return true;
}

#endif // HAL_HAVE_IMU_HEATER
