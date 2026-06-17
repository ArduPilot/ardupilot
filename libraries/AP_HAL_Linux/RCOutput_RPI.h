/*
 * Code by Andy Piper <github@andypiper.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * RCOutput driver for Raspberry Pi boards.
 * Provides serial LED support via SPI, PWM outputs are not supported.
 */
#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/RCOutput.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#if HAL_LINUX_SERIALLED_ENABLED
#include "SerialLED_SPI.h"
#endif

namespace Linux {

/*
 * RCOutput implementation for Raspberry Pi boards.
 *
 * PWM output functions are no-ops since this board typically doesn't
 * have dedicated PWM outputs for motor control. Serial LED support
 * is provided via SPI when HAL_LINUX_SERIALLED_ENABLED is set.
 */
class RCOutput_RPI : public Empty::RCOutput {
public:
    // Initialize the driver
    void init() override;

    // Serial LED functions
    bool set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode = MODE_PWM_NONE, uint32_t clock_mask = 0) override;
    bool set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue) override;
    bool serial_led_send(const uint16_t chan) override;

private:
#if HAL_LINUX_SERIALLED_ENABLED
    SerialLED_SPI _serial_led;
#endif
};

}  // namespace Linux
