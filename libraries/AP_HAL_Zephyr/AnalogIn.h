/*
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
 * Code by @davidbuzz and Claude
 */
#pragma once

#include <AP_HAL/AnalogIn.h>

#ifdef __ZEPHYR__
#include <zephyr/device.h>
#endif

namespace Zephyr {

class AnalogIn;

class AnalogSource : public AP_HAL::AnalogSource {
public:
    AnalogSource(AnalogIn &parent, uint8_t pin);

    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;

private:
    AnalogIn &_parent;
    uint8_t _pin;
};

class AnalogIn : public AP_HAL::AnalogIn {
public:
    /* Reports a fixed 25 degC MCU temperature: this board deliberately returns a
     * constant rather than a reading, because no calibrated sensor is wired. */
#if HAL_WITH_MCU_MONITORING
    float mcu_temperature(void) override { return 25.0f; }
#endif

    AnalogIn();

    void init() override;
    AP_HAL::AnalogSource *channel(int16_t n) override;
    bool valid_analog_pin(uint16_t pin) const override;
    float board_voltage(void) override;

    float read_pin_raw(uint8_t pin);
    float read_pin_voltage(uint8_t pin);

    /* Over-current reporting for the two switched 5V rails. The base class
     * returns 0 for both; overriding surfaces the board's nOC lines as
     * MAV_POWER_STATUS flags, the same information ChibiOS reports from
     * AnalogIn::update_power_flags(). */
    uint16_t power_status_flags(void) override;
    uint16_t accumulated_power_status_flags(void) const override;

private:
    static constexpr uint8_t NUM_ANALOG_PINS = 4;
    static constexpr float ADC_REF_VOLTAGE = 3.3f;
    static constexpr uint16_t ADC_MAX_COUNTS = 4095;  // 12-bit, see seq.resolution

#ifdef __ZEPHYR__
    const struct device *_adc1 = nullptr;
    const struct device *_adc2 = nullptr;
    bool _adc_ready = false;

    /* nOC over-current inputs, active LOW. Schematic-confirmed 2026-07-30. */
    static constexpr uint8_t OC_PERIPH_PIN  = 15;
    static constexpr uint8_t OC_HIPOWER_PIN = 12;
    const struct device *_oc_gpio = nullptr;
    bool _oc_ready = false;
    mutable uint16_t _power_flags = 0;
    mutable uint16_t _accumulated_power_flags = 0;
    void _oc_init(void);
#endif

    AnalogSource *_sources[NUM_ANALOG_PINS] = {};
};

}  // namespace Zephyr
