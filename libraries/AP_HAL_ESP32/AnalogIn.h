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
 * Code by Charles Villard
 */

#pragma once

#include "AP_HAL_ESP32.h"

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)

#define ANALOG_MAX_CHANNELS 8

namespace ESP32 {

class AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class AnalogIn;
    AnalogSource(int16_t pin, float initial_value, uint8_t unit = 1);
    float read_average() override;
    float read_latest() override;
    void set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
	float voltage_average_ratiometric() override;
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms) {}

private:
	//ADC number (1 or 2). ADC2 is unavailable when WIFI on
	uint8_t _unit;

	//Pin number (1-8)
    int16_t _pin;

	//Current computed value (average)
    float _value;
	//Latest fetched raw value from the sensor
    float _latest_value;
	//Number of fetched value since average
    uint8_t _sum_count;
	//Sum of fetched values
    float _sum_value;

    void _add_value();
    float _pin_scaler();

    HAL_Semaphore _semaphore;
};

class AnalogIn : public AP_HAL::AnalogIn {
public:
    friend class AnalogSource;

    void init() override;
    AP_HAL::AnalogSource* channel(int16_t pin) override;
    void _timer_tick();
    float board_voltage() override { return _board_voltage; }

private:
    ESP32::AnalogSource* _channels[ANALOG_MAX_CHANNELS];

    uint32_t _last_run;
    float _board_voltage;

    struct pin_info {
        uint8_t channel;
        float scaling;
    };

    static const pin_info pin_config[];
};

}

#endif // HAL_USE_ADC
