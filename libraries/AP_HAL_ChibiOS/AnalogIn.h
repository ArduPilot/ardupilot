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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#pragma once

#include "AP_HAL_ChibiOS.h"

#define ANALOG_MAX_CHANNELS 16

// number of samples on each channel to gather on each DMA callback
#define ADC_DMA_BUF_DEPTH 8

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)

class ChibiOS::AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class ChibiOS::AnalogIn;
    AnalogSource(int16_t pin, float initial_value);
    float read_average() override;
    float read_latest() override;
    void set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;

private:
    // what value it has
    int16_t _pin;
    float _value;
    float _value_ratiometric;
    float _latest_value;
    uint8_t _sum_count;
    float _sum_value;
    float _sum_ratiometric;
    void _add_value(float v, float vcc5V);
    float _pin_scaler();
    HAL_Semaphore _semaphore;
};

class ChibiOS::AnalogIn : public AP_HAL::AnalogIn {
public:
    friend class AnalogSource;

    void init() override;
    AP_HAL::AnalogSource* channel(int16_t pin) override;
    void _timer_tick(void);
    float board_voltage(void) override { return _board_voltage; }
    float servorail_voltage(void) override { return _servorail_voltage; }
    uint16_t power_status_flags(void) override { return _power_flags; }
    static void adccallback(ADCDriver *adcp);

private:
    void read_adc(uint32_t *val);
    void update_power_flags(void);

    ChibiOS::AnalogSource* _channels[ANALOG_MAX_CHANNELS];

    uint32_t _last_run;
    float _board_voltage;
    float _servorail_voltage;
    float _rssi_voltage;
    uint16_t _power_flags;
    ADCConversionGroup adcgrpcfg;

    struct pin_info {
        uint8_t channel;
        float scaling;
    };
    static const pin_info pin_config[];

    static adcsample_t *samples;
    static uint32_t sample_sum[];
    static uint32_t sample_count;
};

#endif // HAL_USE_ADC
