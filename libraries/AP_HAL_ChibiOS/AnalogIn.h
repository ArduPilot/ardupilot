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

// available ADC channels for allocation
#define ANALOG_MAX_CHANNELS 16

// physical ADC channels per ADC HAL driver
// This is MCU dependent, currently STM32H7 has the highest number of ADC_INs i.e. 20
#define HAL_MAX_ANALOG_IN_CHANNELS 20

#ifndef HAL_NUM_ANALOG_INPUTS
#if defined(HAL_ANALOG3_PINS) || HAL_WITH_MCU_MONITORING
#define HAL_NUM_ANALOG_INPUTS 3
#elif defined(HAL_ANALOG2_PINS)
#define HAL_NUM_ANALOG_INPUTS 2
#else
#define HAL_NUM_ANALOG_INPUTS 1
#endif
#endif

// number of samples on each channel to gather on each DMA callback
#define ADC_DMA_BUF_DEPTH 8

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)

class ChibiOS::AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class ChibiOS::AnalogIn;
    AnalogSource(int16_t pin);
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override WARN_IF_UNUSED;
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
    void timer_tick_adc(uint8_t index);
    float board_voltage(void) override { return _board_voltage; }
    float servorail_voltage(void) override { return _servorail_voltage; }
    uint16_t power_status_flags(void) override { return _power_flags; }
    uint16_t accumulated_power_status_flags(void) const override { return _accumulated_power_flags; }

#if HAL_WITH_MCU_MONITORING
    float mcu_temperature(void) override { return _mcu_temperature; }
    float mcu_voltage(void) override { return _mcu_voltage; }
    float mcu_voltage_max(void) override { return _mcu_voltage_max; }
    float mcu_voltage_min(void) override { return _mcu_voltage_min; }
#endif

private:
    void read_adc(uint8_t index, uint32_t *val);
    void update_power_flags(void);
    void setup_adc(uint8_t index);
    static void adccallback(ADCDriver *adcp);

    ChibiOS::AnalogSource* _channels[ANALOG_MAX_CHANNELS];

    uint32_t _last_run;
    float _board_voltage;
    float _servorail_voltage;
    float _rssi_voltage;
    uint16_t _power_flags;
    uint16_t _accumulated_power_flags;  // bitmask of all _power_flags ever set

    ADCConversionGroup adcgrpcfg[HAL_NUM_ANALOG_INPUTS];

    static uint8_t get_num_grp_channels(uint8_t index);
    static uint8_t get_pin_channel(uint8_t adc_index, uint8_t pin_index);
    static uint8_t get_analog_pin(uint8_t adc_index, uint8_t pin_index);
    static float get_pin_scaling(uint8_t adc_index, uint8_t pin_index);
    static uint8_t get_adc_index(ADCDriver* adcp);

    struct pin_info {
        uint8_t channel;
        uint8_t analog_pin;
        float scaling;
    };
    static const pin_info pin_config[];
#ifdef HAL_ANALOG2_PINS
    static const pin_info pin_config_2[];
#endif
#if defined(HAL_ANALOG3_PINS) || HAL_WITH_MCU_MONITORING
    static const pin_info pin_config_3[];
#endif

    static adcsample_t *samples[HAL_NUM_ANALOG_INPUTS];
    static uint32_t *sample_sum[HAL_NUM_ANALOG_INPUTS];
    static uint32_t sample_count[HAL_NUM_ANALOG_INPUTS];

    HAL_Semaphore _semaphore;

#if HAL_WITH_MCU_MONITORING
    uint16_t _mcu_monitor_sample_count;
    uint32_t _mcu_monitor_temperature_accum;
    uint32_t _mcu_monitor_voltage_accum;
    uint16_t _mcu_vrefint_min;
    uint16_t _mcu_vrefint_max;

    float _mcu_temperature;
    float _mcu_voltage;
    float _mcu_voltage_min;
    float _mcu_voltage_max;
#endif
};

#endif // HAL_USE_ADC
