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

namespace ESP32
{

class AnalogSource : public AP_HAL::AnalogSource
{
public:
    friend class AnalogIn;
    AnalogSource(int16_t ardupin,int16_t pin,float scaler, float initial_value, uint8_t unit);
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms) {}

private:
    //ADC number (1 or 2). ADC2 is unavailable when WIFI on
    uint8_t _unit;

    //adc Pin number (1-8)
    // gpio-adc lower level pin name
    int16_t _pin;

    //human readable Pin number used in ardu params
    int16_t _ardupin;
    //scaling from ADC count to Volts
    int16_t _scaler;
    // gpio pin number on esp32:
    gpio_num_t _gpio;

    //Current computed value (average)
    float _value;
    //Latest fetched raw value from the sensor
    float _latest_value;
    //Number of fetched value since average
    uint8_t _sum_count;
    //Sum of fetched values
    float _sum_value;

    void _add_value();

    HAL_Semaphore _semaphore;
};

class AnalogIn : public AP_HAL::AnalogIn
{
public:
    friend class AnalogSource;

    void init() override;
    AP_HAL::AnalogSource* channel(int16_t pin) override;
    void _timer_tick();
    float board_voltage() override
    {
        return _board_voltage;
    }
    static int8_t find_pinconfig(int16_t ardupin);

private:
    ESP32::AnalogSource* _channels[ANALOG_MAX_CHANNELS]; // list of pointers to active individual AnalogSource objects or nullptr

    uint32_t _last_run;
    float _board_voltage;

    struct pin_info {
        uint8_t channel;  // adc1 pin offset
        float scaling;
        uint8_t ardupin; // eg 3 , as typed into an ardupilot parameter
        uint8_t gpio; // eg 32 for D32 esp pin number/s
    };

    static const pin_info pin_config[];
};

}

#endif // HAL_USE_ADC
