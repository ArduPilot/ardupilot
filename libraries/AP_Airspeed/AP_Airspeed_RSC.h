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
#pragma once

/*
  backend driver for honeywell RSC differential airspeed sensor
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_RSC : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_RSC(AP_Airspeed &_frontend, uint8_t _instance);
    ~AP_Airspeed_RSC(void) {}

    // probe and initialise the sensor
    bool init() override;

    // return the current differential pressureijn Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degress C
    bool get_temperature(float &temperature) override;

private:
    /*
    * Enum for the datarates supported by the sensor
    * N/F - Normal/Fast - 256 KHz/512 KHz
    * DR - Data Rate
    * SPS - Samples per second
    */
    typedef enum {
        N_DR_20_SPS = 0,
        N_DR_45_SPS,
        N_DR_90_SPS,
        N_DR_175_SPS,
        N_DR_330_SPS,
        N_DR_600_SPS,
        N_DR_1000_SPS,
        N_DR_NA,
        F_DR_40_SPS,
        F_DR_90_SPS,
        F_DR_180_SPS,
        F_DR_350_SPS,
        F_DR_660_SPS,
        F_DR_1200_SPS,
        F_DR_2000_SPS,
        F_DR_NA
    } RSC_DATA_RATE;

    // Enum for modes supported by the RSC sensor
    typedef enum {
        NORMAL_MODE = 0,
        NA_MODE,
        FAST_MODE
    } RSC_MODE;

    // Enum for pressure unit
    typedef enum {
        NONE = 0,
        INH2O
    } RSC_PRESSURE_UNIT;

    // Enum for pressure/temperature reading
    typedef enum {
        PRESSURE = 0,
        TEMPERATURE
    } READING_TYPE;

    bool _data_ready();
    void _timer();

    void _eeprom_read(uint16_t address, uint8_t num_bytes, uint8_t *data);
    void _get_coefficients();
    void _get_eeprom_contents();
    void _set_adc_config();
    void _set_data_rate(RSC_DATA_RATE dr);

    void _adc_convert(READING_TYPE type);
    void _adc_read(uint8_t *data);
    float _calculate_pressure();

    float _pressure_range;
    float _pressure_minimum;
    char _pressure_unit[6];
    uint8_t _adc_init_values[4];

    // calculate compensated pressure
    float _coeff_matrix[3][4];

    RSC_DATA_RATE _data_rate;
    RSC_MODE _mode;
    RSC_PRESSURE_UNIT _unit;

    READING_TYPE _type;

    uint8_t _state;
    int16_t _t_raw;
    int32_t _p_raw;

    float _temperature;
    float _pressure;

    float _pressure_sum;
    uint8_t _press_count;

    uint32_t _last_sample_time_ms;

    AP_HAL::DigitalSource *_drdy_pin;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _eeprom_dev;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _adc_dev;
};

