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
  driver for honeywell RSC differential airspeed sensor
  https://https://sensing.honeywell.com/zh-cn/sensors/amplified-board-mount-pressure-sensors/trustability-rsc-series
 */

#include "AP_Airspeed_RSC.h"

// RSC EEPROM Catalog listing
// This part in the EEPROM stores the ascii representation of the sensor chip number
#define RSC_CATALOG_LISTING_MSB                0
#define RSC_CATALOG_LISTING_LSB                15

// RSC EEPROM Serial Number addresses
#define RSC_SERIAL_NO_YYYY_MSB                 16
#define RSC_SERIAL_NO_YYYY_LSB                 19
#define RSC_SERIAL_NO_DDD_MSB                  20
#define RSC_SERIAL_NO_DDD_LSB                  22
#define RSC_SERIAL_NO_XXXX_MSB                 23
#define RSC_SERIAL_NO_XXXX_LSB                 26

// This area in the EEPROM contains the sensor's pressure range in float
#define RSC_PRESSURE_RANGE_LSB                 27
#define RSC_PRESSURE_RANGE_MSB                 30

// This part in the EEPROM contains the lower pressure limit readable
// Value is stored float
#define RSC_PRESSURE_MINIMUM_LSB               31
#define RSC_PRESSURE_MINIMUM_MSB               34

// Unit of measurement of pressure for this particular sensor
// Ex: Pascal, Bar, PSI, inH2O
#define RSC_PRESSURE_UNIT_MSB                  35
#define RSC_PRESSURE_UNIT_LSB                  39

// Ex: Differential, Gauge, absolute
#define RSC_PRESSURE_REFERENCE                 40

// ADC Configuration math
#define RSC_ADC_CONFIG_00                      61
#define RSC_ADC_CONFIG_01                      63
#define RSC_ADC_CONFIG_02                      65
#define RSC_ADC_CONFIG_03                      67

// Offset Coefficient matrix
#define RSC_OFFSET_COEFFICIENT_0_LSB           130
#define RSC_OFFSET_COEFFICIENT_0_MSB           133

#define RSC_OFFSET_COEFFICIENT_1_LSB           134
#define RSC_OFFSET_COEFFICIENT_1_MSB           137

#define RSC_OFFSET_COEFFICIENT_2_LSB           138
#define RSC_OFFSET_COEFFICIENT_2_MSB           141

#define RSC_OFFSET_COEFFICIENT_3_LSB           142
#define RSC_OFFSET_COEFFICIENT_3_MSB           145

// Span Coefficient Matrix
#define RSC_SPAN_COEFFICIENT_0_LSB             210
#define RSC_SPAN_COEFFICIENT_0_MSB             213

#define RSC_SPAN_COEFFICIENT_1_LSB             214
#define RSC_SPAN_COEFFICIENT_1_MSB             217

#define RSC_SPAN_COEFFICIENT_2_LSB             218
#define RSC_SPAN_COEFFICIENT_2_MSB             221

#define RSC_SPAN_COEFFICIENT_3_LSB             222
#define RSC_SPAN_COEFFICIENT_3_MSB             225

// Shape Coefficient Matrix
#define RSC_SHAPE_COEFFICIENT_0_LSB            290
#define RSC_SHAPE_COEFFICIENT_0_MSB            293

#define RSC_SHAPE_COEFFICIENT_1_LSB            294
#define RSC_SHAPE_COEFFICIENT_1_MSB            297

#define RSC_SHAPE_COEFFICIENT_2_LSB            298
#define RSC_SHAPE_COEFFICIENT_2_MSB            301

#define RSC_SHAPE_COEFFICIENT_3_LSB            302
#define RSC_SHAPE_COEFFICIENT_3_MSB            305

// Checksum addresses
#define RSC_CHECKSUM_LSB                       450
#define RSC_CHECKSUM_MSB                       451

// The following section lists ADC Commands/Registers
#define RSC_ADC_RESET_COMMAND                  0x06
#define RSC_DATA_RATE_SHIFT                    5
#define RSC_DATA_RATE_MASK                     0xe0
#define RSC_ADC_REG_MASK                       0x0C
#define RSC_ADC_NUM_BYTES_MASK                 0x03
#define RSC_OPERATING_MODE_MASK                0x18
#define RSC_OPERATING_MODE_SHIFT               3
#define RSC_SET_BITS_MASK                      0x04
#define RSC_ADC_WREG                           0x40

/*
 * The following section contains a list of variations useful for EEPROM
 * calculations and reads.
 */
#define RSC_READ_EEPROM_INSTRUCTION            0x03
#define RSC_EEPROM_ADDRESS_9TH_BIT_MASK        0x100
#define RSC_PRESSURE_RANGE_LEN                 4
#define RSC_PRESSURE_MINIMUM_LEN               4
#define RSC_PRESSURE_UNIT_LEN                  5

// total types of coefficients
#define RSC_COEFF_T_ROW_NO                     3
// total no of coefficients in each type
#define RSC_COEFF_T_COL_NO                     4
// this can be calculated by using the LSB address of the 0th coefficient
// and the MSB of the 3rd coefficient
#define RSC_COEFF_ADDRESS_SPACE_SIZE           16

#define TEMPERATURE_RATIO                      0.03125f

#define RSC_DRDY_PIN 18

extern const AP_HAL::HAL &hal;

AP_Airspeed_RSC::AP_Airspeed_RSC(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

bool AP_Airspeed_RSC::init()
{
    // get eeprom device
    _eeprom_dev = hal.spi->get_device("rsc_eeprom");
    if (!_eeprom_dev) {
        return false;
    }

    // get adc device
    _adc_dev = hal.spi->get_device("rsc_adc");
    if (!_adc_dev) {
        return false;
    }

    // setup data ready pin
    _drdy_pin = hal.gpio->channel(RSC_DRDY_PIN);
    _drdy_pin->mode(HAL_GPIO_INPUT);

    // take semaphore
    if (!_adc_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // get eeprom contents
    _get_eeprom_contents();

    // check airspeed type
    if (strcmp(_pressure_unit, "inH2O") == 0) {
        _unit = INH2O;
    }

    // use initial value to setup adc
    _set_adc_config();

    // set data rate to 175hz
    _set_data_rate(N_DR_175_SPS);

    // convert temperature first
    _adc_convert(TEMPERATURE);

    // release semaphore
    _adc_dev->get_semaphore()->give();

    // timer run at 90hz
    _adc_dev->register_periodic_callback(1000000UL/90U,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_RSC::_timer, void));
    return true;
}

bool AP_Airspeed_RSC::_data_ready()
{
    if (_drdy_pin) {
        return _drdy_pin->read() != 1;
    }

    return false;
}

void AP_Airspeed_RSC::_timer()
{
    uint8_t buf[3];

    if (!_data_ready()) {
        return;
    }

    // read adc value
    _adc_read(buf);

    WITH_SEMAPHORE(sem);

    if (_type == TEMPERATURE) {
        // tempreature is 14 bits in 2s complement representation
        // each byte is shifted to its position in a 16-bit unsigned integer and from 8 more bits to be left-aligned in a 16-bit integer
        _t_raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
        // right-shift signed integer back to get measurement value
        _t_raw >>= 2;
        // apply scaler to get temperature
        _temperature = _t_raw * TEMPERATURE_RATIO;
    } else {
        // pressure is 24 bits in 2s complement representation
        // each byte is shifted to its position in a 32-bit unsigned integer and from 8 more bits to be left-aligned in a 32-bit integer
        _p_raw = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8);
        // right-shift signed integer back to get measurement value
        _p_raw >>= 8;
        // apply scaler to get pressure
        if (_unit == INH2O) {
            _pressure_sum += _calculate_pressure() * INCH_OF_H2O_TO_PASCAL;
        }
        _press_count++;
        _last_sample_time_ms = AP_HAL::millis();
    }

    _state = (_state + 1) % 5;

    if (_state == 0) {
        _adc_convert(TEMPERATURE);
    } else {
        _adc_convert(PRESSURE);
    }
}

void AP_Airspeed_RSC::_get_coefficients()
{
    uint16_t base_address;
    uint8_t l_coeffs[RSC_COEFF_ADDRESS_SPACE_SIZE] = {0};
    uint32_t temp;

    for (uint8_t i = 0; i < RSC_COEFF_T_ROW_NO; i++) {
        base_address = RSC_OFFSET_COEFFICIENT_0_LSB + i * 80;
        _eeprom_read(base_address, RSC_COEFF_ADDRESS_SPACE_SIZE, l_coeffs);

        for (uint8_t j = 0; j < RSC_COEFF_T_COL_NO; j++) {
            temp = l_coeffs[j * 4 + 0] |
                   (l_coeffs[j * 4 + 1] << 8) |
                   (l_coeffs[j * 4 + 2] << 16) |
                   (l_coeffs[j * 4 + 3] << 24);
            _coeff_matrix[i][j] = *(float*)&temp;
        }
    }
}

void AP_Airspeed_RSC::_get_eeprom_contents()
{
    // get pressure range
    _eeprom_read(RSC_PRESSURE_RANGE_LSB, RSC_PRESSURE_RANGE_LEN, (uint8_t *)&_pressure_range);

    // get pressure minimum
    _eeprom_read(RSC_PRESSURE_MINIMUM_LSB, RSC_PRESSURE_MINIMUM_LEN, (uint8_t *)&_pressure_minimum);

    // get pressure unit
    _eeprom_read(RSC_PRESSURE_UNIT_MSB, RSC_PRESSURE_UNIT_LEN, (uint8_t *)&_pressure_unit);

    // get initial ADC values
    _eeprom_read(RSC_ADC_CONFIG_00, 1, &_adc_init_values[0]);
    _eeprom_read(RSC_ADC_CONFIG_01, 1, &_adc_init_values[1]);
    _eeprom_read(RSC_ADC_CONFIG_02, 1, &_adc_init_values[2]);
    _eeprom_read(RSC_ADC_CONFIG_03, 1, &_adc_init_values[3]);

    // get Polynomial/Span/Shape coefficients
    _get_coefficients();
}

void AP_Airspeed_RSC::_set_adc_config()
{
    uint8_t command[5];
    uint8_t reg = 0;
    uint8_t num_bytes = 4;
    // The ADC REG Write command is as follows: 0100 RRNN
    // R - Register Number (0,1,2,3) N - Number of Bytes (0,1,2,3) (0 means 1)
    command[0] = RSC_ADC_WREG|((reg<<2)&RSC_ADC_REG_MASK)|((num_bytes-1)&RSC_ADC_NUM_BYTES_MASK);

    for(uint8_t cnt = 0; cnt < num_bytes; cnt++) {
        command[cnt + 1] = _adc_init_values[cnt];
    }

    // Reset the sensor before configure ADC
    uint8_t reset[1] = {RSC_ADC_RESET_COMMAND};
    _adc_dev->transfer(reset, 1, nullptr, 0);
    hal.scheduler->delay(10);

    // Use initial value to initialize ADC
    _adc_dev->transfer(command, sizeof(command), nullptr, 0);
}

void AP_Airspeed_RSC::_set_data_rate(RSC_DATA_RATE dr)
{
    _data_rate = dr;
    switch (dr) {
        case N_DR_20_SPS:
        case N_DR_45_SPS:
        case N_DR_90_SPS:
        case N_DR_175_SPS:
        case N_DR_330_SPS:
        case N_DR_600_SPS:
        case N_DR_1000_SPS:
            _mode = NORMAL_MODE;
            break;
        case F_DR_40_SPS:
        case F_DR_90_SPS:
        case F_DR_180_SPS:
        case F_DR_350_SPS:
        case F_DR_660_SPS:
        case F_DR_1200_SPS:
        case F_DR_2000_SPS:
            _mode = FAST_MODE;
            break;
        default:
            _mode = NA_MODE;
            break;
    }
}

void AP_Airspeed_RSC::_eeprom_read(uint16_t address, uint8_t num_bytes, uint8_t *data)
{
    uint8_t command[2];
    command[0] = RSC_READ_EEPROM_INSTRUCTION | ((address & RSC_EEPROM_ADDRESS_9TH_BIT_MASK) >> 5);
    command[1] = address & 0xFF;

    _eeprom_dev->transfer(command, 2, data, num_bytes);
}

void AP_Airspeed_RSC::_adc_read(uint8_t *data)
{
    // read ADC value
    _adc_dev->transfer(nullptr, 0, data, 3);
}

void AP_Airspeed_RSC::_adc_convert(READING_TYPE type)
{
    uint8_t command[2] = {0};
    // WREG byte
    command[0] = RSC_ADC_WREG | ((1 << 2) & RSC_ADC_REG_MASK);
    // configuration byte, which includes DataRate, Mode, Pressure/Temperature choice
    command[1] = (((_data_rate << RSC_DATA_RATE_SHIFT) & RSC_DATA_RATE_MASK)
                    | ((_mode << RSC_OPERATING_MODE_SHIFT) & RSC_OPERATING_MODE_MASK)
                    | (((type & 0x01) << 1) | RSC_SET_BITS_MASK));
    // send command
    _adc_dev->transfer(command, 2, nullptr, 0);

    _type = type;
}

float AP_Airspeed_RSC::_calculate_pressure()
{
    int32_t p_raw = _p_raw;

    int16_t t_raw = _t_raw;
    float x = (_coeff_matrix[0][3] * t_raw * t_raw * t_raw);
    float y = (_coeff_matrix[0][2] * t_raw * t_raw);
    float z = (_coeff_matrix[0][1] * t_raw);
    float p_int1 = p_raw - (x + y + z + _coeff_matrix[0][0]);

    x = (_coeff_matrix[1][3] * t_raw * t_raw * t_raw);
    y = (_coeff_matrix[1][2] * t_raw * t_raw);
    z = (_coeff_matrix[1][1] * t_raw);
    float p_int2 = p_int1 / (x + y + z + _coeff_matrix[1][0]);

    x = (_coeff_matrix[2][3] * p_int2 * p_int2 * p_int2);
    y = (_coeff_matrix[2][2] * p_int2 * p_int2);
    z = (_coeff_matrix[2][1] * p_int2);
    float p_comp_fs = x + y + z + _coeff_matrix[2][0];

    float p_comp = (p_comp_fs * _pressure_range) + _pressure_minimum;

    return p_comp;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_RSC::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_press_count > 0) {
        _pressure = _pressure_sum / _press_count;
        _press_count = 0;
        _pressure_sum = 0;
    }

    pressure = _pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_RSC::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    temperature = _temperature;
    return true;
}
