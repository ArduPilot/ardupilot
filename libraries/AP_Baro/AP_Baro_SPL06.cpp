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
#include "AP_Baro_SPL06.h"

#include <utility>

extern const AP_HAL::HAL &hal;

#define SPL06_CHIP_ID                          0x10

#define SPL06_REG_PRESSURE_B2                  0x00    // Pressure MSB Register
#define SPL06_REG_PRESSURE_B1                  0x01    // Pressure middle byte Register
#define SPL06_REG_PRESSURE_B0                  0x02    // Pressure LSB Register
#define SPL06_REG_PRESSURE_START               SPL06_REG_PRESSURE_B2
#define SPL06_PRESSURE_LEN                     3       // 24 bits, 3 bytes
#define SPL06_REG_TEMPERATURE_B2               0x03    // Temperature MSB Register
#define SPL06_REG_TEMPERATURE_B1               0x04    // Temperature middle byte Register
#define SPL06_REG_TEMPERATURE_B0               0x05    // Temperature LSB Register
#define SPL06_REG_TEMPERATURE_START            SPL06_REG_TEMPERATURE_B2
#define SPL06_TEMPERATURE_LEN                  3       // 24 bits, 3 bytes
#define SPL06_REG_PRESSURE_CFG                 0x06    // Pressure config
#define SPL06_REG_TEMPERATURE_CFG              0x07    // Temperature config
#define SPL06_REG_MODE_AND_STATUS              0x08    // Mode and status
#define SPL06_REG_INT_AND_FIFO_CFG             0x09    // Interrupt and FIFO config
#define SPL06_REG_INT_STATUS                   0x0A    // Interrupt and FIFO config
#define SPL06_REG_FIFO_STATUS                  0x0B    // Interrupt and FIFO config
#define SPL06_REG_RST                          0x0C    // Softreset Register
#define SPL06_REG_CHIP_ID                      0x0D    // Chip ID Register
#define SPL06_REG_CALIB_COEFFS_START           0x10
#define SPL06_REG_CALIB_COEFFS_END             0x21

#define SPL06_CALIB_COEFFS_LEN                 (SPL06_REG_CALIB_COEFFS_END - SPL06_REG_CALIB_COEFFS_START + 1)

// TEMPERATURE_CFG_REG
#define SPL06_TEMP_USE_EXT_SENSOR              (1<<7)

// MODE_AND_STATUS_REG
#define SPL06_MEAS_PRESSURE                    (1<<0)  // measure pressure
#define SPL06_MEAS_TEMPERATURE                 (1<<1)  // measure temperature

#define SPL06_MEAS_CFG_CONTINUOUS              (1<<2)
#define SPL06_MEAS_CFG_PRESSURE_RDY            (1<<4)
#define SPL06_MEAS_CFG_TEMPERATURE_RDY         (1<<5)
#define SPL06_MEAS_CFG_SENSOR_RDY              (1<<6)
#define SPL06_MEAS_CFG_COEFFS_RDY              (1<<7)

// INT_AND_FIFO_CFG_REG
#define SPL06_PRESSURE_RESULT_BIT_SHIFT        (1<<2)  // necessary for pressure oversampling > 8
#define SPL06_TEMPERATURE_RESULT_BIT_SHIFT     (1<<3)  // necessary for temperature oversampling > 8

// Don't set oversampling higher than 8 or the measurement time will be higher than 20ms (timer period)
#define SPL06_PRESSURE_OVERSAMPLING            8
#define SPL06_TEMPERATURE_OVERSAMPLING         8

#define SPL06_OVERSAMPLING_TO_REG_VALUE(n)     (ffs(n)-1)

AP_Baro_SPL06::AP_Baro_SPL06(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
{
}

AP_Baro_Backend *AP_Baro_SPL06::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }

    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }

    AP_Baro_SPL06 *sensor = new AP_Baro_SPL06(baro, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_SPL06::_init()
{
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _has_sample = false;

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    uint8_t whoami;
    if (!_dev->read_registers(SPL06_REG_CHIP_ID, &whoami, 1)  ||
        whoami != SPL06_CHIP_ID) {
        // not a SPL06
        return false;
    }

    // read the calibration data
    uint8_t buf[SPL06_CALIB_COEFFS_LEN];
    _dev->read_registers(SPL06_REG_CALIB_COEFFS_START, buf, sizeof(buf));

    _c0 = (buf[0] & 0x80 ? 0xF000 : 0) | ((uint16_t)buf[0] << 4) | (((uint16_t)buf[1] & 0xF0) >> 4);
    _c1 = ((buf[1] & 0x8 ? 0xF000 : 0) | ((uint16_t)buf[1] & 0x0F) << 8) | (uint16_t)buf[2];
    _c00 = (buf[3] & 0x80 ? 0xFFF00000 : 0) | ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (((uint32_t)buf[5] & 0xF0) >> 4);
    _c10 = (buf[5] & 0x8 ? 0xFFF00000 : 0) | (((uint32_t)buf[5] & 0x0F) << 16) | ((uint32_t)buf[6] << 8) | (uint32_t)buf[7];
    _c01 = ((uint16_t)buf[8] << 8) | ((uint16_t)buf[9]);
    _c11 = ((uint16_t)buf[10] << 8) | (uint16_t)buf[11];
    _c20 = ((uint16_t)buf[12] << 8) | (uint16_t)buf[13];
    _c21 = ((uint16_t)buf[14] << 8) | (uint16_t)buf[15];
    _c30 = ((uint16_t)buf[16] << 8) | (uint16_t)buf[17];

    // setup temperature and pressure measurements
    _dev->setup_checked_registers(3, 20);

    _dev->write_register(SPL06_REG_TEMPERATURE_CFG, SPL06_TEMP_USE_EXT_SENSOR | SPL06_OVERSAMPLING_TO_REG_VALUE(SPL06_TEMPERATURE_OVERSAMPLING), true);
    _dev->write_register(SPL06_REG_PRESSURE_CFG, SPL06_OVERSAMPLING_TO_REG_VALUE(SPL06_PRESSURE_OVERSAMPLING), true);

    uint8_t int_and_fifo_reg_value = 0;
    if (SPL06_TEMPERATURE_OVERSAMPLING > 8) {
        int_and_fifo_reg_value |= SPL06_TEMPERATURE_RESULT_BIT_SHIFT;
    }
    if (SPL06_PRESSURE_OVERSAMPLING > 8) {
        int_and_fifo_reg_value |= SPL06_PRESSURE_RESULT_BIT_SHIFT;
    }
    _dev->write_register(SPL06_REG_INT_AND_FIFO_CFG, int_and_fifo_reg_value, true);

    _instance = _frontend.register_sensor();

    _dev->set_device_type(DEVTYPE_BARO_SPL06);
    set_bus_id(_instance, _dev->get_bus_id());
    
    // request 50Hz update
    _timer_counter = -1;
    _dev->register_periodic_callback(20 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_SPL06::_timer, void));

    return true;
}

int32_t AP_Baro_SPL06::raw_value_scale_factor(uint8_t oversampling)
{
    // From the datasheet page 13
    switch(oversampling)
    {
        case 1: return 524288;
        case 2: return 1572864;
        case 4: return 3670016;
        case 8: return 7864320;
        case 16: return 253952;
        case 32: return 516096;
        case 64: return 1040384;
        case 128: return 2088960;
        default: return -1; // invalid
    }
}

// acumulate a new sensor reading
void AP_Baro_SPL06::_timer(void)
{
    uint8_t buf[3];

    if ((_timer_counter == -1) || (_timer_counter == 49)) {
        // First call and every second start a temperature measurement (50Hz call)
        _dev->write_register(SPL06_REG_MODE_AND_STATUS, SPL06_MEAS_TEMPERATURE, false);
        _timer_counter = 0; // Next cycle we are reading the temperature
    } else if (_timer_counter == 0) {
        // A temperature measurement had been started during the previous call
        _dev->read_registers(SPL06_REG_TEMPERATURE_START, buf, sizeof(buf));
        _update_temperature((int32_t)((buf[0] & 0x80 ? 0xFF000000 : 0) | ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]));
        _dev->write_register(SPL06_REG_MODE_AND_STATUS, SPL06_MEAS_PRESSURE, false);
        _timer_counter += 1;
    } else {
        // The rest of the time read the latest pressure and start a new measurement
        _dev->read_registers(SPL06_REG_PRESSURE_START, buf, sizeof(buf));
        _update_pressure((int32_t)((buf[0] & 0x80 ? 0xFF000000 : 0) | ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]));
        _dev->write_register(SPL06_REG_MODE_AND_STATUS, SPL06_MEAS_PRESSURE, false);
        _timer_counter += 1;
    }

    _dev->check_next_register();
}

// transfer data to the frontend
void AP_Baro_SPL06::update(void)
{
    WITH_SEMAPHORE(_sem);

    if (!_has_sample) {
        return;
    }

    _copy_to_frontend(_instance, _pressure, _temperature);
    _has_sample = false;
}

// calculate temperature
void AP_Baro_SPL06::_update_temperature(int32_t temp_raw)
{
    _temp_raw = (float)temp_raw / raw_value_scale_factor(SPL06_TEMPERATURE_OVERSAMPLING);
    const float temp_comp = (float)_c0 / 2 + _temp_raw * _c1;

    WITH_SEMAPHORE(_sem);

    _temperature = temp_comp;
}

// calculate pressure
void AP_Baro_SPL06::_update_pressure(int32_t press_raw)
{
    const float press_raw_sc = (float)press_raw / raw_value_scale_factor(SPL06_PRESSURE_OVERSAMPLING);
    const float pressure_cal = (float)_c00 + press_raw_sc * ((float)_c10 + press_raw_sc * ((float)_c20 + press_raw_sc * _c30));
    const float press_temp_comp = _temp_raw * ((float)_c01 + press_raw_sc * ((float)_c11 + press_raw_sc * _c21));

    const float press_comp = pressure_cal + press_temp_comp;

    if (!pressure_ok(press_comp)) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    _pressure = press_comp;
    _has_sample = true;
}
