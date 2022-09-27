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
#include "AP_Baro_BME680.h"

#if AP_BARO_BME680_ENABLED

#include <utility>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

/* Sensor mode operation */
#define BME680_MODE_SLEEP       0
#define BME680_MODE_FORCED      1
#define BME680_MODE_PARALLEL    2
#define BME680_MODE_SEQUENTIAL  3
#define BME680_MODE BME680_MODE_SEQUENTIAL

/* Sensor Oversampling */
#define BME680_OVERSAMPLING_0   0
#define BME680_OVERSAMPLING_1   1
#define BME680_OVERSAMPLING_2   2
#define BME680_OVERSAMPLING_4   3
#define BME680_OVERSAMPLING_8   4
#define BME680_OVERSAMPLING_16  5
#define BME680_OVERSAMPLING_P BME680_OVERSAMPLING_16
#define BME680_OVERSAMPLING_T BME680_OVERSAMPLING_2

/* Calibration data registers */
#define BME680_REG_CALIB1_ADDR  0x8A
#define BME680_REG_CALIB2_ADDR  0xE9

/* Register sensor ID */
#define BME680_REG_ID           0xD0

/* Sensor Filter */
#define BME680_FILTER_COEFF     2

/* Sensor ID */
#define BME680_ID               0x61

/* Control registers */
#define BME680_REG_RESET        0xE0
#define BME680_REG_CTRL_GAS_0   0x70
#define BME680_REG_CTRL_GAS_1   0x71
#define BME680_REG_CTRL_HEAT    0x72
#define BME680_REG_CTRL_MPAGE   0x73
#define BME680_REG_CTRL_MEAS    0x74

/* Config register */
#define BME680_REG_CONFIG       0x75

/* Register sensor data */
#define BME680_REG_SENSOR_DATA  0x1F

#define BME680_SPI_RD_MSK       0x80

#define BME680_REG_VARIANT_ID   0xF0
#define BME680_RESET_CMD        0xB6

AP_Baro_BME680::AP_Baro_BME680(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
{
}

AP_Baro_Backend *AP_Baro_BME680::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_Baro_BME680 *sensor = new AP_Baro_BME680(baro, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_BME680::_init()
{
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    /* SPI write needs bit mask */
    uint8_t mask = 0xFF;
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        _dev->set_read_flag(BME680_SPI_RD_MSK);
        mask = 0x7F;
    }

    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        /* Setting mem page to 0 */
        if (!_dev->write_register((BME680_REG_CTRL_MPAGE & mask), 0, true) ) {
            return false;
        }
    }

    /* Reseting the sensor */
    if (!_dev->write_register((BME680_REG_RESET & mask), BME680_RESET_CMD, true) ) {
        return false;
    }

    /* Delay after reset */
    hal.scheduler->delay(400);

    uint8_t whoami;
    if (!_dev->read_registers(BME680_REG_ID, &whoami, 1) || 
        (whoami != BME680_ID)) {
        return false;
    }

    /* Reading calibration data */
    _read_calibration();

    /* Setting page to 1 */
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        if (!_dev->write_register((BME680_REG_CTRL_MPAGE & mask), 0x10, true) ) {
            return false;
        }
    }

    _dev->setup_checked_registers(5, 20);

    /* Setting sleep mode - needed to change parameters */
    if (!_dev->write_register((BME680_REG_CTRL_MEAS & mask),(BME680_MODE_SLEEP << 0))) {
        return false;
    }

    /* Setting filter */
    if (!_dev->write_register((BME680_REG_CONFIG & mask), BME680_FILTER_COEFF << 2, true)) {
        return false;
    }

    /* Disabling heater */
    if (!_dev->write_register((BME680_REG_CTRL_HEAT & mask), 0, true)) {
        return false;
    }

    /* Disabling Gas sensor */
    if (!_dev->write_register((BME680_REG_CTRL_GAS_0 & mask), 0, true)) {
        return false;
    }
    if (!_dev->write_register((BME680_REG_CTRL_GAS_1 & mask), 0, true))
    {
        return false;
    }

    /* Reading variant ID - just to check sensor health */
    uint8_t _variantID;
    _dev->read_registers(BME680_REG_VARIANT_ID, &_variantID, sizeof(_variantID));

    /* Setting Oversampling and mode - sensor ready */
    if (!_dev->write_register((BME680_REG_CTRL_MEAS & mask), (BME680_OVERSAMPLING_T << 5) |
                (BME680_OVERSAMPLING_P << 2) | BME680_MODE, true) ) {
        return false;
    }

    _instance = _frontend.register_sensor();

    _dev->set_device_type(DEVTYPE_BARO_BME680);
    set_bus_id(_instance, _dev->get_bus_id());

    // request 50Hz update
    _dev->register_periodic_callback(20 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_BME680::_timer, void));

    return true;
}

void AP_Baro_BME680::_read_calibration(void)
{
    uint8_t buf1[23];
    uint8_t buf2[2];

    _dev->read_registers(BME680_REG_CALIB1_ADDR, buf1,sizeof(buf1));
    _dev->read_registers(BME680_REG_CALIB2_ADDR, buf2,sizeof(buf2));

    /* Temperature calibration data */
    _t1 = ((int16_t)buf2[1] << 8) | buf2[0];
    _t2 = ((int16_t)buf1[1] << 8) | buf1[0];
    _t3 = (int16_t)buf1[2];

    /* Pressure calibration data */
    _p1 = ((int16_t)buf1[5] << 8) | buf1[4];
    _p2 = ((int16_t)buf1[7] << 8) | buf1[6];
    _p3 = (int16_t)buf1[8];
    _p4 = ((int16_t)buf1[11] << 8) | buf1[10];
    _p5 = ((int16_t)buf1[13] << 8) | buf1[12];
    _p6 = (int16_t)buf1[15];
    _p7 = (int16_t)buf1[14];
    _p8 = ((int16_t)buf1[19] << 8) | buf1[18];
    _p9 = ((int16_t)buf1[21] << 8) | buf1[20];
    _p10 = (int16_t)buf1[22];

}

/* acumulate a new sensor reading */
void AP_Baro_BME680::_timer(void)
{
    uint8_t buf[6];

    _dev->read_registers(BME680_REG_SENSOR_DATA , buf, sizeof(buf));

    _update_temperature((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
    _update_pressure((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));

    _dev->check_next_register();
}

// transfer data to the frontend
void AP_Baro_BME680::update(void)
{
    WITH_SEMAPHORE(_sem);

    if (_pressure_count == 0) {
        return;
    }

    _copy_to_frontend(_instance, _pressure_sum/_pressure_count, _temperature);
    _pressure_count = 0;
    _pressure_sum = 0;
}

/* calculate temperature */
void AP_Baro_BME680::_update_temperature(int32_t temp_raw)
{
    int32_t var1, var2, var3, temp_comp;

    var1 = ((int32_t)temp_raw >> 3) - ((int32_t)_t1 << 1);
    var2 = (var1 * (int32_t)_t2) >> 11;
    var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)_t3 << 4)) >> 14;
    _t_fine = var2 + var3;    
    temp_comp = ((_t_fine * 5) + 128) >> 8;

    const float temp = ((float)temp_comp) * 0.01f;

    WITH_SEMAPHORE(_sem);
    _temperature = temp;
}

/* Calculate pressure */
void AP_Baro_BME680::_update_pressure(int32_t press_raw)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t pressure_comp;
    
    /* This value is used to check precedence to multiplication or division
     * in the pressure compensation equation to achieve least loss of precision and
     * avoiding overflows.
     * i.e Comparing value, pres_ovf_check = (1 << 31) >> 1
     */
    const int32_t pres_ovf_check = INT32_C(0x40000000);
    var1 = (((int32_t)_t_fine) >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)_p3 << 5)) >> 3) +
           (((int32_t)_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)_p1) >> 15;
    pressure_comp = 1048576 - press_raw;
    pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));

    if (pressure_comp >= pres_ovf_check) {
        pressure_comp = ((pressure_comp / var1) << 1);
    } else {
        pressure_comp = ((pressure_comp << 1) / var1);
    }
    var1 = ((int32_t)_p9 * (int32_t)(((pressure_comp >> 3) * (pressure_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(pressure_comp >> 2) * (int32_t)_p8) >> 13;
    var3 =
        ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
         (int32_t)_p10) >> 17;
    pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 + ((int32_t)_p7 << 7)) >> 4);

    const float press = (float)pressure_comp;
    if (!pressure_ok(press)) {
        return;
    }

    WITH_SEMAPHORE(_sem);
    _pressure_sum += press;
    _pressure_count++;
}

#endif  // AP_BARO_BME680_ENABLED
