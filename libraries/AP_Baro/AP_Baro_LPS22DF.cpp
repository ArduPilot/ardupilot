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
#include "AP_Baro_LPS22DF.h"

#if AP_BARO_LPS22DF_ENABLED

#include <stdio.h>

extern const AP_HAL::HAL &hal;

// register map
#define LPS22DF_WHO_AM_I_ADDR      0x0F
#define LPS22DF_WHO_AM_I_VAL       0xB4

#define LPS22DF_IF_CTRL_ADDR       0x0E
#define LPS22DF_I2C_I3C_DIS_VAL    (1 << 6)

#define LPS22DF_CTRL_REG1_ADDR     0x10
#define LPS22DF_ODR_75HZ_VAL       (6 << 3)

#define LPS22DF_CTRL_REG2_ADDR     0x11
#define LPS22DF_BDU_VAL            (1 << 3)
#define LPS22DF_LPFP_CFG_VAL       (1 << 4)
#define LPS22DF_EN_LPFP_VAL        (1 << 5)

#define LPS22DF_CTRL_REG3_ADDR     0x12
#define LPS22DF_IF_ADD_INC_VAL     (1 << 0)

#define LPS22DF_STATUS_ADDR        0x27
#define LPS22DF_P_DA_VAL           (1 << 0)
#define LPS22DF_T_DA_VAL           (1 << 1)

#define LPS22DF_PRESS_OUT_XL_ADDR  0x28

#define LPS22DF_TEMP_OUT_ADDR      0x2B

AP_Baro_LPS22DF::AP_Baro_LPS22DF(AP_Baro &baro, AP_HAL::Device &dev)
    : AP_Baro_Backend(baro)
    , _dev(&dev)
{
}

AP_Baro_Backend *AP_Baro_LPS22DF::probe(AP_Baro &baro, AP_HAL::Device &dev)
{
    AP_Baro_LPS22DF *sensor = NEW_NOTHROW AP_Baro_LPS22DF(baro, dev);
    if (!sensor || !sensor->_init()) {
        delete sensor;

        return nullptr;
    }

    return sensor;
}

bool AP_Baro_LPS22DF::_init()
{
    if (!_dev)
        return false;

    _dev->get_semaphore()->take_blocking();

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // top bit is for read on SPI
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI)
        _dev->set_read_flag(0x80);

    if (!_check_whoami()) {
        _dev->get_semaphore()->give();

        return false;
    }

    // turn off sensor for config
    _dev->write_register(LPS22DF_CTRL_REG1_ADDR, 0x00);

    // disable i2c when spi enabled
    _dev->write_register(LPS22DF_IF_CTRL_ADDR,
                         _dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI ?
                         LPS22DF_I2C_I3C_DIS_VAL : 0);

    _dev->write_register(LPS22DF_CTRL_REG2_ADDR, LPS22DF_BDU_VAL |
                                                 LPS22DF_EN_LPFP_VAL |
                                                 LPS22DF_LPFP_CFG_VAL);

    // enable sensor to 75 Hz
    _dev->write_register(LPS22DF_CTRL_REG1_ADDR, LPS22DF_ODR_75HZ_VAL);

    // request 75Hz update
    CallTime = 1000000 / 75;

    _instance = _frontend.register_sensor();

    _dev->set_device_type(DEVTYPE_BARO_LPS22DF);
    set_bus_id(_instance, _dev->get_bus_id());
    
    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(CallTime, FUNCTOR_BIND_MEMBER(&AP_Baro_LPS22DF::_timer, void));

    return true;
}

// check device ID
bool AP_Baro_LPS22DF::_check_whoami(void)
{
    uint8_t whoami;

    if (!_dev->read_registers(LPS22DF_WHO_AM_I_ADDR, &whoami, 1))
	   return false;

    DEV_PRINTF("LPS22DF whoami 0x%02x\n", whoami);

    if (whoami == LPS22DF_WHO_AM_I_VAL)
        return true;

    return false;
}

//  accumulate a new sensor reading
void AP_Baro_LPS22DF::_timer(void)
{
    uint8_t status;

    // use status to check if data is available
    if (!_dev->read_registers(LPS22DF_STATUS_ADDR, &status, 1))
        return;

    if (status & LPS22DF_T_DA_VAL)
        _update_temperature();

    if (status & LPS22DF_P_DA_VAL)
        _update_pressure();
}

// transfer data to the frontend
void AP_Baro_LPS22DF::update(void)
{
    if (_pressure_count == 0)
        return;

    WITH_SEMAPHORE(_sem);

    // average
    _copy_to_frontend(_instance, _pressure_sum / _pressure_count, _temperature);

#ifdef LPS22DF_LL_DEBUG
    DEV_PRINTF("LPS22DF: P %f T %f\n", _pressure_sum / _pressure_count, _temperature);
#endif // LPS22DF_LL_DEBUG

    _pressure_sum = 0;
    _pressure_count = 0;
}

// calculate temperature
void AP_Baro_LPS22DF::_update_temperature(void)
{
    uint8_t temp[2];

    if (!_dev->read_registers(LPS22DF_TEMP_OUT_ADDR, temp, 2))
        return;

    int16_t temp_reg = (int16_t)((uint16_t(temp[1]) << 8) |
                                 uint16_t(temp[0]));

    WITH_SEMAPHORE(_sem);

    // temperature sensitivity is 100 LSB/Celsius
    _temperature = temp_reg * 0.01;
}

// calculate pressure
void AP_Baro_LPS22DF::_update_pressure(void)
{
    uint8_t press[3];

    if (!_dev->read_registers(LPS22DF_PRESS_OUT_XL_ADDR,
                              press, sizeof(press)))
        return;

    int32_t press_reg = ((uint32_t)press[2] << 16) |
                        ((uint32_t)press[1] << 8) |
                        (uint32_t)press[0];

    // pressure sensitivity is 4096 LSB/hPa -> ardupilot requires in Pa
    int32_t pressure_pa = press_reg * (100.0f / 4096);

    WITH_SEMAPHORE(_sem);
    _pressure_sum += pressure_pa;
    _pressure_count++;
}

#endif  // AP_BARO_LPS22DF_ENABLED
