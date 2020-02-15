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
#include "AP_Baro_BMP388.h"

#include <utility>

extern const AP_HAL::HAL &hal;

#define BMP388_MODE_SLEEP  0
#define BMP388_MODE_FORCED 1
#define BMP388_MODE_NORMAL 3
#define BMP388_MODE BMP388_MODE_NORMAL

#define BMP388_ID            0x50

#define BMP388_REG_ID        0x00
#define BMP388_REG_ERR       0x02
#define BMP388_REG_STATUS    0x03
#define BMP388_REG_PRESS     0x04 // 24 bit
#define BMP388_REG_TEMP      0x07 // 24 bit
#define BMP388_REG_TIME      0x0C // 24 bit
#define BMP388_REG_EVENT     0x10
#define BMP388_REG_INT_STS   0x11
#define BMP388_REG_FIFO_LEN  0x12 // 9 bit
#define BMP388_REG_FIFO_DATA 0x14
#define BMP388_REG_FIFO_WTMK 0x15 // 9 bit
#define BMP388_REG_FIFO_CNF1 0x17
#define BMP388_REG_FIFO_CNF2 0x18
#define BMP388_REG_INT_CTRL  0x19
#define BMP388_REG_PWR_CTRL  0x1B
#define BMP388_REG_OSR       0x1C
#define BMP388_REG_ODR       0x1D
#define BMP388_REG_CONFIG    0x1F
#define BMP388_REG_CMD       0x7E

#define BMP388_REG_CAL_P     0x36
#define BMP388_REG_CAL_T     0x31

AP_Baro_BMP388::AP_Baro_BMP388(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> _dev)
    : AP_Baro_Backend(baro)
    , dev(std::move(_dev))
{
}

AP_Baro_Backend *AP_Baro_BMP388::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> _dev)
{
    if (!_dev) {
        return nullptr;
    }

    AP_Baro_BMP388 *sensor = new AP_Baro_BMP388(baro, std::move(_dev));
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_BMP388::init()
{
    if (!dev) {
        return false;
    }
    WITH_SEMAPHORE(dev->get_semaphore());

    has_sample = false;

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // setup to allow reads on SPI
    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }

    // normal mode, temp and pressure
    dev->write_register(BMP388_REG_PWR_CTRL, 0x33, true);
    
    uint8_t whoami;
    if (!read_registers(BMP388_REG_ID, &whoami, 1)  ||
        whoami != BMP388_ID) {
        // not a BMP388
        return false;
    }

    // read the calibration data
    read_registers(BMP388_REG_CAL_P, (uint8_t *)&calib_p, sizeof(calib_p));
    read_registers(BMP388_REG_CAL_T, (uint8_t *)&calib_t, sizeof(calib_t));

    scale_calibration_data();

    dev->setup_checked_registers(4);

    // normal mode, temp and pressure
    dev->write_register(BMP388_REG_PWR_CTRL, 0x33, true);

    instance = _frontend.register_sensor();

    // request 50Hz update
    dev->register_periodic_callback(20 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_BMP388::timer, void));

    return true;
}



//  acumulate a new sensor reading
void AP_Baro_BMP388::timer(void)
{
    uint8_t buf[7];

    if (!read_registers(BMP388_REG_STATUS, buf, sizeof(buf))) {
        return;
    }
    const uint8_t status = buf[0];
    if ((status & 0x20) != 0) {
        // we have pressure data
        update_pressure((buf[3] << 16) | (buf[2] << 8) | buf[1]);
    }
    if ((status & 0x40) != 0) {
        // we have temperature data
        update_temperature((buf[6] << 16) | (buf[5] << 8) | buf[4]);
    }

    dev->check_next_register();
}

// transfer data to the frontend
void AP_Baro_BMP388::update(void)
{
    WITH_SEMAPHORE(_sem);

    if (!has_sample) {
        return;
    }

    _copy_to_frontend(instance, pressure, temperature);
    has_sample = false;
}

/*
  convert calibration data from NVM values to values ready for
  compensation calculations
 */
void AP_Baro_BMP388::scale_calibration_data(void)
{
    // note that this assumes little-endian MCU
    calib.par_t1 = calib_t.nvm_par_t1 * 256.0;
    calib.par_t2 = calib_t.nvm_par_t2 / 1073741824.0f;
    calib.par_t3 = calib_t.nvm_par_t3 / 281474976710656.0f;

    calib.par_p1 = (calib_p.nvm_par_p1 - 16384) / 1048576.0f;
    calib.par_p2 = (calib_p.nvm_par_p2 - 16384) / 536870912.0f;
    calib.par_p3 = calib_p.nvm_par_p3 / 4294967296.0f;
    calib.par_p4 = calib_p.nvm_par_p4 / 137438953472.0;
    calib.par_p5 = calib_p.nvm_par_p5 * 8.0f;
    calib.par_p6 = calib_p.nvm_par_p6 / 64.0;
    calib.par_p7 = calib_p.nvm_par_p7 / 256.0f;
    calib.par_p8 = calib_p.nvm_par_p8 / 32768.0f;
    calib.par_p9 = calib_p.nvm_par_p9 / 281474976710656.0f;
    calib.par_p10 = calib_p.nvm_par_p10 / 281474976710656.0f;
    calib.par_p11 = calib_p.nvm_par_p11 / 36893488147419103232.0f;
}

/*
  update temperature from raw sample
 */
void AP_Baro_BMP388::update_temperature(uint32_t data)
{
    float partial1 = data - calib.par_t1;
    float partial2 = partial1 * calib.par_t2;

    WITH_SEMAPHORE(_sem);
    temperature = partial2 + sq(partial1) * calib.par_t3;
}

/*
  update pressure from raw pressure data
 */
void AP_Baro_BMP388::update_pressure(uint32_t data)
{
    float partial1 = calib.par_p6 * temperature;
    float partial2 = calib.par_p7 * powf(temperature, 2);
    float partial3 = calib.par_p8 * powf(temperature, 3);
    float partial_out1 = calib.par_p5 + partial1 + partial2 + partial3;

    partial1 = calib.par_p2 * temperature;
    partial2 = calib.par_p3 * powf(temperature, 2);
    partial3 = calib.par_p4 * powf(temperature, 3);
    float partial_out2 = data * (calib.par_p1 + partial1 + partial2 + partial3);

    partial1 = powf(data, 2);
    partial2 = calib.par_p9 + calib.par_p10 * temperature;
    partial3 = partial1 * partial2;
    float partial4 = partial3 + powf(data, 3) * calib.par_p11;
    float press = partial_out1 + partial_out2 + partial4;

    WITH_SEMAPHORE(_sem);
    
    pressure = press;
    has_sample = true;
}

/*
  read registers, special SPI handling needed
*/
bool AP_Baro_BMP388::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    // when on I2C we just read normally
    if (dev->bus_type() != AP_HAL::Device::BUS_TYPE_SPI) {
        return dev->read_registers(reg, data, len);
    }
    // for SPI we need to discard the first returned byte. See
    // datasheet for explanation
    uint8_t b[len+2];
    b[0] = reg | 0x80;
    memset(&b[1], 0, len+1);
    if (!dev->transfer(b, len+2, b, len+2)) {
        return false;
    }
    memcpy(data, &b[2], len);
    return true;
}
