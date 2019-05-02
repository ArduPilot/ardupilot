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
  DPS280 barometer driver
 */

#include "AP_Baro_DPS280.h"

#include <utility>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

#define DPS280_REG_PRESS  0x00
#define DPS280_REG_TEMP   0x03
#define DPS280_REG_PCONF  0x06
#define DPS280_REG_TCONF  0x07
#define DPS280_REG_MCONF  0x08
#define DPS280_REG_CREG   0x09
#define DPS280_REG_ISTS   0x0A
#define DPS280_REG_FSTS   0x0B
#define DPS280_REG_RESET  0x0C
#define DPS280_REG_PID    0x0D
#define DPS280_REG_COEF   0x10
#define DPS280_REG_CSRC   0x28

#define DPS280_WHOAMI 0x10


AP_Baro_DPS280::AP_Baro_DPS280(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> _dev)
    : AP_Baro_Backend(baro)
    , dev(std::move(_dev))
{
}

AP_Baro_Backend *AP_Baro_DPS280::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> _dev)
{
    if (!_dev) {
        return nullptr;
    }

    AP_Baro_DPS280 *sensor = new AP_Baro_DPS280(baro, std::move(_dev));
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

/*
  handle bit width for 16 bit config registers
 */
void AP_Baro_DPS280::fix_config_bits16(int16_t &v, uint8_t bits) const
{
    if (v > int16_t((1U<<(bits-1))-1)) {
        v = v - (1U<<bits);
    }
}

/*
  handle bit width for 32 bit config registers
 */
void AP_Baro_DPS280::fix_config_bits32(int32_t &v, uint8_t bits) const
{
    if (v > int32_t((1U<<(bits-1))-1)) {
        v = v - (1U<<bits);
    }
}

/*
  read calibration data
 */
bool AP_Baro_DPS280::read_calibration(void)
{
    uint8_t buf[18];

    if (!dev->read_registers(DPS280_REG_COEF, buf, 18)) {
        return false;
    }

    calibration.C0  = (buf[0] << 4) + ((buf[1] >>4) & 0x0F);
    calibration.C1  = (buf[2] + ((buf[1] & 0x0F)<<8));
    calibration.C00 = ((buf[4]<<4) + (buf[3]<<12)) + ((buf[5]>>4) & 0x0F);
    calibration.C10 = ((buf[5] & 0x0F)<<16) + buf[7] + (buf[6]<<8);
    calibration.C01 = (buf[9] + (buf[8]<<8));
    calibration.C11 = (buf[11] + (buf[10]<<8));
    calibration.C20 = (buf[13] + (buf[12]<<8));
    calibration.C21 = (buf[15] + (buf[14]<<8));
    calibration.C30 = (buf[17] + (buf[16]<<8));

    fix_config_bits16(calibration.C0, 12);
    fix_config_bits16(calibration.C1, 12);
    fix_config_bits32(calibration.C00, 20);
    fix_config_bits32(calibration.C10, 20);
    fix_config_bits16(calibration.C01, 16);
    fix_config_bits16(calibration.C11, 16);
    fix_config_bits16(calibration.C20, 16);
    fix_config_bits16(calibration.C21, 16);
    fix_config_bits16(calibration.C30, 16);

    /* get calibration source */
    if (!dev->read_registers(DPS280_REG_CSRC, &calibration.temp_source, 1)) {
        return false;
    }
    calibration.temp_source &= 0x80;

    return true;
}

bool AP_Baro_DPS280::init()
{
    if (!dev || !dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    dev->set_read_flag(0x80);
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    uint8_t whoami=0;
    if (!dev->read_registers(DPS280_REG_PID, &whoami, 1) ||
        whoami != DPS280_WHOAMI) {
        // not a DPS280
        printf("DPS280 whoami=0x%x\n", whoami);
        dev->get_semaphore()->give();
        return false;
    }

    if (!read_calibration()) {
        dev->get_semaphore()->give();
        return false;
    }

    dev->write_register(DPS280_REG_CREG, 0x0C); // shift for 16x oversampling
    dev->write_register(DPS280_REG_PCONF, 0x54); // 32 Hz, 16x oversample
    dev->write_register(DPS280_REG_TCONF, 0x54 | calibration.temp_source); // 32 Hz, 16x oversample
    dev->write_register(DPS280_REG_MCONF, 0x07); // continuous temp and pressure

    instance = _frontend.register_sensor();

    dev->get_semaphore()->give();

    // request 64Hz update. New data will be available at 32Hz
    dev->register_periodic_callback((1000 / 64) * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_DPS280::timer, void));

    return true;
}

/*
  calculate corrected pressure and temperature
 */
void AP_Baro_DPS280::calculate_PT(int32_t UT, int32_t UP, float &pressure, float &temperature)
{
    const struct dps280_cal &cal = calibration;
    // scaling for 16x oversampling
    const float scaling_16 = 1.0f/253952;

    float temp_scaled;
    float press_scaled;

    temp_scaled = float(UT) * scaling_16;
    temperature = cal.C0 * 0.5f + cal.C1 * temp_scaled;

    press_scaled = float(UP) * scaling_16;

    pressure = cal.C00;
    pressure += press_scaled * (cal.C10 + press_scaled * (cal.C20 + press_scaled * cal.C30));
    pressure += temp_scaled * cal.C01;
    pressure += temp_scaled * press_scaled * (cal.C11 + press_scaled * cal.C21);
}

//  acumulate a new sensor reading
void AP_Baro_DPS280::timer(void)
{
    uint8_t buf[6];
    uint8_t ready;

    if (!dev->read_registers(DPS280_REG_MCONF, &ready, 1) || !(ready & (1U<<4))) {
        // pressure not ready
        return;
    }
    if (!dev->read_registers(DPS280_REG_PRESS, buf, 3)) {
        return;
    }
    if (!dev->read_registers(DPS280_REG_TEMP, &buf[3], 3)) {
        return;
    }

    int32_t press = (buf[2]) + (buf[1]<<8) + (buf[0]<<16);
    int32_t temp  = (buf[5]) + (buf[4]<<8) + (buf[3]<<16);
    fix_config_bits32(press, 24);
    fix_config_bits32(temp, 24);

    float pressure, temperature;

    calculate_PT(temp, press, pressure, temperature);

    if (!pressure_ok(pressure)) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    pressure_sum += pressure;
    temperature_sum += temperature;
    count++;
}

// transfer data to the frontend
void AP_Baro_DPS280::update(void)
{
    if (count == 0) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    _copy_to_frontend(instance, pressure_sum/count, temperature_sum/count);
    pressure_sum = 0;
    temperature_sum = 0;
    count=0;
}
