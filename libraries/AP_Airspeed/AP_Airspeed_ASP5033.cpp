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
  backend driver for airspeed sensor from www.qio-tek.com
  I2C ASP5033 sensor
 */

#include "AP_Airspeed_ASP5033.h"
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL &hal;

#define ASP5033_I2C_ADDR_1     0x6C
#define ASP5033_I2C_ADDR_2     0x6D
#define REG_CMD                0x30
#define REG_PRESS_DATA         0x06
#define REG_TEMP_DATA          0x09
#define REG_PART_ID            0x01
#define REG_PART_ID_SET        0xa4
#define REG_SENSOR_READY       0x08
#define REG_WHOAMI_DEFAULT_ID  0X00
#define REG_WHOAMI_RECHECK_ID  0X66
#define CMD_MEASURE            0x0A


bool AP_Airspeed_ASP5033::init()
{
    // probe the sensor, supporting multiple possible I2C addresses
    const uint8_t addresses[] = { ASP5033_I2C_ADDR_1, ASP5033_I2C_ADDR_2 };
    for (uint8_t address : addresses) {
        dev = hal.i2c_mgr->get_device(get_bus(), address);
        if (!dev) {
            continue;
        }

        WITH_SEMAPHORE(dev->get_semaphore());
        dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        dev->set_retries(2);

        if (!confirm_sensor_id()) {
            continue;
        }

        dev->register_periodic_callback(1000000UL/80U,
                                        FUNCTOR_BIND_MEMBER(&AP_Airspeed_ASP5033::timer, void));
        return true;
    }

    // not found
    return false;
}

/*
  this sensor has an unusual whoami scheme. The part_id is changeable
  via another register. We check the sensor by looking for the
  expected behaviour
*/
bool AP_Airspeed_ASP5033::confirm_sensor_id(void)
{
    uint8_t part_id;
    if (!dev->read_registers(REG_PART_ID_SET, &part_id, 1) ||
        part_id != REG_WHOAMI_DEFAULT_ID) {
        return false;
    }
    if (!dev->write_register(REG_PART_ID_SET, REG_WHOAMI_RECHECK_ID)) {
        return false;
    }
    if (!dev->read_registers(REG_PART_ID, &part_id, 1) ||
        part_id != REG_WHOAMI_RECHECK_ID) {
        return false;
    }
    return true;
}


// read the data from the sensor
void AP_Airspeed_ASP5033::timer()
{
    // request a new measurement cycle begin
    dev->write_register(REG_CMD, CMD_MEASURE);

    uint8_t status;
    if (!dev->read_registers(REG_CMD, &status, 1) ||
        (status & REG_SENSOR_READY) == 0) {
        // no data ready
        return;
    }

    // read pressure and temperature as one block
    uint8_t data[5];
    if (!dev->read_registers(REG_PRESS_DATA, data, sizeof(data))) {
        return;
    }

    // ADC pressure is signed 24 bit
    int32_t press = (data[0]<<24) | (data[1]<<16) | (data[2]<<8);

    // convert back to 24 bit
    press >>= 8;

    // k is a shift based on the pressure range of the device. See
    // table in the datasheet
    constexpr uint8_t k = 7;
    constexpr float press_scale = 1.0 / (1U<<k);

    // temperature is 16 bit signed in units of 1/256 C
    const int16_t temp = (data[3]<<8) | data[4];
    constexpr float temp_scale = 1.0 / 256;

    WITH_SEMAPHORE(sem);
    press_sum += press * press_scale;
    temp_sum += temp * temp_scale;
    press_count++;
    temp_count++;

    last_sample_ms = AP_HAL::millis();
}


// return the current differential_pressure in Pascal
bool AP_Airspeed_ASP5033::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);

    if (AP_HAL::millis() - last_sample_ms > 100) {
        return false;
    }

    if (press_count == 0) {
        pressure = last_pressure;
        return true;
    }

    last_pressure = pressure = press_sum / press_count;

    press_count = 0;
    press_sum = 0;

    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_ASP5033::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);

    if (AP_HAL::millis() - last_sample_ms > 100) {
        return false;
    }
    if (temp_count == 0) {
        temperature = last_temperature;
        return true;
    }

    last_temperature = temperature = temp_sum / temp_count;
    temp_count = 0;
    temp_sum = 0;

    return true;
}
