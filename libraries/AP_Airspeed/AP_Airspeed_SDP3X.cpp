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
  backend driver for airspeed from a I2C SDP3X sensor
 */
#include "AP_Airspeed_SDP3X.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

AP_Airspeed_SDP3X::AP_Airspeed_SDP3X(AP_Airspeed &_frontend) :
    AP_Airspeed_Backend(_frontend)
{
}

// probe and initialise the sensor
bool AP_Airspeed_SDP3X::init()
{
    const struct{
        uint8_t bus;
        uint8_t addr;
    } addresses[] = {
        { 1, SDP3XD0_I2C_ADDR },
        { 0, SDP3XD0_I2C_ADDR },
        { 2, SDP3XD0_I2C_ADDR },
        { 1, SDP3XD1_I2C_ADDR },
        { 0, SDP3XD1_I2C_ADDR },
        { 2, SDP3XD1_I2C_ADDR },
        { 1, SDP3XD2_I2C_ADDR },
        { 0, SDP3XD2_I2C_ADDR },
        { 2, SDP3XD2_I2C_ADDR },
    };
    bool found = false;
    bool ret = false;
    
    for (uint8_t i=0; i<ARRAY_SIZE(addresses); i++)
    {
        _dev = hal.i2c_mgr->get_device(addresses[i].bus, addresses[i].addr);
        if (!_dev)
            continue;
        if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER))
            continue;

        // lots of retries during probe
        _dev->set_retries(10); 

	// step 1 - reset on broadcast
        uint8_t reset_cmd = SDP3X_RESET_CMD;
        ret = _dev->transfer((uint8_t *)&reset_cmd, 1, nullptr, 0);
        if (!ret)
            printf("SDP3X: reset command failed\n");

	// waiting 20ms to reset
	usleep(20000);

	// step 2 - configure
        uint8_t cmd[2] = {(SDP3X_CONT_MEAS_AVG_MODE & 0xFF00) >> 8, SDP3X_CONT_MEAS_AVG_MODE & 0xFF};
        ret = _dev->transfer((uint8_t *)&cmd, 2, nullptr, 0);
        if (!ret)
            printf("SDP3X: continued measure command failed\n");

	// waiting again
	usleep(20000);

	// step 3 - get scale
	uint8_t val[9];
	ret = _dev->transfer(nullptr, 0, &val[0], sizeof(val));    
        if (!ret)
            printf("SDP3X: get scale value failed\n");

    	// Check the CRC
        if (!_crc(&val[0], 2, val[2]) || !_crc(&val[3], 2, val[5]) || !_crc(&val[6], 2, val[8]))
	    printf("SDP3X: CRC failed !\n");

	_scale = (((uint16_t)val[6]) << 8) | val[7];
	printf("Scale value is : %d\n", _scale);
        _collect();
        _dev->get_semaphore()->give();
        found = (_last_sample_time_ms != 0);
        if (found)
        {
	    switch (_scale)
	    {
		case SDP3X_SCALE_PRESSURE_SDP31:
            		printf("SDP31: Found sensor on bus %u address 0x%02x\n",
				addresses[i].bus, addresses[i].addr);
			break;
		case SDP3X_SCALE_PRESSURE_SDP32:
		        printf("SDP32: Found sensor on bus %u address 0x%02x\n",
				addresses[i].bus, addresses[i].addr);
			break;
		case SDP3X_SCALE_PRESSURE_SDP33:
            		printf("SDP33: Found sensor on bus %u address 0x%02x\n",
				addresses[i].bus, addresses[i].addr);
			break;
	    }
            break;
        }
    }
    if (!found)
    {
        printf("SDP3X: no sensor found\n");
        return false;
    }

    // drop to 2 retries for runtime
    _dev->set_retries(2);
    
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_SDP3X::_timer, void));
    return true;
}

// read the values from the sensor
void AP_Airspeed_SDP3X::_collect()
{
	// read 9 bytes from the sensor
	uint8_t val[6];
	int ret = _dev->transfer(nullptr, 0, &val[0], sizeof(val));
	if (!ret)
		printf("SDP3X: Collect error\n");
	// Check the CRC
	if (!_crc(&val[0], 2, val[2]) || !_crc(&val[3], 2, val[5]))
	    printf("Error, BAD CRC collected\n");

	int16_t P = (((int16_t)val[0]) << 8) | val[1];
	int16_t temp = (((int16_t)val[3]) << 8) | val[4];

	float P_float = (float)P;
	float temp_float = (float)temp;

	float scale_float = (float)_scale;

	float diff_press_pa = P_float / scale_float;
	float temperature = temp_float / (float)SDP3X_SCALE_TEMPERATURE;

        if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
	{
		_press = diff_press_pa;
		_temp = temperature;
	        sem->give();
	}
	_last_sample_time_ms = AP_HAL::millis();
}

// 20Hz timer
void AP_Airspeed_SDP3X::_timer()
{
    _collect();
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_SDP3X::get_differential_pressure(float &pressure)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 50)
        return false;
    pressure = _press;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_SDP3X::get_temperature(float &temperature)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 50)
        return false;
    temperature = _temp;
    return true;
}

bool AP_Airspeed_SDP3X::_crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
	uint8_t crc_value = 0xff;

	// calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
	for (unsigned i = 0; i < size; i++)
	{
		crc_value ^= (data[i]);
		for (int bit = 8; bit > 0; --bit)
		{
			if (crc_value & 0x80)
				crc_value = (crc_value << 1) ^ 0x31;
			else
				crc_value = (crc_value << 1);
		}
	}
	// verify checksum
	return (crc_value == checksum);
}
