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

#include "AP_Baro_ICP101XX.h"

#if AP_BARO_ICP101XX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

#include <AP_InertialSensor/AP_InertialSensor_Invensense_registers.h>

extern const AP_HAL::HAL &hal;

#define ICP101XX_ID    0x08
#define CMD_READ_ID    0xefc8
#define CMD_SET_ADDR   0xc595
#define CMD_READ_OTP   0xc7f7
#define CMD_MEAS_LP    0x609c
#define CMD_MEAS_N     0x6825
#define CMD_MEAS_LN    0x70df
#define CMD_MEAS_ULN   0x7866
#define CMD_SOFT_RESET 0x805d

/*
  constructor
 */
AP_Baro_ICP101XX::AP_Baro_ICP101XX(AP_Baro &baro, AP_HAL::Device &_dev)
    : AP_Baro_Backend(baro)
    , dev(&_dev)
{
}

AP_Baro_Backend *AP_Baro_ICP101XX::probe(AP_Baro &baro, AP_HAL::Device &dev)
{
    AP_Baro_ICP101XX *sensor = NEW_NOTHROW AP_Baro_ICP101XX(baro, dev);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_ICP101XX::init()
{
    if (!dev) {
        return false;
    }

    dev->get_semaphore()->take_blocking();

    uint16_t id = 0;
    read_response(CMD_READ_ID, (uint8_t *)&id, 2);
    uint8_t whoami = (id >> 8) & 0x3f; // Product ID Bits 5:0
    if (whoami != ICP101XX_ID) {
        goto failed;
    }

    if (!send_command(CMD_SOFT_RESET)) {
        goto failed;
    }

    // wait for sensor to settle
    hal.scheduler->delay(10);

    if (!read_calibration_data()) {
        goto failed;
    }

    // start a reading
    if (!start_measure(CMD_MEAS_ULN)) {
        goto failed;
    }

    dev->set_retries(0);

    instance = _frontend.register_sensor();

    dev->set_device_type(DEVTYPE_BARO_ICP101XX);
    set_bus_id(instance, dev->get_bus_id());
    
    dev->get_semaphore()->give();

    dev->register_periodic_callback(measure_interval, FUNCTOR_BIND_MEMBER(&AP_Baro_ICP101XX::timer, void));

    return true;

 failed:
    dev->get_semaphore()->give();
    return false;
}

bool AP_Baro_ICP101XX::read_measure_results(uint8_t *buf, uint8_t len)
{
	return dev->transfer(nullptr, 0, buf, len);
}

bool AP_Baro_ICP101XX::read_response(uint16_t cmd, uint8_t *buf, uint8_t len)
{
	uint8_t buff[2];
	buff[0] = (cmd >> 8) & 0xff;
	buff[1] = cmd & 0xff;
	return dev->transfer(&buff[0], 2, buf, len);
}

bool AP_Baro_ICP101XX::send_command(uint16_t cmd)
{
	uint8_t buf[2];
	buf[0] = (cmd >> 8) & 0xff;
	buf[1] = cmd & 0xff;
	return dev->transfer(buf, sizeof(buf), nullptr, 0);
}

bool AP_Baro_ICP101XX::send_command(uint16_t cmd, uint8_t *data, uint8_t len)
{
	uint8_t buf[5];
	buf[0] = (cmd >> 8) & 0xff;
	buf[1] = cmd & 0xff;
	memcpy(&buf[2], data, len);
	return dev->transfer(&buf[0], len + 2, nullptr, 0);
}

int8_t AP_Baro_ICP101XX::cal_crc(uint8_t seed, uint8_t data)
{
	int8_t poly = 0x31;
	int8_t var2;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if ((seed & 0x80) ^ (data & 0x80)) {
			var2 = 1;

		} else {
			var2 = 0;
		}

		seed = (seed & 0x7F) << 1;
		data = (data & 0x7F) << 1;
		seed = seed ^ (uint8_t)(poly * var2);
	}

	return (int8_t)seed;
}

bool AP_Baro_ICP101XX::read_calibration_data(void)
{
    // setup for OTP read
    uint8_t cmd[3] = { 0x00, 0x66, 0x9c };
    if (!send_command(CMD_SET_ADDR, cmd, 3)) {
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t d[3];
        uint8_t crc = 0xff;
        read_response(CMD_READ_OTP, d, 3);
        for (int j = 0; j < 2; j++) {
            crc = (uint8_t)cal_crc(crc, d[j]);
        }
        if (crc != d[2]) {
            return false;
        }
        sensor_constants[i] = (d[0] << 8) | d[1];
    }
    return true;
}

bool AP_Baro_ICP101XX::start_measure(uint16_t mode)
{
	/*
	  From ds-000186-icp-101xx-v1.0.pdf, page 6, table 1

	  Sensor                  Measurement       Max Time
	  Mode                    Time (Forced)
	  Low Power (LP)             1.6 ms          1.8 ms
	  Normal (N)                 5.6 ms          6.3 ms
	  Low Noise (LN)             20.8 ms         23.8 ms
	  Ultra Low Noise(ULN)       83.2 ms         94.5 ms
	*/

	switch (mode) {
	case CMD_MEAS_LP:
		measure_interval = 2000;
		break;

	case CMD_MEAS_LN:
		measure_interval = 24000;
		break;

	case CMD_MEAS_ULN:
		measure_interval = 95000;
		break;

	case CMD_MEAS_N:
	default:
		measure_interval = 7000;
		break;
	}

	if (!send_command(mode)) {
		return false;
	}

	return true;
}

void AP_Baro_ICP101XX::calculate_conversion_constants(const float p_Pa[3], const float p_LUT[3],
                                                      float &A, float &B, float &C)
{
    C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
         p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
         p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
        (p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
         p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
         p_LUT[1] * (p_Pa[2] - p_Pa[0]));
    A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);
    B = (p_Pa[0] - A) * (p_LUT[0] + C);
}

/*
  Convert an output from a calibrated sensor to a pressure in Pa.
  Arguments:
  p_LSB -- Raw pressure data from sensor
  T_LSB -- Raw temperature data from sensor
*/
float AP_Baro_ICP101XX::get_pressure(uint32_t p_LSB, uint32_t T_LSB)
{
    float t = T_LSB - 32768.0;
    float s[3];
    s[0] = LUT_lower + float(sensor_constants[0] * t * t) * quadr_factor;
    s[1] = offst_factor * sensor_constants[3] + float(sensor_constants[1] * t * t) * quadr_factor;
    s[2] = LUT_upper + float(sensor_constants[2] * t * t) * quadr_factor;
    float A, B, C;
    calculate_conversion_constants(p_Pa_calib, s, A, B, C);
    return A + B / (C + p_LSB);
}

void AP_Baro_ICP101XX::convert_data(uint32_t Praw, uint32_t Traw)
{
    // temperature is easy
    float T = -45 + (175.0f / (1U<<16)) * Traw;

    // pressure involves a few more calculations
    float P = get_pressure(Praw, Traw);

    if (!pressure_ok(P)) {
        return;
    }

    WITH_SEMAPHORE(_sem);
      
    accum.psum += P;
    accum.tsum += T;
    accum.count++;
}

void AP_Baro_ICP101XX::timer(void)
{
    uint8_t d[9] {};
    if (read_measure_results(d, 9)) {
        // ignore CRC bytes for now
        uint32_t Traw = (uint32_t(d[0]) << 8) | d[1];
        uint32_t Praw = (uint32_t(d[3]) << 16) | (uint32_t(d[4]) << 8) | d[6];

        convert_data(Praw, Traw);
        start_measure(CMD_MEAS_ULN);
        last_measure_us = AP_HAL::micros();
    } else {
        if (AP_HAL::micros() - last_measure_us > measure_interval*3) {
            // lost a sample
            start_measure(CMD_MEAS_ULN);
            last_measure_us = AP_HAL::micros();
        }
    }
}

void AP_Baro_ICP101XX::update()
{
    WITH_SEMAPHORE(_sem);
    
    if (accum.count > 0) {
        _copy_to_frontend(instance, accum.psum/accum.count, accum.tsum/accum.count);
        accum.psum = accum.tsum = 0;
        accum.count = 0;
    }
}

#endif  // AP_BARO_ICP101XX_ENABLED
