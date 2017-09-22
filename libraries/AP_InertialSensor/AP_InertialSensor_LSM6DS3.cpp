/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_LSM6DS3.h"

#include <utility>
#include <unistd.h>
#include <AP_HAL_Linux/GPIO.h>

extern const AP_HAL::HAL &hal;


////////////////////////////
// LSM6DS3 Gyro Registers //
////////////////////////////
#define WHO_AM_I_ADDR				0x0F
#define LSM6DS3_WHOAMI  			0x69 
#define FIFO_CTRL5		            0x0A
#define CTRL10_C					0x19
#define INT1_CTRL					0x0d
#define CTRL2_G						0x11
#define CTRL1_XL					0x10 
#define CTRL9_XL					0x18
#define OUT_X_L_A					0x28
#define STATUS_REG					0x1E 
#define XL_DATA_READY_MASK			0x1
#define G_DATA_READY_MASK           (0x1 << 1)
#define CTRL1_XL_1600khz            (0x8 << 4)
#define CTRL1_XL_16g                (0x1 << 2)
#define CTRL9_XL_AXIS_VALUES		(0x7 << 3) 
#define CTRL10_C_AXIS_VALUES		(0x7 << 3) 


AP_InertialSensor_LSM6DS3::AP_InertialSensor_LSM6DS3(AP_InertialSensor &imu,AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_InertialSensor_Backend(imu)
	,_dev(std::move(dev))
{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DS3::probe(AP_InertialSensor &_imu,AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) 
	{
        return nullptr;
    }
    AP_InertialSensor_LSM6DS3 *sensor =
        new AP_InertialSensor_LSM6DS3(_imu, std::move(dev));
    if (!sensor || !sensor->_init())
	{
        delete sensor;
        return nullptr;
    }
    return sensor;
}



bool AP_InertialSensor_LSM6DS3::_init()
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    uint8_t tries, whoami;    
    whoami = _register_read(WHO_AM_I_ADDR);
    if (whoami != LSM6DS3_WHOAMI) {	//not LSM6DS3
        hal.console->printf("LSM6DS3: unexpected acc/mag  WHOAMI 0x%x\n", (unsigned)whoami);
        _dev->get_semaphore()->give();
		return false;
    }
        
    for (tries = 0; tries < 5; tries++)
	{
        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
  		_dev->write_register(FIFO_CTRL5, 0x50); //fifo odr 6.6khz , fifo bypass mode
		_accel_init();
		_gyro_init();
		_dev->set_speed(AP_HAL::Device::SPEED_HIGH);
		hal.scheduler->delay(10);
		if (_accel_data_ready() && _gyro_data_ready())
		{ 
			break;
		}
	}// end tries for
    if (5 == tries)
	{
		hal.console->printf("Failed to boot LSM6DS3 5 times\n\n");
        _dev->get_semaphore()->give();
		return false;
    }
    _dev->get_semaphore()->give();
    return true;
}


/*
  get the sensor going
 */
void AP_InertialSensor_LSM6DS3::start(void)
{
    _accel_instance = _imu.register_accel(800, _dev->get_bus_id_devtype(DEVTYPE_LSM6DS3));
    _gyro_instance = _imu.register_gyro(800, _dev->get_bus_id_devtype(DEVTYPE_LSM6DS3));

    // start the timer process to read samples
    _dev->register_periodic_callback(1250, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DS3::_poll_data, void));

}


uint8_t AP_InertialSensor_LSM6DS3::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_LSM6DS3::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val);
}

void AP_InertialSensor_LSM6DS3::_gyro_init()
{
    _register_write(CTRL10_C,CTRL10_C_AXIS_VALUES);
    _register_write(CTRL2_G,0x8C);
    _set_gyro_scale(G_SCALE_2000DPS);
}

void AP_InertialSensor_LSM6DS3::_accel_init()
{
    _register_write(CTRL9_XL,CTRL9_XL_AXIS_VALUES);
    hal.scheduler->delay(1);
    _register_write(CTRL1_XL,CTRL1_XL_1600khz | CTRL1_XL_16g);
    _set_accel_scale(A_SCALE_16G);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DS3::_set_gyro_scale(gyro_scale scale)
{
    /* scales values from datasheet in mdps/digit */
    switch (scale)
	{
	case G_SCALE_125DPS:
        _gyro_scale = 4.375;
        break;
    case G_SCALE_245DPS:
        _gyro_scale = 8.75;
        break;
    case G_SCALE_500DPS:
        _gyro_scale = 17.50;
        break;
	case G_SCALE_1000DPS:
        _gyro_scale = 35;
        break;
    case G_SCALE_2000DPS:
        _gyro_scale = 70;
        break;
    }
    // convert mdps/digit to dps/digit
    _gyro_scale /= 1000;
    //convert dps/digit to (rad/s)/digit
    _gyro_scale *= DEG_TO_RAD;
}

void AP_InertialSensor_LSM6DS3::_set_accel_scale(accel_scale scale)
{
    /*
     * Possible accelerometer scales (and their register bit settings) are:
     * 2 g (00), 4g (10),  8g (11), 16g (01). Here's a bit of an
     * algorithm to calculate g/(ADC tick) based on that 3-bit value:
     */
    _accel_scale = ((float) scale  / 32768.0f);

    // convert to G/LSB to (m/s/s)/LSB */
    _accel_scale *= GRAVITY_MSS;
}

/**
 * Timer process to poll for new data from the LSM6DS3.
 */
void AP_InertialSensor_LSM6DS3::_poll_data()
{
    if (_gyro_data_ready())
	{
        _read_data_transaction_g();
    }
    if (_accel_data_ready())
	{
        _read_data_transaction_a();
    }

    // check next register value for correctness
    if (!_dev->check_next_register())
	{
        _inc_gyro_error_count(_gyro_instance);
    }
    if (!_dev->check_next_register())
	{
        _inc_accel_error_count(_accel_instance);
        
    }
}

bool AP_InertialSensor_LSM6DS3::_accel_data_ready()
{
    uint8_t status = _register_read(STATUS_REG);
    return status & XL_DATA_READY_MASK;
}

bool AP_InertialSensor_LSM6DS3::_gyro_data_ready()
{
    uint8_t status = _register_read(STATUS_REG);
    return status & G_DATA_READY_MASK;
}

void AP_InertialSensor_LSM6DS3::_read_data_transaction_a()
{
    struct sensor_raw_data raw_data = {};
    const uint8_t reg = OUT_X_L_A;
    if (!_dev->transfer(&reg, 1, (uint8_t *) &raw_data, sizeof(raw_data)))
	{
        hal.console->printf("LSM6DS3: error reading accelerometer\n");
        return;
    }
    Vector3f accel_data(raw_data.x, -raw_data.y, -raw_data.z);
    accel_data *= _accel_scale;
    _rotate_and_correct_accel(_accel_instance, accel_data);
    _notify_new_accel_raw_sample(_accel_instance, accel_data, AP_HAL::micros64());

}

/*
 *  read from the data registers and update filtered data
 */
void AP_InertialSensor_LSM6DS3::_read_data_transaction_g()
{
    struct sensor_raw_data raw_data = { };
    //const uint8_t reg = OUT_X_L_G | 0xC0;
    const uint8_t reg = OUT_X_L_G;//miri
    if (!_dev->transfer(&reg, 1, (uint8_t *) &raw_data, sizeof(raw_data)))
	{
        hal.console->printf("LSM6DS3: error reading gyroscope\n");
        return;
    }

    Vector3f gyro_data(raw_data.x, -raw_data.y, -raw_data.z);
    gyro_data *= _gyro_scale;
    _rotate_and_correct_gyro(_gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro_data, AP_HAL::micros64());
   // printf("gyro data x: %lf, y : %lf , z : %lf\n", gyro_data.x, gyro_data.y, gyro_data.z);
}

bool AP_InertialSensor_LSM6DS3::update()
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);
    return true;
}

