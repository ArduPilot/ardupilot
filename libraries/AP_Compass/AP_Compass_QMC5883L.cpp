/*
 * Copyright (C) 2016  Emlid Ltd. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Driver by RadioLink LjWang, Jun 2017
 * GPS compass module See<http://www.radiolink.com>
 */
#include "AP_Compass_QMC5883L.h"

#include <stdio.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#define QMC5883L_REG_CONF1 0x09
#define QMC5883L_REG_CONF2 0x0A

// data output rates for 5883L
#define QMC5883L_ODR_10HZ (0x00 << 2)
#define QMC5883L_ODR_50HZ  (0x01 << 2)
#define QMC5883L_ODR_100HZ (0x02 << 2)
#define QMC5883L_ODR_200HZ (0x03 << 2)

// Sensor operation modes
#define QMC5883L_MODE_STANDBY 0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

#define QMC5883L_RNG_2G (0x00 << 4)
#define QMC5883L_RNG_8G (0x01 << 4)

#define QMC5883L_OSR_512 (0x00 << 6)
#define QMC5883L_OSR_256 (0x01 << 6)
#define QMC5883L_OSR_128	(0x10 << 6)
#define QMC5883L_OSR_64	(0x11	<< 6)

#define QMC5883L_RST 0x80

#define QMC5883L_REG_DATA_OUTPUT_X 0x00
#define QMC5883L_REG_STATUS 0x06

#define QMC5883L_REG_ID 0x0D
#define QMC5883_ID_VAL 0xFF

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_QMC5883L::probe(Compass &compass,
                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
											  bool force_external,
											  enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_Compass_QMC5883L *sensor = new AP_Compass_QMC5883L(compass, std::move(dev),force_external,rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_QMC5883L::AP_Compass_QMC5883L(Compass &compass,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev,
									   bool force_external,
									   enum Rotation rotation)
    : AP_Compass_Backend(compass)
    , _dev(std::move(dev))
    , _rotation(rotation)
	, _force_external(force_external)
{
}

bool AP_Compass_QMC5883L::init()
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    _dev->set_retries(10);

#if 0
    _dump_registers();
#endif

    if(!_check_whoami()){
    	 goto fail;
    }

    if (!_dev->write_register(0x0B, 0x01)||
    	 !_dev->write_register(0x20, 0x40)||
		 !_dev->write_register(0x21, 0x01)||
		 !_dev->write_register(QMC5883L_REG_CONF1,
						QMC5883L_MODE_CONTINUOUS|
						QMC5883L_ODR_100HZ|
						QMC5883L_OSR_512|
						QMC5883L_RNG_8G)) {
    		  	  goto fail;
     }

    // lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    _instance = register_compass();

    printf("%s found on bus %u id %u address 0x%02x\n", name,
           _dev->bus_num(), _dev->get_bus_id(), _dev->get_bus_address());

    set_rotation(_instance, _rotation);

    _dev->set_device_type(DEVTYPE_QMC5883L);
    set_dev_id(_instance, _dev->get_bus_id());

    if (_force_external) {
        set_external(_instance, true);
    }

    //Enable 100HZ
    _dev->register_periodic_callback(10000,
        FUNCTOR_BIND_MEMBER(&AP_Compass_QMC5883L::timer, void));

    return true;

 fail:
    _dev->get_semaphore()->give();
    return false;
}
bool AP_Compass_QMC5883L::_check_whoami()
{
    uint8_t whoami;
    //Affected by other devices,must read registers 0x00 once or reset,after can read the ID registers reliably
    _dev->read_registers(0x00,&whoami,1);
    if (!_dev->read_registers(0x0C, &whoami,1)||
      		whoami != 0x01){
    	return false;
    }
    if (!_dev->read_registers(QMC5883L_REG_ID, &whoami,1)||
    		whoami != QMC5883_ID_VAL){
    	return false;
    }
    return true;
}

void AP_Compass_QMC5883L::timer()
{
    struct PACKED {
    	le16_t rx;
    	le16_t ry;
    	le16_t rz;
    } buffer;

    const float range_scale = 1000.0f / 3000.0f;

    uint8_t status;
    if(!_dev->read_registers(QMC5883L_REG_STATUS,&status,1)){
    	return;
    }
    //new data is ready
    if (!(status & 0x04)) {
    	return;
    }

  if(!_dev->read_registers(QMC5883L_REG_DATA_OUTPUT_X, (uint8_t *) &buffer, sizeof(buffer))){
	  return ;
  }

    auto x = -static_cast<int16_t>(le16toh(buffer.rx));
    auto y = static_cast<int16_t>(le16toh(buffer.ry));
    auto z = -static_cast<int16_t>(le16toh(buffer.rz));

#if 0
    printf("mag.x:%d\n",x);
    printf("mag.y:%d\n",y);
    printf("mag.z:%d\n",z);
#endif

    Vector3f field = Vector3f{x * range_scale , y * range_scale, z * range_scale };

    // rotate to the desired orientation
    if (is_external(_instance)) {
        field.rotate(ROTATION_YAW_90);
    }

    /* rotate raw_field from sensor frame to body frame */
    rotate_field(field, _instance);

    /* publish raw_field (uncorrected point sample) for calibration use */
    publish_raw_field(field, _instance);

    /* correct raw_field for known errors */
    correct_field(field, _instance);

    if (!field_ok(field)) {
        return;
    }

    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _accum += field;
        _accum_count++;
        if(_accum_count == 20){
            _accum.x /= 2;
            _accum.y /= 2;
            _accum.z /= 2;
            _accum_count = 10;
        }
        _sem->give();
    }
}

void AP_Compass_QMC5883L::read()
{
    if (!_sem->take_nonblocking()) {
        return;
    }

    if (_accum_count == 0) {
        _sem->give();
        return;
    }

    Vector3f field(_accum);
    field /= _accum_count;

    publish_filtered_field(field, _instance);

    _accum.zero();
    _accum_count = 0;

    _sem->give();
}

void AP_Compass_QMC5883L::_dump_registers()
{
	  printf("QMC5883L registers dump\n");
	    for (uint8_t reg = QMC5883L_REG_DATA_OUTPUT_X; reg <= 0x30; reg++) {
	        uint8_t v;
	        _dev->read_registers(reg,&v,1);
	        printf("%02x:%02x ", (unsigned)reg, (unsigned)v);
	        if ((reg - ( QMC5883L_REG_DATA_OUTPUT_X-1)) % 16 == 0) {
	            printf("\n");
	        }
	    }
}

