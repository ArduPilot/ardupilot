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
 
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/Device.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>
#include "AP_RangeFinder_VL53L0X.h"
#include "AP_RangeFinder_VL53L1X.h"

class AP_RangeFinder_9xVL53LXX {
public:
	static bool set_addr(int new_addr, int temp_addr, int pin, int orientation);

private:
	static AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t address);
	static bool is_mux(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev);
	static int identify_sensor(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev);

    static void _write_register(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev, uint16_t reg, uint8_t value, uint8_t dev_type = 0);
	static uint8_t _read_register(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev, uint16_t reg, uint8_t dev_type = 0);
	
	static bool _write_check(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev, uint8_t reg, uint8_t value, uint8_t attempts, uint8_t dev_type = 0);
	static bool set_addr_check(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &current_dev, uint8_t new_addr, uint8_t attempts, uint8_t dev_type = 0);
};
