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

#include "AP_RangeFinder_9xVL53LXX.h"
#include <unistd.h>

#define MUX_ADDR 0x20
#define SENSOR_DEFAULT_ADDR 0x29

#define MUX_INPUT_PORT_REG 0x00
#define MUX_OUTPUT_PORT_REG 0x01
#define MUX_POLARITY_INVERSION_REG 0x02
#define MUX_CONFIG_REG 0x03

#define MUX_INPUT_PORT_VAL 0xFF

#define WRITE_CHECK_DELAY_MS 1
#define WRITE_CHECK_ATTEMPTS 5

extern const AP_HAL::HAL &hal;

//<orientation, channel>. -1 no channel
const std::map<int,int> AP_RangeFinder_9xVL53LXX::channel_mapping {
	{0, 0},   // Forward
	{1, 1},   // Forward-Right
	{2, 2},   // Right
	{3, 3},   // Back-Right
	{4, 4},   // Back
	{5, 5},   // Back-Left
	{6, 6},   // Left
	{7, 7},   // Forward-Left
	{24, -1}, // Up
	{25, -1}  // Down
};

AP_HAL::OwnPtr<AP_HAL::I2CDevice> AP_RangeFinder_9xVL53LXX::get_device(uint8_t address) {
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev = hal.i2c_mgr->get_device(1, address);
	
	if(!dev){
		dev = hal.i2c_mgr->get_device(0, address);
	}
	
	return dev;
}

bool AP_RangeFinder_9xVL53LXX::set_addr(uint8_t new_addr, uint8_t temp_addr, uint8_t orientation) {
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> new_dev = get_device(new_addr);
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> temp_dev = get_device(temp_addr);
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> mux_dev = get_device(MUX_ADDR);
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> def_dev = get_device(SENSOR_DEFAULT_ADDR);
	
	int delay_ms = 2; //T_boot = 1.2ms according to datasheet.
	
	uint8_t pin_state = 0;
	uint8_t pin_config = 0;
	int pin = channel_mapping.find(orientation)->second;
	bool new_default = false;
	uint8_t default_sensor_type = 0;
	uint8_t new_sensor_type = 0;
	uint8_t temp_sensor_type = 0;

	//check for IO expander/multiplexer
	if(!mux_dev || !temp_dev || !def_dev || !is_mux(mux_dev)) {
		printf("9xVL53LXX: I2C address configuration not possible.\n");
		return false;
	}
	
	pin_config = _read_register(mux_dev, MUX_CONFIG_REG);
	if(pin >= 0) {
		pin_config &= ~(1<<pin);
	}
	
	//set pin mode to output
	if(!_write_check(mux_dev, MUX_CONFIG_REG, pin_config, WRITE_CHECK_ATTEMPTS)) {
		printf("9xVL53LXX: mux could not set output mode\n");
	}
	
	//read pin states
	pin_state = _read_register(mux_dev, MUX_OUTPUT_PORT_REG);
	
	//disable only target pin
	if(pin >= 0) {
		pin_state &= ~(1<<pin);
		if(!_write_check(mux_dev, MUX_OUTPUT_PORT_REG, pin_state, WRITE_CHECK_ATTEMPTS)) {
			printf("9xVL53LXX: mux could not disable new sensor\n");
		}
	}
	
	//check if 0x29 is new_addr
	new_default = (new_addr == SENSOR_DEFAULT_ADDR);
	
	//check if 0x29 is already in use
	default_sensor_type = identify_sensor(def_dev);
	
	//check if there is already a sensor on new_addr
	new_sensor_type = identify_sensor(new_dev);
	
	//handling default sensor
	if(pin < 0) {
		if(default_sensor_type > 0) {
			if(!new_default) {
				if(new_sensor_type > 0) { //both default and new sensor addresses are used, so changing the address will cause a collision
					printf("9xVL53LXX: error: changing the address of the default sensor to 0x%x will cause a collision, aborting\n", new_addr);
					return false;
				}
				//default used and new address is free, so changing address
				if(!set_addr_check(def_dev, new_addr, WRITE_CHECK_ATTEMPTS, default_sensor_type)) {
					printf("9xVL53LXX: error when trying to set default sensor address to 0x%x, aborting\n", new_addr);
					return false;
				}
			}
			return true;
		}else if(new_sensor_type > 0) { //if default address is free but the new_addr is used, probably means the sensor was already set
			return true;
		}
		//neither default nor new address is used, the configuration must have changed without resetting (power cycling) the sensor array
		printf("9xVL53LXX: error: please check your configuration and reset the sensor array\n");
		return false;
	}
	
	//if there is a sensor at new_addr, report warning and try to resolve the conflict
	//if it cannot be resolved, report error and abort
	if(new_sensor_type > 0 && !new_default) {
		if(default_sensor_type > 0 || (temp_sensor_type = identify_sensor(temp_dev)) > 0) {
			printf("9xVL53LXX: error when trying to set address to 0x%x, aborting\n", new_addr);
			return false;
		}
		
		printf("9xVL53LXX: warning: address conflict found on 0x%x, attempting to resolve\n", new_addr);
		
		if(!set_addr_check(new_dev, temp_addr, WRITE_CHECK_ATTEMPTS, new_sensor_type)) {
			printf("9xVL53LXX: error when trying to set address to 0x%x, aborting\n", new_addr);
			return false;
		}
		default_sensor_type = new_sensor_type; //make sure the presumed default sensor is set to correct address at the end of the procedure (i.e. moved from temp to where it should be)
		temp_sensor_type = new_sensor_type;
	}
	
	//if the new_addr is default, just enable the sensor and it's set
	if(new_default) {
		if(pin >= 0) {
			pin_state |= (1<<pin);
			if(!_write_check(mux_dev, MUX_OUTPUT_PORT_REG, pin_state, WRITE_CHECK_ATTEMPTS)) {
				printf("9xVL53LXX: mux could not enable new sensor on default address\n");
			}
		}
		return true;
	}
	
	//if in use, move to temp_addr
	if(default_sensor_type > 0) {
		if(!set_addr_check(def_dev, temp_addr, WRITE_CHECK_ATTEMPTS, default_sensor_type)) {
			printf("9xVL53LXX: failed to move sensor from SENSOR_DEFAULT_ADDR to temp_addr\n");
		} else {
		    temp_sensor_type = default_sensor_type;
		}
	}
	
	//enable target pin
	if(pin >= 0) {
		pin_state |= (1<<pin);
		if(!_write_check(mux_dev, MUX_OUTPUT_PORT_REG, pin_state, WRITE_CHECK_ATTEMPTS)) {
			printf("9xVL53LXX: mux could not enable new sensor\n");
		}
	}
	
	//wait for sensor to start up
	if(delay_ms > 0) {
		hal.scheduler->delay(delay_ms);
	}
	
	//identify sensor that was just enabled
	default_sensor_type = identify_sensor(def_dev);

	//if new addr is not default, move to new_addr
	if(!set_addr_check(def_dev, new_addr, WRITE_CHECK_ATTEMPTS, default_sensor_type)) {
		printf("9xVL53LXX: failed to move new sensor to new_addr\n");
	}
	
	//if 0x29 was in use, move from temp_addr back to 0x29
	if(default_sensor_type) {
		if(!set_addr_check(temp_dev, SENSOR_DEFAULT_ADDR, WRITE_CHECK_ATTEMPTS, temp_sensor_type)) {
			printf("9xVL53LXX: failed to move default sensor back to SENSOR_DEFAULT_ADDR\n");
		}
	}
	
	return true;
}

bool AP_RangeFinder_9xVL53LXX::is_mux(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev) {
	//TODO: no idea how
	return true;
}

// 0 - unknown, 1 - VL53L0X, 2 - VL53L1X
int AP_RangeFinder_9xVL53LXX::identify_sensor(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev) {
    uint8_t v1, v2;

    v1 = _read_register(_dev, 0x010F, 2);
    v2 = _read_register(_dev, 0x0110, 2);
    if ((v1 == 0xEA) &&
        (v2 == 0xCC)) {
        return 2;
    }

    v1 = _read_register(_dev, 0xC0, 1);
    v2 = _read_register(_dev, 0xC1, 1);
    if (v1 == 0xEE &&
        v2 == 0xAA) {
        return 1;
    }

    return 0;
}

void AP_RangeFinder_9xVL53LXX::_write_register(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev, uint16_t reg, uint8_t value, uint8_t dev_type) {
    if (dev_type <= 1) {
        _dev->write_register((uint8_t)reg, value);
    } else if (dev_type == 2) {
        uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
        _dev->transfer(b, 3, nullptr, 0);
    }
}

bool AP_RangeFinder_9xVL53LXX::_write_check(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev, uint8_t reg, uint8_t value, uint8_t attempts, uint8_t dev_type) {
	if(attempts < 1) {
		attempts = 1;
	}
	
	for(int i = 0; i < attempts; i++) {
		_write_register(_dev, reg, value, dev_type);
		if(value == _read_register(_dev, reg, dev_type)) {
			if(i > 0) {
				printf("9xVL53LXX: _write_check success on attempt %d/%d\n", i+1, WRITE_CHECK_ATTEMPTS);
			}
			return true;
		}
		if(i < attempts-1) {
			hal.scheduler->delay(WRITE_CHECK_DELAY_MS);
		}
	}

	return false;
}

uint8_t AP_RangeFinder_9xVL53LXX::_read_register(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev, uint16_t reg, uint8_t dev_type) {
    uint8_t v = 0;

    if (dev_type <= 1) {
        _dev->read_registers(reg, &v, 1);
    } else if (dev_type == 2) {
        uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
        _dev->transfer(b, 2, &v, 1);
    }

    return v;
}

bool AP_RangeFinder_9xVL53LXX::set_addr_check(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &current_dev, uint8_t new_addr, uint8_t attempts, uint8_t dev_type) {
	if(attempts < 1) {
		attempts = 1;
	}
	
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> new_dev = get_device(new_addr);
	
	for(int i = 0; i < attempts; i++) {
	    if (dev_type <= 1) {
	        AP_RangeFinder_VL53L0X::set_addr(current_dev, new_addr);
	    } else if (dev_type == 2) {
	        AP_RangeFinder_VL53L1X::set_addr(current_dev, new_addr);
	    }
		if(identify_sensor(new_dev) == dev_type) {
			if(i > 0) {
				printf("9xVL53LXX: set_addr_check success on attempt %d/%d\n", i+1, WRITE_CHECK_ATTEMPTS);
			}
			return true;
		}
		if(i < attempts-1) {
			hal.scheduler->delay(WRITE_CHECK_DELAY_MS);
		}
	}
	
	return false;
}
