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

#include "AP_RangeFinder_9xVL53L0X.h"
#include <unistd.h>

#define MUX_ADDR 0x20
#define SENSOR_DEFAULT_ADDR 0x29

#define MUX_INPUT_PORT_REG 0x00
#define MUX_OUTPUT_PORT_REG 0x01
#define MUX_POLARITY_INVERSION_REG 0x02
#define MUX_CONFIG_REG 0x03

#define MUX_INPUT_PORT_VAL 0xFF
#define MUX_CONFIG_VAL 0x00 // All pins set to output.

extern const AP_HAL::HAL &hal;

AP_HAL::OwnPtr<AP_HAL::I2CDevice>& AP_RangeFinder_9xVL53L0X::get_device(uint8_t address) {
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev = hal.i2c_mgr->get_device(1, address)
	
	if(!dev){
		dev = hal.i2c_mgr->get_device(0, address);
	}
	
	return dev;
}

bool AP_RangeFinder_9xVL53L0X::set_addr(uint8_t new_addr, uint8_t temp_addr, uint8_t orientation) {
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> &temp_dev = get_device(temp_addr);
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> &mux_dev = get_device(MUX_ADDR);
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> &def_dev = get_device(SENSOR_DEFAULT_ADDR);
	
	uint8_t pin_state = 0;
	uint8_t pin = channel_mapping[orientation];
	bool new_default = false;
	bool default_used = false;
	
	//check for IO expander (abort if not present)
	if(!is_mux(mux_dev)) {
		return false;
	}
	
	//set all pins as output
	_write_register(mux_dev, MUX_CONFIG_REG, MUX_CONFIG_VAL);
	
	//read pin states
	pin_state = _read_register();
	
	//disable only target pin
	if(pin >= 0) {
		pin_state &= ~(1<<pin);
		_write_register(mux_dev, MUX_OUTPUT_PORT_REG, pin_state);
	}
	
	//check if 0x29 is new_addr
	new_default = (new_addr == SENSOR_DEFAULT_ADDR);
	
	//if the new_addr is default, just enable the sensor and it's set
	if(new_default) {
		if(pin >= 0) {
			pin_state |= (1<<pin);
			_write_register(mux_dev, MUX_OUTPUT_PORT_REG, pin_state);
		}
		return true;
	}
	
	//check if 0x29 is already in use
	default_used = is_sensor(def_dev);
	
	//if in use, move to temp_addr
	if(default_used) {
		AP_RangeFinder_VL53L0X::set_addr(def_dev, temp_addr);
	}
	
	//enable target pin
	if(pin >= 0) {
		pin_state |= (1<<pin);
		_write_register(mux_dev, MUX_OUTPUT_PORT_REG, pin_state);
	}
	
	//if new addr is not default, move to new_addr
	AP_RangeFinder_VL53L0X::set_addr(def_dev, new_addr);
	
	//if 0x29 was in use, move from temp_addr back to 0x29
	if(default_used) {
		AP_RangeFinder_VL53L0X::set_addr(temp_dev, SENSOR_DEFAULT_ADDR);
	}
	
	return true;
}

bool AP_RangeFinder_9xVL53L0X::is_mux(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev) {
	if (_read_register(_dev, MUX_INPUT_PORT_REG) == MUX_INPUT_PORT_VAL) {
		return true;
	}
	return false;
}

bool AP_RangeFinder_9xVL53L0X::is_sensor(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev) {
    uint8_t v1, v2;
    v1 = _read_register(0xC0);
    v2 = _read_register(0xC1);
    if (v1 != 0xEE ||
        v2 != 0xAA) {
        return false;
    }
    return true;
}

void AP_RangeFinder_9xVL53L0X::_write_register(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev, uint8_t reg, uint8_t value) {
	_dev->write_register(reg, value);
}

uint8_t AP_RangeFinder_9xVL53L0X::_read_register(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &_dev, uint8_t reg){
    uint8_t v = 0;
    _dev->read_registers(reg, &v, 1);
    return v;
}