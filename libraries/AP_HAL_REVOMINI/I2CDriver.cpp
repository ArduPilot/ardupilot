/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * I2CDriver.cpp --- AP_HAL_REVOMINI I2C driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.R Written by Roberto Navoni  <info@virtualrobotix.com>, 11 January 2013
 */

#include <AP_HAL/AP_HAL.h>
#include "I2CDriver.h"
#include <i2c.h>

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

#define I2CDELAY 50

#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)

__IO uint32_t  i2ctimeout = I2C_TIMEOUT;

REVOMINII2CDriver::REVOMINII2CDriver(i2c_dev *dev, AP_HAL::Semaphore* semaphore):
	_dev(dev),
	_semaphore(semaphore),
	_lockup_count(0)
{
}
void REVOMINII2CDriver::begin() {

    if(_dev->I2Cx == I2C1)
	i2c_init(_dev, 0, I2C_400KHz_SPEED);
    else
	i2c_init(_dev, 2, I2C_400KHz_SPEED);
}
void REVOMINII2CDriver::end() {}

void REVOMINII2CDriver::setHighSpeed(bool active) {

}

uint8_t REVOMINII2CDriver::write(uint8_t addr, uint8_t len, uint8_t* tx_buffer)
{

	uint8_t numbytes = len;

	uint32_t ret = i2c_write(_dev,  addr, tx_buffer, &numbytes);

	if(ret == 1){
	    _lockup_count ++;  //hal.console->printf_P(PSTR("Failed I2C write1: Event=0x%08X\n"),ret);
//	    hal.gpio->write(20, 1);
	}

	return ret;
}


uint8_t REVOMINII2CDriver::writeRegister(uint8_t addr, uint8_t registerAddress, uint8_t databyte)
{
	uint8_t ibuff[2];

	ibuff[0] = registerAddress;
	ibuff[1] = databyte;
	uint8_t numbytes = 2;

	uint8_t ret = i2c_write(_dev, addr, ibuff, &numbytes);

	if(ret == 1){
	     _lockup_count ++;
//	     hal.gpio->write(20, 1);
	}

	return ret;
}

uint8_t REVOMINII2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 0;}


uint8_t REVOMINII2CDriver::read(uint8_t addr, uint8_t numberBytes, uint8_t* data)
{
	uint8_t ret = i2c_read(_dev, addr, NULL, 0, data, &numberBytes);

	if(ret == 1){
	    _lockup_count ++; //hal.console->printf_P(PSTR("Failed I2C read1: Event=0x%08X\n"),ret);
//	    hal.gpio->write(20, 1);
	}
	return ret;
}

uint8_t REVOMINII2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
	uint8_t ibuff[1];

	ibuff[0] = reg;
	uint8_t numberBytes = 1;

	uint8_t ret = i2c_read(_dev, addr, ibuff, 1, data, &numberBytes);

	if(ret == 1){
	    _lockup_count ++; //hal.console->println_P("i2c timeout read register");
//	    hal.gpio->write(20, 1);
	}

	return ret;
}
uint8_t REVOMINII2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t numberBytes, uint8_t* data)
{
	uint8_t ibuff[1];

	uint8_t ret;

	ibuff[0] = reg;
	uint8_t numbytes = numberBytes;

	ret = i2c_read(_dev, addr, ibuff, 1, data, &numbytes);

	if(ret == 1){
	    _lockup_count ++;
//	    hal.gpio->write(20, 1);
	    return ret;
	}

	uint32_t time = i2ctimeout;
	while(numbytes > 0){
	    if ((time--) == 0)
		{
		_lockup_count ++;
//		hal.gpio->write(20, 1);
		return 1;
		}
	}

	return ret;
}

