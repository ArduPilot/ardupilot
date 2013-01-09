/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * I2CDriver.cpp --- AP_HAL_SMACCM I2C driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 20 December 2012
 */

#include <AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM

#include <AP_HAL.h>
#include <hwf4/i2c.h>
#include <hwf4/gpio.h>

#include "I2CDriver.h"

using namespace SMACCM;

// For now, we are assuming all devices are on a single bus.  This
// will need to be refactored at some point to use a device manager
// like SPI if we need support for multiple busses.
#define I2C_BUS i2c2
#define I2C_SDA pin_b11
#define I2C_SCL pin_b10

void SMACCMI2CDriver::begin()
{
  i2c_init(I2C_BUS, I2C_SDA, I2C_SCL);
  _semaphore.init();
}

// XXX hwf4 doesn't support de-initialization
void SMACCMI2CDriver::end()
{
}

// XXX hwf4 doesn't support non-blocking I2C
void SMACCMI2CDriver::setTimeout(uint16_t ms)
{
}

// XXX hwf4 always uses standard speed
void SMACCMI2CDriver::setHighSpeed(bool active)
{
}

AP_HAL::Semaphore* SMACCMI2CDriver::get_semaphore()
{
  return &_semaphore;
}

uint8_t SMACCMI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
  return i2c_transfer(I2C_BUS, addr, data, len, NULL, 0) ? 0 : 1;
}

uint8_t SMACCMI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
  return i2c_write_reg(I2C_BUS, addr, reg, val) ? 0 : 1;
}

uint8_t SMACCMI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                                        uint8_t len, uint8_t* data)
{
  return i2c_write_regs(I2C_BUS, addr, reg, data, len) ? 0 : 1;
}

uint8_t SMACCMI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
  return i2c_transfer(I2C_BUS, addr, NULL, 0, data, len) ? 0 : 1;
}

uint8_t SMACCMI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
  return i2c_transfer(I2C_BUS, addr, &reg, 1, data, 1) ? 0 : 1;
}

uint8_t SMACCMI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                       uint8_t len, uint8_t* data)
{
  return i2c_transfer(I2C_BUS, addr, &reg, 1, data, len) ? 0 : 1;
}

uint8_t SMACCMI2CDriver::lockup_count()
{
  return 0;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
