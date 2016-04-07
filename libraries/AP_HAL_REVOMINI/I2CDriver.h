/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * I2CDriver.h --- AP_HAL_REVOMINI I2C driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by Roberto Navoni  <info@virtualrobotix.com>, 11 January 2013
 */

#ifndef __AP_HAL_REVOMINI_I2CDRIVER_H__
#define __AP_HAL_REVOMINI_I2CDRIVER_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include <i2c.h>

class REVOMINI::REVOMINII2CDriver : public AP_HAL::I2CDriver {
public:
    REVOMINII2CDriver(i2c_dev *dev, AP_HAL::Semaphore* semaphore);
    void begin();
    void end();
    void setTimeout(uint16_t ms){ _timeoutDelay = ms; }
    void setHighSpeed(bool active);

    /* write: for i2c devices which do not obey register conventions */
    uint8_t write(uint8_t addr, uint8_t len, uint8_t* data);
    /* writeRegister: write a single 8-bit value to a register */
    uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val);
    /* writeRegisters: write bytes to contigious registers */
    uint8_t writeRegisters(uint8_t addr, uint8_t reg,
                                   uint8_t len, uint8_t* data);

    /* read: for i2c devices which do not obey register conventions */
    uint8_t read(uint8_t addr, uint8_t len, uint8_t* data);

    /* readRegister: read from a device register - writes the register,
     * then reads back an 8-bit value. */
    uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data);
    /* readRegister: read contigious device registers - writes the first 
     * register, then reads back multiple bytes */
    uint8_t readRegisters(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t* data);

    uint8_t lockup_count(){ return _lockup_count; }

    AP_HAL::Semaphore* get_semaphore() { return _semaphore; }

private:
    i2c_dev *_dev;
    AP_HAL::Semaphore* _semaphore;
    uint8_t _lockup_count;
    uint16_t _timeoutDelay;
};

#endif // __AP_HAL_REVOMINI_I2CDRIVER_H__
