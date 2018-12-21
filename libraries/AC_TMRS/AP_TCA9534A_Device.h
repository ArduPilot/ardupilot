#pragma once
/*
 * AP_TCA9534ADevice.h
 *
 *  Created on: Apr 10, 2018
 *      Author: Rob
 */

#ifndef AP_TCA9534ADEVICE_H_
#define AP_TCA9534ADEVICE_H_

#include <inttypes.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

class AP_TCA9534A_Device
{

public:
    AP_TCA9534A_Device();
    AP_TCA9534A_Device(uint8_t i2cBus, uint8_t i2cAddress);

    virtual ~AP_TCA9534A_Device();

    bool init();

    void set_payload(uint8_t payload);

    uint8_t get_payload();

private:
    uint8_t i2c_bus_id;
    uint8_t i2c_address;
    uint8_t payload;

//    typedef enum
//    {
//        REGISTER_INPUT = 0x00,
//        REGISTER_OUTPUT = 0x01,
//        REGISTER_POLARITY_INVERSION = 0x02,
//        REGISTER_CONFIGURATION = 0x03,
//    } registers;

    static const uint8_t REGISTER_INPUT = 0x00;
    static const uint8_t REGISTER_OUTPUT = 0x01;
    static const uint8_t REGISTER_POLARITY_INVERSION = 0x02;
    static const uint8_t REGISTER_CONFIGURATION = 0x03;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    void update();

};

#endif /* AP_TCA9534ADEVICE_H_ */
