
#ifndef __AP_HAL_HAL_H__
#define __AP_HAL_HAL_H__

#include "AP_HAL_Namespace.h"

#include "../AP_HAL/UARTDriver.h"
#include "../AP_HAL/SPIDriver.h"
#include "../AP_HAL/AnalogIn.h"
#include "../AP_HAL/Storage.h"
#include "../AP_HAL/GPIO.h"
#include "../AP_HAL/RCInput.h"
#include "../AP_HAL/RCOutput.h"

class AP_HAL::HAL {
public:
    HAL(AP_HAL::UARTDriver* _uartA, // console
        AP_HAL::UARTDriver* _uartB, // 1st GPS
        AP_HAL::UARTDriver* _uartC, // telem1
        AP_HAL::UARTDriver* _uartD, // telem2
        AP_HAL::UARTDriver* _uartE, // 2nd GPS
        AP_HAL::I2CDriver*  _i2c,
        AP_HAL::I2CDriver*  _i2c2,	// for MPU6050 on YUNEEC Platform
        AP_HAL::SPIDeviceManager* _spi,
        AP_HAL::AnalogIn*   _analogin,
        AP_HAL::Storage*    _storage,
        AP_HAL::UARTDriver* _console,
        AP_HAL::GPIO*       _gpio,
        AP_HAL::RCInput*    _rcin,
        AP_HAL::RCOutput*   _rcout,
        AP_HAL::Scheduler*  _scheduler,
        AP_HAL::Util*       _util)
        :
        uartA(_uartA),
        uartB(_uartB),
        uartC(_uartC),
        uartD(_uartD),
        uartE(_uartE),
        i2c(_i2c),
        i2c2(_i2c2),
        spi(_spi),
        analogin(_analogin),
        storage(_storage),
        console(_console),
        gpio(_gpio),
        rcin(_rcin),
        rcout(_rcout),
        scheduler(_scheduler),
        util(_util)
    {}

    virtual void init(int argc, char * const argv[]) const = 0;

    AP_HAL::UARTDriver* uartA;
    AP_HAL::UARTDriver* uartB;
    AP_HAL::UARTDriver* uartC;
    AP_HAL::UARTDriver* uartD;
    AP_HAL::UARTDriver* uartE;
    AP_HAL::I2CDriver*  i2c;
    AP_HAL::I2CDriver*  i2c2;
    AP_HAL::SPIDeviceManager* spi;
    AP_HAL::AnalogIn*   analogin;
    AP_HAL::Storage*    storage;
    AP_HAL::UARTDriver* console;
    AP_HAL::GPIO*       gpio;
    AP_HAL::RCInput*    rcin;
    AP_HAL::RCOutput*   rcout;
    AP_HAL::Scheduler*  scheduler;
    AP_HAL::Util*       util;
};

#endif // __AP_HAL_HAL_H__

