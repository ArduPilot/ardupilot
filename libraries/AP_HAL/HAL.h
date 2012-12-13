
#ifndef __AP_HAL_HAL_H__
#define __AP_HAL_HAL_H__

#include "AP_HAL_Namespace.h"

#include "../AP_HAL/UARTDriver.h"
#include "../AP_HAL/SPIDriver.h"
#include "../AP_HAL/AnalogIn.h"
#include "../AP_HAL/Storage.h"
#include "../AP_HAL/Console.h"
#include "../AP_HAL/GPIO.h"
#include "../AP_HAL/RCInput.h"
#include "../AP_HAL/RCOutput.h"

class AP_HAL::HAL {
public:
    HAL(AP_HAL::UARTDriver* _uartA,
        AP_HAL::UARTDriver* _uartB,
        AP_HAL::UARTDriver* _uartC,
        AP_HAL::I2CDriver*  _i2c,
        AP_HAL::SPIDeviceManager* _spi,
        AP_HAL::AnalogIn*   _analogin,
        AP_HAL::Storage*    _storage,
        AP_HAL::ConsoleDriver* _console,
        AP_HAL::GPIO*       _gpio,
        AP_HAL::RCInput*    _rcin,
        AP_HAL::RCOutput*   _rcout,
        AP_HAL::Scheduler*  _scheduler)
        :
        uartA(_uartA),
        uartB(_uartB),
        uartC(_uartC),
        i2c(_i2c),
        spi(_spi),
        analogin(_analogin),
        storage(_storage),
        console(_console),
        gpio(_gpio),
        rcin(_rcin),
        rcout(_rcout),
        scheduler(_scheduler)
    {}

    virtual void init(int argc, char * const argv[]) const = 0;

    AP_HAL::UARTDriver* uartA;
    AP_HAL::UARTDriver* uartB;
    AP_HAL::UARTDriver* uartC;
    AP_HAL::I2CDriver*  i2c;
    AP_HAL::SPIDeviceManager* spi;
    AP_HAL::AnalogIn*   analogin;
    AP_HAL::Storage*    storage;
    AP_HAL::ConsoleDriver* console;
    AP_HAL::GPIO*       gpio;
    AP_HAL::RCInput*    rcin;
    AP_HAL::RCOutput*   rcout;
    AP_HAL::Scheduler*  scheduler;
};

#endif // __AP_HAL_HAL_H__

