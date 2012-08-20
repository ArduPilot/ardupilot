
#ifndef __AP_HAL_HAL_H__
#define __AP_HAL_HAL_H__

#include "AP_HAL_Namespace.h"

#include "../AP_HAL/UARTDriver.h"
#include "../AP_HAL/SPIDriver.h"
#include "../AP_HAL/AnalogIn.h"
#include "../AP_HAL/Storage.h"
#include "../AP_HAL/Log.h"
#include "../AP_HAL/Console.h"
#include "../AP_HAL/GPIO.h"
#include "../AP_HAL/PPMInput.h"
#include "../AP_HAL/PWMOutput.h"

class AP_HAL::HAL {
public:
    HAL(AP_HAL::UARTDriver* _uart0,
        AP_HAL::UARTDriver* _uart1,
        AP_HAL::UARTDriver* _uart2,
        AP_HAL::UARTDriver* _uart3,
        AP_HAL::I2CDriver*  _i2c,
        AP_HAL::SPIDriver*  _spi,
        AP_HAL::AnalogIn*   _analogIn,
        AP_HAL::Storage*    _storage,
        AP_HAL::Log*        _log,
        AP_HAL::Console*    _console,
        AP_HAL::GPIO*       _gpio,
        AP_HAL::PPMInput*   _ppmin,
        AP_HAL::PWMOutput*  _pwmout)
        :
        uart0(_uart0),
        uart1(_uart1),
        uart2(_uart2),
        uart3(_uart3),
        i2c(_i2c),
        spi(_spi),
        analogIn(_analogIn),
        storage(_storage),
        log(_log),
        console(_console),
        gpio(_gpio),
        ppmin(_ppmin),
        pwmout(_pwmout)
    {}

    AP_HAL::UARTDriver* uart0;
    AP_HAL::UARTDriver* uart1;
    AP_HAL::UARTDriver* uart2;
    AP_HAL::UARTDriver* uart3;

    AP_HAL::I2CDriver*  i2c;
    AP_HAL::SPIDriver*  spi;
    AP_HAL::AnalogIn*   analogIn;

    AP_HAL::Storage*    storage;
    AP_HAL::Log*        log;
    AP_HAL::Console*    console;
    AP_HAL::GPIO*       gpio;
    AP_HAL::PPMInput*   ppmin;
    AP_HAL::PWMOutput*  pwmout;
};

#endif // __AP_HAL_HAL_H__

