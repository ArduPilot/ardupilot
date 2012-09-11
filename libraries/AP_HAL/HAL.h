
#ifndef __AP_HAL_HAL_H__
#define __AP_HAL_HAL_H__

#include "AP_HAL_Namespace.h"

#include "../AP_HAL/UARTDriver.h"
#include "../AP_HAL/SPIDriver.h"
#include "../AP_HAL/AnalogIn.h"
#include "../AP_HAL/Storage.h"
#include "../AP_HAL/Dataflash.h"
#include "../AP_HAL/GPIO.h"
#include "../AP_HAL/RCInput.h"
#include "../AP_HAL/RCOutput.h"

class AP_HAL::HAL {
public:
    HAL(AP_HAL::UARTDriver* _uart0,
        AP_HAL::UARTDriver* _uart1,
        AP_HAL::UARTDriver* _uart2,
        AP_HAL::UARTDriver* _uart3,
        AP_HAL::I2CDriver*  _i2c,
        AP_HAL::SPIDriver*  _spi,
        AP_HAL::AnalogIn*   _analogin,
        AP_HAL::Storage*    _storage,
        AP_HAL::Dataflash*  _dataflash,
        AP_HAL::BetterStream* _console,
        AP_HAL::GPIO*       _gpio,
        AP_HAL::RCInput*    _rcin,
        AP_HAL::RCOutput*   _rcout,
        AP_HAL::Scheduler*  _scheduler)
        :
        uart0(_uart0),
        uart1(_uart1),
        uart2(_uart2),
        uart3(_uart3),
        i2c(_i2c),
        spi(_spi),
        analogin(_analogin),
        storage(_storage),
        dataflash(_dataflash),
        console(_console),
        gpio(_gpio),
        rcin(_rcin),
        rcout(_rcout),
        scheduler(_scheduler)
    {}

    virtual void init(void* opts) const = 0;

    AP_HAL::UARTDriver* uart0;
    AP_HAL::UARTDriver* uart1;
    AP_HAL::UARTDriver* uart2;
    AP_HAL::UARTDriver* uart3;
    AP_HAL::I2CDriver*  i2c;
    AP_HAL::SPIDriver*  spi;
    AP_HAL::AnalogIn*   analogin;
    AP_HAL::Storage*    storage;
    AP_HAL::Dataflash*  dataflash;
    AP_HAL::BetterStream* console;
    AP_HAL::GPIO*       gpio;
    AP_HAL::RCInput*    rcin;
    AP_HAL::RCOutput*   rcout;
    AP_HAL::Scheduler*  scheduler;
};

#endif // __AP_HAL_HAL_H__

