
#ifndef __AP_HAL_AVR_HAL_AVR_H__
#define __AP_HAL_AVR_HAL_AVR_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

/**
 * HAL_AVR class derives from HAL but provides an AVR-specific
 * init method.
 */

class AP_HAL_AVR::HAL_AVR : public AP_HAL::HAL {
public:
    HAL_AVR(
        AP_HAL::UARTDriver* _uart0,
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
        AP_HAL::RCInput*    _rcin,
        AP_HAL::RCOutput*   _rcout,
        AP_HAL::Scheduler*  _scheduler)
        : AP_HAL::HAL(  _uart0, _uart1, _uart2, _uart3,
                        _i2c, _spi, _analogIn, _storage,
                        _log, _console, _gpio, _rcin,
                        _rcout, _scheduler) {}

    void init(void* opts) const;
};
#endif // __AP_HAL_AVR_HAL_AVR_H__

