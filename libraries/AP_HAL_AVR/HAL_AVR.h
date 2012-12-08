
#ifndef __AP_HAL_AVR_HAL_AVR_H__
#define __AP_HAL_AVR_HAL_AVR_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"
#include "utility/ISRRegistry.h"

/**
 * HAL_AVR class derives from HAL but provides an AVR-specific
 * init method.
 */

class AP_HAL_AVR::HAL_AVR : public AP_HAL::HAL {
public:
    HAL_AVR(
        AP_HAL::UARTDriver* _uartA,
        AP_HAL::UARTDriver* _uartB,
        AP_HAL::UARTDriver* _uartC,
        AP_HAL::I2CDriver*  _i2c,
        AP_HAL::SPIDeviceManager*  _spi,
        AP_HAL::AnalogIn*   _analogin,
        AP_HAL::Storage*    _storage,
        AP_HAL::ConsoleDriver* _console,
        AP_HAL::GPIO*       _gpio,
        AP_HAL::RCInput*    _rcin,
        AP_HAL::RCOutput*   _rcout,
        AP_HAL::Scheduler*  _scheduler)
        : AP_HAL::HAL(  _uartA, _uartB, _uartC,
                        _i2c, _spi, _analogin, _storage,
                        _console, _gpio, _rcin,
                        _rcout, _scheduler) {}

    void init(void* opts) const;
    AP_HAL_AVR::ISRRegistry isr_registry;
};
#endif // __AP_HAL_AVR_HAL_AVR_H__

