#pragma once

#include "string.h"
#include "utility/functor.h"

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDevice;
    class I2CDeviceManager;
    class I2CDriver;
    class Device;

    class SPIDevice;
    class SPIDeviceDriver;
    class SPIDeviceManager;

    class AnalogSource;
    class AnalogIn;
    class Storage;
    class DigitalSource;
    class GPIO;
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Semaphore;
    class OpticalFlow;
    
    class Util;

    /* Utility Classes */
    class Print;
    class Stream;
    class BetterStream;

    /* Typdefs for function pointers (Procedure, Member Procedure) 

       For member functions we use the FastDelegate delegates class
       which allows us to encapculate a member function as a type
     */
    typedef void(*Proc)(void);
    FUNCTOR_TYPEDEF(MemberProc, void);

    /**
     * Global names for all of the existing SPI devices on all platforms.
     */

    enum SPIDeviceType {
        SPIDevice_Dataflash         = 0,
        SPIDevice_ADS7844           = 1,
        SPIDevice_MS5611            = 2,
        SPIDevice_MPU6000           = 3,
        SPIDevice_ADNS3080_SPI0     = 4,
        SPIDevice_ADNS3080_SPI3     = 5,
        SPIDevice_MPU9250           = 6,
        SPIDevice_L3GD20            = 7,
        SPIDevice_LSM303D           = 8,
        SPIDevice_LSM9DS0_AM        = 9,
        SPIDevice_LSM9DS0_G         = 10,
        SPIDevice_Ublox             = 11,
        SPIDevice_RASPIO            = 12
    };

    // Must be implemented by the concrete HALs.
    const HAL& get_HAL();
}
