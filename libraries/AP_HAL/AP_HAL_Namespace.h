
#ifndef __AP_HAL_NAMESPACE_H__
#define __AP_HAL_NAMESPACE_H__

#include "string.h"
#include "utility/FastDelegate.h"

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDriver;

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
    typedef fastdelegate::FastDelegate0<> MemberProc;

    /**
     * Global names for all of the existing SPI devices on all platforms.
     */

    enum SPIDevice {
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
        SPIDevice_Ublox             = 11
    };

}

// macro to hide the details of AP_HAL::MemberProc
#define AP_HAL_MEMBERPROC(func) fastdelegate::MakeDelegate(this, func)

#endif // __AP_HAL_NAMESPACE_H__
